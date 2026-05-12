#!/usr/bin/env python3
"""
hover.py — NOMAD Hover Mode Controller
=======================================
Standalone script that commands the drone to hold its current position
(LOITER mode on ArduPilot). Also exposes hover/land control through the
shared web_server state so the web UI can trigger it.

Usage
-----
  Activate hover while drone is already airborne:
    python hover.py

  Activate hover then land after N seconds:
    python hover.py --land-after 30

  Activate hover and return to web-UI control:
    python hover.py --web

  Disable hover (initiate landing):
    python hover.py --land

  Dry run (no hardware):
    python hover.py --dry-run
"""

import argparse
import logging
import sys
import time
import threading
import signal

# ── Optional hardware imports (same pattern as drone_control.py) ──────────────
try:
    from pymavlink import mavutil
except ImportError:
    mavutil = None

try:
    from web_server import get_state, set_state, start_server_background
    _web_server_available = True
except ImportError:
    _web_server_available = False
    def get_state(key): return None
    def set_state(**kw): pass

# ── Constants ─────────────────────────────────────────────────────────────────
SERIAL_PORT   = '/dev/serial0'
BAUD_RATE     = 57600

MODE_STABILIZE = 0
MODE_GUIDED    = 4
MODE_LOITER    = 5   # ArduPilot LOITER — holds GPS position + altitude
MODE_LAND      = 9

HOVER_CHECK_INTERVAL = 0.5   # seconds between state checks while hovering
LAND_SETTLE_TIME     = 6     # seconds to wait after issuing LAND before disarming

# ── Logging ───────────────────────────────────────────────────────────────────
logger = logging.getLogger('nomad.hover')
logger.setLevel(logging.INFO)
_sh = logging.StreamHandler(sys.stdout)
_sh.setFormatter(logging.Formatter('%(asctime)s %(levelname)s [hover]: %(message)s'))
logger.addHandler(_sh)
_fh = logging.FileHandler('nomad_flight.log')
_fh.setFormatter(logging.Formatter('%(asctime)s %(levelname)s [hover]: %(message)s'))
logger.addHandler(_fh)

# ── Shared hover state (read by web_server routes added below) ────────────────
_hover_state = {
    'active':     False,   # True while LOITER is engaged
    'countdown':  None,    # int seconds remaining in pre-hover countdown, or None
    'landing':    False,   # True once land sequence has started
}
_hover_lock = threading.Lock()

def _hset(**kw):
    with _hover_lock:
        _hover_state.update(kw)

def _hget(key):
    with _hover_lock:
        return _hover_state.get(key)


# ── MAVLink helpers ───────────────────────────────────────────────────────────
def connect_mavlink(dry_run=False):
    if dry_run or mavutil is None:
        logger.info('MAVLink: skipped (dry-run or not installed)')
        return None
    try:
        master = mavutil.mavlink_connection(SERIAL_PORT, baud=BAUD_RATE)
        master.wait_heartbeat(timeout=10)
        logger.info('MAVLink connected (sys=%d comp=%d)',
                    master.target_system, master.target_component)
        return master
    except Exception as exc:
        logger.error('MAVLink connection failed: %s', exc)
        return None


def set_mode(master, mode_id, mode_name=''):
    """Send a flight-mode change and log it."""
    if master is None:
        logger.info('[DRY-RUN] set_mode → %s (%d)', mode_name or mode_id, mode_id)
        return
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id)
    logger.info('Mode → %s', mode_name or str(mode_id))


def send_zero_velocity(master):
    """
    Send a zero-velocity body-frame command so the FC doesn't drift while
    we're waiting for LOITER to engage. Safe to call repeatedly.
    """
    if master is None:
        return
    master.mav.set_position_target_local_ned_send(
        0,
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111,  # ignore pos & accel, use velocity + yaw rate
        0, 0, 0,
        0, 0, 0,             # zero velocity
        0, 0, 0,
        0, 0)


def disarm_drone(master):
    if master is None:
        logger.info('[DRY-RUN] disarm')
        return
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        0, 0, 0, 0, 0, 0, 0)
    logger.info('Drone DISARMED')


# ── Core hover/land actions ───────────────────────────────────────────────────
def activate_hover(master):
    """
    Engage LOITER mode so ArduPilot holds the drone's current GPS position
    and altitude without any velocity commands from us.
    """
    logger.info('Activating HOVER (LOITER mode)…')
    send_zero_velocity(master)
    time.sleep(0.1)
    set_mode(master, MODE_LOITER, 'LOITER')
    _hset(active=True, landing=False)
    set_state(mode='hover')
    logger.info('HOVER active — drone holding position. Press Ctrl-C or call --land to descend.')


def deactivate_hover_land(master):
    """
    Exit hover and initiate a controlled landing.
    ArduPilot LAND mode descends and auto-disarms on touchdown.
    """
    if _hget('landing'):
        return   # already landing
    logger.info('Deactivating hover — initiating LAND sequence…')
    _hset(active=False, landing=True)
    set_mode(master, MODE_LAND, 'LAND')
    set_state(mode='landing')
    time.sleep(LAND_SETTLE_TIME)
    disarm_drone(master)
    set_state(mode='idle')
    _hset(landing=False)
    logger.info('Landing complete.')


def hover_with_countdown(master, countdown_seconds=5):
    """
    Display a countdown, then engage hover (LOITER).
    Used by the web UI 'HOVER' button.
    """
    logger.info('Hover countdown starting: %d seconds…', countdown_seconds)
    for i in range(countdown_seconds, 0, -1):
        _hset(countdown=i)
        logger.info('Hover in %d…', i)
        time.sleep(1)
    _hset(countdown=None)
    activate_hover(master)


# ── Blocking hover loop (used by standalone CLI) ──────────────────────────────
def run_hover_loop(master, land_after=None):
    """
    Block until:
      • Ctrl-C is pressed
      • land_after seconds have elapsed (if set)
      • The web_server emergency_stop flag is set
      • The web_server mode is changed away from 'hover' by another process
    """
    start = time.time()
    logger.info('Hover loop running (land_after=%s)…', land_after)

    def _sigint(sig, frame):
        logger.info('Ctrl-C received — landing.')
        deactivate_hover_land(master)
        sys.exit(0)

    signal.signal(signal.SIGINT, _sigint)

    while True:
        # Check for timed auto-land
        if land_after is not None and (time.time() - start) >= land_after:
            logger.info('land_after=%d reached — landing.', land_after)
            deactivate_hover_land(master)
            break

        # Respect emergency stop from web server
        if get_state('emergency_stop'):
            logger.critical('Emergency stop detected — landing immediately.')
            deactivate_hover_land(master)
            break

        # Another process may have switched mode away (e.g. drone_control resuming)
        current_mode = get_state('mode')
        if current_mode not in (None, 'hover'):
            logger.info('Mode changed to %r externally — exiting hover loop.', current_mode)
            break

        time.sleep(HOVER_CHECK_INTERVAL)


# ── Web-server API extensions ─────────────────────────────────────────────────
def register_hover_routes(app, master_ref):
    """
    Add /api/hover and /api/hover/land routes to the existing Flask app.
    Call this after importing `app` from web_server if you want the web UI
    to be able to trigger hover. master_ref is a list so the value can be
    updated after connect (mutable container trick).
    """
    from flask import jsonify, request as freq

    @app.route('/api/hover', methods=['POST'])
    def api_hover():
        """
        Begin a countdown then engage hover (LOITER).
        Body: { "countdown": 5 }   (default 5 seconds)
        """
        data      = freq.json or {}
        countdown = int(data.get('countdown', 5))
        master    = master_ref[0]

        if get_state('emergency_stop'):
            return jsonify({'error': 'Emergency stop is active'}), 403

        # Run countdown + hover in a background thread so the HTTP response
        # returns immediately and the countdown ticks in the background.
        t = threading.Thread(
            target=hover_with_countdown,
            args=(master, countdown),
            daemon=True,
        )
        t.start()
        return jsonify({'success': True, 'message': f'Hover countdown started ({countdown}s)'})

    @app.route('/api/hover/land', methods=['POST'])
    def api_hover_land():
        """Disable hover and initiate landing."""
        master = master_ref[0]
        if not _hget('active'):
            return jsonify({'error': 'Hover is not currently active'}), 400
        t = threading.Thread(
            target=deactivate_hover_land,
            args=(master,),
            daemon=True,
        )
        t.start()
        return jsonify({'success': True, 'message': 'Landing initiated'})

    @app.route('/api/hover/status', methods=['GET'])
    def api_hover_status():
        with _hover_lock:
            return jsonify(dict(_hover_state))


# ── CLI ───────────────────────────────────────────────────────────────────────
def parse_args():
    p = argparse.ArgumentParser(
        description='NOMAD hover mode controller — ArduPilot LOITER',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    p.add_argument('--land',       action='store_true',
                   help='Immediately exit hover and land (overrides --hover)')
    p.add_argument('--land-after', type=int, default=None, metavar='SECONDS',
                   help='Hover for N seconds then auto-land')
    p.add_argument('--countdown',  type=int, default=5, metavar='SECONDS',
                   help='Pre-hover countdown in seconds (default: 5)')
    p.add_argument('--no-countdown', action='store_true',
                   help='Skip countdown and engage hover immediately')
    p.add_argument('--web',        action='store_true',
                   help='Start web server alongside hover control')
    p.add_argument('--port',       type=int, default=5000,
                   help='Web server port (default: 5000)')
    p.add_argument('--dry-run',    action='store_true',
                   help='No hardware interaction — simulate commands only')
    return p.parse_args()


def main():
    args   = parse_args()
    master = connect_mavlink(dry_run=args.dry_run)

    # ── Optional web server ───────────────────────────────────────────────────
    if args.web and _web_server_available:
        from web_server import app
        master_ref = [master]
        register_hover_routes(app, master_ref)
        start_server_background('0.0.0.0', args.port)
        logger.info('Web server started on port %d', args.port)

    # ── --land flag: exit hover, descend, done ────────────────────────────────
    if args.land:
        _hset(active=True)   # pretend we were hovering so deactivate works
        deactivate_hover_land(master)
        return

    # ── Normal hover activation ───────────────────────────────────────────────
    if args.no_countdown:
        activate_hover(master)
    else:
        hover_with_countdown(master, countdown_seconds=args.countdown)

    # ── Block until done ──────────────────────────────────────────────────────
    run_hover_loop(master, land_after=args.land_after)


if __name__ == '__main__':
    main()
