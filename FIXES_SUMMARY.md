# NOMAD Setup - Fixes and Updates

## Fixed Issues

### 1. ✅ Pip Wheel Error ("Getting requirements to build wheel")
**Problem**: Package compilation failures on Raspberry Pi
**Solution**:
- Pinned specific versions in `requirements.txt` (pre-built wheel versions)
- Created `setup.sh` script that:
  - Installs system dependencies and build tools
  - Upgrades pip, setuptools, and wheel
  - Installs packages in optimal order
  
**Usage**:
```bash
bash setup.sh
source venv/bin/activate
pip install -r requirements.txt
```

### 2. ✅ Expanded COCO Dataset
**Added 25+ everyday items** to `coco.names`:
- Building elements: door, window, wall, floor, ceiling, staircase
- Furniture: table, desk, cabinet, shelves, closet
- Rooms: bathroom, kitchen, bedroom, livingroom
- Environment: tree, grass, sand, water, sky, clouds
- Safety: obstacle, railing

### 3. ✅ Virtual Environment (venv)
**Do you need venv?** YES - Recommended!
- **venv**: Python's built-in virtual environment tool
- **venv-1**: Not a standard tool - use just `venv`
- **Benefits**: Isolates dependencies, prevents conflicts with system Python, easy to remove/reset

**Quick setup**:
```bash
python3 -m venv venv
source venv/bin/activate
```

### 4. ✅ Easy Destination Configuration
**Changed destinations from hardcoded to configurable**

**How to change destinations**:
1. Open `destinations.py`
2. Add or modify entries in `DESTINATIONS` dictionary:
```python
DESTINATIONS = {
    'home': (40.7128, -74.0060),      # Add your location
    'warehouse': (51.5074, -0.1278),   # Add another location
}
```
3. Run: `python drone_control.py`
4. Enter destination name when prompted (case-insensitive)

**Flight parameter settings** (in `destinations.py`):
- `TAKEOFF_ALTITUDE`: Height in meters
- `FLIGHT_SPEED`: Forward velocity (m/s)
- `OBSTACLE_TURN_RATE`: Rotation speed when avoiding
- `OBSTACLE_OBJECTS`: Objects to avoid (add/remove as needed)

## Files Created/Modified

### New Files
- `setup.sh` - Automated setup script
- `destinations.py` - Easy configuration file
- `test_setup.py` - Improved with visual feedback

### Modified Files
- `requirements.txt` - Pinned versions, removed problematic packages
- `drone_control.py` - Uses imports from destinations.py
- `coco.names` - Expanded with 25+ everyday items
- `README.md` - Updated with venv and configuration instructions

## Quick Summary

| Issue | Before | After |
|-------|--------|-------|
| Pip errors | Generic packages | Pinned versions + setup script |
| COCO items | 80 standard classes | 105+ classes (added everyday items) |
| Virtual env | Not mentioned | Full venv setup guide + setup.sh |
| Destinations | Hardcoded in code | Easy `destinations.py` config |
| Configuration | Hard to change | Simple text file editing |

## Next Steps

1. Run setup: `bash setup.sh`
2. Download YOLO files (see README)
3. Configure hardware connections
4. Edit `destinations.py` with your GPS coordinates
5. Test: `python test_setup.py`
6. Run drone: `python drone_control.py`