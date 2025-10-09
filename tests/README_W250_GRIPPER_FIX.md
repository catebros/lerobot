# W250 Gripper Loop Fix - Documentation

## Problem Solved
The W250 gripper was entering an **infinite open/close loop** when using the Interbotix ROS2 API. This was caused by the `tmr_gripper_state` timer callback running at 50Hz and continuously publishing gripper commands.

## Solution Implemented
1. **Disabled the timer callback** (`tmr_gripper_state`) immediately after connection
2. **Direct effort commands** - Publish gripper effort commands once without triggering the timer
3. **Auto-stop mechanism** - A background thread sends `effort=0` after 0.5s to stop movement
4. **Clean disconnect** - Cancels timer, stops gripper movement, and disables torque

## Files Modified
- `src/lerobot/robots/w250/w250_interbotix.py`
  - Line 200-204: Disabled timer callback on connect
  - Line 448-486: New gripper control using direct effort commands with auto-stop
  - Line 491-513: Enhanced disconnect to stop gripper cleanly
- `tests/test_w250_gripper.py` - Adjusted timing for new implementation
- `src/lerobot/teleoperators/w250joystick/w250joystick.py` - Fixed gripper increment directions

## How It Works

### Gripper Control Flow
```
1. send_action(gripper.pos=1.0) called
   ↓
2. Calculate effort (+value for open, -value for close)
   ↓
3. Publish effort command ONCE to ROS2
   ↓
4. Start background thread with 0.5s timer
   ↓
5. Gripper moves (motor receives PWM/current)
   ↓
6. After 0.5s: Thread publishes effort=0 to stop
   ↓
7. Gripper stops (no loop!)
```

### Key Parameters
- **Auto-stop delay**: 0.5 seconds (configurable in line 476)
- **Gripper threshold**: 0.1 (prevents duplicate commands)
- **Effort values**:
  - Open: +gripper_value (~+250 PWM)
  - Close: -gripper_value (~-250 PWM)
  - Stop: 0.0

## Testing

### Basic Gripper Test
```bash
cd ~/lerobot/tests
python3 test_w250_gripper.py
```
**Expected behavior:**
- ✅ Gripper opens and closes smoothly
- ✅ No infinite loops
- ✅ Clean disconnect

### Single Command Test
```bash
python3 test_w250_single_gripper_command.py
```
**Tests:**
- Single open command
- Single close command
- Duplicate command filtering (threshold)
- No loop after disconnect

### Diagnostic Tests (for debugging)
These tests were created to diagnose the loop issue:
- `test_w250_gripper_loop_diagnosis.py` - Monitors all gripper commands
- `test_w250_minimal_gripper.py` - Minimal test to isolate the issue
- `test_w250_gripper_inspect.py` - Inspects gripper object methods

**You can delete these after confirming everything works.**

## Joystick Control

### Button Mapping
- **LB (Left Bumper)**: Open gripper (increment +0.02)
- **RB (Right Bumper)**: Close gripper (increment -0.02)

### Why Small Increments?
- Increments of 0.02 provide fine control
- Multiple button presses accumulate to 0.1+ to trigger actual movement
- Prevents accidental gripper commands from brief button taps

## Configuration

### Adjusting Auto-Stop Time
In `w250_interbotix.py` line 476:
```python
time.sleep(0.5)  # Wait for gripper to move
```
- **Increase** if gripper doesn't fully open/close
- **Decrease** for faster response (may not complete movement)

### Adjusting Gripper Threshold
In `w250_interbotix.py` line 62:
```python
self._gripper_threshold: float = 0.1
```
- **Increase** to reduce gripper command frequency
- **Decrease** for more responsive gripper (may cause more commands)

## Troubleshooting

### Gripper doesn't move
- Check that ROS2 control node is running:
  ```bash
  ros2 node list | grep wx250
  ```
- Verify gripper effort value:
  ```bash
  ros2 topic echo /wx250/commands/joint_single
  ```

### Gripper still loops
- Check that timer was cancelled (should see in logs):
  ```
  INFO - Gripper state timer DISABLED to prevent loops
  ```
- If not, the timer cancellation may have failed

### Gripper moves too slow/fast
- Adjust the effort value by changing `gripper_pressure` in config:
  ```python
  config = W250InterbotixConfig(
      gripper_pressure=0.5,  # 0.0 to 1.0
      gripper_pressure_lower_limit=150,
      gripper_pressure_upper_limit=350,
  )
  ```

## API Reference

### Interbotix Gripper Methods (DO NOT USE - cause loops)
❌ `gripper.grasp(delay)` - Closes gripper but triggers timer callback
❌ `gripper.release(delay)` - Opens gripper but triggers timer callback
❌ `gripper.set_pressure(pressure)` - Only sets pressure, doesn't move gripper

### Our Implementation (SAFE)
✅ Direct effort publishing via `gripper.core.pub_single.publish()`
✅ Auto-stop thread to prevent loops
✅ Timer callback disabled on connect

## Credits
Fixed by analyzing the Interbotix gripper source code and identifying the timer callback loop issue.
