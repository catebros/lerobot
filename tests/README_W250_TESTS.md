# W250 Robot Tests - User Guide

Complete testing suite for the WidowX-250 robot with Interbotix interface and joystick teleoperation.

## 📋 Available Tests

### 🚀 Quick Tests (< 30 seconds)

#### 1. **Quick Check** - Fast verification
```bash
python3 test_w250_quick_check.py
```
**What it does:** Verifies basic functionality in under 30 seconds
- ✓ Connection
- ✓ Observation reading
- ✓ One arm movement
- ✓ Gripper open/close
- ✓ Clean disconnect

**When to use:** Quick verification after code changes or before starting work

---

### 🔧 Component Tests (1-3 minutes)

#### 2. **Connection Test**
```bash
python3 test_w250_connection.py
```
**What it does:** Tests connection and calibration
- Robot connection
- ROS2 setup verification
- Calibration to home position

#### 3. **Joint Movement Test**
```bash
python3 test_w250_joints.py
```
**What it does:** Tests all arm joints individually
- Reading joint positions
- Small movements
- Return to home
- Non-blocking verification

#### 4. **Gripper Test**
```bash
python3 test_w250_gripper.py
```
**What it does:** Comprehensive gripper testing
- Position reading
- Open/close commands
- Multiple cycles
- No loop verification

#### 5. **Observation Test**
```bash
python3 test_w250_observations.py
```
**What it does:** Tests observation interface
- Reading all features
- Timing verification
- Camera integration (if configured)

---

### 🎯 Integration Tests (3-5 minutes)

#### 6. **Complete Integration Test** ⭐ RECOMMENDED
```bash
python3 test_w250_complete_integration.py
```
**What it does:** End-to-end test of all functionality
- Connection and calibration
- All joint movements
- Gripper control
- Combined arm + gripper movements (pick and place simulation)
- Observation interface
- Clean disconnect

**When to use:**
- After major code changes
- Before deploying to production
- To verify complete setup

---

### 🎮 Teleoperation Tests

#### 7. **Joystick Teleoperation Test** 🕹️
```bash
python3 test_w250_joystick_teleop.py
```

**Prerequisites:**
1. Logitech F710 gamepad connected
2. Mode switch set to 'X' (XInput mode)
3. ROS2 joy node running:
   ```bash
   ros2 run joy joy_node
   ```

**What it does:**
- Tests gamepad connection
- Individual control testing
- **Interactive free control mode**
- Full teleoperation workflow

**Controls:**
- **Left Stick X**: Waist rotation
- **Left Stick Y**: Shoulder up/down
- **Right Stick X**: Wrist angle
- **Right Stick Y**: Elbow up/down
- **LT/RT**: Wrist rotate
- **LB**: Open gripper
- **RB**: Close gripper
- **Y**: Reset to home
- **Start**: Exit test

---

### 🔍 Diagnostic Tests (Advanced)

#### 8. **Single Gripper Command Test**
```bash
python3 test_w250_single_gripper_command.py
```
**Purpose:** Verify gripper receives exactly one command per action

#### 9. **Gripper Loop Diagnosis**
```bash
python3 test_w250_gripper_loop_diagnosis.py
```
**Purpose:** Monitor all gripper commands to detect loops

#### 10. **Gripper Inspection**
```bash
python3 test_w250_gripper_inspect.py
```
**Purpose:** Inspect gripper object methods and attributes

---

## 🎯 Recommended Test Workflow

### For First-Time Setup
```bash
# 1. Quick check
python3 test_w250_quick_check.py

# 2. If successful, run full integration
python3 test_w250_complete_integration.py

# 3. If using joystick, test teleoperation
# (Start joy node first: ros2 run joy joy_node)
python3 test_w250_joystick_teleop.py
```

### For Daily Development
```bash
# Quick verification before starting work
python3 test_w250_quick_check.py

# Test specific component you're working on
python3 test_w250_gripper.py  # or joints, observations, etc.
```

### Before Deployment
```bash
# Full integration test
python3 test_w250_complete_integration.py

# Teleoperation test (if applicable)
python3 test_w250_joystick_teleop.py
```

### When Debugging Issues
```bash
# Test individual components
python3 test_w250_connection.py
python3 test_w250_joints.py
python3 test_w250_gripper.py

# For gripper issues specifically
python3 test_w250_single_gripper_command.py
python3 test_w250_gripper_loop_diagnosis.py
```

---

## 📊 Test Output Guide

### Success Indicators ✓
- `✓` marks indicate successful test steps
- `INFO` level logs show normal operation
- Clean disconnect with no loops

### Warning Indicators ⚠
- `⚠` marks indicate non-critical issues
- May indicate timing issues or optional features
- Usually safe to ignore if test passes

### Error Indicators ✗
- `✗` marks indicate test failures
- `ERROR` level logs show problems
- Test will exit with code 1

---

## 🔧 Troubleshooting

### Test fails at connection
```bash
# Check ROS2 is running
ros2 node list

# Check if control node is running
ros2 launch interbotix_xsarm_control xsarm_control.launch.py robot_model:=wx250
```

### Gripper test fails
```bash
# Run diagnostic test
python3 test_w250_gripper_loop_diagnosis.py

# Check gripper timer is disabled (should see in logs):
# "INFO - Gripper state timer DISABLED to prevent loops"
```

### Joystick test fails
```bash
# Check joy node is running
ros2 node list | grep joy

# Start joy node if needed
ros2 run joy joy_node

# Check joystick is detected
ros2 topic echo /joy
# (You should see output when moving joystick)
```

### Tests are slow
```bash
# Tests involve real robot movements - expected times:
# - Quick check: ~25-30 seconds
# - Component tests: 1-3 minutes
# - Integration test: 3-5 minutes
# - Joystick test: Variable (interactive)
```

---

## 🎮 Joystick Setup Guide

### Hardware Setup
1. **Connect Logitech F710 gamepad** via USB or wireless dongle
2. **Set mode switch to 'X'** (XInput mode, on back of gamepad)
3. **Turn on gamepad** (LED should light up)

### Software Setup
```bash
# Install ROS2 joy package (if not installed)
sudo apt install ros-<distro>-joy

# Start joy node
ros2 run joy joy_node

# Verify joystick is detected
ros2 topic echo /joy
# Move joysticks - you should see axes values changing
```

### Gamepad Not Detected?
```bash
# Check USB connection
lsusb | grep Logitech

# Check device permissions
ls -l /dev/input/js*

# Add user to input group if needed
sudo usermod -a -G input $USER
# (Log out and back in for changes to take effect)
```

---

## 📝 Creating Custom Tests

### Template for New Tests
```python
#!/usr/bin/env python3
import logging
import sys
import time
from pathlib import Path

repo_root = Path(__file__).parent.parent
if (repo_root / "src" / "lerobot").exists():
    sys.path.insert(0, str(repo_root / "src"))

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def test_my_feature():
    from lerobot.robots.w250 import W250Interbotix, W250InterbotixConfig

    config = W250InterbotixConfig(
        robot_model="wx250",
        robot_name="wx250",
    )
    robot = W250Interbotix(config)

    try:
        robot.connect(calibrate=True)

        # Your test code here

        robot.disconnect()
        return True
    except Exception as e:
        logger.error(f"Test failed: {e}")
        try:
            robot.disconnect()
        except:
            pass
        return False

if __name__ == "__main__":
    success = test_my_feature()
    sys.exit(0 if success else 1)
```

---

## 🐛 Reporting Issues

If tests fail consistently:

1. **Note the exact error message**
2. **Check which test fails** (connection, joints, gripper, etc.)
3. **Run diagnostic tests** to gather more info
4. **Check logs** for specific error details
5. **Provide test output** when reporting issues

---

## 📚 Additional Resources

- **Gripper Fix Documentation**: `README_W250_GRIPPER_FIX.md`
- **Interbotix Documentation**: https://docs.trossenrobotics.com/
- **ROS2 Joy Package**: https://index.ros.org/p/joy/

---

**Happy Testing! 🚀**
