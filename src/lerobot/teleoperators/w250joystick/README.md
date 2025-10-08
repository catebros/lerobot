# W250 Logitech F710 Gamepad Teleoperator

This teleoperator enables control of the WidowX-250 robot using a Logitech F710 gamepad via ROS2.

## Hardware Setup

1. **Connect the F710 gamepad**:
   - Via USB cable, or
   - Via wireless USB dongle

2. **Set the mode switch**:
   - **IMPORTANT**: The mode switch on the back of the F710 must be set to **'X'** (XInput mode)
   - DirectInput mode ('D') is not supported

3. **Verify connection**:
   ```bash
   # List connected joysticks
   ls /dev/input/js*

   # Test with jstest (optional)
   jstest /dev/input/js0
   ```

## ROS2 Setup

1. **Install ROS2 joy package** (if not already installed):
   ```bash
   sudo apt install ros-${ROS_DISTRO}-joy
   ```

2. **Run the joy node**:
   ```bash
   ros2 run joy joy_node
   ```

3. **Verify joy messages** (optional):
   ```bash
   ros2 topic echo /joy
   ```

## Control Mapping

### Analog Sticks (Continuous Control)

- **Left Stick X (←→)**: Waist rotation
- **Left Stick Y (↑↓)**: Shoulder (arm up/down)
- **Right Stick X (←→)**: Wrist angle
- **Right Stick Y (↑↓)**: Elbow (extend/contract)

### Triggers (Analog)

- **LT (Left Trigger)**: Wrist rotate counter-clockwise
- **RT (Right Trigger)**: Wrist rotate clockwise

### Shoulder Buttons

- **LB (Left Bumper)**: Open gripper (hold)
- **RB (Right Bumper)**: Close gripper (hold)

### Face Buttons

- **Y (Triangle)**: Reset to home position
- **A, B, X**: Currently unused

### System Buttons

- **Back**: Toggle intervention (emergency stop)
- **Start**: Rerecord episode

## Control Strategy

The teleoperator uses **incremental position control**:

1. **Accumulative positions**: Joint positions accumulate over time as you hold the sticks
2. **Multi-joint control**: Move multiple joints simultaneously
3. **Smooth movements**: Positions are clamped to valid ranges (-1 to 1 for arm joints, 0 to 1 for gripper)
4. **Deadzone**: Small stick movements (< 0.1 by default) are ignored to prevent drift

## Running the Teleoperator

### Basic Usage

```python
from lerobot.teleoperators.w250joystick import W250JoystickConfig, W250JoystickTeleop
from lerobot.robots.w250 import W250InterbotixConfig, W250Interbotix

# Configure teleoperator
teleop_config = W250JoystickConfig(
    x_step_size=0.015,      # Shoulder increment
    y_step_size=0.015,      # Waist increment
    z_step_size=0.015,      # Elbow increment
    wrist_step_size=0.01,   # Wrist angle/rotate increment
    deadzone=0.15
)
teleop = W250JoystickTeleop(teleop_config)

# Configure robot
robot_config = W250InterbotixConfig(
    robot_model="wx250s",
    moving_time=0.5  # Faster for teleoperation
)
robot = W250Interbotix(robot_config)

# Connect
teleop.connect()
robot.connect(calibrate=False)

# Control loop
while True:
    action = teleop.get_action()  # Returns absolute joint positions
    events = teleop.get_teleop_events()

    if events.get('is_intervention', True):
        robot.send_action(action)
```

### Test Script

Run the included test script:

```bash
cd examples
python test_w250_joycon.py
```

## Configuration Parameters

### Step Sizes

Control how fast joints move per iteration:

- `x_step_size`: Shoulder movement increment (default: 0.01)
- `y_step_size`: Waist rotation increment (default: 0.01)
- `z_step_size`: Elbow movement increment (default: 0.01)
- `wrist_step_size`: Wrist angle/rotate increment (default: 0.008)

**Tip**: Increase for faster movement, decrease for finer control

### Deadzone

Joystick deadzone to prevent drift (default: 0.1):

```python
teleop_config = W250JoystickConfig(
    deadzone=0.15  # Larger deadzone = less sensitive
)
```

## Axis and Button Mappings (XInput Mode)

These are the default mappings for F710 in XInput mode. Adjust if needed:

### Axes
- 0: Left stick X
- 1: Left stick Y
- 2: Left trigger (LT)
- 3: Right stick X
- 4: Right stick Y
- 5: Right trigger (RT)

### Buttons
- 0: A
- 1: B
- 2: X
- 3: Y
- 4: LB
- 5: RB
- 6: Back
- 7: Start
- 8: Logitech (center)
- 9: Left stick button (L3)
- 10: Right stick button (R3)

## Troubleshooting

### Gamepad not detected

```bash
# Check if device exists
ls /dev/input/js*

# Check permissions
sudo chmod a+rw /dev/input/js0

# Verify joy node is running
ros2 node list | grep joy
```

### Wrong mode

If buttons don't work correctly, check the mode switch is set to **'X'** (not 'D').

### Inverted axes

If movement directions feel wrong, you can adjust signs in the code:

```python
# In get_joint_increments() method
increments["shoulder.pos"] = -left_stick_y * self.config.x_step_size  # Toggle the minus
```

### Trigger not working

Triggers may require calibration. Check raw values:

```bash
ros2 topic echo /joy
```

LT/RT should go from -1.0 (unpressed) to 1.0 (fully pressed).

## Physical Robot vs Simulation

This teleoperator works with both simulation and physical robots. For physical robots:

1. Ensure robot is powered and connected
2. Use `calibrate=False` when connecting to avoid automatic movements
3. Start with smaller step sizes for safety
4. Keep emergency stop (Back button) easily accessible

## Safety

- **Emergency Stop**: Press the Back button to toggle intervention (stops robot movement)
- **Home Position**: Press Y button to reset all joints to home (0 position, gripper half-open)
- **Intervention Flag**: By default, intervention is enabled. Toggle with Back button.

## Advanced Usage

### Custom Button Mappings

Modify `config_w250joystick.py` to change button assignments:

```python
button_y: int = 3       # Y button - Reset to home position
button_back: int = 6    # Back button - Toggle intervention
button_start: int = 7   # Start button - Rerecord episode
```

### Adjusting Control Speed

Fine-tune movement speed by adjusting step sizes:

```python
# Slower, more precise control
teleop_config = W250JoystickConfig(
    x_step_size=0.005,
    y_step_size=0.005,
    z_step_size=0.005,
    wrist_step_size=0.003,
)

# Faster control
teleop_config = W250JoystickConfig(
    x_step_size=0.025,
    y_step_size=0.025,
    z_step_size=0.025,
    wrist_step_size=0.015,
)
```
