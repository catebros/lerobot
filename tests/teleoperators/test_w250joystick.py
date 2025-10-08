#!/usr/bin/env python

# Copyright 2025 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from unittest.mock import MagicMock, Mock, patch

import pytest

from lerobot.teleoperators.w250joystick import W250JoystickConfig, W250JoystickTeleop


def _make_joy_message(axes=None, buttons=None):
    """Create a mock Joy message."""
    msg = Mock()
    msg.axes = axes if axes is not None else [0.0] * 6
    msg.buttons = buttons if buttons is not None else [0] * 12
    return msg


def _make_ros2_mock():
    """Create mocks for ROS2 components."""
    # Mock rclpy module
    rclpy_mock = MagicMock()
    rclpy_mock.ok.return_value = False  # Start as not initialized
    rclpy_mock.init = MagicMock(side_effect=lambda: setattr(rclpy_mock, "ok", lambda: True))
    rclpy_mock.executors.SingleThreadedExecutor = MagicMock

    return rclpy_mock


@pytest.fixture
def w250_teleop():
    """Create a W250JoystickTeleop instance with mocked ROS2."""
    with (
        patch("lerobot.teleoperators.w250joystick.w250joystick.rclpy", _make_ros2_mock()),
        patch("lerobot.teleoperators.w250joystick.w250joystick.Node"),
    ):
        config = W250JoystickConfig()
        teleop = W250JoystickTeleop(config)
        yield teleop
        if teleop.is_connected():
            teleop.disconnect()


@pytest.fixture
def connected_w250_teleop():
    """Create a connected W250JoystickTeleop instance with mocked ROS2."""
    with (
        patch("lerobot.teleoperators.w250joystick.w250joystick.rclpy", _make_ros2_mock()),
        patch("lerobot.teleoperators.w250joystick.w250joystick.Node"),
    ):
        config = W250JoystickConfig()
        teleop = W250JoystickTeleop(config)

        # Mock the joycon_node
        teleop.joycon_node = MagicMock()
        teleop.joycon_node.get_axes_values = MagicMock(return_value=[0.0] * 6)
        teleop.joycon_node.get_button_state = MagicMock(return_value=False)
        teleop.joycon_node.update_state = MagicMock()
        teleop.joycon_node.intervention_flag = False
        teleop.joycon_node.episode_end_status = None
        teleop.joycon_node.open_gripper_command = False
        teleop.joycon_node.close_gripper_command = False

        # Mock executor and thread
        teleop.executor = MagicMock()
        teleop.ros_thread = MagicMock()
        teleop.ros_thread.is_alive = MagicMock(return_value=True)

        yield teleop
        if teleop.is_connected():
            teleop.disconnect()


def test_config_creation():
    """Test that config can be created with default values."""
    config = W250JoystickConfig()
    assert config.ros2_node_name == "w250_joycon_teleop"
    assert config.joy_topic == "/joy"
    assert config.use_gripper is True
    assert config.deadzone == 0.1
    assert config.x_step_size == 0.01
    assert config.y_step_size == 0.01
    assert config.z_step_size == 0.01


def test_config_custom_values():
    """Test that config can be created with custom values."""
    config = W250JoystickConfig(
        joy_topic="/custom_joy",
        use_gripper=False,
        deadzone=0.2,
        x_step_size=0.02,
    )
    assert config.joy_topic == "/custom_joy"
    assert config.use_gripper is False
    assert config.deadzone == 0.2
    assert config.x_step_size == 0.02


def test_teleop_creation(w250_teleop):
    """Test that teleoperator can be created."""
    assert w250_teleop is not None
    assert w250_teleop.name == "w250joystick"
    assert w250_teleop.config_class == W250JoystickConfig


def test_action_features_with_gripper(w250_teleop):
    """Test action features when gripper is enabled."""
    features = w250_teleop.action_features
    assert features["dtype"] == "float32"
    assert features["shape"] == (4,)
    assert features["names"] == {"delta_x": 0, "delta_y": 1, "delta_z": 2, "gripper": 3}


def test_action_features_without_gripper():
    """Test action features when gripper is disabled."""
    with (
        patch("lerobot.teleoperators.w250joystick.w250joystick.rclpy", _make_ros2_mock()),
        patch("lerobot.teleoperators.w250joystick.w250joystick.Node"),
    ):
        config = W250JoystickConfig(use_gripper=False)
        teleop = W250JoystickTeleop(config)
        features = teleop.action_features
        assert features["dtype"] == "float32"
        assert features["shape"] == (3,)
        assert features["names"] == {"delta_x": 0, "delta_y": 1, "delta_z": 2}


def test_is_connected(w250_teleop):
    """Test connection status."""
    assert not w250_teleop.is_connected()


def test_get_action_zero_movement(connected_w250_teleop):
    """Test getting action with zero movement."""
    action = connected_w250_teleop.get_action()

    assert "delta_x" in action
    assert "delta_y" in action
    assert "delta_z" in action
    assert "gripper" in action

    assert action["delta_x"] == 0.0
    assert action["delta_y"] == 0.0
    assert action["delta_z"] == 0.0
    assert action["gripper"] == 1  # STAY


def test_get_action_analog_stick_movement(connected_w250_teleop):
    """Test getting action with analog stick movement."""
    # Mock stick values (above deadzone)
    connected_w250_teleop.joycon_node.get_axes_values.return_value = [0.5, -0.8, 0.0, 0.0, 0.0, 0.0]

    action = connected_w250_teleop.get_action()

    # With stick_x = 0.5 and stick_y = -0.8
    # delta_x = -(-0.8) * 0.01 = 0.008
    # delta_y = 0.5 * 0.01 = 0.005
    assert action["delta_x"] == pytest.approx(0.008)
    assert action["delta_y"] == pytest.approx(0.005)
    assert action["delta_z"] == 0.0


def test_get_action_dpad_up(connected_w250_teleop):
    """Test getting action with D-pad up pressed."""

    def button_state_side_effect(button_idx):
        return button_idx == connected_w250_teleop.config.button_up

    connected_w250_teleop.joycon_node.get_button_state.side_effect = button_state_side_effect

    action = connected_w250_teleop.get_action()

    assert action["delta_z"] == connected_w250_teleop.config.z_step_size


def test_get_action_dpad_down(connected_w250_teleop):
    """Test getting action with D-pad down pressed."""

    def button_state_side_effect(button_idx):
        return button_idx == connected_w250_teleop.config.button_down

    connected_w250_teleop.joycon_node.get_button_state.side_effect = button_state_side_effect

    action = connected_w250_teleop.get_action()

    assert action["delta_z"] == -connected_w250_teleop.config.z_step_size


def test_get_action_open_gripper(connected_w250_teleop):
    """Test getting action with open gripper command."""
    connected_w250_teleop.joycon_node.open_gripper_command = True
    connected_w250_teleop.joycon_node.close_gripper_command = False

    action = connected_w250_teleop.get_action()

    assert action["gripper"] == 2  # OPEN


def test_get_action_close_gripper(connected_w250_teleop):
    """Test getting action with close gripper command."""
    connected_w250_teleop.joycon_node.open_gripper_command = False
    connected_w250_teleop.joycon_node.close_gripper_command = True

    action = connected_w250_teleop.get_action()

    assert action["gripper"] == 0  # CLOSE


def test_get_action_deadzone(connected_w250_teleop):
    """Test that small stick movements are filtered by deadzone."""
    # Mock stick values below deadzone (0.1)
    connected_w250_teleop.joycon_node.get_axes_values.return_value = [0.05, -0.08, 0.0, 0.0, 0.0, 0.0]

    action = connected_w250_teleop.get_action()

    # All movements should be zero due to deadzone
    assert action["delta_x"] == 0.0
    assert action["delta_y"] == 0.0


def test_get_teleop_events_default(connected_w250_teleop):
    """Test getting teleop events with default state."""
    from lerobot.teleoperators.utils import TeleopEvents

    events = connected_w250_teleop.get_teleop_events()

    assert events[TeleopEvents.IS_INTERVENTION] is False
    assert events[TeleopEvents.TERMINATE_EPISODE] is False
    assert events[TeleopEvents.SUCCESS] is False
    assert events[TeleopEvents.RERECORD_EPISODE] is False


def test_get_teleop_events_intervention(connected_w250_teleop):
    """Test getting teleop events with intervention active."""
    from lerobot.teleoperators.utils import TeleopEvents

    connected_w250_teleop.joycon_node.intervention_flag = True

    events = connected_w250_teleop.get_teleop_events()

    assert events[TeleopEvents.IS_INTERVENTION] is True


def test_get_teleop_events_success(connected_w250_teleop):
    """Test getting teleop events with success status."""
    from lerobot.teleoperators.utils import TeleopEvents

    connected_w250_teleop.joycon_node.episode_end_status = TeleopEvents.SUCCESS

    events = connected_w250_teleop.get_teleop_events()

    assert events[TeleopEvents.SUCCESS] is True
    assert events[TeleopEvents.TERMINATE_EPISODE] is False


def test_get_teleop_events_rerecord(connected_w250_teleop):
    """Test getting teleop events with rerecord status."""
    from lerobot.teleoperators.utils import TeleopEvents

    connected_w250_teleop.joycon_node.episode_end_status = TeleopEvents.RERECORD_EPISODE

    events = connected_w250_teleop.get_teleop_events()

    assert events[TeleopEvents.RERECORD_EPISODE] is True
    assert events[TeleopEvents.TERMINATE_EPISODE] is True


def test_get_action_not_connected(w250_teleop):
    """Test that get_action raises error when not connected."""
    with pytest.raises(RuntimeError, match="JoyCon controller not connected"):
        w250_teleop.get_action()


def test_calibrate(connected_w250_teleop):
    """Test calibrate method (should be no-op)."""
    connected_w250_teleop.calibrate()  # Should not raise


def test_is_calibrated(connected_w250_teleop):
    """Test is_calibrated method."""
    assert connected_w250_teleop.is_calibrated() is True


def test_configure(connected_w250_teleop):
    """Test configure method (should be no-op)."""
    connected_w250_teleop.configure()  # Should not raise


def test_send_feedback(connected_w250_teleop):
    """Test send_feedback method (should be no-op)."""
    connected_w250_teleop.send_feedback({"test": "data"})  # Should not raise


def test_feedback_features(w250_teleop):
    """Test that feedback features is empty."""
    assert w250_teleop.feedback_features == {}
