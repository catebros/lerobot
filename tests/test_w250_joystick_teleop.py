#!/usr/bin/env python3
"""
Test W250 Joystick with RViz Visualization
Control the RViz robot model with Logitech F710 gamepad by publishing joint states
"""

import logging
import sys
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

# Add lerobot to path if not installed
repo_root = Path(__file__).parent.parent
if (repo_root / "src" / "lerobot").exists():
    sys.path.insert(0, str(repo_root / "src"))

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


class RVizRobotController(Node):
    """Controls RViz robot by publishing to /wx250/joint_states"""
    
    def __init__(self, robot_name='wx250'):
        super().__init__('rviz_joystick_controller')
        
        self.robot_name = robot_name
        
        # Publisher to the robot's namespaced joint_states topic
        topic_name = f'/{robot_name}/joint_states'
        self.joint_pub = self.create_publisher(JointState, topic_name, 10)
        
        # Joint names for WX250 (must match URDF exactly)
        self.joint_names = [
            'waist',
            'shoulder',
            'elbow',
            'wrist_angle',
            'wrist_rotate',
            'gripper',
            'left_finger',
            'right_finger'
        ]
        
        # Current positions (home position in radians)
        self.positions = {
            'waist': 0.0,
            'shoulder': 0.0,
            'elbow': 0.0,
            'wrist_angle': 0.0,
            'wrist_rotate': 0.0,
            'gripper': 0.0,
            'left_finger': 0.037,   # Open position
            'right_finger': -0.037,  # Open position
        }
        
        # Scaling factors for joystick to joint angle conversion
        # These keep the robot within its joint limits
        self.scale = {
            'waist': 3.14,        # ±180 degrees
            'shoulder': 1.8,      # ±103 degrees
            'elbow': 1.9,         # ±109 degrees  
            'wrist_angle': 1.8,   # ±103 degrees
            'wrist_rotate': 3.14, # ±180 degrees
        }
        
        logger.info(f"RViz controller initialized")
        logger.info(f"Publishing joint states to: {topic_name}")
    
    def update_from_action(self, action):
        """
        Update joint positions from joystick action
        Action values should be in range [-1, 1] from joystick
        """
        if 'waist.pos' in action:
            self.positions['waist'] = action['waist.pos'] * self.scale['waist']
        if 'shoulder.pos' in action:
            self.positions['shoulder'] = action['shoulder.pos'] * self.scale['shoulder']
        if 'elbow.pos' in action:
            self.positions['elbow'] = action['elbow.pos'] * self.scale['elbow']
        if 'wrist_angle.pos' in action:
            self.positions['wrist_angle'] = action['wrist_angle.pos'] * self.scale['wrist_angle']
        if 'wrist_rotate.pos' in action:
            self.positions['wrist_rotate'] = action['wrist_rotate.pos'] * self.scale['wrist_rotate']
        if 'gripper.pos' in action:
            # Convert 0-1 gripper value to finger positions
            # gripper.pos: 0 = open (0.037), 1 = closed (0.015)
            gripper_val = action['gripper.pos']
            finger_pos = 0.037 - gripper_val * (0.037 - 0.015)
            self.positions['left_finger'] = finger_pos
            self.positions['right_finger'] = -finger_pos
            self.positions['gripper'] = gripper_val * 0.5
    
    def publish_state(self):
        """Publish current joint states to RViz"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ''
        msg.name = self.joint_names
        msg.position = [
            self.positions['waist'],
            self.positions['shoulder'],
            self.positions['elbow'],
            self.positions['wrist_angle'],
            self.positions['wrist_rotate'],
            self.positions['gripper'],
            self.positions['left_finger'],
            self.positions['right_finger'],
        ]
        
        # Include velocities and efforts (zeros) for completeness
        msg.velocity = [0.0] * len(self.joint_names)
        msg.effort = [0.0] * len(self.joint_names)
        
        self.joint_pub.publish(msg)
    
    def reset_home(self):
        """Reset to home position"""
        logger.info("Resetting to home position...")
        self.positions = {
            'waist': 0.0,
            'shoulder': 0.0,
            'elbow': 0.0,
            'wrist_angle': 0.0,
            'wrist_rotate': 0.0,
            'gripper': 0.0,
            'left_finger': 0.037,
            'right_finger': -0.037,
        }


def test_joystick_rviz():
    """Test joystick control with RViz visualization"""

    logger.info("=" * 60)
    logger.info("W250 JOYSTICK CONTROL WITH RVIZ")
    logger.info("=" * 60)
    logger.info("\nPrerequisites:")
    logger.info("  1. Logitech F710 gamepad connected via USB")
    logger.info("  2. Mode switch on back set to 'X' (XInput mode)")
    logger.info("  3. Terminal 1 - ROS2 joy node:")
    logger.info("     $ ros2 run joy joy_node")
    logger.info("  4. Terminal 2 - RViz WITHOUT joint_state_publisher:")
    logger.info("     $ ros2 launch interbotix_xsarm_descriptions \\")
    logger.info("       xsarm_description.launch.py robot_model:=wx250 \\")
    logger.info("       use_sim:=true use_rviz:=true")
    logger.info("\n⚠️  IMPORTANT: Do NOT use use_joint_pub:=true!")
    logger.info("   This script will publish the joint states instead.")
    logger.info("=" * 60)

    input("\nPress ENTER when ready to start...")

    # Initialize ROS2
    rclpy.init()

    try:
        from lerobot.teleoperators.w250joystick import W250JoystickTeleop, W250JoystickConfig
    except ImportError as e:
        logger.error(f"✗ Failed to import teleoperator: {e}")
        rclpy.shutdown()
        return False

    # Create controller and teleoperator
    controller = RVizRobotController(robot_name='wx250')
    
    teleop_config = W250JoystickConfig(
        joy_topic="/joy",
        ros2_node_name="w250_joystick_rviz_teleop",
    )
    teleop = W250JoystickTeleop(teleop_config)

    try:
        # ==================== Setup ====================
        logger.info("\n" + "=" * 60)
        logger.info("PHASE 1: Connecting Teleoperator")
        logger.info("=" * 60)

        logger.info("\nConnecting teleoperator...")
        teleop.connect()
        time.sleep(1)
        logger.info("✓ Teleoperator connected")

        # Publish initial state to establish connection
        logger.info("\nPublishing initial robot state to RViz...")
        for _ in range(5):  # Publish a few times to ensure it's received
            controller.publish_state()
            rclpy.spin_once(controller, timeout_sec=0.01)
            time.sleep(0.1)
        logger.info("✓ Initial state published")

        logger.info("\n🎮 Gamepad Controls:")
        logger.info("  Left Stick:")
        logger.info("    X-axis → Waist rotation (left/right)")
        logger.info("    Y-axis → Shoulder (forward/back)")
        logger.info("  Right Stick:")
        logger.info("    X-axis → Wrist angle (tilt)")
        logger.info("    Y-axis → Elbow (up/down)")
        logger.info("  Triggers:")
        logger.info("    LT/RT  → Wrist rotate")
        logger.info("  Bumpers:")
        logger.info("    LB     → Open gripper")
        logger.info("    RB     → Close gripper")
        logger.info("  Buttons:")
        logger.info("    Y      → Reset to home position")
        logger.info("    START  → Exit")

        # ==================== Control Loop ====================
        logger.info("\n" + "=" * 60)
        logger.info("PHASE 2: Active Control - Move the robot in RViz!")
        logger.info("=" * 60)
        logger.info("\nStarting control loop in 3 seconds...")
        logger.info("You should see the robot in RViz respond to your joystick!")
        time.sleep(3)

        control_active = True
        last_status_time = time.time()
        iteration = 0

        logger.info("\n📊 CONTROL ACTIVE - The robot in RViz should now move!")
        logger.info("=" * 60)

        while control_active:
            # Get action from teleoperator
            action = teleop.get_action()
            events = teleop.get_teleop_events()

            # Update robot position from joystick
            controller.update_from_action(action)
            
            # Publish to RViz
            controller.publish_state()

            # Check for home reset
            if events.get("stop_episode", False):
                logger.info("\n🏠 Y button pressed - resetting to home")
                controller.reset_home()

            # Check for exit
            if events.get("rerecord_episode", False):
                logger.info("\n📍 START pressed - exiting control loop")
                control_active = False

            # Spin ROS2 to process callbacks
            rclpy.spin_once(controller, timeout_sec=0.0)

            # Print status every 2 seconds
            if time.time() - last_status_time > 2.0:
                pos = controller.positions
                logger.info(f"  [{iteration//40}s] Joint Positions: "
                           f"waist={pos['waist']:.2f}, "
                           f"shoulder={pos['shoulder']:.2f}, "
                           f"elbow={pos['elbow']:.2f}, "
                           f"wrist_angle={pos['wrist_angle']:.2f}, "
                           f"wrist_rotate={pos['wrist_rotate']:.2f}, "
                           f"gripper_open={pos['left_finger']:.3f}")
                last_status_time = time.time()

            iteration += 1
            time.sleep(0.05)  # 20Hz control loop

        logger.info("=" * 60)
        logger.info("✓ Control loop finished")

        # ==================== Cleanup ====================
        logger.info("\n" + "=" * 60)
        logger.info("PHASE 3: Cleanup")
        logger.info("=" * 60)

        logger.info("\nReturning to home position...")
        controller.reset_home()
        for _ in range(10):
            controller.publish_state()
            rclpy.spin_once(controller, timeout_sec=0.01)
            time.sleep(0.1)
        logger.info("✓ Returned to home")

        logger.info("\nDisconnecting teleoperator...")
        teleop.disconnect()
        logger.info("✓ Teleoperator disconnected")

        logger.info("\nShutting down ROS2...")
        controller.destroy_node()
        rclpy.shutdown()
        logger.info("✓ ROS2 shutdown complete")

    except KeyboardInterrupt:
        logger.warning("\n\n⚠️  Test interrupted by user (Ctrl+C)")
        logger.info("Cleaning up...")

        try:
            teleop.disconnect()
        except:
            pass

        try:
            controller.destroy_node()
        except:
            pass

        try:
            rclpy.shutdown()
        except:
            pass

        return False

    except Exception as e:
        logger.error(f"\n✗ Test failed: {e}")
        import traceback
        traceback.print_exc()

        # Cleanup on error
        try:
            teleop.disconnect()
        except:
            pass

        try:
            controller.destroy_node()
        except:
            pass

        try:
            rclpy.shutdown()
        except:
            pass

        return False

    # ==================== TEST SUMMARY ====================
    logger.info("\n" + "=" * 60)
    logger.info("✓ RVIZ JOYSTICK CONTROL TEST PASSED!")
    logger.info("=" * 60)
    logger.info("\nTest Summary:")
    logger.info("  ✓ Teleoperator connected successfully")
    logger.info("  ✓ Joint states published to /wx250/joint_states")
    logger.info("  ✓ RViz robot model controlled via joystick")
    logger.info("  ✓ Clean disconnect and shutdown")
    logger.info("\n🎮 Your joystick now controls the RViz visualization!")
    logger.info("=" * 60)

    return True


if __name__ == "__main__":
    success = test_joystick_rviz()
    sys.exit(0 if success else 1)