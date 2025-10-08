# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
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

"""
Configuration class for WidowX-250 robot using Interbotix ROS2 API
"""

import logging
from dataclasses import dataclass, field
from typing import Any, Dict, Optional
from pathlib import Path

# Import the base Robot configuration
from ..robot import RobotConfig

logger = logging.getLogger(__name__)


@dataclass
class W250InterbotixConfig(RobotConfig):
    """
    Configuration for WidowX-250 robot using Interbotix Python API with ROS2
    
    This config maintains compatibility with the LeRobot interface while using
    the official Interbotix ROS2 implementation underneath.
    """
    
    # Robot model name for Interbotix API (closest match to WidowX-250)
    robot_model: str = "wx250"
    
    # ROS2 namespace for the robot
    robot_name: str = "wx250"
    
    # Joint group name (standard Interbotix configuration)
    group_name: str = "arm"
    
    # Gripper joint name (standard Interbotix configuration) 
    gripper_name: str = "gripper"
    
    # Movement timing parameters
    moving_time: float = 2.0  # Time in seconds for arm movements
    accel_time: float = 0.3   # Acceleration time in seconds
    
    # Gripper control parameters
    gripper_pressure: float = 0.5  # Gripper pressure (0.0-1.0)
    gripper_pressure_lower_limit: int = 150  # Minimum effort (PWM)
    gripper_pressure_upper_limit: int = 350  # Maximum effort (PWM)
    
    # ROS2 specific settings
    init_node: bool = True  # True - Interbotix API creates robot_manipulation node internally
    use_moveit: bool = False  # Whether to use MoveIt for motion planning
    
    # Camera configurations (inherited from base RobotConfig)
    cameras: Dict[str, Any] = field(default_factory=dict)
    
    # Motor calibration settings
    calibration_dir: Path = Path("~/.cache/huggingface/lerobot/calibration")
    
    # Safety limits
    max_relative_target: Optional[float] = None  # Maximum relative movement per command
    disable_torque_on_disconnect: bool = True
    
    # Motion profile settings (Interbotix specific)
    profile_type: str = "time_based"  # "time_based" or "velocity_based"
    
    def __post_init__(self):
        """Validate configuration after initialization"""
        super().__post_init__()
        
        # Validate robot model
        valid_models = ["wx200", "wx250", "wx250s", "vx250", "vx300", "vx300s"]
        if self.robot_model not in valid_models:
            logger.warning(f"Robot model '{self.robot_model}' not in standard Interbotix models: {valid_models}")
        
        # Validate pressure settings
        if not 0.0 <= self.gripper_pressure <= 1.0:
            raise ValueError(f"gripper_pressure must be between 0.0 and 1.0, got {self.gripper_pressure}")
        
        if self.gripper_pressure_lower_limit >= self.gripper_pressure_upper_limit:
            raise ValueError("gripper_pressure_lower_limit must be less than gripper_pressure_upper_limit")
        
        # Validate timing
        if self.accel_time > self.moving_time / 2:
            raise ValueError("accel_time must be <= moving_time / 2")
        
        # Ensure calibration directory exists
        self.calibration_dir = Path(self.calibration_dir).expanduser()
        self.calibration_dir.mkdir(parents=True, exist_ok=True)
        
        logger.info(f"W250InterbotixConfig initialized:")
        logger.info(f"  Robot model: {self.robot_model}")
        logger.info(f"  Robot name: {self.robot_name}")
        logger.info(f"  Group name: {self.group_name}")
        logger.info(f"  Moving time: {self.moving_time}s")
        logger.info(f"  Gripper pressure: {self.gripper_pressure}")


# Register this configuration class
@RobotConfig.register_subclass("w250_interbotix")
class W250InterbotixConfigRegistered(W250InterbotixConfig):
    """Registered version of W250InterbotixConfig for LeRobot framework"""
    pass