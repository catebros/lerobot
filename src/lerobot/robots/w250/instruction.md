# Terminal 1
conda create -n lerobot_ros2 python=3.10 -y
conda activate lerobot_ros2
source /opt/ros/humble/setup.bash
source ~/interbotix_ws/install/setup.bash

ros2 launch interbotix_xsarm_control xsarm_control.launch.py robot_model:=wx250 use_sim:=true


# Terminal 2
conda activate lerobot_ros2

source /opt/ros/humble/setup.bash
source ~/interbotix_ws/install/setup.bash

cd ~/lerobot/tests

python test_w250_connection.py


ros2 run joy joy_node