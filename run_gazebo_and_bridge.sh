#!/bin/bash
eval "$(conda shell.bash hook)"

conda deactivate
# Load environment variables from .env
source .env

# Trap Ctrl+C and kill both child processes
trap 'echo "Stopping..."; kill $GAZEBO_PID $BRIDGE_PID 2>/dev/null; exit' SIGINT

# Start Gazebo in the background using WORLD_PATH from .env
ign gazebo -r "$WORLD_PATH" &
GAZEBO_PID=$!

# Activate conda environment (with shell hook)
conda activate robot_ros_env

# Start the bridge in the background
ros2 run ros_gz_bridge parameter_bridge /model/robot/link/camera/depth/image@sensor_msgs/msg/Image@ignition.msgs.Image /cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist &
BRIDGE_PID=$!

# Wait for both processes
wait $GAZEBO_PID $BRIDGE_PID
