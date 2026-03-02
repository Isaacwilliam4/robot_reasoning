
# Robot Reasoning ROS Workspace

## Setup and Running Instructions

### 1. Install the Conda Environment

First, create the required conda environment from the provided `environment.yaml` file:

```bash
conda env create -f environment.yaml
```

If the environment already exists, you can update it with:

```bash
conda env update -f environment.yaml
```

### 2. Build the ROS 2 Workspace

Activate the conda environment:

```bash
conda activate robot_ros_env
```

Then build the workspace:

```bash
colcon build
```

After building, source the setup script (choose the one for your shell):

```bash
source install/setup.sh   # for bash
# or
source install/setup.zsh  # for zsh
```

### 3. Set the World Path in a .env File

Create a `.env` file in the root of the workspace (where this README is located) and add the following line:

```
WORLD_PATH=/absolute/path/to/depth_camera_world.sdf
```

Replace `/absolute/path/to/depth_camera_world.sdf` with the actual path to your `depth_camera_world.sdf` file (e.g., `src/camera_pkg/worlds/depth_camera_world.sdf`).

### 4. Run Gazebo and the ROS-Gazebo Bridge

In a new terminal (do NOT activate the conda environment), run:

```bash
./run_gazebo_and_bridge.sh
```

This will start Gazebo and the ROS-Gazebo bridge. The script will handle conda activation internally.

### 5. Launch the ROS 2 Stack

In another terminal, activate the conda environment:

```bash
conda activate robot_ros_env
```

Then launch the stack:

```bash
ros2 launch camera_pkg run_stack.launch.py
```

---

You should now have the simulation and ROS 2 nodes running. If you encounter issues, ensure all steps were followed and that your `.env` file is correctly set up.
