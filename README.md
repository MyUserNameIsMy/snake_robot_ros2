# 5-DOF Snake Robot Control System
![Snake Robot](./5_dof_robot.jpg)

This project provides a complete ROS 2 Humble environment for simulating and controlling a 5-joint snake robot using **MoveIt2** and **Dynamixel** servos. The entire stack runs within a **Docker container** on **WSL2**, leveraging GPU acceleration for smooth RViz performance.

## 🚀 System Architecture
- **OS:** Windows 11 (WSL2 with Ubuntu 22.04)
- **Middleware:** ROS 2 Humble
- **Planning:** MoveIt 2
- **Hardware Interface:** `dynamixel_hardware` (ros2_control)
- **Containerization:** Docker with NVIDIA Container Toolkit

---

## 🛠️ Prerequisites

1. **Windows Setup:**
   - Install [Docker Desktop](https://www.docker.com/products/docker-desktop/) with WSL2 backend enabled.
   - Install [usbipd-win](https://github.com/dorssel/usbipd-win) for real hardware USB passthrough.

2. **WSL2 Permissions:**
   Allow the container to access your X-Server for GUI support:
   ```bash
   echo "export XAUTHORITY=~/.Xauthority" >> ~/.bashrc
   source ~/.bashrc
   ```

## 1️⃣ Phase 1: Docker Environment Setup
You will install Docker inside WSL2 Ubuntu rather than using Docker Desktop to ensure better compatibility with low-level hardware communication.

### Step 1: Install Docker Engine
Follow the [official Ubuntu Docker installation guide](https://docs.docker.com/engine/install/ubuntu/).

### Step 2: Create Docker Compose Configuration
Place the following `docker-compose.yml` in `~/ros2-moveit-snake/`. This configures GPU passthrough and volumes for your workspace.

```yaml
services:
  cpu:
    image: moveit/moveit2:${DOCKER_IMAGE}
    container_name: moveit2_container
    privileged: true
    network_mode: host
    command: /bin/bash
    working_dir: /root/ws_moveit
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix
      - $XAUTHORITY:/root/.Xauthority
      - ./ros2-moveit-snake:/root/ws_moveit/src
    environment:
      QT_X11_NO_MITSHM: 1
      DISPLAY: $DISPLAY

  gpu:
    image: moveit/moveit2:${DOCKER_IMAGE}
    container_name: moveit2_container
    privileged: true
    devices:
      - /dev/ttyUSB0:/dev/ttyUSB0
    network_mode: host
    command: /bin/bash
    deploy:
      resources:
        reservations:
          devices:
            - driver: nvidia
              count: 1
              capabilities: [gpu]
    working_dir: /root/ws_moveit
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix
      - $XAUTHORITY:/root/.Xauthority
      - ./ros2-moveit-snake:/root/ws_moveit/src
    environment:
      QT_X11_NO_MITSHM: 1
      DISPLAY: $DISPLAY
      NVIDIA_VISIBLE_DEVICES: all
      NVIDIA_DRIVER_CAPABILITIES: all
```

## 2️⃣ Phase 2: SDK & Software Installation
This phase establishes the "bridge" between ROS 2 software and the physical MX-28 hardware.

### Step 1: Launch & Enter Container
```bash
DOCKER_IMAGE=humble-humble-tutorial-source docker compose run cpu
```

Once inside the container, it is recommended to update the package list and install a text editor like Vim:
```bash
sudo apt update
sudo apt install vim
```

### Step 2: Install Dynamixel SDK (Python)
Inside the container terminal, run:

```bash
cd /root/ws_moveit/src
git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
cd DynamixelSDK/python
sudo python3 setup.py install
```

### Step 3: Setup Robot Description and Build
You need to import your robot description package (containing URDF and meshes) into the workspace before building.

1. **Clone Robot Description:**
   Navigate to the `src` directory and copy robot_description folder from 
   snake_robot_ros2 tutorial. This ensures 
   your URDF and meshes are available for MoveIt.

2. **Build the Workspace:**
   ```bash
   cd /root/ws_moveit
   
   # Source ROS 2 environment
   source /opt/ros/humble/setup.bash

   # Build the workspace
   colcon build --symlink-install
   source install/setup.bash
   ```

   **Verify the Build:**
   To check if everything was imported correctly, run the following command from `~/ws_moveit`:
   ```bash
   ros2 launch robot_description display.launch.py
   ```
   
   If successful, you should see the robot model in RViz:
   ![Success](./snake_rviz_first_step.png)

   ### ⚠️ Troubleshooting RViz
   If you see an error like the image below (no robot model visible):
   ![Error](./snake_rviz_first_step_error.png)

   **Solution:**
   1. Change the **Fixed Frame** to `world`.
   2. Click the **Add** button in the bottom left.
   3. Scroll down and select **RobotModel**.
   ![Add Robot Model](./snake_rviz_first_step_robot_model.png)
   4. In the left panel, expand **RobotModel** and set the **Description Topic** to `/robot_description`.
   ![Set Description Topic](./snake_rviz_first_step_description_topic.png)

## 3️⃣ Phase 3: MoveIt2 Configuration (Simulation)
Use the Setup Assistant to generate the robot's motion planning parameters.

### Step 1: Launch Setup Assistant
```bash
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```

### Step 2: Load Robot Model
1. Click **Create New MoveIt Configuration Package**.
2. Select the `snake_robot.urdf.xacro` file.
   ![Load URDF](./moveit2_step_1.png)
3. Click **Load Files**. You should see "Success".
   ![Success](./moveit_2_step_1_success.png)

### Step 3: Self-Collisions
1. Click **Self-Collisions** on the left.
2. Click **Generate Collision Matrix**.
   ![Generate Collision](./moveit2_step_2.png)

### Step 4: Virtual Joints
1. Click **Virtual Joints**.
2. Add a virtual joint to attach the robot to the world (if needed, usually `world` to `base_link`).
   ![Virtual Joint](./moveit2_step_3.png)

### Step 5: Planning Groups
1. Click **Planning Groups**.
2. Click **Add Group**.
   - **Group Name:** `snake_arm`
   - **Kinematic Solver:** `kdl_kinematics_plugin/KDLKinematicsPlugin`
3. Click **Add Kin. Chain**.
   - **Base Link:** `base_link`
   - **Tip Link:** `end_effector`
   ![Planning Group](./moveit2_step_4.png)

### Step 6: Robot Poses
1. Click **Robot Poses**.
2. Add predefined poses (e.g., Home, Up, Right).
   ![Add Poses](./moveit2_step_5.png)
   ![All Poses](./moveit2_step_5_all.png)

### Step 7: End Effectors & Passive Joints
1. **End Effectors:** No changes needed.
   ![End Effectors](./moveit2_step_6.png)
2. **Passive Joints:** No changes needed.
   ![Passive Joints](./moveit2_step_7.png)

### Step 8: ROS 2 Control
1. Click **ROS 2 Control**.
2. Click **Add Interface** (or Auto Detect).
   ![ROS 2 Control](./moveit2_step_8.png)

### Step 9: Controllers
1. **ROS 2 Controllers:** Generate the controllers.
   ![ROS 2 Controllers](./moveit2_step_9.png)
2. **MoveIt Controllers:** Generate MoveIt controllers.
   ![MoveIt Controllers](./moveit2_step_10.png)

### Step 10: Perception & Launch Files
1. **Perception:** No changes.
   ![Perception](./moveit2_step_11.png)
2. **Launch Files:** No changes.
   ![Launch Files](./moveit2_step_12.png)

### Step 11: Author Information
1. Click **Author Information**.
2. Enter your name and email.
   ![Author Info](./moveit2_step_13.png)

### Step 12: Generate Package
1. Click **Configuration Files**.
2. Set the output directory (e.g., `~/ws_moveit/src/snake_moveit_config`).
3. Click **Generate Package**.
   ![Generate Package](./moveit2_step_14.png)
   ![Success](./moveit2_step_14_success.png)

### Step 13: Build and Test
1. Build the new package:
   ```bash
   colcon build --packages-select snake_moveit_config
   source install/setup.bash
   ```
2. Run the demo:
   ```bash
   ros2 launch snake_moveit_config demo.launch.py
   ```

### ⚠️ Troubleshooting
If you encounter an error like this:
![Error](./moveit2_test_1.png)

You may need to manually update the configuration files.

**1. Fix `joint_limits.yaml`**
Edit `~/ws_moveit/src/snake_moveit_config/config/joint_limits.yaml`:
```yaml
# joint_limits.yaml allows the dynamics properties specified in the URDF to be overwritten or augmented as needed

# For beginners, we downscale velocity and acceleration limits.
# You can always specify higher scaling factors (<= 1.0) in your motion requests.  # Increase the values below to 1.0 to always move at maximum speed.
default_velocity_scaling_factor: 0.1
default_acceleration_scaling_factor: 0.1

# Specific joint properties can be changed with the keys [max_position, min_position, max_velocity, max_acceleration]
# Joint limits can be turned off with [has_velocity_limits, has_acceleration_limits]
joint_limits:
  joint_1:
    has_velocity_limits: true
    max_velocity: 1.0
    has_acceleration_limits: true
    max_acceleration: 1.0
  joint_2:
    has_velocity_limits: true
    max_velocity: 1.0
    has_acceleration_limits: true
    max_acceleration: 1.0
  joint_3:
    has_velocity_limits: true
    max_velocity: 1.0
    has_acceleration_limits: true
    max_acceleration: 1.0
  joint_4:
    has_velocity_limits: true
    max_velocity: 1.0
    has_acceleration_limits: true
    max_acceleration: 1.0
  joint_5:
    has_velocity_limits: true
    max_velocity: 1.0
    has_acceleration_limits: true
    max_acceleration: 1.0
```

**2. Fix `moveit_controllers.yaml`**
Edit `~/ws_moveit/src/snake_moveit_config/config/moveit_controllers.yaml`:
```yaml
# MoveIt uses this configuration to know which controller to talk to
moveit_controller_manager: moveit_simple_controller_manager/MoveItSimpleControllerManager

moveit_simple_controller_manager:
  controller_names:
    - snake_arm_controller

  snake_arm_controller:
    # This must match the action namespace of your controller
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    default: true
    joints:
      - joint_1
      - joint_2
      - joint_3
      - joint_4
      - joint_5
```

## 4️⃣ Phase 4: Real Robot Control (The Bridge)
The Shadow Node listens to simulation joint states and sends them to the physical motors.

### Step 1: Attach USB to WSL
Run this in Windows PowerShell (Admin) to forward the U2D2 adapter to Linux:

```powershell
usbipd list
usbipd attach --wsl --busid <BUS_ID_OF_U2D2>
```

### Step 2: Run the Shadow Node
In a second container terminal, execute your bridge script (configured for IDs 1–5 and Protocol 2.0):

```bash
python3 /root/ws_moveit/src/snake_description/scripts/snake_bridge.py
```

### Step 3: Execution
1. Move the robot interactive markers in RViz.
2. Click "Plan and Execute".
3. The physical MX-28 servos will mirror the simulation.
