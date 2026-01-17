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
   - Install [NVIDIA Drivers](https://www.nvidia.com/Download/index.aspx) on the Windows host.
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
Place the following `docker-compose.yml` in `~/ros2-snake/`. This configures GPU passthrough and volumes for your workspace.

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
DOCKER_IMAGE=humble-tutorial docker compose run --entrypoint /bin/bash moveit_gpu
```

### Step 2: Install Dynamixel SDK (Python)
Inside the container terminal, run:

```bash
cd /root/ws_moveit/src
git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
cd DynamixelSDK/python
sudo python3 setup.py install
```

### Step 3: Build the Workspace
```bash
cd /root/ws_moveit
# Source MoveIt tutorial environment
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash

# Build robot description and config
colcon build --symlink-install
source install/setup.bash
```

## 3️⃣ Phase 3: MoveIt2 Configuration (Simulation)
Use the Setup Assistant to generate the robot's motion planning parameters.

1. **Launch Assistant:** `ros2 launch moveit_setup_assistant setup_assistant.launch.py`
2. **Define Planning Group:** Create a group named `snake_arm` using the `joint_chain` (Joint 1 to Joint 5).
3. **Define Poses:** Add the Home, Right, and Left poses previously calibrated.
4. **Test Simulation:**

```bash
ros2 launch snake_moveit_config demo.launch.py
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
