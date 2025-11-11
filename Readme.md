# Usage

## Quick Start with Docker (Recommended)

### 1. Clone the repository
```bash
git clone git@github.com:silverleopard12/drone_show.git
cd drone_show
```

### 2. Pull the Docker image
```bash
docker pull ds70768/droneshow_trajectory_docker:latest
```

### 3. Allow X server access (for GUI/RViz)
```bash
xhost +local:docker
```

### 4. Run the Docker container
```bash
docker run -it --rm \
    --network host \
    --privileged \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -v $(pwd):/workspace/ego-planner-swarm \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    ds70768/droneshow_trajectory_docker:latest
```

### 5. Build the workspace inside the container
```bash
cd /workspace/ego-planner-swarm
colcon build --symlink-install
source install/setup.bash
```

### 6. Run the planner
See [Section 3. Running the Code](#3-running-the-code) below for detailed instructions.

---

## Manual Installation (Alternative)

### 1. Required Libraries
* vtk (A dependency library for PCL installation, need to check Qt during compilation)
* PCL

### 2. Prerequisites
It might be due to some incorrect settings in my publish/subscribe configurations. Using ROS2's default FastDDS causes significant lag during program execution. The reason hasn't been identified yet. Please follow the steps below to change the DDS to cyclonedds.

#### 2.1 Install cyclonedds
```bash
sudo apt install ros-humble-rmw-cyclonedds-cpp
```

#### 2.2 Change default DDS
```bash
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
source ~/.bashrc
```

#### 2.3 Verify the change
```bash
ros2 doctor --report | grep "RMW middleware"
```
If the output shows rmw_cyclonedds_cpp, the modification is successful.

---

## 3. Running the Code

### 3.1 Launch Rviz
```bash
ros2 launch ego_planner rviz.launch.py
```

### 3.2 Run the planning program
Open a new terminal and execute:

**For Docker users:** Open a new terminal and run:
```bash
docker exec -it <container_name> bash
cd /workspace/ego-planner-swarm
source install/setup.bash
```

**Available launch options:**

* **Single drone**
```bash
ros2 launch ego_planner single_run_in_sim.launch.py
```

* **Swarm**
```bash
ros2 launch ego_planner swarm.launch.py
```

* **Large swarm**
```bash
ros2 launch ego_planner swarm_large.launch.py
```

* **Additional parameters (optional):**
    * `use_mockamap`: Map generation method. Default: False (uses Random Forest), True uses mockamap.
    * `use_dynamic`: Whether to consider dynamics. Default: False (disabled), True enables dynamics.

**Example with parameters:**
```bash
ros2 launch ego_planner single_run_in_sim.launch.py use_mockamap:=True use_dynamic:=False
```
