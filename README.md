# go2w_sim_ros2
[![Linux platform](https://img.shields.io/badge/platform-Ubuntu--22.04-green.svg)](https://releases.ubuntu.com/22.04/)
[![Python](https://img.shields.io/badge/python-3.10-red.svg)](https://docs.python.org/3/whatsnew/3.10.html)
[![ROS2](https://img.shields.io/badge/ROS2-Humble-purple.svg)](https://docs.ros.org/en/humble/index.html)
[![IsaacSim](https://img.shields.io/badge/IsaacSim-5.0.0-blue.svg)](https://docs.isaacsim.omniverse.nvidia.com/latest/index.html)
[![Gazebo](https://img.shields.io/badge/Gazebo-11.0-orange.svg)](https://classic.gazebosim.org/)

---
This package allows ROS2 control of a Unitree Go2-W quadruped robot being simulated in Isaac Sim and Gazebo.

## TODO

|  | Feature                      |
|--------|--------------------------------------|
| ✅ | Gazebo and Isaac Simulation             |
| ✅ | Deployment of Isaac Lab trained models in Isaac Sim and Gazebo                |
| ✅ | ROS2 sensors communication for perception                |
| ❌ | [Livox Mid360 simulation for Isaac Sim](https://github.com/Livox-SDK/livox_laser_simulation/issues/30)   |
| ⬜ | Localization and Navigation Stack      |

## Prerequisites
- **Ubuntu 22.04**
- **ROS 2 Humble**
- **Python**
  - Gazebo: Python **3.10** (default on Ubuntu 22.04)
  - Isaac Sim: Python **3.11** (per Isaac Sim’s ROS bridge instructions)
- One simulator:
  - **Gazebo 11 (Classic)** — follow the [official install guide](https://classic.gazebosim.org/tutorials?tut=install_ubuntu)
  - **Isaac Sim 5.0** — follow the [official install guide](https://github.com/isaac-sim/IsaacSim/blob/main/README.md) and the **ROS 2 (Py 3.11)** setup: [Enabling rclpy / custom workspaces](https://docs.isaacsim.omniverse.nvidia.com/5.0.0/installation/install_ros.html#enabling-rclpy-custom-ros-2-packages-and-workspaces-with-python-3-11)

---

## 2. Build:
```bash
git clone --recurse-submodules https://github.com/JansSolanoVega/go2w-sim-ros2.git
cd go2w-sim-ros2
```
For building gazebo simulation, follow the rl-sar installation [README](https://github.com/fan-ziqi/rl_sar/blob/main/README.md) and then: 

```bash
cd rl_sar
./build.sh
```
## 3. Start simulation
### Option A – **Gazebo 11 Classic**
Without the environment of isaac activated:
```bash
source install/setup.bash
ros2 launch rl_sar gazebo.launch.py rname:=go2w
```

In a new terminal
```bash
source install/setup.bash
ros2 run rl_sar rl_sim
```

### Option B – **Isaac Sim 5.0**
```bash
source /opt/ros/ros_py311/install/local_setup.bash
conda activate isaaclab
python isaac_go2w_ros2.py
```
## Troubleshooting

- Python mismatch (3.10 vs 3.11): Gazebo builds on system Python 3.10; Isaac Sim ROS bridge expects Python 3.11. Use separate terminals/envs.
- LiDAR in Isaac: Mid360 sim is not yet available upstream (see issue linked above).