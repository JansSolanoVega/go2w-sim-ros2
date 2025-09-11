# go2w_sim_ros2


This package allows ROS2 control of a Unitree Go2-W quadruped robot being simulated in Isaac Sim and Gazebo.

| Tested With        | Version   |
|--------------------|----------|
| Ubuntu            | 22.04    |
| ROS2              | Humble   |
| Isaac Sim         | 5.0.0    |
| Gazebo            | 11.0     |

TODO:

|  | Feature                      |
|--------|--------------------------------------|
| ✅ | Joint position targets             |
| ⬜ | Joint velocity targets                |
| ✅ | Joint-level specification of Kp and Kd (proportional and derivative gain) |
| ❌ | Joint torque control   |
| ✅ | Joint state and IMU      |
| ✅ | Head LiDAR  |
| ⬜ | Front camera      |
| ⬜ | Terrain selection |

## Installation

## Usage
- Currently I am publishing imu and lidar using the clock of simulation, given the 20 fps. Even though time stamps are in fact publishing at 200hz, they appear at 60hz when recording rosbag data.
- Reducing the dt does not break the locomotion (1/200 to 1/600), so by dividing by 3 when can compensate the simulation fps and when can get 200hz. However timestamps are not at that frequency.
- Other solution might be to use not clock time but simulation time for pubishing data?