import re
from typing import Optional
import numpy as np
from isaacsim.core.utils.rotations import quat_to_rot_matrix
from isaacsim.robot.policy.examples.controllers import PolicyController
from isaacsim.core.utils.types import ArticulationAction

from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class CmdVelSubscriber(Node):
    """ROS2 node that listens to /cmd_vel and updates base command."""
    def __init__(self, runner):
        super().__init__('go2w_cmdvel_listener')
        self.runner = runner
        self.subscription = self.create_subscription(
            Twist, '/unitree_go2w/cmd_vel', self.listener_callback, 10)

    def listener_callback(self, msg: Twist):
        # Map Twist to [vx, vy, wz]
        self.runner._base_command = np.array([msg.linear.x, msg.linear.y, msg.angular.z], dtype=float)

class JoystickPublisher(Node):
    """ROS2 node that converts joystick input into Twist commands."""
    def __init__(self):
        super().__init__('joystick_to_cmdvel')

        # Publisher to your robot's cmd_vel
        self.publisher = self.create_publisher(Twist, '/unitree_go2w/cmd_vel', 10)

        # Subscriber to joystick events
        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        # Config: axis mapping (adjust if needed)
        self.axis_linear_x = 1     # left stick vertical
        self.axis_linear_y = 0     # left stick horizontal
        self.axis_angular_z = 3    # right stick horizontal
        self.enable_button = 4     # LB button must be pressed

        # Scaling factors
        self.scale_linear = 0.75    # m/s
        self.scale_angular = 0.75   # rad/s

    def joy_callback(self, joy_msg: Joy):
        twist = Twist()

        # Only move if enable button is pressed
        if joy_msg.buttons[self.enable_button]:
            twist.linear.x = self.scale_linear * joy_msg.axes[self.axis_linear_x]
            twist.linear.y = self.scale_linear * joy_msg.axes[self.axis_linear_y]
            twist.angular.z = self.scale_angular * joy_msg.axes[self.axis_angular_z]
        else:
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.angular.z = 0.0

        self.publisher.publish(twist)

class GO2WPolicy(PolicyController):
    """GO2-W: legs (pose control) + wheels (velocity control), 57-dim observation."""
    POS_IDX = np.array([0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11], dtype=int)   # 12 leg joints (position controlled)
    VEL_IDX = np.array([12, 13, 14, 15], dtype=int)                         # 4 wheel joints (velocity controlled)

    def __init__(
        self,
        prim_path: str,
        name: str = "go2w",
        usd_path: Optional[str] = None,
        position: Optional[np.ndarray] = None,
        orientation: Optional[np.ndarray] = None,
        policy_path: Optional[str] = None,
        env_yaml_path: Optional[str] = None,
    ) -> None:
        super().__init__(name, prim_path, None, usd_path, position, orientation)
        if not policy_path or not env_yaml_path:
            raise ValueError("Must provide policy_path and env_yaml_path.")
        self.load_policy(policy_path, env_yaml_path)

        self._previous_action = np.zeros(16, dtype=float)  # (12 pos + 4 vel)
        self._policy_counter = 0
        self.clip_observations = 100.0
        self.decimation = 4  # compute action every N physics steps

        # Scales (match your training)
        self.ang_vel_scale  = 0.25
        self.commands_scale = 1.0
        self.dof_pos_scale  = 1.0
        self.dof_vel_scale  = 0.05

        # Joint order mapping (old -> new)
        old_order = [
            'FL_hip_joint', 'FR_hip_joint', 'RL_hip_joint', 'RR_hip_joint',
            'FL_thigh_joint', 'FR_thigh_joint', 'RL_thigh_joint', 'RR_thigh_joint',
            'FL_calf_joint', 'FR_calf_joint', 'RL_calf_joint', 'RR_calf_joint',
            'FL_foot_joint', 'FR_foot_joint', 'RL_foot_joint', 'RR_foot_joint'
        ]
        new_order = [
            'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
            'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
            'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint',
            'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint',
            'FR_foot_joint', 'FL_foot_joint', 'RR_foot_joint', 'RL_foot_joint'
        ]
        self.idx_map = [old_order.index(j) for j in new_order]
        self.idx_map_reverse = [new_order.index(j) for j in old_order]

        self._default_pos = None
        self._dof_count = None
        self.action_scale = None

    # -------- Observation (57) --------
    def _compute_observation(self, command: np.ndarray) -> np.ndarray:
        ang_vel_I = self.robot.get_angular_velocity()
        _, q_IB = self.robot.get_world_pose()
        R_IB = quat_to_rot_matrix(q_IB)
        R_BI = R_IB.transpose()
        ang_vel_b = R_BI @ ang_vel_I
        gravity_b = R_BI @ np.array([0.0, 0.0, -1.0])

        # Reordered joint states
        self.current_joint_pos = self.robot.get_joint_positions()[self.idx_map]
        self.current_joint_vel = self.robot.get_joint_velocities()[self.idx_map]

        dof_pos_rel = self.current_joint_pos - self._default_pos
        dof_pos_rel[self.VEL_IDX] = 0.0  # wheels: no pos error

        obs = np.zeros(57, dtype=float)
        obs[0:3]   = ang_vel_b * self.ang_vel_scale
        obs[3:6]   = gravity_b
        obs[6:9]   = command * self.commands_scale
        obs[9:25]  = dof_pos_rel * self.dof_pos_scale
        obs[25:41] = self.current_joint_vel * self.dof_vel_scale
        obs[41:57] = self._previous_action
        return np.clip(obs, -self.clip_observations, self.clip_observations)

    # -------- Control --------
    def forward(self, dt: float, command: np.ndarray) -> None:
        if self._policy_counter % self.decimation == 0:
            obs = self._compute_observation(command)
            self.actions = self._compute_action(obs)
            self._previous_action = self.actions

        actions_scaled = self.actions * self.action_scale

        # Create targets (NaN entries are ignored)
        q = np.asarray(self.robot.get_joint_positions(), dtype=float).reshape(-1)
        target_pos = np.full_like(q, np.nan, dtype=float)
        target_vel = np.full_like(q, np.nan, dtype=float)

        # Legs: position targets, Wheels: velocity targets
        target_pos[self.POS_IDX] = self._default_pos[self.POS_IDX] + actions_scaled[self.POS_IDX]
        target_vel[self.VEL_IDX] = actions_scaled[self.VEL_IDX]

        action = ArticulationAction(
            joint_positions=target_pos[self.idx_map_reverse],
            joint_velocities=target_vel[self.idx_map_reverse],
        )
        self.robot.apply_action(action)
        self._policy_counter += 1

    def initialize(self, physics_sim_view=None) -> None:
        super().initialize(
            physics_sim_view=physics_sim_view,
            effort_modes="force",
            control_mode="effort",
            set_articulation_props=False,
        )
        q = np.asarray(self.robot.get_joint_positions(), dtype=float).reshape(-1)
        self._dof_count = q.size
        self._default_pos = np.asarray(self.default_pos, dtype=float).reshape(-1)[self.idx_map]

        # Per-joint action scales
        self.action_scale = np.zeros(self._dof_count, dtype=float)
        for i in self.POS_IDX:
            name = self.robot.dof_names[i]
            if re.search(r".*_hip_joint$", name):
                self.action_scale[i] = 0.125
            else:
                self.action_scale[i] = 0.25
        self.action_scale[self.VEL_IDX] = 5.0
        self.action_scale = self.action_scale[self.idx_map]
        self._previous_action = np.zeros(self._dof_count, dtype=float)
