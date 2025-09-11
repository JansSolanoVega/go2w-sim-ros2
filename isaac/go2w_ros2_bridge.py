import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist, TransformStamped
from sensor_msgs.msg import PointCloud2, PointField, Image, Imu, CameraInfo
from sensor_msgs.msg import JointState
from sensor_msgs_py import point_cloud2
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
import numpy as np
import time
from pxr import Gf
from cv_bridge import CvBridge

# Isaac Sim sensor APIs
import omni
import omni.graph.core as og
import omni.replicator.core as rep
import omni.syntheticdata._syntheticdata as sd

ext_manager = omni.kit.app.get_app().get_extension_manager()
ext_manager.set_extension_enabled_immediate("omni.isaac.ros2_bridge", True)
from isaacsim.ros2.bridge import read_camera_info
from rclpy.qos import qos_profile_sensor_data

# class RobotDataManager(Node):
#     """
#     Publishes ROS 2 topics for a single robot:
#       /odom, /pose, /joint_states, /imu
#       /points (PointCloud2 from RTX LiDAR)
#       /camera/color/image_raw
#       /camera/depth/image_raw
#       /camera/camera_info
#     Subscribes:
#       /cmd_vel
#     """

#     def __init__(self, runner, lidar=None, camera=None):
#         super().__init__("robot_data_manager")
#         self.runner = runner  # expects runner._robot.robot articulation
#         self.lidar = lidar
#         self.camera = camera
#         self.bridge = CvBridge()

#         # Pubs/Subs
#         self.pub_odom = self.create_publisher(Odometry, "/odom", 10)
#         self.pub_pose = self.create_publisher(PoseStamped, "/pose", 10)
#         self.pub_js = self.create_publisher(JointState, "/joint_states", 10)
#         self.pub_imu  = self.create_publisher(Imu, "/imu", qos_profile_sensor_data)
#         self.lidar_pub = self.create_publisher(PointCloud2, "unitree_go2w/lidar/point_cloud", 10)

#         self.pub_points = self.create_publisher(PointCloud2, "/points", 10)
#         self.pub_img_color = self.create_publisher(Image, "/unitree_go2w/front_cam/image_raw", 10)
#         self.pub_cam_info = self.create_publisher(CameraInfo, "/unitree_go2w/front_cam/info", 10)

#         self.sub_cmd = self.create_subscription(Twist, "/unitree_go2w/cmd_vel", self._on_cmd_vel, 10)

#         # TF broadcasters
#         self.tf_dyn = TransformBroadcaster(self)
#         self._publish_static_frames()

#         # Buffers for IMU linear acceleration
#         self.prev_lin_vel_b = np.zeros(3)
#         self.prev_time_sec = self.get_clock().now().nanoseconds * 1e-9

#         # Rates
#         self.odom_freq = 20.0
#         self.lidar_freq = 15.0
#         self.imu_freq   = 200.0
#         self.cam_freq = 30.0
#         self._t_last_odom = time.time()
#         self._t_last_lidar = time.time()
#         self._t_last_cam = time.time()
#         self._t_last_imu   = time.time()

#     # ------------------- Sub -------------------
#     def _on_cmd_vel(self, msg: Twist):
#         self.runner._base_command = np.array([msg.linear.x, msg.linear.y, msg.angular.z], dtype=float)

#     # ------------------- Static TFs -------------------
#     def _publish_static_frames(self):
#         # imu_link relative to base_link
#         imu_broadcaster = StaticTransformBroadcaster(self)
#         t = TransformStamped()
#         t.header.stamp = self.get_clock().now().to_msg()
#         t.header.frame_id = "base_link"
#         t.child_frame_id = "imu_link"
#         t.transform.rotation.w = 1.0
#         imu_broadcaster.sendTransform(t)

#         # lidar_frame
#         lidar_broadcaster = StaticTransformBroadcaster(self)
#         t = TransformStamped()
#         t.header.stamp = self.get_clock().now().to_msg()
#         t.header.frame_id = "base_link"
#         t.child_frame_id = "lidar_frame"
#         t.transform.translation.x = 0.2
#         t.transform.translation.z = 0.2
#         t.transform.rotation.w = 1.0
#         lidar_broadcaster.sendTransform(t)

#         # camera_link
#         camera_broadcaster = StaticTransformBroadcaster(self)
#         t = TransformStamped()
#         t.header.stamp = self.get_clock().now().to_msg()
#         t.header.frame_id = "base_link"
#         t.child_frame_id = "camera_link"
#         t.transform.translation.x = 0.4
#         t.transform.translation.z = 0.2
#         t.transform.rotation.w = 1.0
#         camera_broadcaster.sendTransform(t)

#     # ------------------- Main tick -------------------
#     def tick(self):
#         now = time.time()

#         # Odom + Joints
#         if now - self._t_last_odom >= 1.0 / self.odom_freq:
#             self._t_last_odom = now
#             self._publish_odom_pose_js()

#         if now - self._t_last_imu >= 1.0 / self.imu_freq:
#             self._t_last_imu = now
#             self._publish_imu_fast()

#         # Camera
#         # if self.camera and (now - self._t_last_cam >= 1.0 / self.cam_freq):
#         #     self._t_last_cam = now
#         #     self.pub_color_image()

#         # Handle subs
#         rclpy.spin_once(self, timeout_sec=0.0)

#     # ------------------- Publishers -------------------
#     def _publish_odom_pose_js(self):
#         art = self.runner._robot.robot
#         pos, quat = art.get_world_pose()  # (xyz, wxyz)
#         lin_vel = art.get_linear_velocity()
#         ang_vel = art.get_angular_velocity()
#         q = art.get_joint_positions()
#         dq = art.get_joint_velocities()

#         # Odometry
#         odom = Odometry()
#         odom.header.stamp = self.get_clock().now().to_msg()
#         odom.header.frame_id = "odom"
#         odom.child_frame_id = "base_link"
#         odom.pose.pose.position.x = float(pos[0])
#         odom.pose.pose.position.y = float(pos[1])
#         odom.pose.pose.position.z = float(pos[2])
#         odom.pose.pose.orientation.w = float(quat[0])
#         odom.pose.pose.orientation.x = float(quat[1])
#         odom.pose.pose.orientation.y = float(quat[2])
#         odom.pose.pose.orientation.z = float(quat[3])
#         odom.twist.twist.linear.x = float(lin_vel[0])
#         odom.twist.twist.linear.y = float(lin_vel[1])
#         odom.twist.twist.linear.z = float(lin_vel[2])
#         odom.twist.twist.angular.x = float(ang_vel[0])
#         odom.twist.twist.angular.y = float(ang_vel[1])
#         odom.twist.twist.angular.z = float(ang_vel[2])
#         self.pub_odom.publish(odom)

#         # TF odom->base_link
#         t = TransformStamped()
#         t.header = odom.header
#         t.child_frame_id = "base_link"
#         t.transform.translation.x = float(pos[0])
#         t.transform.translation.y = float(pos[1])
#         t.transform.translation.z = float(pos[2])
#         t.transform.rotation = odom.pose.pose.orientation
#         self.tf_dyn.sendTransform(t)

#         # PoseStamped
#         ps = PoseStamped()
#         ps.header = odom.header
#         ps.pose = odom.pose.pose
#         self.pub_pose.publish(ps)

#         # JointState
#         js = JointState()
#         js.header = odom.header
#         js.name = list(art.dof_names)
#         js.position = [float(v) for v in q]
#         js.velocity = [float(v) for v in dq]
#         self.pub_js.publish(js)

#     def _publish_imu(self, quat_wxyz, lin_vel_w, ang_vel_w):
#         from scipy.spatial.transform import Rotation as R
#         R_WB = R.from_quat([quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0]]).as_matrix()
#         v_b = R_WB.T @ np.array(lin_vel_w)
#         w_b = R_WB.T @ np.array(ang_vel_w)

#         now_sec = self.get_clock().now().nanoseconds * 1e-9
#         dt = max(1e-6, now_sec - self.prev_time_sec)
#         a_b = (v_b - self.prev_lin_vel_b) / dt
#         g_b = R_WB.T @ np.array([0, 0, -9.81])
#         a_b += g_b

#         msg = Imu()
#         msg.header.stamp = self.get_clock().now().to_msg()
#         msg.header.frame_id = "imu_link"
#         msg.orientation.w = float(quat_wxyz[0])
#         msg.orientation.x = float(quat_wxyz[1])
#         msg.orientation.y = float(quat_wxyz[2])
#         msg.orientation.z = float(quat_wxyz[3])
#         msg.angular_velocity.x = float(w_b[0])
#         msg.angular_velocity.y = float(w_b[1])
#         msg.angular_velocity.z = float(w_b[2])
#         msg.linear_acceleration.x = float(a_b[0])
#         msg.linear_acceleration.y = float(a_b[1])
#         msg.linear_acceleration.z = float(a_b[2])
#         self.pub_imu.publish(msg)

#         self.prev_lin_vel_b = v_b
#         self.prev_time_sec = now_sec

#     def _publish_imu_fast(self):
#         art = self.runner._robot.robot
#         _, quat = art.get_world_pose()
#         lin_vel = art.get_linear_velocity()
#         ang_vel = art.get_angular_velocity()
#         self._publish_imu(quat, lin_vel, ang_vel)

#     def setup_rtx_lidar_ros2(self,
#                             lidar_prim_path="/World/robot/base/lidar",     # where your SensorManager created it
#                             frame_id="lidar_frame",
#                             pc_topic="unitree_go2w/lidar/point_cloud"):
#         """
#         Creates the Replicator pipelines to publish LiDAR to ROS 2.
#         Call once during setup, AFTER the lidar prim exists.
#         """
#         sensor_path = lidar_prim_path
#         hydra_texture = rep.create.render_product(sensor_path, [1, 1], name="RTX_LidarRP")
#         writer_pc = rep.writers.get("RtxLidar" + "ROS2PublishPointCloud")
#         writer_pc.initialize(topicName=pc_topic, frameId=frame_id)
#         writer_pc.attach([hydra_texture])


#     def pub_color_image(self):
#         render_product = self.camera._render_product_path
#         step_size = 1
#         topic_name = "/unitree_go2w/front_cam/image_raw"
#         frame_id = "camera_link"

#         rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(sd.SensorType.Rgb.name)
#         writer = rep.writers.get(rv + "ROS2PublishImage")
#         writer.initialize(
#             frameId=frame_id,
#             nodeNamespace="",
#             queueSize=1,
#             topicName=topic_name,
#         )
#         writer.attach([render_product])

#         # Control execution rate
#         gate_path = omni.syntheticdata.SyntheticData._get_node_path(rv + "IsaacSimulationGate", render_product)
#         og.Controller.attribute(gate_path + ".inputs:step").set(step_size)

# +++ add at top with other imports +++
from rclpy.qos import qos_profile_sensor_data
import omni.graph.core as og
import omni.physx as physx
from scipy.spatial.transform import Rotation as R
import subprocess
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from scipy.spatial.transform import Rotation as R

class RobotDataManager(Node):
    def __init__(self, runner, lidar=None, camera=None, physics_dt=1/200, lidar_freq = 10.0, imu_freq=200.0, odom_freq=50.0):
        super().__init__("robot_data_manager")
        # self.declare_parameter("use_sim_time", True)

        self.lidar = lidar
        self.runner = runner

        self.physics_dt = float(physics_dt)
        self.imu_freq  = float(imu_freq)
        self.odom_freq = float(odom_freq)
        self.lidar_freq = float(lidar_freq)

        # decimation
        self.imu_decim  = max(1, int(round((1.0/self.imu_freq)/self.physics_dt)))  # = 1 for 200 Hz @ 1/200
        self.odom_decim = max(1, int(round((1.0/self.odom_freq)/self.physics_dt))) # = 4 for 50 Hz
        self.lidar_decim = max(1, int(round((1.0/self.lidar_freq)/self.physics_dt))) # = 4 for 50 Hz
        self._imu_step = 0
        self._odom_step = 0
        self._lidar_step = 0

        # single /clock publisher (best effort, keep_last=1)
        clock_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.pub_clock = self.create_publisher(Clock, "/clock", clock_qos)

        # other pubs
        self.pub_imu  = self.create_publisher(Imu, "/imu", clock_qos)  # high-rate sensor QoS is fine here
        self.pub_odom = self.create_publisher(Odometry, "/odom", 10)
        self.pub_pose = self.create_publisher(PoseStamped, "/pose", 10)
        self.pub_js   = self.create_publisher(JointState, "/joint_states", 10)
        self.pub_lidar = self.create_publisher(PointCloud2, "unitree_go2w/lidar/point_cloud", 10)
        self.tf_dyn   = TransformBroadcaster(self)
        self._publish_static_frames()

        # IMU state & our own sim-time accumulator
        self.sim_t = 0.0
        self.prev_lin_vel_b = np.zeros(3)

        self.prev_stamp = None

    def _stamp(self) -> Time:
        sec = int(self.sim_t)
        nsec = int((self.sim_t - sec) * 1e9)
        t = Time(); t.sec = sec; t.nanosec = nsec
        return t

    # call this from your runner's physics callback
    def on_physics_step(self, step_size: float):
        # advance our sim time
        self.sim_t += step_size

        # publish /clock exactly once
        clk = Clock(); clk.clock = self._stamp()
        self.pub_clock.publish(clk)

        # IMU @ 200 Hz (every step at 1/200)
        self._imu_step += 1
        if (self._imu_step % self.imu_decim) == 0:
            self._publish_imu_fast()
            self._imu_step = 0

        # og.Controller.evaluate_sync(self._imu_graph)

        # Lidar @ 10 Hz (every 20 steps at 1/200)
        self._lidar_step += 1
        if (self._lidar_step % self.lidar_freq) == 0:
            self.publish_lidar_data(self.lidar.get_data()["data"].reshape(-1, 3))
            self._lidar_step = 0

        # Odom/Pose/JS @ 50 Hz (every 4 steps at 1/200)
        self._odom_step += 1
        if (self._odom_step % self.odom_decim) == 0:
            self._publish_odom_pose_js()
            self._odom_step = 0

    def _publish_imu_fast(self):
        art = self.runner._robot.robot
        _, quat = art.get_world_pose()
        lin_vel = art.get_linear_velocity()
        ang_vel = art.get_angular_velocity()
        self._publish_imu(quat, lin_vel, ang_vel)

    def _publish_imu(self, quat_wxyz, lin_vel_w, ang_vel_w):
        dt = self.imu_decim * self.physics_dt  # exact physics time between IMU ticks

        R_WB = R.from_quat([quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0]]).as_matrix()
        v_b = R_WB.T @ np.array(lin_vel_w)
        w_b = R_WB.T @ np.array(ang_vel_w)

        a_b = (v_b - self.prev_lin_vel_b) / dt
        a_b += R_WB.T @ np.array([0.0, 0.0, -9.81])

        msg = Imu()
        msg.header.stamp = self._stamp()
        msg.header.frame_id = "imu_link"
        msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z = map(float, quat_wxyz[:4])
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = map(float, w_b[:3])
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = map(float, a_b[:3])
        self.pub_imu.publish(msg)

        self.prev_lin_vel_b = v_b

    def _publish_odom_pose_js(self):
        art = self.runner._robot.robot
        pos, quat = art.get_world_pose()
        lin_vel   = art.get_linear_velocity()
        ang_vel   = art.get_angular_velocity()
        q         = art.get_joint_positions()
        dq        = art.get_joint_velocities()

        stamp_now = self._stamp()
        
        odom = Odometry()
        odom.header.stamp = stamp_now
        odom.header.frame_id = "odom"
        odom.child_frame_id  = "base_link"
        odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = map(float, pos[:3])
        odom.pose.pose.orientation.w, odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z = map(float, quat[:4])
        odom.twist.twist.linear.x,  odom.twist.twist.linear.y,  odom.twist.twist.linear.z  = map(float, lin_vel[:3])
        odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z = map(float, ang_vel[:3])
        self.pub_odom.publish(odom)

        t = TransformStamped()
        t.header = odom.header
        t.child_frame_id = "base_link"
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = map(float, pos[:3])
        t.transform.rotation = odom.pose.pose.orientation
        self.tf_dyn.sendTransform(t)

        ps = PoseStamped(); ps.header = odom.header; ps.pose = odom.pose.pose
        self.pub_pose.publish(ps)

        js = JointState(); js.header = odom.header
        js.name = list(art.dof_names)
        js.position = [float(v) for v in q]
        js.velocity = [float(v) for v in dq]
        self.pub_js.publish(js)

    def _on_cmd_vel(self, msg: Twist):
        self.runner._base_command = np.array([msg.linear.x, msg.linear.y, msg.angular.z], dtype=float)

    def _publish_static_frames(self):
        # imu_link relative to base_link
        imu_broadcaster = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "base_link"
        t.child_frame_id = "imu_link"
        t.transform.rotation.w = 1.0
        imu_broadcaster.sendTransform(t)

        # lidar_frame
        lidar_broadcaster = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "base_link"
        t.child_frame_id = "lidar_frame"
        t.transform.translation.x = 0.2
        t.transform.translation.z = 0.2
        t.transform.rotation.w = 1.0
        lidar_broadcaster.sendTransform(t)

        # camera_link
        camera_broadcaster = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "base_link"
        t.child_frame_id = "camera_link"
        t.transform.translation.x = 0.4
        t.transform.translation.z = 0.2
        t.transform.rotation.w = 1.0
        camera_broadcaster.sendTransform(t)
        
    # def setup_rtx_lidar_ros2(self,
    #                         lidar_prim_path="/World/robot/base/lidar",     # where your SensorManager created it
    #                         frame_id="lidar_frame",
    #                         pc_topic="unitree_go2w/lidar/point_cloud"):
    #     """
    #     Creates the Replicator pipelines to publish LiDAR to ROS 2.
    #     Call once during setup, AFTER the lidar prim exists.
    #     """
    #     sensor_path = lidar_prim_path
    #     hydra_texture = rep.create.render_product(sensor_path, [1, 1], name="RTX_LidarRP")
    #     writer_pc = rep.writers.get("RtxLidar" + "ROS2PublishPointCloud")
    #     writer_pc.initialize(topicName=pc_topic, frameId=frame_id)
    #     writer_pc.attach([hydra_texture])

    def publish_lidar_data(self, points):
        point_cloud = PointCloud2()
        point_cloud.header.frame_id = "lidar_frame"
        point_cloud.header.stamp = self._stamp()#self.get_clock().now().to_msg()
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        point_cloud = point_cloud2.create_cloud(point_cloud.header, fields, points)
        self.pub_lidar.publish(point_cloud)  