from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist, TransformStamped
from sensor_msgs.msg import PointCloud2, PointField, Imu
from sensor_msgs.msg import JointState
from sensor_msgs_py import point_cloud2
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
import numpy as np
from pxr import Gf

# Isaac Sim sensor APIs
import omni
import rclpy

ext_manager = omni.kit.app.get_app().get_extension_manager()
ext_manager.set_extension_enabled_immediate("omni.isaac.ros2_bridge", True)

from scipy.spatial.transform import Rotation as R
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from scipy.spatial.transform import Rotation as R

import omni.replicator.core as rep
import omni.syntheticdata._syntheticdata as sd
import omni.graph.core as og
from rclpy.parameter import Parameter

class RobotDataManager(Node):
    def __init__(self, runner, lidar=None, camera=None, physics_dt=1/200, lidar_freq = 10.0, imu_freq=200.0, odom_freq=50.0):
        super().__init__("robot_data_manager")#, parameter_overrides=[rclpy.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        
        self.lidar = lidar
        self.runner = runner
        self.camera = camera

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

        clock_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
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
        self.pub_color_image()

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

        # IMU
        self._imu_step += 1
        if (self._imu_step % self.imu_decim) == 0:
            self._publish_imu_fast()
            self._imu_step = 0

        # Lidar
        self._lidar_step += 1
        if (self._lidar_step % self.lidar_decim) == 0:
            self.publish_lidar_data_lio(self.lidar.get_data()["data"].reshape(-1, 3))
            self._lidar_step = 0

        # Odom/Pose/JS
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

    def pub_color_image(self):
        render_product = self.camera._render_product_path
        step_size = 1
        topic_name = "/unitree_go2w/front_cam/image_raw"
        frame_id = "camera_link"

        rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(sd.SensorType.Rgb.name)
        writer = rep.writers.get(rv + "ROS2PublishImage")
        writer.initialize(
            frameId=frame_id,
            nodeNamespace="",
            queueSize=1,
            topicName=topic_name,
        )
        writer.attach([render_product])

        # Control execution rate
        gate_path = omni.syntheticdata.SyntheticData._get_node_path(rv + "IsaacSimulationGate", render_product)
        og.Controller.attribute(gate_path + ".inputs:step").set(step_size)
    
    def publish_lidar_data_lio(self, points_xyz: np.ndarray,
                       spin_hz: float = 10.0,
                       n_scan: int = 32,
                       ring_mode: str = "zeros",   # "zeros" or "modulo"
                       intensities: np.ndarray | None = None):
        """
        Publish dense XYZIRT cloud expected by LIO pipelines.
        - points_xyz: Nx3 float array (may contain NaNs)
        - spin_hz: LiDAR spin rate; used to synthesize per-point time (deskew)
        - n_scan: vertical channels if you want a modulo-based fake 'ring'
        - ring_mode: "zeros" (safe) or "modulo" (i % n_scan)
        - intensities: optional Nx1 float array; None -> fill 1.0
        """
        if points_xyz is None:
            return

        pts = np.asarray(points_xyz, dtype=np.float32).reshape(-1, 3)
        # remove NaNs -> dense cloud
        mask = np.isfinite(pts).all(axis=1)
        pts = pts[mask]
        if pts.size == 0:
            return
        N = pts.shape[0]

        # intensity
        if intensities is not None:
            intensities = np.asarray(intensities, dtype=np.float32).reshape(-1)
            intensities = intensities[:len(points_xyz)][mask]
        else:
            intensities = np.ones(N, dtype=np.float32)

        # ring (synthetic)
        if ring_mode == "modulo" and n_scan > 1:
            ring = (np.arange(N, dtype=np.int32) % n_scan).astype(np.float32)
        else:
            ring = np.zeros(N, dtype=np.float32)

        # per-point relative time (0 .. scan_period)
        scan_period = 1.0 / float(spin_hz) if spin_hz > 0 else 0.1
        time_rel = np.linspace(0.0, scan_period, N, dtype=np.float32)

        # stack XYZIRT as float32 for clean 24B alignment
        xyzirt = np.column_stack((pts, intensities, ring, time_rel)).astype(np.float32)

        # header from your sim-time
        header = PointCloud2().header
        header.frame_id = "lidar_frame"
        header.stamp = self._stamp()

        fields = [
            PointField(name="x",         offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name="y",         offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name="z",         offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name="ring",      offset=16, datatype=PointField.FLOAT32, count=1),  # keep float for alignment
            PointField(name="time",      offset=20, datatype=PointField.FLOAT32, count=1),
        ]

        cloud = point_cloud2.create_cloud(header, fields, xyzirt.tolist())
        cloud.is_dense = True
        self.pub_lidar.publish(cloud)
