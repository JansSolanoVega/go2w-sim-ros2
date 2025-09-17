import omni
import numpy as np
from pxr import Gf
import omni.replicator.core as rep
from isaacsim.sensors.camera import Camera
import isaacsim.core.utils.numpy.rotations as rot_utils
from isaacsim.sensors.rtx import LidarRtx

class SensorManager:
    """
    Attach RTX LiDAR and Camera to a single robot at /World/robot/base.
    """

    def __init__(self, base_prim: str = "/World/robot/base"):
        self.base_prim = base_prim

    # ---------------- RTX LiDAR ----------------
    def add_rtx_lidar(
        self,
        config: str = "XT32_SD10", #{Example_Rotary, VLS_128, XT32_SD10}
        translation: tuple[float, float, float] = (0.0, 0.0, 0.1),
        orientation_wxyz: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
    ):
        """
        Creates a single RTX LiDAR and returns its annotator.
        """
        sensor_attributes = {'omni:sensor:Core:scanRateBaseHz': 30.0}#10.0}
        sensor = LidarRtx(
            prim_path=f"{self.base_prim}/lidar",
            translation=translation,
            orientation=orientation_wxyz,
            config_file_name=config,
            **sensor_attributes,
        )

        sensor.initialize()
        pc_annot = rep.AnnotatorRegistry.get_annotator("IsaacCreateRTXLidarScanBuffer")
        #pc_annot = rep.AnnotatorRegistry.get_annotator("IsaacExtractRTXSensorPointCloudNoAccumulator")
        
        pc_annot.attach([sensor._render_product])
        
        return pc_annot
    
    def add_rtx_lidar_accumulator(
        self,
        config: str = "Example_Rotary",#,"Velodyne_VLS128" Hesai_XT32_SD10
        translation: tuple[float, float, float] = (0.0, 0.0, 0.1),
        orientation_wxyz: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
    ):
        """
        Creates a single RTX LiDAR and returns its annotator.
        """
        sensor_attributes = {'omni:sensor:Core:scanRateBaseHz': 30.0}
        sensor = LidarRtx(
            prim_path=f"{self.base_prim}/lidar",
            translation=translation,
            orientation=orientation_wxyz,
            config_file_name=config,
            **sensor_attributes,
        )

        sensor.initialize()
        writer = rep.WriterRegistry.get("RtxLidar" + "ROS2PublishPointCloud")
        writer.initialize(
            topicName="unitree_go2w/lidar/point_cloud",
            frameId="lidar_frame",
        )
        writer.attach([sensor._render_product])
        
        return None

    # ---------------- Camera ----------------
    def add_camera(
        self,
        frequency_hz: float = 30.0,
        resolution: tuple[int, int] = (320, 240),
        translation: tuple[float, float, float] = (0.4, 0.0, 0.2),
        rpy_deg: tuple[float, float, float] = (0.0, 0.0, 0.0),
        camera_rel_path: str = "front_cam",
        focal_length: float = 1.5,
    ):
        """
        Creates a single Camera prim and returns it.
        """
        quat = rot_utils.euler_angles_to_quats(np.array(rpy_deg), degrees=True)
        cam_prim = f"{self.base_prim}/{camera_rel_path}"
        cam = Camera(
            prim_path=cam_prim,
            translation=np.array(translation, dtype=float),
            frequency=frequency_hz,
            resolution=resolution,
            orientation=quat,
        )
        cam.initialize()
        cam.set_focal_length(focal_length)
        return cam
