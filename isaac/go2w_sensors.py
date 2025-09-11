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
        config: str = "Example_Rotary",
        translation: tuple[float, float, float] = (0.2, 0.0, 0.2),
        orientation_wxyz: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
    ):
        """
        Creates a single RTX LiDAR and returns its annotator.
        """
        _, sensor = omni.kit.commands.execute(
            "IsaacSensorCreateRtxLidar",
            path="/lidar",
            parent=self.base_prim,
            config=config,
            translation=translation,
            orientation=Gf.Quatd(*orientation_wxyz),
        )

        rp = rep.create.render_product(str(sensor.GetPath()), resolution=(16, 16))
        pc_annot = rep.AnnotatorRegistry.get_annotator("IsaacExtractRTXSensorPointCloudNoAccumulator")
        pc_annot.attach([rp])
        
        return pc_annot
        # sensor_attributes = {'omni:sensor:Core:scanRateBaseHz': 10.0}

        # # Create the RTX Lidar with the specified attributes.
        # sensor = LidarRtx(
        #     prim_path=f"{self.base_prim}/lidar",
        #     translation=translation,
        #     orientation=orientation_wxyz,
        #     config_file_name=config,
        #     **sensor_attributes,
        # )
        # return sensor

    # ---------------- Camera ----------------
    def add_camera(
        self,
        frequency_hz: float = 30.0,
        resolution: tuple[int, int] = (640, 480),
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
