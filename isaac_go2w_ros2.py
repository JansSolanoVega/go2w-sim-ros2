#!/usr/bin/env python3
# Run GO2-W policy in Isaac Sim (no isaaclab, no CLI args).
# - Loads Simple Warehouse
# - Spawns GO2W from USD
# - Attaches custom GO2WPolicy (legs: pos, wheels: vel)
# - Keyboard teleop (arrow keys / N, M)
# - ROS2 teleop via /cmd_vel (geometry_msgs/Twist)

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import os
import numpy as np
import carb
import omni.appwindow
from isaacsim.core.api import World
from isaacsim.core.utils.prims import define_prim
from isaacsim.storage.native import get_assets_root_path
from isaac.go2w_ctrl import GO2WPolicy, CmdVelSubscriber
from isaac.go2w_ros2_bridge import RobotDataManager
from isaac.go2w_sensors import SensorManager
import rclpy

POLICY_PT   = "ckpts/policy.pt"
ENV_YAML    = "cfg/env.yaml"
ROBOT_USD   = "assets/go2w.usd"
ROBOT_PRIM  = "/World/robot"

# ---------------------------- Runner ----------------------------
class Go2wRunner:
    def __init__(self, physics_dt: float, render_dt: float) -> None:
        self._world = World(stage_units_in_meters=1.0, physics_dt=physics_dt, rendering_dt=render_dt)

        assets_root_path = get_assets_root_path()
        if assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")

        # Spawn warehouse scene
        prim = define_prim("/World/Warehouse", "Xform")
        prim.GetReferences().AddReference(os.path.join(assets_root_path, "Isaac/Environments/Simple_Warehouse/warehouse_multiple_shelves.usd"))
        
        # Spawn the robot with our custom policy
        self._robot = GO2WPolicy(
            prim_path=ROBOT_PRIM,
            name="go2w",
            usd_path=ROBOT_USD,
            position=np.array([0.0, 0.0, 0.6]),
            policy_path=POLICY_PT,
            env_yaml_path=ENV_YAML,
        )

        self._base_command = np.zeros(3)
        self._cmdvel_node = None

        # Keyboard mappings
        self._input_keyboard_mapping = {
            "UP": [1.0, 0.0, 0.0],        "NUMPAD_8": [1.0, 0.0, 0.0],
            "DOWN": [-1.0, 0.0, 0.0],     "NUMPAD_2": [-1.0, 0.0, 0.0],
            "RIGHT": [0.0, -1.0, 0.0],    "NUMPAD_6": [0.0, -1.0, 0.0],
            "LEFT": [0.0, 1.0, 0.0],      "NUMPAD_4": [0.0, 1.0, 0.0],
            "N": [0.0, 0.0, 1.0],         "NUMPAD_7": [0.0, 0.0, 1.0],
            "M": [0.0, 0.0, -1.0],        "NUMPAD_9": [0.0, 0.0, -1.0],
        }

        self.needs_reset = False
        self.first_step = True

    def setup(self) -> None:
        # Keyboard listener
        self._appwindow = omni.appwindow.get_default_app_window()
        self._input = carb.input.acquire_input_interface()
        self._keyboard = self._appwindow.get_keyboard()
        self._sub_keyboard = self._input.subscribe_to_keyboard_events(self._keyboard, self._on_keyboard)

        # Physics callback
        self._world.add_physics_callback("go2w_forward", callback_fn=self.on_physics_step)

        # ROS2 init
        # Create sensors
        sm = SensorManager()
        lidar = sm.add_rtx_lidar()
        camera = None #sm.add_camera()

        # Create ROS2 manager
        rclpy.init()
        self._pub_data_node = RobotDataManager(self, lidar, camera)
        self._cmdvel_node = CmdVelSubscriber(self)

        # self._pub_data_node.setup_rtx_lidar_ros2()


    def on_physics_step(self, step_size: float) -> None:
        if self.first_step:
            self._robot.initialize()
            self.first_step = False
        elif self.needs_reset:
            self._world.reset(True)
            self.needs_reset = False
            self.first_step = True
        else:
            self._robot.forward(step_size, self._base_command)

        # Spin ROS node briefly
        self._pub_data_node.on_physics_step(step_size)

        if self._cmdvel_node is not None:
            rclpy.spin_once(self._cmdvel_node, timeout_sec=0.0)

    def run(self) -> None:
        while simulation_app.is_running():
            self._world.step(render=True)
            if self._world.is_stopped():
                self.needs_reset = True

    def _on_keyboard(self, event, *args, **kwargs) -> bool:
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            if event.input.name in self._input_keyboard_mapping:
                self._base_command += np.array(self._input_keyboard_mapping[event.input.name])
        elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
            if event.input.name in self._input_keyboard_mapping:
                self._base_command -= np.array(self._input_keyboard_mapping[event.input.name])
        return True


def main():
    physics_dt = 1.0 / 200.0
    render_dt = 1.0 / 60.0

    runner = Go2wRunner(physics_dt=physics_dt, render_dt=render_dt)

    simulation_app.update()
    runner._world.reset()
    simulation_app.update()

    runner.setup()
    simulation_app.update()

    runner.run()

    # Shutdown ROS cleanly
    if runner._cmdvel_node is not None:
        runner._pub_data_node.destroy_node()
        runner._cmdvel_node.destroy_node()
    rclpy.shutdown()

    simulation_app.close()


if __name__ == "__main__":
    main()
