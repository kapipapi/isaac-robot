from collections import OrderedDict
import carb
from omni.isaac.core.objects.cuboid import DynamicCuboid, VisualCuboid
from omni.isaac.core.prims.rigid_prim import RigidPrim
from omni.isaac.core.tasks.base_task import BaseTask
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.rotations import euler_angles_to_quat
from omni.isaac.core.utils.stage import add_reference_to_stage, get_stage_units
from omni.isaac.franka import Franka
from omni.isaac.core.utils.prims import is_prim_path_valid
from omni.isaac.core.utils.string import find_unique_string_name
from omni.isaac.sensor import Camera
from typing import Optional
import numpy as np


class FollowTarget(BaseTask):
    def __init__(
        self,
        name: str = "franka_follow_target",
    ) -> None:
        super().__init__(name=name, offset=None)
        
        self._robot = None

        self._target_name = None
        self._target = None
        self._target_prim_path = None
        self._target_position = None
        self._target_orientation = None
        self._target_visual_material = None
        if self._target_position is None:
            self._target_position = np.array([0.5, 0, 0.5]) / get_stage_units()

        self._obstacle_cubes = OrderedDict()

        return
    
    def set_up_scene(self, scene):
        super().set_up_scene(scene)
        scene.add_default_ground_plane()

        # set up target parameters
        self._target_prim_path = find_unique_string_name(
            initial_name="/World/TargetCube", is_unique_fn=lambda x: not is_prim_path_valid(x)
        )
        self._target_name = find_unique_string_name(
            initial_name="target", is_unique_fn=lambda x: not self.scene.object_exists(x)
        )
        self._target_orientation = euler_angles_to_quat(np.array([np.pi, -np.pi/2, 0.0]))

        self.set_params(
            target_prim_path=self._target_prim_path,
            target_position=self._target_position,
            target_orientation=self._target_orientation,
            target_name=self._target_name,
        )

        self._robot = self.set_robot()
        scene.add(self._robot)

        camera = Camera(
            prim_path="/World/camera",
            position=np.array([0.30, 0.0, 1.2]),
            frequency=20,
            resolution=(256, 256),
            orientation=euler_angles_to_quat(np.array([0, 35, 0]), degrees=True),
        )

        camera.set_focal_length(1)
        camera.set_clipping_range(0.1, 1000)

        camera2 = Camera(
            prim_path="/World/Franka/panda_link8/camera2",
            translation=np.array([0.0, 0.0, 0.0]),
            frequency=20,
            resolution=(256, 256),
            orientation=euler_angles_to_quat(np.array([90, -90, 90+135]), degrees=True),
        )

        camera2.set_focal_length(1)
        camera2.set_clipping_range(0.1, 1000)

        self.add_obstacle([0.5, 0.0, 0.5])

        self._task_objects[self._robot.name] = self._robot
        self._move_task_objects_to_their_frame()

        return
    
    def set_params(
        self,
        target_prim_path: Optional[str] = None,
        target_name: Optional[str] = None,
        target_position: Optional[np.ndarray] = None,
        target_orientation: Optional[np.ndarray] = None,
    ) -> None:
        """[summary]

        Args:
            target_prim_path (Optional[str], optional): [description]. Defaults to None.
            target_name (Optional[str], optional): [description]. Defaults to None.
            target_position (Optional[np.ndarray], optional): [description]. Defaults to None.
            target_orientation (Optional[np.ndarray], optional): [description]. Defaults to None.
        """
        if target_prim_path is not None:
            if self._target is not None:
                del self._task_objects[self._target.name]
            self._target = self.scene.add(
                VisualCuboid(
                    name=target_name,
                    prim_path=target_prim_path,
                    position=target_position,
                    orientation=target_orientation,
                    color=np.array([1, 0, 0]),
                    size=1.0,
                    scale=np.array([0.03, 0.03, 0.03]) / get_stage_units(),
                )
            )
            self._task_objects[self._target.name] = self._target
            self._target_visual_material = self._target.get_applied_visual_material()
        else:
            self._target.set_local_pose(position=target_position, orientation=target_orientation)
        return
    
    def get_params(self) -> dict:
        """[summary]

        Returns:
            dict: [description]
        """
        params_representation = dict()
        params_representation["target_prim_path"] = {"value": self._target.prim_path, "modifiable": True}
        params_representation["target_name"] = {"value": self._target.name, "modifiable": True}
        position, orientation = self._target.get_local_pose()
        params_representation["target_position"] = {"value": position, "modifiable": True}
        params_representation["target_orientation"] = {"value": orientation, "modifiable": True}
        params_representation["robot_name"] = {"value": self._robot.name, "modifiable": False}
        return params_representation
    
    def get_observations(self) -> dict:
        """[summary]

        Returns:
            dict: [description]
        """
        joints_state = self._robot.get_joints_state()
        target_position, target_orientation = self._target.get_local_pose()
        return {
            self._robot.name: {
                "joint_positions": np.array(joints_state.positions),
                "joint_velocities": np.array(joints_state.velocities),
            },
            self._target.name: {"position": np.array(target_position), "orientation": np.array(target_orientation)},
        }
    
    def set_robot(self) -> Franka:
        """[summary]

        Returns:
            Franka: [description]
        """
        return Franka(
            prim_path="/World/Franka",
            position=[0.15, 0.0, 1],
            orientation=euler_angles_to_quat(np.array([0.0, np.pi-np.pi/4, 0.0])),
        )

    def calculate_metrics(self) -> dict:
        """[summary]
        """
        raise NotImplementedError

    def is_done(self) -> bool:
        """[summary]
        """
        raise NotImplementedError

    def target_reached(self) -> bool:
        """[summary]

        Returns:
            bool: [description]
        """
        end_effector_position, _ = self._robot.end_effector.get_world_pose()
        target_position, _ = self._target.get_world_pose()
        if np.mean(np.abs(np.array(end_effector_position) - np.array(target_position))) < (0.035 / get_stage_units()):
            return True
        else:
            return False

    def pre_step(self, time_step_index: int, simulation_time: float) -> None:
        """[summary]

        Args:
            time_step_index (int): [description]
            simulation_time (float): [description]
        """
        if self._target_visual_material is not None:
            if hasattr(self._target_visual_material, "set_color"):
                if self.target_reached():
                    self._target_visual_material.set_color(color=np.array([0, 1.0, 0]))
                else:
                    self._target_visual_material.set_color(color=np.array([1.0, 0, 0]))

        return

    def post_reset(self) -> None:
        """[summary]
        """
        return
    
    def add_object(self, path:str, name:str, position: np.ndarray = None, orientation: np.ndarray = np.array([0,0,0,0]), scale: np.ndarray = np.array([0,0,0])):
        """[summary]

        example path "/Isaac/Environments/Simple_Warehouse/Props/SM_RackShelf_01.usd"

        Args:
            position (np.ndarray, optional): [description]. Defaults to np.array([0.5, 0.1, 1.0]).
        """
        _assets_root_path = get_assets_root_path()
        if _assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return


        object_prim_path = find_unique_string_name(
            initial_name=f"/World/{name}", is_unique_fn=lambda x: not is_prim_path_valid(x)
        )

        object_name = find_unique_string_name(initial_name=name, is_unique_fn=lambda x: not self.scene.object_exists(x))

        add_reference_to_stage(
            usd_path=_assets_root_path+path, 
            prim_path=object_prim_path
        )
        
        print(_assets_root_path)

        if position is None:
            position = np.array([-2.0, -1.5, 1.0]) / get_stage_units()

        object_ = self.scene.add(
            RigidPrim(
                prim_path=object_prim_path,
                name=object_name,
                position=position / get_stage_units(),
                orientation=orientation,
                scale=scale,
            )
        )

        return object_

    def add_obstacle(self, position: np.ndarray = None):
        """[summary]

        Args:
            position (np.ndarray, optional): [description]. Defaults to np.array([0.5, 0.1, 1.0]).
        """
        # TODO: move to task frame if there is one
        cube_prim_path = find_unique_string_name(
            initial_name="/World/ObstacleCube", is_unique_fn=lambda x: not is_prim_path_valid(x)
        )
        cube_name = find_unique_string_name(initial_name="cube", is_unique_fn=lambda x: not self.scene.object_exists(x))
        
        if position is None:
            position = np.array([0.5, 0.1, 1.0]) / get_stage_units()
        
        cube = self.scene.add(
            DynamicCuboid(
                name=cube_name,
                position=position + self._offset,
                prim_path=cube_prim_path,
                size=0.1 / get_stage_units(),
                color=np.array([0, 0, 1.0]),
            )
        )

        self._obstacle_cubes[cube.name] = cube

        return cube

    def remove_obstacle(self, name: Optional[str] = None) -> None:
        """[summary]

        Args:
            name (Optional[str], optional): [description]. Defaults to None.
        """
        if name is not None:
            self.scene.remove_object(name)
            del self._obstacle_cubes[name]
        else:
            obstacle_to_delete = list(self._obstacle_cubes.keys())[-1]
            self.scene.remove_object(obstacle_to_delete)
            del self._obstacle_cubes[obstacle_to_delete]
        return

    def get_obstacle_to_delete(self) -> None:
        """[summary]

        Returns:
            [type]: [description]
        """
        obstacle_to_delete = list(self._obstacle_cubes.keys())[-1]
        return self.scene.get_object(obstacle_to_delete)

    def obstacles_exist(self) -> bool:
        """[summary]

        Returns:
            bool: [description]
        """
        if len(self._obstacle_cubes) > 0:
            return True
        else:
            return False

    def cleanup(self) -> None:
        """[summary]
        """
        obstacles_to_delete = list(self._obstacle_cubes.keys())
        for obstacle_to_delete in obstacles_to_delete:
            self.scene.remove_object(obstacle_to_delete)
            del self._obstacle_cubes[obstacle_to_delete]
        return
