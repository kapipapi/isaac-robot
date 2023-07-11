from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})

from tasks.follow_target import FollowTarget
from omni.isaac.franka.controllers import RMPFlowController
from omni.isaac.core import World
from omni.isaac.sensor import Camera

import numpy as np
import keyboard

my_world = World(stage_units_in_meters=1.0)

camera = Camera(
    prim_path="/World/camera",
    position=np.array([0.5, 0.0, 1.0]),
    frequency=20,
    resolution=(256, 256),
    orientation=np.array([0.68221, 0.19074, -0.21278, -0.673]),
)

camera.set_focal_length(2)
camera.set_clipping_range(0.1, 1000)

my_task = FollowTarget(name="follow_target_task")
my_world.add_task(my_task)
my_world.reset()

task_params = my_world.get_task("follow_target_task").get_params()
franka_name = task_params["robot_name"]["value"]
target_name = task_params["target_name"]["value"]

my_franka = my_world.scene.get_object(franka_name)
target = my_world.scene.get_object(target_name)

my_controller = RMPFlowController(name="target_follower_controller", robot_articulation=my_franka)
articulation_controller = my_franka.get_articulation_controller()

my_task.add_shelf(np.array([0.724714473014445, 0.09992469865078797, 0.5841155230778812]))
my_task.add_shelf(np.array([0.7150064396873923, 0.0999246986507883, 0.953007231813732]))

pressed = 0
offset = 0.05

while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_playing():
        if my_world.current_time_step_index == 0:
            my_world.reset()
            my_controller.reset()

        observations = my_world.get_observations()
        pos = observations[target_name]["position"]

        if pressed == 0:
            if keyboard.is_pressed('left'):
                print('You Pressed left!')
                pressed = 1
                pos = pos + [0, offset, 0]
                target.set_local_pose(pos)

            elif keyboard.is_pressed('right'):
                print('You Pressed right!')
                pressed = 2
                pos = pos + [0, -offset, 0]
                target.set_local_pose(pos)

            elif keyboard.is_pressed('down'):
                print('You Pressed down!')
                pressed = 3
                pos = pos + [-offset, 0, 0]
                target.set_local_pose(pos)

            elif keyboard.is_pressed('up'):
                print('You Pressed up!')
                pressed = 4
                pos = pos + [offset, 0, 0]
                target.set_local_pose(pos)

        elif pressed == 1 and not keyboard.is_pressed('left'):
            pressed = 0
        elif pressed == 2 and not keyboard.is_pressed('right'):
            pressed = 0
        elif pressed == 3 and not keyboard.is_pressed('down'):
            pressed = 0
        elif pressed == 4 and not keyboard.is_pressed('up'):
            pressed = 0

        actions = my_controller.forward(
            target_end_effector_position=observations[target_name]["position"],
            target_end_effector_orientation=observations[target_name]["orientation"],
        )

        articulation_controller.apply_action(actions)

simulation_app.close()
