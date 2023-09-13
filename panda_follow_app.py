from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})


from tasks.follow_target import FollowTarget
from omni.isaac.franka.controllers import RMPFlowController
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core import World

import numpy as np
import keyboard
my_world = World(stage_units_in_meters=1.0)

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

pressed = 0
offset = 0.05

current_axis = 0
axis = ["side", "depth", "upper"]

def get_axis_offset_vec(axis, offset):
    if axis == "side":
        return [0, offset, 0]
    

    if axis == "depth":
        return [offset, 0, 0]
    

    if axis == "upper":
        return [0, 0, offset]
    
    return [0,0,0]

last_time_move = my_world.current_time
last_time_classification = my_world.current_time

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
                pressed = 1
                print('LEFT')
                last_time_classification = my_world.current_time

            elif keyboard.is_pressed('right'):
                pressed = 2
                print('RIGHT')
                last_time_classification = my_world.current_time

        diff_clsf = my_world.current_time - last_time_classification

        if 5 > diff_clsf > 1:
            if pressed == 0:
                print('RELAX - no move (', my_world.current_time - last_time_move, ')')
                last_time_classification = my_world.current_time

            if pressed == 1:
                pressed = 0
                print('LEFT on axis ', axis[current_axis])
                last_time_move = my_world.current_time
                pos = pos + get_axis_offset_vec(axis[current_axis], offset)
                target.set_local_pose(pos)

            if pressed == 2:
                pressed = 0
                print('RIGHT on axis ', axis[current_axis])
                last_time_move = my_world.current_time
                pos = pos - get_axis_offset_vec(axis[current_axis], offset)
                target.set_local_pose(pos)
            
        diff = my_world.current_time - last_time_move
        if diff > 5:
            print("NO MOVE MORE THAN 5 SECOND NEXT AXIS")
            last_time_classification = my_world.current_time
            last_time_move = my_world.current_time
            current_axis += 1
            if current_axis == 3:
                print("============ END OF ACTIVITY ============")
            else:
                print("NOW AXIS ", axis[current_axis])

        # if pressed == 0:
        #     if keyboard.is_pressed('left'):
        #         print('You Pressed left!')
        #         pressed = 1
        #         pos = pos + [0, offset, 0]
        #         target.set_local_pose(pos)
        #     elif keyboard.is_pressed('right'):
        #         print('You Pressed right!')
        #         pressed = 2
        #         pos = pos + [0, -offset, 0]
        #         target.set_local_pose(pos)
        #     elif keyboard.is_pressed('down'):
        #         print('You Pressed down!')
        #         pressed = 3
        #         pos = pos + [-offset, 0, 0]
        #         target.set_local_pose(pos)
        #     elif keyboard.is_pressed('up'):
        #         print('You Pressed up!')
        #         pressed = 4
        #         pos = pos + [offset, 0, 0]
        #         target.set_local_pose(pos)
        #     elif keyboard.is_pressed('k'):
        #         print('You Pressed up!')
        #         pressed = 7
        #         pos = pos + [0, 0, offset]
        #         target.set_local_pose(pos)
        #     elif keyboard.is_pressed('m'):
        #         print('You Pressed up!')
        #         pressed = 8
        #         pos = pos - [0, 0, offset]
        #         target.set_local_pose(pos)
        #     elif keyboard.is_pressed('space'):
        #         print('You Pressed space (',"open" if is_opening else "close",' gripper)!')
        #         pressed = 5
        #         gripper_positions = my_franka.gripper.get_joint_positions()
        #         if is_opening:
        #             my_franka.gripper.apply_action(
        #                 ArticulationAction(joint_positions=[gripper_positions[0] + (0.005), gripper_positions[1] + (0.005)])
        #             )
        #         else:
        #             my_franka.gripper.apply_action(
        #                 ArticulationAction(joint_positions=[gripper_positions[0] - (0.005), gripper_positions[1] - (0.005)])
        #             )
        #     elif keyboard.is_pressed('shift'):
        #         print('You Pressed shift (change mode)!')
        #         pressed = 6
        #         is_opening = not is_opening
        #         print("Now mode is: ", "open" if is_opening else "close")
        # elif pressed == 1 and not keyboard.is_pressed('left'):
        #     pressed = 0
        # elif pressed == 2 and not keyboard.is_pressed('right'):
        #     pressed = 0
        # elif pressed == 3 and not keyboard.is_pressed('down'):
        #     pressed = 0
        # elif pressed == 4 and not keyboard.is_pressed('up'):
        #     pressed = 0
        # elif pressed == 5 and not keyboard.is_pressed('space'):
        #     pressed = 0
        # elif pressed == 6 and not keyboard.is_pressed('shift'):
        #     pressed = 0
        # elif pressed == 7 and not keyboard.is_pressed('k'):
        #     pressed = 0
        # elif pressed == 8 and not keyboard.is_pressed('m'):
        #     pressed = 0

        actions = my_controller.forward(
            target_end_effector_position=observations[target_name]["position"],
            target_end_effector_orientation=observations[target_name]["orientation"],
        )

        articulation_controller.apply_action(actions)

simulation_app.close()
