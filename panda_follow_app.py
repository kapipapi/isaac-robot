from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})

from tasks.follow_target import FollowTarget
from omni.isaac.franka.controllers import RMPFlowController
from omni.isaac.core import World

my_world = World(stage_units_in_meters=1.0)

my_task = FollowTarget(name="follow_target_task")
my_world.add_task(my_task)
my_world.reset()

task_params = my_world.get_task("follow_target_task").get_params()
franka_name = task_params["robot_name"]["value"]
target_name = task_params["target_name"]["value"]

my_franka = my_world.scene.get_object(franka_name)

my_controller = RMPFlowController(name="target_follower_controller", robot_articulation=my_franka)

articulation_controller = my_franka.get_articulation_controller()


obstacle = my_task.add_obstacle()
print(obstacle)

while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_playing():
        if my_world.current_time_step_index == 0:
            my_world.reset()
            my_controller.reset()

        observations = my_world.get_observations()

        actions = my_controller.forward(
            target_end_effector_position=observations[target_name]["position"],
            target_end_effector_orientation=observations[target_name]["orientation"],
        )

        articulation_controller.apply_action(actions)

simulation_app.close()
