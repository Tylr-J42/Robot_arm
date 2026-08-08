"""
Launch the pick-and-pour task script WITH the MoveIt config parameters loaded.

moveit_pick_and_pour.py uses MoveItPy, which needs robot_description,
robot_description_semantic, kinematics.yaml, joint_limits and the planning
pipelines on its own node -- a bare `python3 moveit_pick_and_pour.py` has none
of that and will fail. This wrapper builds those params from the tyler_arm
config package and runs the script as a node with them attached.

This does NOT start move_group. Run the planner + the Pi first, then this:

    # dev machine, terminal 1
    ros2 launch tyler_arm bringup_real.launch.py
    # Pi, terminal 2
    python3 arm_pi_node.py
    # dev machine, terminal 3
    ros2 launch /home/tyler/Desktop/Robot_arm/pick_and_pour.launch.py
"""

from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("Robot_URDF", package_name="tyler_arm")
        .to_moveit_configs()
    )

    # MoveItPy reads the param `planning_pipelines.pipeline_names` (a nested
    # key), but MoveItConfigsBuilder emits `planning_pipelines` as a plain list
    # (what move_group wants). Without reshaping it, MoveItPy sees no pipelines
    # and dies with "Failed to load planning pipelines from parameter server".
    params = moveit_config.to_dict()
    params["planning_pipelines"] = {
        "pipeline_names": ["ompl", "pilz_industrial_motion_planner"],
    }
    # Default plan-request params used by arm.plan() with no arguments.
    params["plan_request_params"] = {
        "planning_attempts": 5,
        "planning_pipeline": "ompl",
        "planner_id": "RRTConnectkConfigDefault",
        "max_velocity_scaling_factor": 0.1,
        "max_acceleration_scaling_factor": 0.1,
        "planning_time": 5.0,
    }
    # Named namespace for the straight-line (Pilz LIN) moves; the script asks
    # for it via PlanRequestParameters(moveit, "pilz_lin").
    params["pilz_lin"] = {
        "plan_request_params": {
            "planning_attempts": 5,
            "planning_pipeline": "pilz_industrial_motion_planner",
            "planner_id": "LIN",
            # LIN maps a Cartesian path onto joints; near-singular stretches
            # spike joint accel, so keep these low or LIN violates joint limits.
            "max_velocity_scaling_factor": 0.05,
            "max_acceleration_scaling_factor": 0.02,
            "planning_time": 5.0,
        }
    }

    task_node = Node(
        # No package: run the script by absolute path (it is chmod +x with a
        # python3 shebang). Keep this path in sync if you move the file.
        executable="/home/tyler/Desktop/Robot_arm/moveit_pick_and_pour.py",
        name="pick_and_pour",
        output="screen",
        parameters=[params],
    )

    return LaunchDescription([task_node])
