"""
Launch the vision-guided cube pick WITH the MoveIt config parameters loaded.

Same rationale as pick_and_pour.launch.py: moveit_pick_cube.py uses MoveItPy,
which needs robot_description, robot_description_semantic, kinematics.yaml,
joint_limits and the planning pipelines on its OWN node. Running the script
bare gets none of that and fails immediately.

This starts ONLY the task node -- no move_group, and NO RViz. Both come from
the tyler_arm bringup in terminal 1; if RViz is not on screen, that launch is
what is missing, not this one.

Run the planner + the Pi first:

    # dev machine, terminal 1
    ros2 launch tyler_arm bringup_real.launch.py
    # Pi, terminal 2
    python3 arm_pi_node.py

    # dev machine, terminal 3 -- capture, draw in RViz, do NOT move:
    ros2 launch /home/tyler/Desktop/Robot_arm/pick_cube.launch.py plan_only:=true
    # ...and once the cube frame lands where the real cube is:
    ros2 launch /home/tyler/Desktop/Robot_arm/pick_cube.launch.py

Arguments (all optional):
    plan_only:=true       plan and publish, never execute
    cube_xyz:=-0.1,0.5,0.02   bench mode, skip the camera entirely
    no_scene:=true        skip the table/cube collision objects
    accel_scaling:=0.02   acceleration of the turret/approach/lift moves --
                          lower this first if the supply browns out
    velocity_scaling:=0.03    speed of those same moves
    lin_velocity_scaling:=0.01 / lin_accel_scaling:=0.02
                          the straight-line descent and retreat
    extra_args:="--burst 40 --max-rms 2.5"    anything else the script takes

Do not run camera_tf_publisher.py alongside this -- both open the camera.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def _launch_setup(context, *args, **kwargs):
    moveit_config = (
        MoveItConfigsBuilder("Robot_URDF", package_name="tyler_arm")
        .to_moveit_configs()
    )

    # MoveItPy reads `planning_pipelines.pipeline_names` (a nested key), but
    # MoveItConfigsBuilder emits `planning_pipelines` as a plain list (what
    # move_group wants). Without reshaping it MoveItPy sees no pipelines and
    # dies with "Failed to load planning pipelines from parameter server".
    params = moveit_config.to_dict()
    params["planning_pipelines"] = {
        "pipeline_names": ["ompl", "pilz_industrial_motion_planner"],
    }
    def scaling(name):
        return float(LaunchConfiguration(name).perform(context))

    # Governs every JOINT-SPACE move: the turret pre-turn, the approach, the
    # lift, and the curved fallbacks -- i.e. everything except the LIN descent
    # and retreat, which read the pilz_lin block below.
    #
    # These are the numbers to lower when the supply browns out. All six motors
    # run at once on an OMPL approach (the descent only moves 2nd/3rd/5th),
    # so peak draw here is the worst in the whole task. Acceleration is the
    # bigger lever: the current transient comes from accelerating the rotors,
    # not from the steady rate.
    params["plan_request_params"] = {
        "planning_attempts": 5,
        "planning_pipeline": "ompl",
        "planner_id": "RRTConnectkConfigDefault",
        "max_velocity_scaling_factor": scaling("velocity_scaling"),
        "max_acceleration_scaling_factor": scaling("accel_scaling"),
        "planning_time": 5.0,
    }
    # The straight-line descent/retreat. Lower than the joint-space moves
    # because the Pi restarts the steppers from a dead stop at every Pilz
    # waypoint with no acceleration ramp, so a high rate here loses steps
    # under gravity -- see VISION_PIPELINE.md.
    params["pilz_lin"] = {
        "plan_request_params": {
            "planning_attempts": 5,
            "planning_pipeline": "pilz_industrial_motion_planner",
            "planner_id": "LIN",
            "max_velocity_scaling_factor": scaling("lin_velocity_scaling"),
            "max_acceleration_scaling_factor": scaling("lin_accel_scaling"),
            "planning_time": 5.0,
        }
    }

    script_args = []
    if LaunchConfiguration("plan_only").perform(context).lower() == "true":
        script_args.append("--plan-only")
    if LaunchConfiguration("no_scene").perform(context).lower() == "true":
        script_args.append("--no-scene")
    hold = LaunchConfiguration("hold").perform(context).lower()
    if hold == "true":
        script_args.append("--hold")
    elif hold == "false":
        script_args.append("--no-hold")
    cube_xyz = LaunchConfiguration("cube_xyz").perform(context)
    if cube_xyz:
        # ONE token with '=', never ["--cube-xyz", value]. The reachable
        # workspace is the -X/+Y quadrant, so the value normally starts with a
        # minus, and argparse reads a separate "-0.1,..." as an option name and
        # dies with "expected one argument".
        script_args.append(f"--cube-xyz={cube_xyz}")
    base_deg = LaunchConfiguration("start_base_deg").perform(context)
    if base_deg:
        script_args.append(f"--start-base-deg={base_deg}")   # '=' : may be negative
    orientation = LaunchConfiguration("orientation").perform(context)
    if orientation:
        script_args += ["--orientation", orientation]
    yaw = LaunchConfiguration("yaw").perform(context)
    if yaw:
        script_args += ["--yaw", yaw]
    yaw_offset = LaunchConfiguration("yaw_offset").perform(context)
    if yaw_offset:
        script_args.append(f"--yaw-offset={yaw_offset}")     # '=' : may be negative
    if LaunchConfiguration("require_straight").perform(context).lower() == "true":
        script_args.append("--require-straight")
    extra = LaunchConfiguration("extra_args").perform(context)
    if extra:
        script_args += extra.split()

    return [Node(
        # No package: run the script by absolute path (chmod +x, python3
        # shebang). Keep this path in sync if you move the file.
        executable="/home/tyler/Desktop/Robot_arm/moveit_pick_cube.py",
        name="pick_cube",
        output="screen",
        parameters=[params],
        arguments=script_args,
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "plan_only", default_value="false",
            description="plan and publish to RViz, but never move the arm"),
        DeclareLaunchArgument(
            "no_scene", default_value="false",
            description="skip the table/cube collision objects"),
        DeclareLaunchArgument(
            "cube_xyz", default_value="",
            description="bench mode: skip the camera, pick here, e.g. -0.1,0.5,0.02"),
        DeclareLaunchArgument(
            "hold", default_value="",
            description="keep TF published after the task so you can look in "
                        "RViz; '' = auto (on with plan_only)"),
        DeclareLaunchArgument(
            "start_base_deg", default_value="",
            description="rotate the turret this many degrees before picking; "
                        "'' keeps the script default of 0 (no pre-turn)"),
        DeclareLaunchArgument(
            "orientation", default_value="",
            description="down (default) | fixed | vision"),
        DeclareLaunchArgument(
            "require_straight", default_value="false",
            description="fail rather than fall back to a curved path if the "
                        "straight-line descent cannot be planned"),
        DeclareLaunchArgument(
            "yaw", default_value="",
            description="what sets the wrist spin with orientation=down: "
                        "cube (default, squares the jaws to the cube's faces) "
                        "| azimuth (follows the turret bearing)"),
        DeclareLaunchArgument(
            "yaw_offset", default_value="",
            description="degrees added to the chosen yaw, to trim the finger "
                        "alignment against the real gripper"),
        DeclareLaunchArgument(
            "velocity_scaling", default_value="0.05",
            description="speed of the joint-space moves (turret, approach, "
                        "lift) as a fraction of the joint limits. Lower this "
                        "if the supply browns out on the approach."),
        DeclareLaunchArgument(
            "accel_scaling", default_value="0.03",
            description="acceleration of the joint-space moves. The bigger "
                        "lever for brownouts -- peak current comes from "
                        "accelerating, so drop this before velocity_scaling."),
        DeclareLaunchArgument(
            "lin_velocity_scaling", default_value="0.01",
            description="speed of the straight-line descent and retreat"),
        DeclareLaunchArgument(
            "lin_accel_scaling", default_value="0.02",
            description="acceleration of the straight-line descent and retreat"),
        DeclareLaunchArgument(
            "extra_args", default_value="",
            description="passed straight to moveit_pick_cube.py, e.g. '--burst 40'"),
        OpaqueFunction(function=_launch_setup),
    ])
