import time

import numpy as np
from lerobot.model.kinematics import RobotKinematics
from lerobot.processor import RobotAction, RobotObservation, RobotProcessorPipeline
from lerobot.processor.converters import (
    robot_action_observation_to_transition,
    transition_to_robot_action,
)
from lerobot.robots.so100_follower.config_so100_follower import SO100FollowerConfig
from lerobot.robots.so100_follower.robot_kinematic_processor import (
    EEBoundsAndSafety,
    EEReferenceAndDelta,
    GripperVelocityToJoint,
    InverseKinematicsEEToJoints,
)
from lerobot.robots.so100_follower.so100_follower import SO100Follower
from lerobot.teleoperators.phone.config_phone import PhoneConfig, PhoneOS
from lerobot.utils.robot_utils import busy_wait
from lerobot.utils.visualization_utils import init_rerun, log_rerun_data
from teleop_android import AndroidPhone, MapPhoneActionToRobotAction

#:


FPS = 30

PATH_URDF = "../../setup-arm/SO-ARM100/Simulation/SO101/so101_new_calib.urdf"

#:

# Initialize the robot
robot_config = SO100FollowerConfig(
    port="/dev/tty.usbmodem5A460829821", id="arm_follower_0", use_degrees=True
)
robot = SO100Follower(robot_config)

# Initialize the telooperator
teleop_config = PhoneConfig(phone_os=PhoneOS.ANDROID)
teleop_config.camera_offset = np.array([0.0, -0.01, 0.05])
teleop_device = AndroidPhone(config=teleop_config)

# NOTE: It is highly recommended to use the urdf in the SO-ARM100 repo: https://github.com/TheRobotStudio/SO-ARM100/blob/main/Simulation/SO101/so101_new_calib.urdf
kinematics_solver = RobotKinematics(
    urdf_path=PATH_URDF,
    target_frame_name="gripper_frame_link",
    joint_names=list(robot.bus.motors.keys()),
)

# Build pipeline to convert phone action to ee pose action to joint action
phone_to_robot_joints_processor = RobotProcessorPipeline[
    tuple[RobotAction, RobotObservation], RobotAction
](
    steps=[
        MapPhoneActionToRobotAction(),
        EEReferenceAndDelta(
            kinematics=kinematics_solver,
            end_effector_step_sizes={"x": 0.5, "y": 0.5, "z": 0.5},
            motor_names=list(robot.bus.motors.keys()),
            use_latched_reference=True,
        ),
        EEBoundsAndSafety(
            end_effector_bounds={"min": [-1.0, -1.0, -1.0], "max": [1.0, 1.0, 1.0]},
            max_ee_step_m=0.10,
        ),
        GripperVelocityToJoint(
            speed_factor=20.0,
        ),
        InverseKinematicsEEToJoints(
            kinematics=kinematics_solver,
            motor_names=list(robot.bus.motors.keys()),
            initial_guess_current_joints=True,
        ),
    ],
    to_transition=robot_action_observation_to_transition,
    to_output=transition_to_robot_action,
)

# Connect to the robot and teleoperator
robot.connect()
teleop_device.connect()

# Init rerun viewer
init_rerun(session_name="phone_so101_teleop")

if not robot.is_connected or not teleop_device.is_connected:
    raise ValueError("Robot or teleop is not connected!")

print("Starting teleop loop. Move your phone to teleoperate the robot...")
while True:
    t0 = time.perf_counter()

    # Get robot observation
    robot_obs = robot.get_observation()

    # Get teleop action
    phone_obs = teleop_device.get_action()

    # Phone -> EE pose -> Joints transition
    joint_action = phone_to_robot_joints_processor((phone_obs, robot_obs))

    # print("Robot obs")
    # print(robot_obs)
    # print("Joint action")
    # print(joint_action)
    # print("====\n")

    # Send action to robot
    _ = robot.send_action(joint_action)

    # Visualize
    log_rerun_data(observation=phone_obs, action=joint_action)

    busy_wait(max(1.0 / FPS - (time.perf_counter() - t0), 0.0))
