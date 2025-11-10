# Example usage - this file requires the optional rerun dependency
# Install with: uv sync --extra rerun

# In ARCore world coordinates:
# - Y is aligned with gravity and points up. This remains fixed.
# - X and Z are chosen at session start, typically based on the phone's initial facing
#   so that Z roughly matches the initial forward direction and X the initial right.
# - After initialization, the world X/Y/Z axes are fixed in space; they do not rotate
#   with the phone. The phone's camera pose moves/rotates within this fixed frame.

import time

import numpy as np
import rerun as rr
import transforms3d as t3d
from lerobot.model.kinematics import RobotKinematics
from lerobot.processor import RobotAction, RobotObservation, RobotProcessorPipeline
from lerobot.processor.converters import (
    robot_action_observation_to_transition,
    transition_to_robot_action,
)
from lerobot.robots.so100_follower.robot_kinematic_processor import (
    EEBoundsAndSafety,
    EEReferenceAndDelta,
    InverseKinematicsEEToJoints,
)
from lerobot.teleoperators.phone.config_phone import PhoneConfig, PhoneOS
from rerun import blueprint as rrb
from teleop_android import (
    AndroidPhone,
    GripperToJoint,
    MapPhoneActionToRobotAction,
    Pose,
    WristJoints,
)

#: Constants

# WARN: Set this to the path of your URDF file for the SO101, you must edit it to
# add "lower_arm_frame_link", see README.
PATH_URDF = "../../setup-arm/SO-ARM100/Simulation/SO101/so101_new_calib.urdf"

FPS = 30

MOTOR_TO_RERUN = {
    "shoulder_pan": {
        "path": "/world_robot/so101_new_calib/base_link/shoulder_pan",
        "axis": [0, 0, -1],
        "pos_init": -8.835164835164836,
    },
    "shoulder_lift": {
        "path": "/world_robot/so101_new_calib/base_link/shoulder_pan/shoulder_link/shoulder_lift",
        "axis": [0, 1, 0],
        "pos_init": -102.37362637362638,
    },
    "elbow_flex": {
        "path": "/world_robot/so101_new_calib/base_link/shoulder_pan/shoulder_link/shoulder_lift/upper_arm_link/elbow_flex",
        "axis": [0, 0, 1],
        "pos_init": 97.0989010989011,
    },
    "wrist_flex": {
        "path": "/world_robot/so101_new_calib/base_link/shoulder_pan/shoulder_link/shoulder_lift/upper_arm_link/elbow_flex/lower_arm_link/wrist_flex",
        "axis": [0, 0, 1],
        "pos_init": 64.21978021978022,
    },
    "wrist_roll": {
        "path": "/world_robot/so101_new_calib/base_link/shoulder_pan/shoulder_link/shoulder_lift/upper_arm_link/elbow_flex/lower_arm_link/wrist_flex/wrist_link/wrist_roll",
        "axis": [0, -1, 0],
        "pos_init": -2.241758241758242,
    },
    "gripper": {
        "path": "/world_robot/so101_new_calib/base_link/shoulder_pan/shoulder_link/shoulder_lift/upper_arm_link/elbow_flex/lower_arm_link/wrist_flex/wrist_link/wrist_roll/gripper_link/gripper",
        "axis": [0, -1, 0],
        "pos_init": 1.1019283746556474,
    },
}

XYZ_AXIS_NAMES = ["x", "y", "z"]
RPY_AXIS_NAMES = ["roll", "pitch", "yaw"]
XYZ_AXIS_COLORS = [[(231, 76, 60), (39, 174, 96), (52, 120, 219)]]

TF_RUB2FLU = np.array([[0, 0, -1, 0], [-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 1]])
TF_XYZW_TO_WXYZ = np.array([[0, 0, 0, 1], [1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0]])
TF_WXYZ_TO_XYZW = np.array([[0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1], [1, 0, 0, 0]])


#: Init Rerun

blueprint = rrb.Horizontal(
    rrb.Vertical(
        rrb.TimeSeriesView(
            origin="position",
            name="Position",
            overrides={
                "/position": rr.SeriesLines.from_fields(
                    names=XYZ_AXIS_NAMES, colors=XYZ_AXIS_COLORS
                ),  # type: ignore[arg-type]
            },
        ),
        rrb.TimeSeriesView(
            origin="orientation",
            name="Orientation",
            overrides={
                "/orientation": rr.SeriesLines.from_fields(
                    names=RPY_AXIS_NAMES, colors=XYZ_AXIS_COLORS
                ),  # type: ignore[arg-type]
            },
        ),
    ),
    rrb.Vertical(
        rrb.Spatial3DView(
            origin="/world_phone",
            name="Phone position in the world",
            time_ranges=rrb.VisibleTimeRanges(
                timeline="log_time",
                start=rrb.TimeRangeBoundary.cursor_relative(seconds=-60),
                end=rrb.TimeRangeBoundary.cursor_relative(seconds=0),
            ),
        ),
        rrb.Spatial3DView(
            origin="/world_robot",
            name="Robot position in the world",
        ),
    ),
    column_shares=[0.45, 0.55],
)

rr.init("test_teleop", spawn=True, default_blueprint=blueprint)

# Set FLU coordinate system
rr.log("/", rr.ViewCoordinates.FLU, static=True)

# Create a pinhole camera with no images to aid visualizations
rr.log(
    "/world_phone/phone",
    rr.Pinhole(
        focal_length=(500.0, 500.0),
        resolution=(640, 480),
        image_plane_distance=0.5,
        camera_xyz=rr.ViewCoordinates.FLU,
    ),
    static=True,
)

rr.log_file_from_path(PATH_URDF, entity_path_prefix="/world_robot", static=True)

#: Setup robot pipeline and teleoperator

kinematics_solver = RobotKinematics(
    urdf_path=PATH_URDF,
    target_frame_name="lower_arm_frame_link",
    joint_names=list(MOTOR_TO_RERUN.keys()),
)

phone_to_robot_joints_processor = RobotProcessorPipeline[
    tuple[RobotAction, RobotObservation], RobotAction
](
    steps=[
        MapPhoneActionToRobotAction(),
        EEReferenceAndDelta(
            kinematics=kinematics_solver,
            end_effector_step_sizes={"x": 0.5, "y": 0.5, "z": 0.5},
            motor_names=list(MOTOR_TO_RERUN.keys()),
            use_latched_reference=True,
        ),
        EEBoundsAndSafety(
            end_effector_bounds={"min": [-1.0, -1.0, -1.0], "max": [1.0, 1.0, 1.0]},
            max_ee_step_m=0.50,
        ),
        GripperToJoint(clip_min=-5.0, clip_max=95.0),
        InverseKinematicsEEToJoints(
            kinematics=kinematics_solver,
            motor_names=list(MOTOR_TO_RERUN.keys()),
            initial_guess_current_joints=True,
        ),
        WristJoints(
            motor_names=list(MOTOR_TO_RERUN.keys()),
        ),
    ],
    to_transition=robot_action_observation_to_transition,
    to_output=transition_to_robot_action,
)

teleop_device = AndroidPhone(config=PhoneConfig(phone_os=PhoneOS.ANDROID))


def callback_pose_android(message: Pose) -> None:
    # Data from ARCore is in RUB coordinate system
    position_rub = message["position"]
    orientation_rub = message["orientation"]
    position_rub = np.array([position_rub["x"], position_rub["y"], position_rub["z"]])
    orientation_rub_quaternion_wxyz = np.array(
        [
            orientation_rub["w"],
            orientation_rub["x"],
            orientation_rub["y"],
            orientation_rub["z"],
        ]
    )
    orientation_rub_matrix = t3d.quaternions.quat2mat(orientation_rub_quaternion_wxyz)

    # Transform data RUB to FLU coordinate system, to plot time series
    orientation_flu_matrix = (
        TF_RUB2FLU[:3, :3] @ orientation_rub_matrix @ TF_RUB2FLU[:3, :3].T
    )
    position_flu = TF_RUB2FLU[:3, :3] @ position_rub
    pose_flu = t3d.affines.compose(position_flu, orientation_flu_matrix, [1, 1, 1])

    # forward, left, up -> roll, pitch, yaw
    orientation_flu_euler = np.degrees(
        np.array(t3d.euler.mat2euler(pose_flu[:3, :3], axes="sxyz"))
    )
    orientation_flu_quaternion_wxyz = t3d.quaternions.mat2quat(pose_flu[:3, :3])
    orientation_flu_quaternion_xyzw = TF_WXYZ_TO_XYZW @ orientation_flu_quaternion_wxyz

    rr.log("/position", rr.Scalars(pose_flu[:3, 3]))
    rr.log("/orientation", rr.Scalars(orientation_flu_euler))
    rr.log(
        "/world_phone/trajectory_phone",
        rr.Points3D([pose_flu[:3, 3]]),
    )
    rr.log(
        "/world_phone/phone",
        rr.Transform3D(
            translation=pose_flu[:3, 3],
            rotation=rr.Quaternion(xyzw=orientation_flu_quaternion_xyzw),
        ),
    )


teleop_device.connect()
# Add an additional callback to the internal _teleop_server
teleop_device._teleop_server.subscribe_pose(callback_pose_android)


# Simulated robot state
robot_obs = {}
for motor, data in MOTOR_TO_RERUN.items():
    robot_obs[f"{motor}.pos"] = data["pos_init"]

step = 1
while True:
    t0 = time.perf_counter()

    if step % 3 == 0:
        for motor, data in MOTOR_TO_RERUN.items():
            degrees = robot_obs[f"{motor}.pos"]
            rotation = rr.RotationAxisAngle(axis=data["axis"], degrees=degrees)
            rr.log(
                data["path"],
                rr.Transform3D(rotation=rotation),
            )

    # Get teleop action
    phone_obs = teleop_device.get_action()

    # Phone -> EE pose -> Joints transition
    joint_action = phone_to_robot_joints_processor((phone_obs, robot_obs))

    # Update simulated robot state
    robot_obs = joint_action

    step += 1
    # NOTE: We use `time.sleep` instead of LeRobot's `busy_wait`, because the latter kills perf
    time.sleep(max(1.0 / FPS - (time.perf_counter() - t0), 0.0))
