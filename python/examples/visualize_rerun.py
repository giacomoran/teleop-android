# Example usage - this file requires the optional rerun dependency
# Install with: uv sync --extra rerun

# In ARCore world coordinates:
# - Y is aligned with gravity and points up. This remains fixed.
# - X and Z are chosen at session start, typically based on the phone's initial facing
#   so that Z roughly matches the initial forward direction and X the initial right.
# - After initialization, the world X/Y/Z axes are fixed in space; they do not rotate
#   with the phone. The phone's camera pose moves/rotates within this fixed frame.

import numpy as np
import rerun as rr
import transforms3d as t3d
from rerun import blueprint as rrb
from teleop_android import Pose, TeleopServer

#: Constants

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
    rrb.Spatial3DView(
        origin="/world",
        name="World position",
        time_ranges=rrb.VisibleTimeRanges(
            timeline="log_time",
            start=rrb.TimeRangeBoundary.cursor_relative(seconds=-60),
            end=rrb.TimeRangeBoundary.cursor_relative(seconds=0),
        ),
    ),
    column_shares=[0.45, 0.55],
)

rr.init("test_teleop", spawn=True, default_blueprint=blueprint)

# Set FLU coordinate system
rr.log("/", rr.ViewCoordinates.FLU, static=True)

# Create a pinhole camera with no images to aid visualizations
rr.log(
    "/world/phone",
    rr.Pinhole(
        focal_length=(500.0, 500.0),
        resolution=(640, 480),
        image_plane_distance=0.5,
        camera_xyz=rr.ViewCoordinates.FLU,
    ),
    static=True,
)


#: Pose Updates


def callback(message: Pose) -> None:
    # Data from ARCore is in RUB coordinate system
    position_rub = message["position"]
    orientation_rub = message["orientation"]
    position_rub = np.array([position_rub["x"], position_rub["y"], position_rub["z"]])
    orientation_rub_quaternion_xyzw = np.array(
        [
            orientation_rub["x"],
            orientation_rub["y"],
            orientation_rub["z"],
            orientation_rub["w"],
        ]
    )
    orientation_rub_quaternion_wxyz = TF_XYZW_TO_WXYZ @ orientation_rub_quaternion_xyzw
    orientation_rub_matrix = t3d.quaternions.quat2mat(orientation_rub_quaternion_wxyz)

    # Transform data RUB to FLU coordinate system, to plot time series
    tf_rub2flu_orientation = TF_RUB2FLU[:3, :3]
    orientation_flu_matrix = (
        tf_rub2flu_orientation @ orientation_rub_matrix @ tf_rub2flu_orientation.T
    )
    position_flu = tf_rub2flu_orientation @ position_rub
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
        "/world/trajectory_phone",
        rr.Points3D([pose_flu[:3, 3]]),
    )
    rr.log(
        "/world/phone",
        rr.Transform3D(
            translation=pose_flu[:3, 3],
            rotation=rr.Quaternion(xyzw=orientation_flu_quaternion_xyzw),
        ),
    )


teleop = TeleopServer()
teleop.subscribe_pose(callback)
teleop.run()
