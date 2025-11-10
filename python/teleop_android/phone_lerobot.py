# The code here has been modified from LeRobot phone teleoperation implementation,
# the goal is to make the Android app compatible with LeRoboot.
# REFS: https://github.com/huggingface/lerobot/tree/main/src/lerobot/teleoperators/phone

import copy
import logging
import math
import threading
import time
from dataclasses import dataclass, field
from typing import Optional

import numpy as np
import transforms3d as t3d
from lerobot.configs.types import FeatureType, PipelineFeatureType, PolicyFeature
from lerobot.processor import (
    ProcessorStepRegistry,
    RobotAction,
    RobotActionProcessorStep,
    TransitionKey,
)
from lerobot.teleoperators.phone.teleop_phone import BasePhone, PhoneConfig
from lerobot.teleoperators.teleoperator import Teleoperator
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from lerobot.utils.rotation import Rotation

from .server import Control, Pose, TeleopServer

logger = logging.getLogger(__name__)

#: Utils

TF_RUB2FLU = np.array([[0, 0, -1, 0], [-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 1]])
TF_XYZW_TO_WXYZ = np.array([[0, 0, 0, 1], [1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0]])
TF_WXYZ_TO_XYZW = np.array([[0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1], [1, 0, 0, 0]])


def are_close(a, b=None, lin_tol=1e-9, ang_tol=1e-9):
    """
    Check if two transformation matrices are close to each other within specified tolerances.

    REFS: https://github.com/SpesRobotics/teleop/blob/main/teleop/__init__.py

    Parameters:
        a (numpy.ndarray): The first transformation matrix.
        b (numpy.ndarray, optional): The second transformation matrix. If not provided, it defaults to the identity matrix.
        lin_tol (float, optional): The linear tolerance for closeness. Defaults to 1e-9.
        ang_tol (float, optional): The angular tolerance for closeness. Defaults to 1e-9.

    Returns:
        bool: True if the matrices are close, False otherwise.
    """
    if b is None:
        b = np.eye(4)
    d = np.linalg.inv(a) @ b
    if not np.allclose(d[:3, 3], np.zeros(3), atol=lin_tol):
        return False
    yaw = math.atan2(d[1, 0], d[0, 0])
    pitch = math.asin(-d[2, 0])
    roll = math.atan2(d[2, 1], d[2, 2])
    rpy = np.array([roll, pitch, yaw])
    return np.allclose(rpy, np.zeros(3), atol=ang_tol)


def slerp(q1, q2, t):
    """
    Spherical linear interpolation between two quaternions.

    REFS: https://github.com/SpesRobotics/teleop/blob/main/teleop/__init__.py
    """
    q1 = q1 / np.linalg.norm(q1)
    q2 = q2 / np.linalg.norm(q2)

    dot = np.dot(q1, q2)

    # If the dot product is negative, use the shortest path
    if dot < 0.0:
        q2 = -q2
        dot = -dot

    DOT_THRESHOLD = 0.9995
    if dot > DOT_THRESHOLD:
        # Linear interpolation fallback for nearly identical quaternions
        result = q1 + t * (q2 - q1)
        return result / np.linalg.norm(result)

    theta_0 = np.arccos(dot)
    theta = theta_0 * t

    q3 = q2 - q1 * dot
    q3 = q3 / np.linalg.norm(q3)

    return q1 * np.cos(theta) + q3 * np.sin(theta)


def interpolate_transforms(T1, T2, alpha):
    """
    Interpolate between two 4x4 transformation matrices using SLERP + linear translation.

    REFS: https://github.com/SpesRobotics/teleop/blob/main/teleop/__init__.py

    Args:
        T1 (np.ndarray): Start transform (4x4)
        T2 (np.ndarray): End transform (4x4)
        alpha (float): Interpolation factor [0, 1]

    Returns:
        np.ndarray: Interpolated transform (4x4)
    """
    assert T1.shape == (4, 4) and T2.shape == (4, 4)
    assert 0.0 <= alpha <= 1.0

    # Translation
    t1 = T1[:3, 3]
    t2 = T2[:3, 3]
    t_interp = (1 - alpha) * t1 + alpha * t2

    # Rotation
    R1 = T1[:3, :3]
    R2 = T2[:3, :3]
    q1 = t3d.quaternions.mat2quat(R1)
    q2 = t3d.quaternions.mat2quat(R2)

    # SLERP
    q_interp = slerp(q1, q2, alpha)
    R_interp = t3d.quaternions.quat2mat(q_interp)

    # Final transform
    T_interp = np.eye(4)
    T_interp[:3, :3] = R_interp
    T_interp[:3, 3] = t_interp

    return T_interp


#: Processors


@ProcessorStepRegistry.register("map_phone_action_to_robot_action")
@dataclass
class MapPhoneActionToRobotAction(RobotActionProcessorStep):
    """
    Maps calibrated phone pose actions to standardized robot action inputs.

    This processor step acts as a bridge between the phone teleoperator's output
    and the robot's expected action format. It remaps the phone's 6-DoF pose
    (position and rotation) to the robot's target end-effector pose, applying
    necessary axis inversions and swaps. It also interprets platform-specific
    button presses to generate a gripper command.
    """

    _enabled_prev: bool = field(default=False, init=False, repr=False)

    def action(self, action: RobotAction) -> RobotAction:
        """
        Processes the phone action dictionary to create a robot action dictionary.

        Args:
            act: The input action dictionary from the phone teleoperator.

        Returns:
            A new action dictionary formatted for the robot controller.

        Raises:
            ValueError: If 'pos' or 'rot' keys are missing from the input action.
        """
        # Pop them from the action
        enabled = bool(action.pop("phone.enabled"))
        # Position delta for the phone
        pos = action.pop("phone.pos")
        # Orientation delta for the phone
        rot = action.pop("phone.rot")
        inputs = action.pop("phone.raw_inputs")  # noqa: F841 TODO:

        if pos is None or rot is None:
            raise ValueError("pos and rot must be present in action")

        rot_quaternion_xyzw = rot.as_quat()
        rot_quaternion_wxyz = TF_XYZW_TO_WXYZ @ rot_quaternion_xyzw
        delta_orientation_euler = np.degrees(
            np.array(t3d.euler.quat2euler(rot_quaternion_wxyz, axes="sxyz"))
        )
        rotvec_identity = Rotation.from_matrix(np.eye(3)).as_rotvec()

        # TODO:
        # # Map certain inputs to certain actions
        # if self.platform == PhoneOS.IOS:
        #     gripper_vel = float(inputs.get("a3", 0.0))
        # else:
        #     a = float(inputs.get("reservedButtonA", 0.0))
        #     b = float(inputs.get("reservedButtonB", 0.0))
        #     gripper_vel = (
        #         a - b
        #     )  # Positive if a is pressed, negative if b is pressed, 0 if both or neither are pressed

        action["enabled"] = enabled
        # "target_{x,y,z}" represent a position delta (see EEReferenceAndDelta implementation)
        action["target_x"] = pos[0] if enabled else 0.0
        action["target_y"] = pos[1] if enabled else 0.0
        action["target_z"] = pos[2] if enabled else 0.0
        # "target_{wx,wy,wz}" represent a rotation delta, as rotvec
        # These are used by downstream LeRobot processor steps, but we expect the end
        # effector to be the lower arm, of which we don't want to control the orientation.
        action["target_wx"] = rotvec_identity[0]
        action["target_wy"] = rotvec_identity[1]
        action["target_wz"] = rotvec_identity[2]
        # Same as "enabled", the EEReferenceAndDelta step pops "enabled" so we store
        # an additional copy that is popped by WristJoints.
        action["wrist_enabled"] = enabled
        action["wrist_delta_degrees_flex"] = delta_orientation_euler[1]
        action["wrist_delta_degrees_roll"] = delta_orientation_euler[0]
        # TODO:
        action["gripper_vel"] = 0.0  # Still send gripper action when disabled
        return action

    def transform_features(
        self, features: dict[PipelineFeatureType, dict[str, PolicyFeature]]
    ) -> dict[PipelineFeatureType, dict[str, PolicyFeature]]:
        for feat in ["enabled", "pos", "rot", "raw_inputs"]:
            features[PipelineFeatureType.ACTION].pop(f"phone.{feat}", None)

        for feat in [
            "enabled",
            "target_x",
            "target_y",
            "target_z",
            "target_wx",
            "target_wy",
            "target_wz",
            "wrist_enabledwrist_delta_degrees_flex",
            "wrist_delta_degrees_roll",
            "gripper_vel",
        ]:
            features[PipelineFeatureType.ACTION][f"{feat}"] = PolicyFeature(
                type=FeatureType.ACTION, shape=(1,)
            )

        return features


@ProcessorStepRegistry.register("wrist_joints")
@dataclass
class WristJoints(RobotActionProcessorStep):
    """
    TODO:
    """

    motor_names: list[str]

    pos_wrist_reference: np.ndarray | None = field(default=None, init=False, repr=False)
    _enabled_prev: bool = field(default=False, init=False, repr=False)
    _pos_wrist_disabled: np.ndarray | None = field(default=None, init=False, repr=False)

    def action(self, action: RobotAction) -> RobotAction:
        observation = self.transition.get(TransitionKey.OBSERVATION).copy()

        if observation is None:
            raise ValueError(
                "Joints observation is require for computing wrist position"
            )

        assert "wrist_flex" in self.motor_names and "wrist_roll" in self.motor_names
        pos_wrist_obs = np.array(
            [
                float(observation["wrist_flex.pos"]),
                float(observation["wrist_roll.pos"]),
            ],
            dtype=float,
        )

        enabled = bool(action.pop("wrist_enabled"))
        delta_degrees_flex = float(action.pop("wrist_delta_degrees_flex"))
        delta_degrees_roll = float(action.pop("wrist_delta_degrees_roll"))

        pos_wrist_desired = None

        if enabled:
            # Latched reference mode: latch reference at the rising edge
            if not self._enabled_prev or self.pos_wrist_reference is None:
                self.pos_wrist_reference = pos_wrist_obs
            pos_wrist_reference = (
                self.pos_wrist_reference
                if self.pos_wrist_reference is not None
                else pos_wrist_obs
            )

            pos_wrist_desired = pos_wrist_reference + np.array(
                [delta_degrees_flex, delta_degrees_roll]
            )

            self._pos_wrist_disabled = pos_wrist_desired.copy()
        else:
            # While disabled, keep sending the same command to avoid drift.
            if self._pos_wrist_disabled is None:
                # If we've never had an enabled command yet, freeze current FK pose once.
                self._pos_wrist_disabled = pos_wrist_obs.copy()
            pos_wrist_desired = self._pos_wrist_disabled.copy()

        action["wrist_flex.pos"] = float(pos_wrist_desired[0])
        action["wrist_roll.pos"] = float(pos_wrist_desired[1])

        self._enabled_prev = enabled
        return action

    def reset(self):
        """Resets the internal state of the processor."""
        self.pos_wrist_reference = None
        self._enabled_prev = False
        self._pos_wrist_disabled = None

    def transform_features(
        self, features: dict[PipelineFeatureType, dict[str, PolicyFeature]]
    ) -> dict[PipelineFeatureType, dict[str, PolicyFeature]]:
        for feat in [
            "wrist_enabled",
            "wrist_delta_degrees_flex",
            "wrist_delta_degrees_roll",
        ]:
            features[PipelineFeatureType.ACTION].pop(f"{feat}", None)

        for feat in ["wrist_flex.pos", "wrist_roll.pos"]:
            features[PipelineFeatureType.ACTION][f"{feat}"] = PolicyFeature(
                type=FeatureType.ACTION, shape=(1,)
            )

        return features


#: Phone


# TODO: Review comments and log messages
class AndroidPhone(BasePhone, Teleoperator):
    name = "android_phone"

    def __init__(self, config: PhoneConfig):
        super().__init__(config)
        self.config = config

        self._teleop_server = None

        self._thread_android = None
        self._lock_android = threading.Lock()
        # Pose and control updated by the Android callback, lock `self._lock_android` to read them
        self._pose_android: Optional[Pose] = None
        self._control_android: Optional[Control] = None

        self._pose_phone_init = None
        self._pose_phone_prev = None

    @property
    def is_connected(self) -> bool:
        return self._teleop_server is not None

    def connect(self) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        logger.info("Starting teleop stream for Android...")
        self._teleop_server = TeleopServer()
        self._teleop_server.subscribe_pose(self._callback_pose_android)
        self._teleop_server.subscribe_control(self._callback_control_android)
        self._thread_android = threading.Thread(
            target=self._teleop_server.run, daemon=True
        )
        self._thread_android.start()
        logger.info(f"{self} connected, teleop stream started.")

        self._enabled = False

    def _callback_pose_android(self, pose: Pose) -> None:
        with self._lock_android:
            self._pose_android = pose

    def _callback_control_android(self, control: Control) -> None:
        with self._lock_android:
            self._control_android = control

    def disconnect(self) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        self._teleop_server = None
        if self._thread_android and self._thread_android.is_alive():
            self._thread_android.join(timeout=1.0)
            self._thread_android = None
            self._pose_android = None
            self._control_android = None

    # TODO: Document that LeRobot expects FLU coordinate system
    def get_action(self) -> dict:
        RESULT_NOT_ENABLED = {
            "phone.enabled": False,
            "phone.pos": np.array([0, 0, 0]),
            "phone.rot": Rotation.from_matrix(np.eye(3)),
            "phone.raw_inputs": {},
        }

        ##: Read latest pose and control data received from the Android phone

        with self._lock_android:
            pose = copy.deepcopy(self._pose_android)
            control = copy.deepcopy(self._control_android)

        if control is None or pose is None:
            return RESULT_NOT_ENABLED

        ##: Parse data from the Android phone

        self._enabled_prev = self._enabled
        self._enabled = bool(control["isActive"])
        scale = 0.5 if bool(control["isFineControl"]) else 1.0
        # TODO: Gripper vel

        position_rub = np.array(
            [
                pose["position"]["x"],
                pose["position"]["y"],
                pose["position"]["z"],
            ]
        )
        orientation_rub_quaternion_wxyz = np.array(
            [
                pose["orientation"]["w"],
                pose["orientation"]["x"],
                pose["orientation"]["y"],
                pose["orientation"]["z"],
            ]
        )

        orientation_rub_matrix = t3d.quaternions.quat2mat(
            orientation_rub_quaternion_wxyz
        )

        # Transform data RUB to FLU coordinate system
        orientation_matrix = (
            TF_RUB2FLU[:3, :3] @ orientation_rub_matrix @ TF_RUB2FLU[:3, :3].T
        )
        position = TF_RUB2FLU[:3, :3] @ position_rub

        pose_phone = t3d.affines.compose(position, orientation_matrix, [1, 1, 1])

        ##: Handle edge cases

        # Begin "enabled" phone movement
        if not self._enabled_prev and self._enabled:
            assert self._pose_phone_init is None and self._pose_phone_prev is None

        # Stop "enabled" phone movement
        if self._enabled_prev and not self._enabled:
            self._pose_phone_init = None
            self._pose_phone_prev = None
            # Note that self._pose_robot is retained, we need to keep track of it across
            # disjoint phone movements
            return RESULT_NOT_ENABLED

        if not self._enabled_prev and not self._enabled:
            return RESULT_NOT_ENABLED

        # Pose jump protection
        if self._pose_phone_prev is not None:
            if not are_close(
                pose_phone,
                self._pose_phone_prev,
                lin_tol=0.05,
                ang_tol=math.radians(35),
            ):
                logger.warning("Pose jump detected, resetting the pose")
                self._pose_phone_init = None
                self._pose_phone_prev = pose_phone
                # Note that self._pose_robot is retained, we need to keep track of it across
                # disjoint phone movements
                return RESULT_NOT_ENABLED
        self._pose_phone_prev = pose_phone

        # We get here:
        # - right after jumps
        # - right after the user enables movement
        if self._pose_phone_init is None:
            self._pose_phone_init = pose_phone

        ##: Compute robot phone

        delta_position = pose_phone[:3, 3] - self._pose_phone_init[:3, 3]
        delta_orientation = pose_phone[:3, :3] @ self._pose_phone_init[:3, :3].T

        if scale < 1.0:
            self._pose_robot = interpolate_transforms(
                self._pose_robot_init, self._pose_robot, scale
            )

        ##: Convert to LeRobot data

        rot_quaternion_wxyz = t3d.quaternions.mat2quat(delta_orientation)
        rot_quaternion_xyzw = TF_WXYZ_TO_XYZW @ rot_quaternion_wxyz
        rot = Rotation.from_quat(rot_quaternion_xyzw)
        pos = delta_position

        raw_inputs = control

        assert self._enabled
        return {
            "phone.enabled": self._enabled,
            "phone.pos": pos,
            "phone.rot": rot,
            "phone.raw_inputs": raw_inputs,
        }

    def calibrate(self) -> None:
        print(
            "Hold the phone so that: top edge points forward in same direction as the robot (robot +x) and screen points up (robot +z)"
        )
        print("Hold the control pad and start moving...\n")

        while True:
            with self._lock_android:
                control = copy.deepcopy(self._control_android)
            if control and bool(control["isActive"]):
                break
            time.sleep(0.01)

        print("Calibration done\n")

    @property
    def is_calibrated(self) -> bool:
        return (self._pose_phone_init is not None) and (
            self._pose_robot_init is not None
        )
