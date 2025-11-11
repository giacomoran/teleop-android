# REFS: https://github.com/huggingface/lerobot/blob/main/src/lerobot/teleoperators/phone/teleop_phone.py

import copy
import logging
import math
import threading
import time
from typing import Optional

import numpy as np
import transforms3d as t3d
from lerobot.teleoperators.phone.teleop_phone import BasePhone, PhoneConfig
from lerobot.teleoperators.teleoperator import Teleoperator
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from lerobot.utils.rotation import Rotation

from .lerobot_utils import (
    TF_RUB2FLU,
    TF_WXYZ_TO_XYZW,
    are_close,
    interpolate_transforms,
)
from .server import Control, Pose, TeleopServer

logger = logging.getLogger(__name__)

#:


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

        # Store initial phone pose when user starts touching (used as reference for relative movement)
        # Reset to None when user stops touching or when pose jump is detected
        self._pose_phone_init = None
        self._pose_phone_prev = None
        # Store initial control pad Y position to calculate relative gripper movement
        # Reset to None when user stops touching or when pose jump is detected
        self._y_control_pad_init: Optional[float] = None

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

        control_pad_y = float(control.get("y", 0.0))

        # Transform RUB (used by ARCore) to FLU (used by LeRobot) coordinate system
        orientation_matrix = (
            TF_RUB2FLU[:3, :3] @ orientation_rub_matrix @ TF_RUB2FLU[:3, :3].T
        )
        position = TF_RUB2FLU[:3, :3] @ position_rub

        # Create 4x4 pose matrix, combining position, orientation, and scale
        pose_phone = t3d.affines.compose(position, orientation_matrix, [1, 1, 1])

        ##: Handle edge cases

        # Begin "enabled" phone movement
        if not self._enabled_prev and self._enabled:
            assert self._pose_phone_init is None and self._pose_phone_prev is None
            assert self._y_control_pad_init is None

        # Stop "enabled" phone movement
        if self._enabled_prev and not self._enabled:
            self._pose_phone_init = None
            self._pose_phone_prev = None
            self._y_control_pad_init = None
            # Note that self._pose_robot is retained, we need to keep track of it across
            # disjoint phone movements
            return RESULT_NOT_ENABLED

        # Already not "enabled"
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
                self._y_control_pad_init = None
                # Note that self._pose_robot is retained, we need to keep track of it across
                # disjoint phone movements
                return RESULT_NOT_ENABLED
        self._pose_phone_prev = pose_phone

        # We get here:
        # - right after jumps
        # - right after the user enables movement
        if self._pose_phone_init is None:
            self._pose_phone_init = pose_phone

        # Latch control pad y reference when enabled starts
        if self._y_control_pad_init is None:
            self._y_control_pad_init = float(control.get("y", 0.0))

        ##: Compute deltas

        if scale < 1.0:
            pose_phone = interpolate_transforms(
                self._pose_phone_init, pose_phone, scale
            )

        delta_position = pose_phone[:3, 3] - self._pose_phone_init[:3, 3]
        delta_orientation = pose_phone[:3, :3] @ self._pose_phone_init[:3, :3].T

        delta_y_control_pad = control_pad_y - self._y_control_pad_init

        ##: Convert to LeRobot data
        # See `lerobot_processors.py` for how this data is used. We tried as much as possible
        # to stick to LeRobot's original phone teleop implementation.

        rot_quaternion_wxyz = t3d.quaternions.mat2quat(delta_orientation)
        rot_quaternion_xyzw = TF_WXYZ_TO_XYZW @ rot_quaternion_wxyz
        rot = Rotation.from_quat(rot_quaternion_xyzw)
        pos = delta_position

        raw_inputs = control.copy()
        raw_inputs["delta_y_control_pad"] = delta_y_control_pad

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
