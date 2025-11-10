from dataclasses import dataclass, field

import numpy as np
import transforms3d as t3d
from lerobot.configs.types import FeatureType, PipelineFeatureType, PolicyFeature
from lerobot.processor import (
    ProcessorStepRegistry,
    RobotAction,
    RobotActionProcessorStep,
    TransitionKey,
)
from lerobot.utils.rotation import Rotation

from .lerobot_utils import (
    TF_XYZW_TO_WXYZ,
)

#: MapPhoneActionToRobotAction


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
        inputs = action.pop("phone.raw_inputs")

        if pos is None or rot is None:
            raise ValueError("pos and rot must be present in action")

        rot_quaternion_xyzw = rot.as_quat()
        rot_quaternion_wxyz = TF_XYZW_TO_WXYZ @ rot_quaternion_xyzw
        delta_orientation_euler = np.degrees(
            np.array(t3d.euler.quat2euler(rot_quaternion_wxyz, axes="sxyz"))
        )
        rotvec_identity = Rotation.from_matrix(np.eye(3)).as_rotvec()

        delta_y_control_pad = float(inputs.get("delta_y_control_pad", 0.0))

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
        # Same as "enabled", store an additional copy for GripperToJoint
        action["gripper_enabled"] = enabled
        action["gripper_delta_y_control_pad"] = delta_y_control_pad if enabled else 0.0
        # Unused but required by other LeRobot processors
        action["gripper_vel"] = 0.0
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
            "wrist_enabled",
            "wrist_delta_degrees_flex",
            "wrist_delta_degrees_roll",
            "gripper_enabled",
            "gripper_delta_y_control_pad",
            "gripper_vel",
        ]:
            features[PipelineFeatureType.ACTION][f"{feat}"] = PolicyFeature(
                type=FeatureType.ACTION, shape=(1,)
            )

        return features


#: WristJoints


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


#: GripperToJoint


@ProcessorStepRegistry.register("gripper_to_joint")
@dataclass
class GripperToJoint(RobotActionProcessorStep):
    """
    TODO:
    """

    clip_min: float = 0.0
    clip_max: float = 100.0
    speed_factor: float = 50.0

    pos_gripper_reference: float | None = field(default=None, init=False, repr=False)
    _enabled_prev: bool = field(default=False, init=False, repr=False)
    _pos_gripper_disabled: float | None = field(default=None, init=False, repr=False)

    def action(self, action: RobotAction) -> RobotAction:
        observation = self.transition.get(TransitionKey.OBSERVATION).copy()

        enabled = bool(action.pop("gripper_enabled"))
        delta_y_control_pad = float(action.pop("gripper_delta_y_control_pad"))
        action.pop("ee.gripper_vel")  # Unused

        if observation is None:
            raise ValueError(
                "Joints observation is require for computing robot kinematics"
            )

        pos_gripper = float(observation["gripper.pos"])

        pos_gripper_desired = None

        if enabled:
            # Latched reference mode: latch reference at the rising edge
            if not self._enabled_prev or self.pos_gripper_reference is None:
                self.pos_gripper_reference = pos_gripper
            pos_gripper_reference = (
                self.pos_gripper_reference
                if self.pos_gripper_reference is not None
                else pos_gripper
            )

            # Clip the control pad delta to [-1, 1] for safety
            delta_y_control_pad = float(np.clip(delta_y_control_pad, -1.0, 1.0))
            # Multiply by speed_factor to convert control pad delta to gripper position delta
            delta_gripper_pos = delta_y_control_pad * self.speed_factor

            # Add delta to reference position
            pos_gripper_desired = pos_gripper_reference + delta_gripper_pos

            # Clip between clip_min and clip_max
            pos_gripper_desired = float(
                np.clip(pos_gripper_desired, self.clip_min, self.clip_max)
            )

            self._pos_gripper_disabled = pos_gripper_desired
        else:
            # While disabled, keep sending the same command to avoid drift.
            if self._pos_gripper_disabled is None:
                # If we've never had an enabled command yet, freeze current position once.
                self._pos_gripper_disabled = pos_gripper
            pos_gripper_desired = self._pos_gripper_disabled

        action["ee.gripper_pos"] = pos_gripper_desired

        self._enabled_prev = enabled
        return action

    def reset(self):
        """Resets the internal state of the processor."""
        self.pos_gripper_reference = None
        self._enabled_prev = False
        self._pos_gripper_disabled = None

    def transform_features(
        self, features: dict[PipelineFeatureType, dict[str, PolicyFeature]]
    ) -> dict[PipelineFeatureType, dict[str, PolicyFeature]]:
        for feat in [
            "gripper_enabled",
            "gripper_delta_y_control_pad",
            "ee.gripper_vel",
        ]:
            features[PipelineFeatureType.ACTION].pop(f"{feat}", None)

        features[PipelineFeatureType.ACTION]["ee.gripper_pos"] = PolicyFeature(
            type=FeatureType.ACTION, shape=(1,)
        )

        return features
