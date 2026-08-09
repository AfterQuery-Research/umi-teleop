"""Dual-YAM robot backend with strict calibration and idle semantics."""

from __future__ import annotations

import threading
import time
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Protocol

import numpy as np
from numpy.typing import NDArray

from umi_teleop_fixed.calibration import calibration_path, load_gripper_calibrations
from umi_teleop_fixed.can import (
    CANInterfaceInfo,
    discover_can_interfaces,
    readiness_error,
    resolve_can_interface,
)
from umi_teleop_fixed.i2rt_runtime import close_i2rt_robot, new_control_threads
from umi_teleop_fixed.se3 import PoseMatrix

SIDES = ("left", "right")
YAM_JOINT_LIMITS = (
    (-2.61799, 3.14159),
    (0.0, 3.66519),
    (0.0, 3.14159),
    (-1.69297, 1.5708),
    (-1.5708, 1.5708),
    (-2.0944, 2.0944),
)
READY_ARM_JOINTS = (0.0, 0.4, 0.8, 0.0, 0.0, 0.0)
DEFAULT_START_POSITION = (*READY_ARM_JOINTS, 1.0, *READY_ARM_JOINTS, 1.0)
ZERO_ARM_JOINTS = (0.0,) * 6
ZERO_POSITION = (*ZERO_ARM_JOINTS, 1.0, *ZERO_ARM_JOINTS, 1.0)
START_MOVE_STEP_SIZE = 0.01
START_MOVE_RATE_HZ = 30.0
START_MOVE_TOLERANCE = 0.035
START_MOVE_TIMEOUT_SECONDS = 30.0


@dataclass(frozen=True)
class ArmState:
    """One coherent robot-arm observation in command coordinates."""

    joint_positions: NDArray[np.float64]
    gripper_position: float
    ee_pose: PoseMatrix
    received_at: float


class Arm(Protocol):
    def read_state(self) -> ArmState: ...

    def solve_ik(
        self, target_pose: PoseMatrix, seed: NDArray[np.float64]
    ) -> NDArray[np.float64]: ...

    def command(self, joint_positions: NDArray[np.float64], gripper_position: float) -> None: ...

    def idle(self) -> None: ...

    def close(self) -> None: ...


class YamKinematics:
    """Six-arm-joint FK/IK at the physical LINEAR_4310 grasp site."""

    def __init__(self) -> None:
        try:
            import mujoco
            from i2rt.robots.get_robot import combine_arm_and_gripper_xml
            from i2rt.robots.kinematics import Kinematics
            from i2rt.robots.utils import ArmType, GripperType
        except ImportError as error:
            raise ImportError("The pinned i2rt dependency is required for robot control") from error

        model_path = combine_arm_and_gripper_xml(ArmType.YAM, GripperType.LINEAR_4310)
        self._kinematics = Kinematics(model_path, "grasp_site")
        model = mujoco.MjModel.from_xml_path(model_path)
        if model.nq != 8:
            raise RuntimeError(
                f"expected the YAM + LINEAR_4310 model to have 8 coordinates, got {model.nq}"
            )
        self._joint_limits = np.asarray(model.jnt_range[:6], dtype=np.float64).copy()

    @staticmethod
    def _model_configuration(joints: object) -> NDArray[np.float64]:
        """Append fixed finger coordinates required by the physical gripper model."""
        return np.concatenate((_validated_joints(joints), np.zeros(2, dtype=np.float64)))

    def fk(self, joints: NDArray[np.float64]) -> PoseMatrix:
        configuration = self._model_configuration(joints)
        return np.asarray(self._kinematics.fk(configuration), dtype=np.float64).copy()

    def ik(self, target: PoseMatrix, seed: NDArray[np.float64]) -> NDArray[np.float64]:
        initial = self._model_configuration(seed)
        success, solution = self._kinematics.ik(
            target,
            "grasp_site",
            init_q=initial,
            max_iters=20,
            pos_threshold=5e-4,
            ori_threshold=5e-4,
        )
        model_result = np.asarray(solution, dtype=np.float64)
        if model_result.shape != (8,) or not np.all(np.isfinite(model_result)):
            raise RuntimeError("YAM + LINEAR_4310 IK must return eight finite coordinates")
        result = _validated_joints(model_result[:6])
        if not success:
            raise RuntimeError("IK did not converge")
        lower, upper = self._joint_limits[:, 0], self._joint_limits[:, 1]
        if np.any(result < lower) or np.any(result > upper):
            raise RuntimeError("IK solution exceeds conservative YAM joint limits")
        return result


def _validated_joints(values: object) -> NDArray[np.float64]:
    joints = np.asarray(values, dtype=np.float64)
    if joints.shape != (6,) or not np.all(np.isfinite(joints)):
        raise RuntimeError("YAM arm state must contain six finite joint positions")
    return joints.copy()


def validate_dual_position(values: object) -> NDArray[np.float64]:
    """Validate six joints and one normalized gripper target for each arm."""
    position = np.asarray(values, dtype=np.float64)
    if position.shape != (14,) or not np.all(np.isfinite(position)):
        raise ValueError("start_position must contain 14 finite values")
    for offset, side in ((0, "left"), (7, "right")):
        for index, (lower, upper) in enumerate(YAM_JOINT_LIMITS):
            value = float(position[offset + index])
            if value < lower or value > upper:
                raise ValueError(
                    f"start_position {side} joint {index + 1} is outside [{lower}, {upper}]"
                )
        gripper = float(position[offset + 6])
        if gripper < 0.0 or gripper > 1.0:
            raise ValueError(f"start_position {side} gripper must be within [0, 1]")
    return position.copy()


def _position_targets(
    values: object,
) -> tuple[dict[str, NDArray[np.float64]], dict[str, float]]:
    position = validate_dual_position(values)
    joints = {"left": position[:6], "right": position[7:13]}
    grippers = {"left": float(position[6]), "right": float(position[13])}
    return joints, grippers


class I2RTArm:
    """One real YAM connected through the pinned i2rt dependency."""

    def __init__(
        self,
        robot: object,
        kinematics: YamKinematics,
        control_threads: tuple[threading.Thread, ...],
        *,
        monotonic: Callable[[], float] = time.monotonic,
    ) -> None:
        self._robot = robot
        self._kinematics = kinematics
        self._control_threads = control_threads
        self._monotonic = monotonic
        self._closed = False

    def read_state(self) -> ArmState:
        workers = (self._robot._server_thread, *self._control_threads)
        dead_workers = [thread.name for thread in workers if not thread.is_alive()]
        if not self._robot.motor_chain.running or dead_workers:
            detail = ", ".join(dead_workers) or "motor chain stopped"
            raise RuntimeError(f"i2rt robot worker is not healthy: {detail}")
        observations = self._robot.get_observations()
        joints = _validated_joints(observations["joint_pos"])
        gripper_values = np.asarray(observations["gripper_pos"], dtype=np.float64)
        if gripper_values.shape != (1,) or not np.all(np.isfinite(gripper_values)):
            raise RuntimeError("YAM gripper state must contain one finite position")
        return ArmState(
            joint_positions=joints,
            gripper_position=float(gripper_values[0]),
            ee_pose=self._kinematics.fk(joints),
            received_at=self._monotonic(),
        )

    def solve_ik(self, target_pose: PoseMatrix, seed: NDArray[np.float64]) -> NDArray[np.float64]:
        return self._kinematics.ik(target_pose, seed)

    def command(self, joint_positions: NDArray[np.float64], gripper_position: float) -> None:
        joints = _validated_joints(joint_positions)
        if not np.isfinite(gripper_position):
            raise RuntimeError("gripper command must be finite")
        self._robot.command_joint_pos(np.append(joints, gripper_position))

    def idle(self) -> None:
        self._robot.enter_gravity_comp_idle()

    def close(self) -> None:
        if not self._closed:
            self._closed = True
            close_i2rt_robot(self._robot, self._control_threads)


ArmFactory = Callable[[str, tuple[float, float]], Arm]


def create_i2rt_arm(channel: str, gripper_limits: tuple[float, float]) -> Arm:
    """Connect one real arm, capturing the i2rt threads needed for clean shutdown."""
    try:
        from i2rt.robots.get_robot import get_yam_robot
        from i2rt.robots.utils import ArmType, GripperType
    except ImportError as error:
        raise ImportError("The pinned i2rt dependency is required for robot control") from error

    existing_threads = set(threading.enumerate())
    robot = get_yam_robot(
        channel=channel,
        arm_type=ArmType.YAM,
        gripper_type=GripperType.LINEAR_4310,
        zero_gravity_mode=True,
        gripper_limits_override=np.asarray(gripper_limits),
        enable_auto_recovery=False,
    )
    control_threads = new_control_threads(existing_threads)
    if not control_threads:
        close_i2rt_robot(robot, ())
        raise RuntimeError("could not identify i2rt's CAN control worker")
    return I2RTArm(robot, YamKinematics(), control_threads)


class DualYamBackend:
    """Own two arms and keep command/idle transitions symmetric."""

    def __init__(self, arms: Mapping[str, Arm]) -> None:
        if set(arms) != set(SIDES):
            raise ValueError("dual backend requires exactly left and right arms")
        self._arms = dict(arms)
        self._closed = False

    @classmethod
    def open(
        cls,
        *,
        robot_id: str,
        left_adapter_serial: str,
        right_adapter_serial: str,
        calibration_root: Path | None = None,
        arm_factory: ArmFactory = create_i2rt_arm,
        interfaces: list[CANInterfaceInfo] | None = None,
    ) -> DualYamBackend:
        """Validate both links and calibrations before connecting either robot."""
        if left_adapter_serial.casefold() == right_adapter_serial.casefold():
            raise ValueError("left and right CAN adapter serials must be different")
        discovered = discover_can_interfaces() if interfaces is None else interfaces
        serials = {"left": left_adapter_serial, "right": right_adapter_serial}
        selected = {
            side: resolve_can_interface(serial, interfaces=discovered)
            for side, serial in serials.items()
        }
        for side in SIDES:
            error = readiness_error(selected[side], bitrate=1_000_000)
            if error is not None:
                raise RuntimeError(f"{side} CAN adapter {serials[side]} is not ready: {error}")

        path = calibration_path(robot_id, root=calibration_root)
        calibrations = load_gripper_calibrations(path, expected_robot_id=robot_id)
        missing = [side for side in SIDES if side not in calibrations]
        if missing:
            raise RuntimeError(f"missing gripper calibration for {', '.join(missing)} in {path}")
        for side in SIDES:
            if calibrations[side].adapter_serial.casefold() != serials[side].casefold():
                raise RuntimeError(
                    f"{side} gripper calibration belongs to CAN adapter "
                    f"{calibrations[side].adapter_serial}, not {serials[side]}"
                )

        arms: dict[str, Arm] = {}
        try:
            for side in SIDES:
                arms[side] = arm_factory(
                    selected[side].name,
                    calibrations[side].gripper_limits,
                )
        except BaseException:
            for arm in arms.values():
                arm.close()
            raise
        return cls(arms)

    def read_states(self) -> dict[str, ArmState]:
        return {side: self._arms[side].read_state() for side in SIDES}

    def solve_ik(
        self,
        targets: Mapping[str, PoseMatrix],
        seeds: Mapping[str, NDArray[np.float64]],
    ) -> dict[str, NDArray[np.float64]]:
        """Solve both sides before the caller is allowed to send either command."""
        return {
            side: self._arms[side].solve_ik(targets[side], seeds[side]) for side in SIDES
        }

    def command(
        self,
        joint_targets: Mapping[str, NDArray[np.float64]],
        gripper_targets: Mapping[str, float],
    ) -> None:
        for side in SIDES:
            self._arms[side].command(joint_targets[side], gripper_targets[side])

    def move_to_position(
        self,
        position: object,
        *,
        position_name: str = "start_position",
        step_size: float = START_MOVE_STEP_SIZE,
        rate_hz: float = START_MOVE_RATE_HZ,
        tolerance: float = START_MOVE_TOLERANCE,
        timeout_seconds: float = START_MOVE_TIMEOUT_SECONDS,
        monotonic: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
        emit: Callable[[str], None] = print,
    ) -> None:
        """Smoothly move both arms to one measured, validated joint position."""
        if not position_name.strip():
            raise ValueError("position_name must not be empty")
        for name, value in (
            ("step_size", step_size),
            ("rate_hz", rate_hz),
            ("tolerance", tolerance),
            ("timeout_seconds", timeout_seconds),
        ):
            if not np.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be positive")
        target_joints, target_grippers = _position_targets(position)
        try:
            states = self.read_states()
            start_joints = {side: states[side].joint_positions for side in SIDES}
            start_grippers = {side: states[side].gripper_position for side in SIDES}
            maximum_delta = max(
                *(
                    float(np.max(np.abs(target_joints[side] - start_joints[side])))
                    for side in SIDES
                ),
                *(abs(target_grippers[side] - start_grippers[side]) for side in SIDES),
            )
            steps = max(1, int(np.ceil(maximum_delta / step_size)))
            emit(
                f"Moving both arms to {position_name} in {steps} steps "
                f"at {rate_hz:.0f} Hz."
            )
            started = monotonic()
            next_step = started
            for step_index in range(1, steps + 1):
                if monotonic() - started >= timeout_seconds:
                    raise TimeoutError(f"timed out while moving both arms to {position_name}")
                alpha = step_index / steps
                waypoint_joints = {
                    side: start_joints[side]
                    + alpha * (target_joints[side] - start_joints[side])
                    for side in SIDES
                }
                waypoint_grippers = {
                    side: start_grippers[side]
                    + alpha * (target_grippers[side] - start_grippers[side])
                    for side in SIDES
                }
                self.command(waypoint_joints, waypoint_grippers)
                self.read_states()
                next_step += 1.0 / rate_hz
                sleep(max(0.0, next_step - monotonic()))

            while True:
                states = self.read_states()
                maximum_error = max(
                    *(
                        float(
                            np.max(
                                np.abs(target_joints[side] - states[side].joint_positions)
                            )
                        )
                        for side in SIDES
                    ),
                    *(
                        abs(target_grippers[side] - states[side].gripper_position)
                        for side in SIDES
                    ),
                )
                if maximum_error <= tolerance:
                    # Keep the exact target active while waiting for teleoperation.
                    self.command(target_joints, target_grippers)
                    emit(f"Reached {position_name} (maximum error {maximum_error:.4f}).")
                    return
                if monotonic() - started >= timeout_seconds:
                    raise TimeoutError(
                        f"both arms did not reach {position_name} within "
                        f"{timeout_seconds:.1f}s (maximum error {maximum_error:.4f})"
                    )
                self.command(target_joints, target_grippers)
                next_step += 1.0 / rate_hz
                sleep(max(0.0, next_step - monotonic()))
        except BaseException:
            self.idle()
            raise

    def idle(self) -> None:
        errors = []
        for side in SIDES:
            try:
                self._arms[side].idle()
            except BaseException as error:
                errors.append(f"{side}: {error}")
        if errors:
            raise RuntimeError("could not idle both arms: " + "; ".join(errors))

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        errors = []
        try:
            self.idle()
        except BaseException as error:
            errors.append(str(error))
        for side in SIDES:
            try:
                self._arms[side].close()
            except BaseException as error:
                errors.append(f"{side} close: {error}")
        if errors:
            raise RuntimeError("; ".join(errors))

    def __enter__(self) -> DualYamBackend:
        return self

    def __exit__(self, *_exc_info: object) -> None:
        self.close()
