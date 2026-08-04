"""Hardware-free inverse-kinematics fallback policy for teleoperation."""

from __future__ import annotations

import math
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from threading import Condition, Thread
from time import monotonic
from typing import Literal

import numpy as np

from umi_teleop.controller import checked_rotation_matrix, quat_to_matrix, rotation_angle

IkMode = Literal["normal", "degraded_ik", "holding_ik"]
SolveIk = Callable[[np.ndarray, np.ndarray], tuple[bool, np.ndarray]]
ValidateJoints = Callable[[np.ndarray], str | None]

DEFAULT_LINE_SEARCH_ALPHAS = (1.0, 0.75, 0.50, 0.25, 0.10)
DEFAULT_ORIENTATION_RELAXATIONS = (1.0, 0.50, 0.25, 0.0)


def checked_pose_matrix(pose: np.ndarray, name: str = "pose") -> np.ndarray:
    """Return a finite homogeneous transform with a proper rotation."""
    out = np.asarray(pose, dtype=np.float64)
    if out.shape != (4, 4) or not np.isfinite(out).all():
        raise ValueError(f"{name} must be a finite 4x4 matrix")
    if not np.allclose(out[3], [0.0, 0.0, 0.0, 1.0], atol=1e-9, rtol=0.0):
        raise ValueError(f"{name} must have a homogeneous final row")
    result = out.copy()
    result[:3, :3] = checked_rotation_matrix(result[:3, :3], f"{name} rotation")
    return result


def _matrix_to_quat_xyzw(rotation: np.ndarray) -> np.ndarray:
    """Convert a checked rotation matrix to a normalized XYZW quaternion."""
    matrix = checked_rotation_matrix(rotation)
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = 2.0 * math.sqrt(trace + 1.0)
        quat = np.array(
            [
                (matrix[2, 1] - matrix[1, 2]) / scale,
                (matrix[0, 2] - matrix[2, 0]) / scale,
                (matrix[1, 0] - matrix[0, 1]) / scale,
                0.25 * scale,
            ]
        )
    else:
        axis = int(np.argmax(np.diag(matrix)))
        if axis == 0:
            scale = 2.0 * math.sqrt(max(0.0, 1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]))
            quat = np.array(
                [
                    0.25 * scale,
                    (matrix[0, 1] + matrix[1, 0]) / scale,
                    (matrix[0, 2] + matrix[2, 0]) / scale,
                    (matrix[2, 1] - matrix[1, 2]) / scale,
                ]
            )
        elif axis == 1:
            scale = 2.0 * math.sqrt(max(0.0, 1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]))
            quat = np.array(
                [
                    (matrix[0, 1] + matrix[1, 0]) / scale,
                    0.25 * scale,
                    (matrix[1, 2] + matrix[2, 1]) / scale,
                    (matrix[0, 2] - matrix[2, 0]) / scale,
                ]
            )
        else:
            scale = 2.0 * math.sqrt(max(0.0, 1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]))
            quat = np.array(
                [
                    (matrix[0, 2] + matrix[2, 0]) / scale,
                    (matrix[1, 2] + matrix[2, 1]) / scale,
                    0.25 * scale,
                    (matrix[1, 0] - matrix[0, 1]) / scale,
                ]
            )
    norm = float(np.linalg.norm(quat))
    if norm < 1e-12:
        raise ValueError("rotation produced a zero quaternion")
    return quat / norm


def interpolate_rotation(start: np.ndarray, end: np.ndarray, alpha: float) -> np.ndarray:
    """Spherically interpolate two rotation matrices."""
    if not math.isfinite(alpha) or not 0.0 <= alpha <= 1.0:
        raise ValueError("rotation interpolation alpha must be between zero and one")
    start_quat = _matrix_to_quat_xyzw(start)
    end_quat = _matrix_to_quat_xyzw(end)
    dot = float(np.dot(start_quat, end_quat))
    if dot < 0.0:
        end_quat = -end_quat
        dot = -dot
    dot = float(np.clip(dot, -1.0, 1.0))
    if dot > 0.9995:
        quat = start_quat + alpha * (end_quat - start_quat)
        quat /= np.linalg.norm(quat)
    else:
        angle = math.acos(dot)
        sin_angle = math.sin(angle)
        quat = (
            math.sin((1.0 - alpha) * angle) / sin_angle * start_quat
            + math.sin(alpha * angle) / sin_angle * end_quat
        )
    return quat_to_matrix(quat)


def interpolate_pose(
    start: np.ndarray,
    end: np.ndarray,
    position_alpha: float,
    orientation_alpha: float | None = None,
) -> np.ndarray:
    """Interpolate translation and orientation independently between two poses."""
    start_pose = checked_pose_matrix(start, "start pose")
    end_pose = checked_pose_matrix(end, "end pose")
    if not math.isfinite(position_alpha) or not 0.0 <= position_alpha <= 1.0:
        raise ValueError("position interpolation alpha must be between zero and one")
    if orientation_alpha is None:
        orientation_alpha = position_alpha
    if not math.isfinite(orientation_alpha) or not 0.0 <= orientation_alpha <= 1.0:
        raise ValueError("orientation interpolation alpha must be between zero and one")
    result = np.eye(4, dtype=np.float64)
    result[:3, 3] = start_pose[:3, 3] + position_alpha * (end_pose[:3, 3] - start_pose[:3, 3])
    result[:3, :3] = interpolate_rotation(start_pose[:3, :3], end_pose[:3, :3], orientation_alpha)
    return result


@dataclass(frozen=True)
class IkFallbackResult:
    mode: IkMode
    joints: np.ndarray
    target_pose: np.ndarray
    reference_pose: np.ndarray
    position_alpha: float
    orientation_relaxation: float
    attempts: int
    full_target_failure: str | None = None
    final_failure: str | None = None

    @property
    def orientation_alpha(self) -> float:
        return self.position_alpha * self.orientation_relaxation


@dataclass(frozen=True)
class AsyncIkCompletion:
    """One completed IK request, including enough timing data to reject stale work."""

    request_id: int
    generation: int
    submitted_at: float
    completed_at: float
    outcome: IkFallbackResult


@dataclass(frozen=True)
class _AsyncIkRequest:
    request_id: int
    generation: int
    submitted_at: float
    desired_pose: np.ndarray
    last_good_pose: np.ndarray
    last_good_joints: np.ndarray


def _validated_fractions(
    values: Sequence[float],
    *,
    name: str,
    allow_zero: bool,
) -> tuple[float, ...]:
    result = tuple(float(value) for value in values)
    if not result or not all(math.isfinite(value) for value in result):
        raise ValueError(f"{name} must contain finite values")
    minimum = 0.0 if allow_zero else np.nextafter(0.0, 1.0)
    if any(value < minimum or value > 1.0 for value in result):
        qualifier = "zero and one" if allow_zero else "greater than zero and at most one"
        raise ValueError(f"{name} values must be between {qualifier}")
    if not math.isclose(result[0], 1.0):
        raise ValueError(f"{name} must start with 1.0")
    if any(result[index] <= result[index + 1] for index in range(len(result) - 1)):
        raise ValueError(f"{name} must be strictly decreasing")
    return result


class FallbackIkSolver:
    """Try exact IK, then a bounded stateful search toward the last safe pose.

    A failed IK iteration on the YAM Pi is expensive enough that evaluating the entire Cartesian
    search grid in one control tick would stall teleoperation. This class tries only a bounded
    number of fallback candidates per tick and carries its search position forward. Once a
    degraded candidate succeeds, it is remembered as the safe preference while the exact target
    continues to be retried every tick.
    """

    def __init__(
        self,
        solve: SolveIk,
        validate_joints: ValidateJoints,
        *,
        line_search_alphas: Sequence[float] = DEFAULT_LINE_SEARCH_ALPHAS,
        orientation_relaxations: Sequence[float] = DEFAULT_ORIENTATION_RELAXATIONS,
        fallback_attempts_per_cycle: int = 2,
    ):
        self.solve = solve
        self.validate_joints = validate_joints
        self.line_search_alphas = _validated_fractions(
            line_search_alphas, name="line-search alphas", allow_zero=False
        )
        self.orientation_relaxations = _validated_fractions(
            orientation_relaxations, name="orientation relaxations", allow_zero=True
        )
        if fallback_attempts_per_cycle < 1:
            raise ValueError("fallback_attempts_per_cycle must be at least one")
        self.fallback_attempts_per_cycle = int(fallback_attempts_per_cycle)
        self._candidates = tuple(
            (position_alpha, orientation_relaxation)
            for position_alpha in self.line_search_alphas
            for orientation_relaxation in self.orientation_relaxations
            if not (math.isclose(position_alpha, 1.0) and math.isclose(orientation_relaxation, 1.0))
        )
        if not self._candidates:
            raise ValueError("fallback grid must contain at least one non-exact candidate")
        self.reset()

    def reset(self) -> None:
        self._preferred_index: int | None = None
        self._search_index = 0

    def _attempt(
        self,
        pose: np.ndarray,
        initial_joints: np.ndarray,
    ) -> tuple[np.ndarray | None, str | None]:
        success, candidate_joints = self.solve(pose, initial_joints.copy())
        joints = np.asarray(candidate_joints, dtype=np.float64)
        if not success:
            return None, "IK solver did not converge"
        if joints.shape != initial_joints.shape or not np.isfinite(joints).all():
            return None, "IK solver returned invalid joint data"
        limit_reason = self.validate_joints(joints)
        if limit_reason is not None:
            return None, f"IK target {limit_reason}"
        return joints.copy(), None

    def _fallback_indices(self) -> list[int]:
        if self._preferred_index is None:
            start = self._search_index
            return [
                (start + offset) % len(self._candidates)
                for offset in range(min(self.fallback_attempts_per_cycle, len(self._candidates)))
            ]
        preferred = self._preferred_index
        if self.fallback_attempts_per_cycle == 1:
            return [preferred]
        indices = [max(0, preferred - 1), preferred]
        deduplicated: list[int] = []
        for index in indices:
            if index not in deduplicated:
                deduplicated.append(index)
        return deduplicated[: self.fallback_attempts_per_cycle]

    def solve_target(
        self,
        desired_pose: np.ndarray,
        last_good_pose: np.ndarray,
        last_good_joints: np.ndarray,
    ) -> IkFallbackResult:
        desired = checked_pose_matrix(desired_pose, "desired pose")
        reference = checked_pose_matrix(last_good_pose, "last-good pose")
        initial = np.asarray(last_good_joints, dtype=np.float64)
        if initial.ndim != 1 or not initial.size or not np.isfinite(initial).all():
            raise ValueError("last-good joints must be a finite nonempty vector")

        exact_joints, exact_failure = self._attempt(desired, initial)
        attempts = 1
        if exact_joints is not None:
            self.reset()
            return IkFallbackResult(
                mode="normal",
                joints=exact_joints,
                target_pose=desired,
                reference_pose=reference,
                position_alpha=1.0,
                orientation_relaxation=1.0,
                attempts=attempts,
            )

        indices = self._fallback_indices()
        final_failure = exact_failure
        for index in indices:
            position_alpha, orientation_relaxation = self._candidates[index]
            candidate_pose = interpolate_pose(
                reference,
                desired,
                position_alpha,
                position_alpha * orientation_relaxation,
            )
            candidate_joints, failure = self._attempt(candidate_pose, initial)
            attempts += 1
            final_failure = failure
            if candidate_joints is not None:
                self._preferred_index = index
                self._search_index = 0
                return IkFallbackResult(
                    mode="degraded_ik",
                    joints=candidate_joints,
                    target_pose=candidate_pose,
                    reference_pose=reference,
                    position_alpha=position_alpha,
                    orientation_relaxation=orientation_relaxation,
                    attempts=attempts,
                    full_target_failure=exact_failure,
                )

        self._preferred_index = None
        self._search_index = (indices[-1] + 1) % len(self._candidates)
        return IkFallbackResult(
            mode="holding_ik",
            joints=initial.copy(),
            target_pose=reference.copy(),
            reference_pose=reference,
            position_alpha=0.0,
            orientation_relaxation=0.0,
            attempts=attempts,
            full_target_failure=exact_failure,
            final_failure=final_failure,
        )


class AsyncFallbackIkSolver:
    """Run one arm's stateful fallback solver outside the real-time control loop.

    There is at most one pending request. If control produces targets faster than IK can solve
    them, the not-yet-started request is replaced by the newest target instead of building an
    unbounded stale-work queue. The worker owns ``FallbackIkSolver`` and its kinematics model.
    """

    def __init__(self, solver: FallbackIkSolver, *, name: str):
        self.solver = solver
        self.name = name
        self._condition = Condition()
        self._pending: _AsyncIkRequest | None = None
        self._latest: AsyncIkCompletion | None = None
        self._worker_error: BaseException | None = None
        self._next_request_id = 1
        self._last_delivered_id = 0
        self._closed = False
        self._thread = Thread(
            target=self._run,
            name=f"umi-ik-{name}",
            daemon=True,
        )
        self._thread.start()

    def submit(
        self,
        desired_pose: np.ndarray,
        last_good_pose: np.ndarray,
        last_good_joints: np.ndarray,
        *,
        generation: int,
        submitted_at: float | None = None,
    ) -> int:
        """Replace pending work with the newest target and return its request ID."""
        timestamp = monotonic() if submitted_at is None else float(submitted_at)
        request = _AsyncIkRequest(
            request_id=0,
            generation=int(generation),
            submitted_at=timestamp,
            desired_pose=np.asarray(desired_pose, dtype=np.float64).copy(),
            last_good_pose=np.asarray(last_good_pose, dtype=np.float64).copy(),
            last_good_joints=np.asarray(last_good_joints, dtype=np.float64).copy(),
        )
        with self._condition:
            self._raise_if_unavailable()
            request = _AsyncIkRequest(
                request_id=self._next_request_id,
                generation=request.generation,
                submitted_at=request.submitted_at,
                desired_pose=request.desired_pose,
                last_good_pose=request.last_good_pose,
                last_good_joints=request.last_good_joints,
            )
            self._next_request_id += 1
            self._pending = request
            self._condition.notify()
            return request.request_id

    def poll(self, *, generation: int) -> AsyncIkCompletion | None:
        """Return each completed result at most once, ignoring prior control generations."""
        with self._condition:
            self._raise_if_unavailable()
            completion = self._latest
            if completion is None or completion.request_id <= self._last_delivered_id:
                return None
            self._last_delivered_id = completion.request_id
            if completion.generation != generation:
                return None
            return completion

    def close(self, timeout_s: float = 0.0) -> bool:
        """Request worker shutdown; return whether it stopped within the optional timeout."""
        with self._condition:
            if not self._closed:
                self._closed = True
                self._pending = None
                self._condition.notify_all()
        if timeout_s > 0.0:
            self._thread.join(timeout=timeout_s)
        return not self._thread.is_alive()

    def _raise_if_unavailable(self) -> None:
        if self._worker_error is not None:
            raise RuntimeError(
                f"{self.name} IK worker failed: {self._worker_error}"
            ) from self._worker_error
        if self._closed:
            raise RuntimeError(f"{self.name} IK worker is closed")

    def _run(self) -> None:
        active_generation: int | None = None
        try:
            while True:
                with self._condition:
                    self._condition.wait_for(lambda: self._closed or self._pending is not None)
                    if self._closed:
                        return
                    request = self._pending
                    self._pending = None
                assert request is not None
                if request.generation != active_generation:
                    self.solver.reset()
                    active_generation = request.generation
                outcome = self.solver.solve_target(
                    request.desired_pose,
                    request.last_good_pose,
                    request.last_good_joints,
                )
                completion = AsyncIkCompletion(
                    request_id=request.request_id,
                    generation=request.generation,
                    submitted_at=request.submitted_at,
                    completed_at=monotonic(),
                    outcome=outcome,
                )
                with self._condition:
                    self._latest = completion
        except BaseException as exc:
            with self._condition:
                self._worker_error = exc
                self._condition.notify_all()


def fallback_diagnostic(side: str, result: IkFallbackResult, desired_pose: np.ndarray) -> str:
    """Format an actionable per-arm fallback diagnostic."""
    if result.mode == "normal":
        return f"{side}=normal"
    desired = checked_pose_matrix(desired_pose, "desired pose")
    desired_mm = 1000.0 * desired[:3, 3]
    reference_mm = 1000.0 * result.reference_pose[:3, 3]
    desired_rotation = math.degrees(
        rotation_angle(desired[:3, :3] @ result.reference_pose[:3, :3].T)
    )
    prefix = (
        f"{side}={result.mode} alpha={result.position_alpha:.2f} "
        f"orientation_relax={result.orientation_relaxation:.2f} attempts={result.attempts} "
        f"desired_xyz_mm=[{desired_mm[0]:+.1f},{desired_mm[1]:+.1f},{desired_mm[2]:+.1f}] "
        f"last_good_xyz_mm=[{reference_mm[0]:+.1f},{reference_mm[1]:+.1f},"
        f"{reference_mm[2]:+.1f}] desired_from_last_rot={desired_rotation:.1f}deg"
    )
    if result.mode == "degraded_ik":
        return f"{prefix} full_failure={result.full_target_failure}"
    return (
        f"{prefix} full_failure={result.full_target_failure} "
        f"fallback_failure={result.final_failure}"
    )
