from __future__ import annotations

import math
import time
from threading import Event

import numpy as np
import pytest

from umi_teleop.controller import rotation_angle
from umi_teleop.ik_policy import (
    AsyncFallbackIkSolver,
    FallbackIkSolver,
    fallback_diagnostic,
    interpolate_pose,
    interpolate_rotation,
)


def z_rotation(degrees: float) -> np.ndarray:
    angle = math.radians(degrees)
    cosine, sine = math.cos(angle), math.sin(angle)
    return np.array(
        [[cosine, -sine, 0.0], [sine, cosine, 0.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )


def pose(x: float = 0.0, rotation_degrees: float = 0.0) -> np.ndarray:
    result = np.eye(4, dtype=np.float64)
    result[0, 3] = x
    result[:3, :3] = z_rotation(rotation_degrees)
    return result


def test_pose_interpolation_separates_position_and_orientation():
    result = interpolate_pose(pose(), pose(1.0, 90.0), 0.75, 0.25)
    np.testing.assert_allclose(result[:3, 3], [0.75, 0.0, 0.0])
    assert math.degrees(rotation_angle(result[:3, :3])) == pytest.approx(22.5)

    halfway = interpolate_rotation(z_rotation(170.0), z_rotation(-170.0), 0.5)
    assert math.degrees(rotation_angle(halfway)) == pytest.approx(180.0)


def test_exact_solution_is_normal_and_resets_search():
    calls = []

    def solve(target, initial):
        calls.append(target.copy())
        return True, initial + 0.1

    solver = FallbackIkSolver(solve, lambda _joints: None)
    result = solver.solve_target(pose(0.2), pose(), np.zeros(6))

    assert result.mode == "normal"
    assert result.attempts == 1
    assert result.position_alpha == 1.0
    assert result.orientation_relaxation == 1.0
    np.testing.assert_allclose(result.joints, 0.1)
    assert len(calls) == 1


def test_line_search_chooses_largest_reachable_position_candidate():
    def solve(target, initial):
        return target[0, 3] <= 0.5 + 1e-9, initial + target[0, 3]

    solver = FallbackIkSolver(
        solve,
        lambda _joints: None,
        line_search_alphas=(1.0, 0.5, 0.1),
        orientation_relaxations=(1.0, 0.0),
        fallback_attempts_per_cycle=2,
    )
    result = solver.solve_target(pose(1.0), pose(), np.zeros(6))

    assert result.mode == "degraded_ik"
    assert result.position_alpha == 0.5
    assert result.orientation_relaxation == 1.0
    assert result.attempts == 3
    np.testing.assert_allclose(result.target_pose[:3, 3], [0.5, 0.0, 0.0])


def test_orientation_relaxes_at_full_position_before_translation():
    def solve(target, initial):
        angle = math.degrees(rotation_angle(target[:3, :3]))
        success = target[0, 3] == pytest.approx(1.0) and angle <= 45.0 + 1e-6
        return bool(success), initial + angle / 100.0

    solver = FallbackIkSolver(
        solve,
        lambda _joints: None,
        line_search_alphas=(1.0, 0.5),
        orientation_relaxations=(1.0, 0.5, 0.0),
        fallback_attempts_per_cycle=2,
    )
    result = solver.solve_target(pose(1.0, 90.0), pose(), np.zeros(6))

    assert result.mode == "degraded_ik"
    assert result.position_alpha == 1.0
    assert result.orientation_relaxation == 0.5
    assert math.degrees(rotation_angle(result.target_pose[:3, :3])) == pytest.approx(45.0)


def test_joint_limit_rejection_participates_in_fallback_search():
    def solve(target, _initial):
        return True, np.full(6, target[0, 3])

    def validate(joints):
        return "joint 1 exceeds margin" if joints[0] > 0.5 else None

    solver = FallbackIkSolver(
        solve,
        validate,
        line_search_alphas=(1.0, 0.5),
        orientation_relaxations=(1.0, 0.0),
        fallback_attempts_per_cycle=2,
    )
    result = solver.solve_target(pose(1.0), pose(), np.zeros(6))

    assert result.mode == "degraded_ik"
    assert result.position_alpha == 0.5
    assert "joint 1 exceeds margin" in result.full_target_failure


def test_unreachable_target_holds_last_safe_command_and_continues_search():
    attempted_x = []

    def solve(target, initial):
        attempted_x.append(float(target[0, 3]))
        return False, initial + 99.0

    initial = np.arange(6, dtype=np.float64)
    solver = FallbackIkSolver(
        solve,
        lambda _joints: None,
        line_search_alphas=(1.0, 0.5, 0.1),
        orientation_relaxations=(1.0, 0.0),
        fallback_attempts_per_cycle=1,
    )
    first = solver.solve_target(pose(1.0), pose(), initial)
    second = solver.solve_target(pose(1.0), pose(), initial)

    assert first.mode == second.mode == "holding_ik"
    assert not first.search_exhausted
    np.testing.assert_array_equal(first.joints, initial)
    np.testing.assert_array_equal(first.target_pose, np.eye(4))
    # Exact is retried every cycle; the fallback cursor moves from alpha=1 to alpha=0.5.
    assert attempted_x == pytest.approx([1.0, 1.0, 1.0, 0.5])


def test_unreachable_target_reports_only_after_complete_fallback_sweep():
    solver = FallbackIkSolver(
        lambda _target, initial: (False, initial),
        lambda _joints: None,
        line_search_alphas=(1.0, 0.5, 0.1),
        orientation_relaxations=(1.0, 0.0),
        fallback_attempts_per_cycle=2,
    )

    first = solver.solve_target(pose(1.0), pose(), np.zeros(6))
    second = solver.solve_target(pose(1.0), pose(), np.zeros(6))
    third = solver.solve_target(pose(1.0), pose(), np.zeros(6))

    assert first.mode == second.mode == third.mode == "holding_ik"
    assert not first.search_exhausted and not second.search_exhausted
    assert third.search_exhausted


def test_fallback_sweep_freezes_one_target_while_exact_retry_uses_latest():
    attempted_x: list[float] = []

    def solve(target, initial):
        attempted_x.append(float(target[0, 3]))
        return False, initial

    solver = FallbackIkSolver(
        solve,
        lambda _joints: None,
        line_search_alphas=(1.0, 0.5),
        orientation_relaxations=(1.0, 0.0),
        fallback_attempts_per_cycle=1,
    )

    first = solver.solve_target(pose(1.0), pose(), np.zeros(6))
    second = solver.solve_target(pose(2.0), pose(), np.zeros(6))
    third = solver.solve_target(pose(3.0), pose(), np.zeros(6))

    assert not first.search_exhausted and not second.search_exhausted
    assert third.search_exhausted
    # Exact retries follow the live target, while the bounded fallback grid proves one fixed
    # target unreachable instead of combining candidates from unrelated tracker samples.
    assert attempted_x == pytest.approx([1.0, 1.0, 2.0, 0.5, 3.0, 0.5])


def test_degraded_preference_is_reused_after_trying_one_better_candidate():
    attempted_x = []

    def solve(target, initial):
        attempted_x.append(float(target[0, 3]))
        return target[0, 3] <= 0.5 + 1e-9, initial + target[0, 3]

    solver = FallbackIkSolver(
        solve,
        lambda _joints: None,
        line_search_alphas=(1.0, 0.5, 0.1),
        orientation_relaxations=(1.0, 0.0),
        fallback_attempts_per_cycle=2,
    )
    first = solver.solve_target(pose(1.0), pose(), np.zeros(6))
    attempted_x.clear()
    second = solver.solve_target(pose(1.0), pose(), np.zeros(6))

    assert first.mode == second.mode == "degraded_ik"
    assert second.position_alpha == 0.5
    # Exact, then the immediately better alpha=1/orientation-relaxed candidate, then preference.
    assert attempted_x == pytest.approx([1.0, 1.0, 0.5])


def test_single_attempt_budget_reuses_known_degraded_preference():
    def solve(target, initial):
        return target[0, 3] <= 0.5 + 1e-9, initial + target[0, 3]

    solver = FallbackIkSolver(
        solve,
        lambda _joints: None,
        line_search_alphas=(1.0, 0.5),
        orientation_relaxations=(1.0, 0.0),
        fallback_attempts_per_cycle=1,
    )
    assert solver.solve_target(pose(1.0), pose(), np.zeros(6)).mode == "holding_ik"
    assert solver.solve_target(pose(1.0), pose(), np.zeros(6)).mode == "degraded_ik"
    reused = solver.solve_target(pose(1.0), pose(), np.zeros(6))

    assert reused.mode == "degraded_ik"
    assert reused.position_alpha == 0.5
    assert reused.attempts == 2


def test_holding_diagnostic_contains_requested_failure_context():
    solver = FallbackIkSolver(
        lambda _target, initial: (False, initial),
        lambda _joints: None,
        line_search_alphas=(1.0, 0.5),
        orientation_relaxations=(1.0, 0.0),
        fallback_attempts_per_cycle=1,
    )
    result = solver.solve_target(pose(0.2, 60.0), pose(0.1), np.zeros(6))
    diagnostic = fallback_diagnostic("right", result, pose(0.2, 60.0))

    assert "right=holding_ik" in diagnostic
    assert "desired_xyz_mm=[+200.0,+0.0,+0.0]" in diagnostic
    assert "last_good_xyz_mm=[+100.0,+0.0,+0.0]" in diagnostic
    assert "desired_from_last_rot=60.0deg" in diagnostic
    assert "IK solver did not converge" in diagnostic


def test_fallback_configuration_is_validated():
    with pytest.raises(ValueError, match="start with 1.0"):
        FallbackIkSolver(lambda _pose, q: (True, q), lambda _q: None, line_search_alphas=(0.5,))
    with pytest.raises(ValueError, match="strictly decreasing"):
        FallbackIkSolver(
            lambda _pose, q: (True, q),
            lambda _q: None,
            orientation_relaxations=(1.0, 0.5, 0.5),
        )


def test_async_solver_keeps_only_latest_pending_target():
    started = Event()
    release = Event()
    attempted_x: list[float] = []

    def solve(target, initial):
        x = float(target[0, 3])
        attempted_x.append(x)
        if x == 1.0:
            started.set()
            assert release.wait(timeout=1.0)
        return True, initial + x

    worker = AsyncFallbackIkSolver(
        FallbackIkSolver(solve, lambda _joints: None),
        name="test",
    )
    try:
        first_id = worker.submit(pose(1.0), pose(), np.zeros(6), generation=4)
        assert started.wait(timeout=1.0)

        submitted = time.monotonic()
        worker.submit(pose(2.0), pose(), np.zeros(6), generation=4)
        latest_id = worker.submit(pose(3.0), pose(), np.zeros(6), generation=4)
        assert time.monotonic() - submitted < 0.1
        release.set()

        completions = []
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            if completion := worker.poll(generation=4):
                completions.append(completion)
                if completion.request_id == latest_id:
                    break
            time.sleep(0.005)

        assert first_id < latest_id
        assert completions[-1].request_id == latest_id
        np.testing.assert_allclose(completions[-1].desired_pose, pose(3.0))
        np.testing.assert_allclose(completions[-1].outcome.joints, 3.0)
        assert attempted_x == [1.0, 3.0]
    finally:
        release.set()
        assert worker.close(timeout_s=1.0)


def test_async_solver_does_not_deliver_an_old_generation():
    worker = AsyncFallbackIkSolver(
        FallbackIkSolver(lambda _target, initial: (True, initial), lambda _joints: None),
        name="test",
    )
    try:
        worker.submit(pose(), pose(), np.zeros(6), generation=1)
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline and worker._latest is None:
            time.sleep(0.005)
        assert worker.poll(generation=2) is None
        assert worker.poll(generation=1) is None
    finally:
        assert worker.close(timeout_s=1.0)
