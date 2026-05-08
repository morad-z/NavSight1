"""Convention tests for the NavSight EKF Z-up world frame fix.

Background
----------
NavSight has had two coexisting world frames in different parts of the code:
the Madgwick filter and IMU pipeline are Z-up (gravity along world +Z, matches
Android sensor convention), and the EKF state evolution post-c1c15b2 was Y-up
(gravity along world -Y, yaw extracted around world Y). The mismatch produces
a ~180 degree heading offset on real walks.

This file pins the math identities the EKF should implement after the Z-up
fix lands. Run on Windows directly:

    python scripts/test_z_up_conventions.py

Each test asserts a specific identity. A failing test means a change to the
C++ code violated the convention. Tests are pure-numpy so no OpenCV install
is required.

Conventions enforced
--------------------
- World frame: ENU (X=East, Y=North, Z=Up), right-handed.
- Body frame: Android sensor (X=right, Y=screen-top, Z=out-of-screen).
- R_GtoI: world->body rotation (a vector in world coords becomes its body-frame
  representation when left-multiplied by R_GtoI).
- Yaw nav convention: compass-CW positive, North=0, East=+pi/2, range [-pi, pi].
- Yaw axis: world Z (vertical / gravity-anti-parallel). Body Z when phone is
  held screen-up flat.
"""

from __future__ import annotations

import math
import sys

import numpy as np


# ---------------------------------------------------------------------------
# Convention helpers (must match the C++ implementation)
# ---------------------------------------------------------------------------


def Rx(rad: float) -> np.ndarray:
    c, s = math.cos(rad), math.sin(rad)
    return np.array([[1.0, 0.0, 0.0],
                     [0.0, c, -s],
                     [0.0, s, c]])


def Ry(rad: float) -> np.ndarray:
    c, s = math.cos(rad), math.sin(rad)
    return np.array([[c, 0.0, s],
                     [0.0, 1.0, 0.0],
                     [-s, 0.0, c]])


def Rz(rad: float) -> np.ndarray:
    c, s = math.cos(rad), math.sin(rad)
    return np.array([[c, -s, 0.0],
                     [s, c, 0.0],
                     [0.0, 0.0, 1.0]])


def quat_to_rot_b2w_hamilton(q: np.ndarray) -> np.ndarray:
    """Hamilton (w, x, y, z) -> 3x3 body-to-world rotation matrix.

    Matches IMUPreintegrator's q0_, q1_, q2_, q3_ layout.
    """
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def imu_get_heading(q: np.ndarray) -> float:
    """Mirror of IMUPreintegrator::getHeading (Z-up CW-positive nav yaw)."""
    w, x, y, z = q
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw_math = math.atan2(siny_cosp, cosy_cosp)
    return -yaw_math  # negate for CW-positive nav


def ekf_get_yaw_zup(R_GtoI: np.ndarray, roll: float = 0.0, pitch: float = 0.0) -> float:
    """Mirror of post-fix EKFState::getYaw (Z-up world, CW-positive nav).

    R_GtoI is world->body. yaw is rotation around world-Z. The R_align
    sandwich removes Madgwick roll and pitch (which themselves are Z-up
    extractions), so for a pure-yaw R_GtoI the sandwich reduces to identity.
    """
    R_align = Ry(pitch) @ Rx(roll)
    R_aligned = R_align @ R_GtoI @ R_align.T
    return math.atan2(R_aligned[1, 0], R_aligned[0, 0])


def ekf_get_yaw_yup_buggy(R_GtoI: np.ndarray, roll: float = 0.0, pitch: float = 0.0) -> float:
    """Mirror of c1c15b2 (current bug) Y-up getYaw — kept for cross-check."""
    R_align = Ry(pitch) @ Rx(roll)
    R_aligned = R_align @ R_GtoI @ R_align.T
    return math.atan2(-R_aligned[0, 2], R_aligned[0, 0])


def init_R_from_azimuth(azimuth_rad: float) -> np.ndarray:
    """Mirror of Tracker::setInitialHeading matrix construction.

    For body at compass heading psi (CW-positive nav), this is the world->body
    rotation in ENU world (X=East, Y=North, Z=Up). Verified via worked examples
    in the docstring.
    """
    c, s = math.cos(azimuth_rad), math.sin(azimuth_rad)
    return np.array([[c, -s, 0.0],
                     [s, c, 0.0],
                     [0.0, 0.0, 1.0]])


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


PASS = "\033[92mPASS\033[0m"
FAIL = "\033[91mFAIL\033[0m"
TOL = 1e-6


def _approx_equal(a: float, b: float, tol: float = TOL) -> bool:
    diff = abs(a - b)
    while diff > math.pi:
        diff = abs(diff - 2 * math.pi)
    return diff < tol


def test_setInitialHeading_roundtrip_with_zup_getYaw() -> tuple[bool, str]:
    """For each azimuth psi in test set, init_R(psi) -> getYaw_zup -> psi."""
    azimuths = [0.0, math.pi / 4, math.pi / 2, math.pi, -math.pi / 2,
                3 * math.pi / 4, -3 * math.pi / 4, 0.123]
    for psi in azimuths:
        R = init_R_from_azimuth(psi)
        recovered = ekf_get_yaw_zup(R)
        if not _approx_equal(recovered, psi):
            return False, f"psi={math.degrees(psi):.2f}° -> getYaw={math.degrees(recovered):.2f}°"
    return True, ""


def test_setInitialHeading_roundtrip_with_yup_yields_degenerate() -> tuple[bool, str]:
    """The current Y-up getYaw on the same matrix always returns 0 or pi.

    This is the BUG: Tracker init builds Z-rotation matrix which has R[0,2]=0
    for any azimuth, so the Y-up getYaw collapses to 0 or pi.
    """
    psi_east = math.pi / 2
    R = init_R_from_azimuth(psi_east)
    yup = ekf_get_yaw_yup_buggy(R)
    # For the Z-rotation matrix, R[0,2] = 0 always. The Y-up form returns
    # atan2(0, cos(pi/2)) = atan2(0, 0) which numpy treats as 0 (or sometimes
    # an arbitrary value at the branch). Either way, not pi/2.
    if abs(yup - psi_east) < 0.5:
        return False, f"Y-up form unexpectedly returned correct yaw {math.degrees(yup):.2f}° — bug not present?"
    return True, f"Y-up form returns {math.degrees(yup):.2f}° (degenerate) for true psi={math.degrees(psi_east):.2f}° — confirms bug."


def test_madgwick_quat_yaw_matches_ekf_get_yaw() -> tuple[bool, str]:
    """Madgwick body->world quat -> imu.getHeading should match EKF getYaw on R_GtoI = R_b2w.T.

    For Madgwick (Hamilton, Z-up), a pure-yaw rotation by yaw_math CCW around
    world Z gives quaternion q = (cos(yaw_math/2), 0, 0, sin(yaw_math/2)).
    imu.getHeading returns -yaw_math (CW-positive nav).
    The world->body matrix is R_b2w.T. With Z-up getYaw on R_GtoI:
        R_GtoI = Rz(yaw_math).T = Rz(-yaw_math)
        R_GtoI[1,0] = -sin(yaw_math), R_GtoI[0,0] = cos(yaw_math)
        getYaw(R_GtoI) = atan2(-sin(yaw_math), cos(yaw_math)) = -yaw_math
    So imu.getHeading() == ekf.getYaw(R_GtoI). Both are CW-positive nav yaw.
    """
    for yaw_math in [0.0, math.pi / 4, -math.pi / 3, math.pi / 2, -math.pi]:
        c2 = math.cos(yaw_math / 2)
        s2 = math.sin(yaw_math / 2)
        q = np.array([c2, 0.0, 0.0, s2])
        R_b2w = quat_to_rot_b2w_hamilton(q)
        R_GtoI = R_b2w.T

        nav_from_quat = imu_get_heading(q)
        nav_from_ekf = ekf_get_yaw_zup(R_GtoI)

        if not _approx_equal(nav_from_quat, nav_from_ekf):
            return False, (f"yaw_math={math.degrees(yaw_math):.2f}° -> "
                           f"getHeading={math.degrees(nav_from_quat):.2f}°, "
                           f"getYaw={math.degrees(nav_from_ekf):.2f}°")
    return True, ""


def test_stationary_gravity_cancels_in_zup() -> tuple[bool, str]:
    """Phone screen-up stationary: body accel = (0, 0, +9.81). World specific
    force after Z-up gravity subtract should be ~0 -> no velocity drift.

    a_world_total = R_GtoI.T @ a_body
    a_world_kinematic = a_world_total + g_world  (where g_world = (0,0,-9.81))
    """
    a_body = np.array([0.0, 0.0, 9.81])  # gravity along body +Z (Android screen-up)
    R_GtoI = np.eye(3)  # body frame == world frame (rare but valid for test)

    g_zup = np.array([0.0, 0.0, -9.81])
    a_world_kinematic_zup = R_GtoI.T @ a_body + g_zup
    if np.linalg.norm(a_world_kinematic_zup) > 1e-9:
        return False, f"Z-up: residual specific force = {a_world_kinematic_zup} (should be 0)"

    g_yup = np.array([0.0, -9.81, 0.0])
    a_world_kinematic_yup = R_GtoI.T @ a_body + g_yup
    # Y-up bug: residual along Y (-9.81) and Z (+9.81) — leaks into velocity!
    leak_norm = np.linalg.norm(a_world_kinematic_yup)
    if leak_norm < 1e-3:
        return False, "Y-up bug should leak gravity but didn't — test setup wrong"
    return True, f"Z-up: residual = 0; Y-up bug leaks {leak_norm:.2f} m/s^2 — confirms current bug."


def test_visual_delta_yup_extraction_is_zero_on_zup_matrix() -> tuple[bool, str]:
    """Tracker.cpp:2144 visual_delta_heading_y_up = atan2(R[0,2], R[0,0]).

    For a Z-up R_aligned (which Madgwick roll/pitch produce), the [0,2]
    element is 0 for any pure-yaw rotation. So visual_delta_y_up is
    structurally zero — the visual yaw correction has been a no-op.
    """
    for psi in [math.pi / 6, math.pi / 3, -math.pi / 4]:
        R_aligned = Rz(psi)  # what R_align * R_GtoI * R_align.T produces for pure yaw
        delta_yup = math.atan2(R_aligned[0, 2], R_aligned[0, 0])
        if abs(delta_yup) > 1e-9:
            return False, f"psi={math.degrees(psi):.2f}° -> visual_delta_y_up={math.degrees(delta_yup):.6f}° (should be 0)"

        # Z-up extraction recovers psi correctly:
        delta_zup = math.atan2(R_aligned[1, 0], R_aligned[0, 0])
        if not _approx_equal(delta_zup, psi):
            return False, f"psi={math.degrees(psi):.2f}° -> Z-up delta={math.degrees(delta_zup):.2f}° (should match)"
    return True, "Y-up delta is 0 for Z-up R_aligned (confirms no-op); Z-up delta recovers yaw."


def test_h_jacobian_world_frame_is_constant_e_z() -> tuple[bool, str]:
    """For left-mult update R_new = exp(delta_theta_world) @ R_old, the
    yaw-measurement Jacobian is h = e_z_world = (0, 0, +1), CONSTANT
    regardless of state R.

    Test for several non-trivial states and verify dy/d(delta_theta_z) ≈ +1
    while dy/d(delta_theta_x) ≈ 0 and dy/d(delta_theta_y) ≈ 0.

    Pre-2026-05-09 the code had h = +R @ e_z_world (body-frame style). For
    pure-yaw states this coincides with (0, 0, 1) because Rz doesn't move
    e_z. For vertical-phone states (R has pitch ~ -pi/2), R @ e_z_world is
    body Y-axis, so the H Jacobian had wrong direction and the EKF's yaw
    correction was distributed over the wrong body axes — Step 7 chi² then
    rejected every loop closure on real walks (sim 1778262445638: 100/100
    chi² rejects).
    """
    eps = 1e-4

    # Vertical phone facing North: R_w2b = [[1,0,0],[0,0,1],[0,1,0]]
    R_vertical_N = np.array([[1.0, 0.0, 0.0],
                             [0.0, 0.0, 1.0],
                             [0.0, 1.0, 0.0]])
    # Compass East: Rz(+pi/2) world->body
    R_east = init_R_from_azimuth(math.pi / 2)
    # Identity
    R_identity = np.eye(3)

    states = [
        ("identity", R_identity),
        ("compass East (flat)", R_east),
        ("vertical North", R_vertical_N),
    ]

    for name, R_old in states:
        yaw_old = ekf_get_yaw_zup(R_old)
        for axis_name, axis in [("x", np.array([1.0, 0.0, 0.0])),
                                 ("y", np.array([0.0, 1.0, 0.0])),
                                 ("z", np.array([0.0, 0.0, 1.0]))]:
            d_theta = eps * axis
            skew = np.array([[0, -d_theta[2], d_theta[1]],
                             [d_theta[2], 0, -d_theta[0]],
                             [-d_theta[1], d_theta[0], 0]])
            R_new = (np.eye(3) + skew) @ R_old
            yaw_new = ekf_get_yaw_zup(R_new)

            # Wrap-aware diff
            diff = yaw_new - yaw_old
            while diff > math.pi: diff -= 2 * math.pi
            while diff < -math.pi: diff += 2 * math.pi

            d_yaw_d_eps = diff / eps
            expected = 1.0 if axis_name == "z" else 0.0
            if abs(d_yaw_d_eps - expected) > 0.01:
                return False, (f"state={name} axis={axis_name}: "
                               f"d_yaw/d_eps={d_yaw_d_eps:.3f} expected {expected:.0f}")
    return True, "h_world = e_z_world constant for all states ✓"


def test_setInitialHeading_east_gives_correct_body_axes() -> tuple[bool, str]:
    """Walk through the example: facing East, what does R_GtoI mean?

    User facing East: compass azimuth = +pi/2 nav-CW.
    init_R = Rz(+pi/2) (the matrix Tracker.cpp:322 builds) =
        [[0, -1, 0], [1, 0, 0], [0, 0, 1]]

    World frame ENU: X=East, Y=North, Z=Up.
    World North vector in world coords: (0, 1, 0).
    World East vector in world coords: (1, 0, 0).

    R_GtoI maps world->body. So the body coords of world East:
        R_GtoI @ (1,0,0) = (0, 1, 0) = body Y axis.
    This means body Y points in the direction of world East.

    Body Y is "screen-top" (Android convention). User facing East with
    phone held screen-up: screen-top points to the direction of motion = East.
    So body Y aligning with world East is correct. ✓
    """
    R = init_R_from_azimuth(math.pi / 2)
    east_in_body = R @ np.array([1.0, 0.0, 0.0])
    expected = np.array([0.0, 1.0, 0.0])  # should be body +Y
    if np.linalg.norm(east_in_body - expected) > 1e-9:
        return False, f"world East in body = {east_in_body}, expected {expected}"
    return True, "Facing East: world East -> body +Y axis (screen-top forward) ✓"


def test_madgwick_init_with_pending_yaw_recovers_compass_heading() -> tuple[bool, str]:
    """Regression for the 2026-05-09 bug: Madgwick used to initialize with
    yaw=0 regardless of the magnetometer-derived setInitialHeading.

    The fix: tryInitMadgwickLocked now uses
        yaw = -pending_madgwick_yaw_nav_   (math-CCW from compass-CW)
    so that imu.getHeading() returns the input azimuth.

    For roll=pitch=0 and yaw_math=-azimuth_nav, the quaternion is
        q = (cos(yaw_math/2), 0, 0, sin(yaw_math/2))
    and getHeading returns -atan2(2(q0·q3 + q1·q2), 1 - 2(q2² + q3²)) = -yaw_math = +azimuth_nav.
    """
    azimuths_nav = [0.0, math.pi / 4, math.pi / 2, math.pi - 0.01,
                    -math.pi / 2, -3 * math.pi / 4, 0.987]
    for azim_nav in azimuths_nav:
        yaw_math = -azim_nav  # pending compass heading -> Madgwick math yaw
        c2 = math.cos(yaw_math / 2)
        s2 = math.sin(yaw_math / 2)
        q = np.array([c2, 0.0, 0.0, s2])  # roll=pitch=0
        # Mirror IMUPreintegrator::getHeading
        nav = imu_get_heading(q)
        if not _approx_equal(nav, azim_nav):
            return False, (f"azim_nav={math.degrees(azim_nav):.2f}° -> "
                           f"getHeading={math.degrees(nav):.2f}°")
    return True, "Madgwick init with pending yaw -> getHeading recovers compass azimuth ✓"


def test_yup_buggy_get_yaw_returns_negated_for_y_axis_rotation() -> tuple[bool, str]:
    """Pin the existing test_ekf_state.cpp:35-50 finding:

    A Ry(+90) matrix passed to the buggy Y-up getYaw returns -pi/2 (or 0
    for atan2 ambiguity at the branch). After Z-up fix, Ry(+90) is no
    longer a "yaw" rotation — it's a pitch rotation. getYaw should return
    0 for pitch-only rotations.
    """
    R = Ry(math.pi / 2)
    # Z-up form: yaw extracted from atan2(R[1,0], R[0,0]) = atan2(0, 0) ~ 0
    yaw_zup = ekf_get_yaw_zup(R)
    if abs(yaw_zup) > 1e-6 and abs(abs(yaw_zup) - math.pi) > 1e-6:
        return False, f"Z-up getYaw on Ry(90) = {math.degrees(yaw_zup):.2f}° (should be 0 or branch)"

    # Z-up extraction on a pure Z rotation:
    R_zrot = Rz(math.pi / 2)
    yaw_zup_z = ekf_get_yaw_zup(R_zrot)
    if not _approx_equal(yaw_zup_z, math.pi / 2):
        return False, f"Z-up getYaw on Rz(90) = {math.degrees(yaw_zup_z):.2f}° (should be 90)"
    return True, "Y-axis rotation -> pitch (yaw=0); Z-axis rotation -> yaw=psi ✓"


# ---------------------------------------------------------------------------
# Runner
# ---------------------------------------------------------------------------


TESTS = [
    ("setInitialHeading round-trip with Z-up getYaw",
     test_setInitialHeading_roundtrip_with_zup_getYaw),
    ("setInitialHeading + Y-up getYaw is degenerate (confirms bug)",
     test_setInitialHeading_roundtrip_with_yup_yields_degenerate),
    ("Madgwick imu.getHeading == EKF getYaw on inverted matrix",
     test_madgwick_quat_yaw_matches_ekf_get_yaw),
    ("Stationary gravity cancels in Z-up; leaks in Y-up (confirms bug)",
     test_stationary_gravity_cancels_in_zup),
    ("visual_delta_y_up extracts 0 from Z-up R_aligned (confirms no-op)",
     test_visual_delta_yup_extraction_is_zero_on_zup_matrix),
    ("H Jacobian (left-mult/world-δθ): h = e_z_world constant for all states",
     test_h_jacobian_world_frame_is_constant_e_z),
    ("Facing East: world East maps to body +Y (screen-top)",
     test_setInitialHeading_east_gives_correct_body_axes),
    ("Y-axis rotation is pitch in Z-up (not yaw)",
     test_yup_buggy_get_yaw_returns_negated_for_y_axis_rotation),
    ("Madgwick init with pending yaw recovers compass heading (regression for 2026-05-09 bug)",
     test_madgwick_init_with_pending_yaw_recovers_compass_heading),
]


def main() -> int:
    print("NavSight Z-up world frame convention tests")
    print("=" * 72)
    failures = 0
    for name, fn in TESTS:
        try:
            ok, msg = fn()
        except Exception as e:  # pragma: no cover
            ok, msg = False, f"exception: {e}"
        tag = PASS if ok else FAIL
        print(f"[{tag}] {name}")
        if msg:
            print(f"        {msg}")
        if not ok:
            failures += 1
    print("=" * 72)
    print(f"{len(TESTS) - failures}/{len(TESTS)} passed")
    return 0 if failures == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
