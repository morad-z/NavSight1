"""Offline proof for ScaleEstimatorVI (Hesch-Martinelli closed-form VI scale).

Faithfully reimplements ScaleEstimatorVI::solve() (app/src/main/cpp/ScaleEstimatorVI.cpp)
in numpy and drives it with SYNTHETIC keyframe pairs from a KNOWN trajectory (true per-pair
metric displacement s_true), so we can prove:
  (A) clean body-frame inputs -> solve recovers s_true              [solver math correct]
  (B) camera-frame t_vis used as body-frame (the live bug)         -> s wrong (~3x)   [reproduce]
  (C) fix t_vis_body = R_bc^T * t_vis_cam                          -> s recovers       [validate fix]
  (D) near-constant velocity                                       -> s ill-conditioned [degeneracy]

No device, no OpenCV, no C++ build needed. Run: python scripts/test_scale_estimator.py
"""
import numpy as np

G = np.array([0.0, 0.0, -9.81])          # world gravity, Z-up ENU (matches solver default)
MIN_PAIRS = 4


def rot_z(yaw):
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def solve_scale_vi(pairs):
    """Exact port of ScaleEstimatorVI::solve(). pairs: list of dicts with keys
    R_w_b (3x3 body->world), t_vis_body (3, unit), delta_p_body (3), delta_v_body (3), dt.
    Returns (ok, s, var_s, v0)."""
    N = len(pairs)
    if N < MIN_PAIRS:
        return False, None, None, None
    # gamma_i = sum_{k<i} (R_w_b_k . delta_v_k + g . dt_k);  gamma_0 = 0
    gamma = [np.zeros(3) for _ in range(N)]
    for i in range(1, N):
        pk = pairs[i - 1]
        gamma[i] = gamma[i - 1] + pk['R_w_b'] @ pk['delta_v_body'] + G * pk['dt']
    AtA = np.zeros((4, 4))
    Atb = np.zeros(4)
    A_blocks, b_blocks = [], []
    for i in range(N):
        p = pairs[i]
        a_col = p['R_w_b'] @ p['t_vis_body']                       # 3
        dpw = p['R_w_b'] @ p['delta_p_body']                       # 3
        b_i = dpw + gamma[i] * p['dt'] + 0.5 * G * (p['dt'] ** 2)  # 3
        A_i = np.zeros((3, 4))
        A_i[:, 0] = a_col
        A_i[0, 1] = -p['dt']; A_i[1, 2] = -p['dt']; A_i[2, 3] = -p['dt']
        AtA += A_i.T @ A_i
        Atb += A_i.T @ b_i
        A_blocks.append(A_i); b_blocks.append(b_i)
    # x = (AtA)^-1 Atb  (SVD-solve as in cv::solve(DECOMP_SVD))
    x, *_ = np.linalg.lstsq(AtA, Atb, rcond=None)
    s = x[0]; v0 = x[1:]
    # residual variance + var_s = sigma2 * (AtA)^-1[0,0]
    rss = 0.0
    for i in range(N):
        r = A_blocks[i] @ x - b_blocks[i]
        rss += float(r @ r)
    dof = 3 * N - 4
    sigma2 = rss / dof if dof > 0 else rss
    AtA_inv = np.linalg.pinv(AtA)
    var_s = sigma2 * AtA_inv[0, 0]
    # solver's acceptance gate + the Observer-C relsigma gate
    ok = np.isfinite(s) and 0.01 < s < 10.0 and var_s > 0.0
    return ok, s, var_s, v0


def solve_scale_tls(pairs):
    """Total-Least-Squares variant: same A,b assembly as solve_scale_vi, but solve A x = b
    via TLS (SVD of [A|b]) which accounts for noise in the (noisy) visual scale column —
    avoids the OLS errors-in-variables attenuation of s. Returns (ok, s, v0)."""
    N = len(pairs)
    if N < MIN_PAIRS:
        return False, None, None
    gamma = [np.zeros(3) for _ in range(N)]
    for i in range(1, N):
        pk = pairs[i - 1]
        gamma[i] = gamma[i - 1] + pk['R_w_b'] @ pk['delta_v_body'] + G * pk['dt']
    A = np.zeros((3 * N, 4)); b = np.zeros(3 * N)
    for i in range(N):
        p = pairs[i]
        a_col = p['R_w_b'] @ p['t_vis_body']
        b_i = p['R_w_b'] @ p['delta_p_body'] + gamma[i] * p['dt'] + 0.5 * G * (p['dt'] ** 2)
        A[3*i:3*i+3, 0] = a_col
        A[3*i+0, 1] = -p['dt']; A[3*i+1, 2] = -p['dt']; A[3*i+2, 3] = -p['dt']
        b[3*i:3*i+3] = b_i
    M = np.hstack([A, b.reshape(-1, 1)])           # (3N x 5)
    _, _, Vt = np.linalg.svd(M, full_matrices=False)
    v = Vt[-1]                                      # smallest singular vector
    if abs(v[-1]) < 1e-12:
        return False, None, None
    x = -v[:4] / v[-1]
    s = x[0]
    ok = np.isfinite(s) and 0.01 < s < 10.0
    return ok, s, x[1:]


def gen_pairs(speeds, dt, R_GtoI, R_bc, t_vis_mode):
    """Build synthetic per-pair data for a trajectory moving forward (world +Y) at the
    given per-pair speeds. R_GtoI = world->body (constant for simplicity), R_bc = body->camera.
    t_vis_mode: 'body_correct' | 'camera_bug' | 'camera_fixed'.
    Per-pair true displacement magnitude = speed*dt (this is s_true for that pair)."""
    R_w_b = R_GtoI.T                       # body->world
    R_GtoC = R_bc @ R_GtoI                 # world->camera
    fwd_world = np.array([0.0, 1.0, 0.0])  # walking/riding forward = world +Y
    pairs = []
    v_prev = None
    for k, sp in enumerate(speeds):
        v_world = sp * fwd_world                       # world velocity this pair
        # world accel over the pair (from velocity change); first pair from rest
        a_world = ((v_world - (v_prev if v_prev is not None else np.zeros(3))) / dt)
        v_prev = v_world
        # specific force in body (accelerometer): f_body = R_GtoI (a_world - g)
        f_body = R_GtoI @ (a_world - G)
        # Forster short-pair preintegration (R_rel ~ I over one frame):
        delta_v_body = f_body * dt
        delta_p_body = 0.5 * f_body * (dt ** 2)
        # the metric world displacement this pair (what s*t_vis must equal):
        disp_world = v_world * dt + 0.5 * a_world * (dt ** 2)
        s_true = np.linalg.norm(disp_world)
        dir_world = disp_world / s_true if s_true > 1e-9 else fwd_world
        t_vis_cam = R_GtoC @ dir_world                 # unit dir in CAMERA frame (recoverPose output)
        if t_vis_mode == 'body_correct':
            t_vis_body = R_GtoI @ dir_world            # the ideal body-frame unit dir
        elif t_vis_mode == 'camera_bug':
            t_vis_body = t_vis_cam                      # BUG: camera-frame used as body-frame
        elif t_vis_mode == 'camera_fixed':
            t_vis_body = R_bc.T @ t_vis_cam             # FIX: camera->body
        else:
            raise ValueError(t_vis_mode)
        pairs.append(dict(R_w_b=R_w_b, t_vis_body=t_vis_body,
                          delta_p_body=delta_p_body, delta_v_body=delta_v_body, dt=dt))
    return pairs, np.mean(speeds) * dt   # mean per-pair displacement ~ what single-s recovers


def report(name, pairs, s_expect):
    ok, s, var_s, v0 = solve_scale_vi(pairs)
    relsig = (np.sqrt(var_s) / s) if (s and s > 1e-9 and var_s is not None) else float('nan')
    speed = s / DT if s else float('nan')
    print(f"  {name:34s} ok={ok}  s={s if s is None else round(s,4)}  "
          f"s_expect~{s_expect:.4f}  ratio={ (s/s_expect) if s else float('nan'):.2f}  "
          f"relsig={relsig:.2f}  vi_speed={speed*3.6:.1f} km/h"
          + ("  [PASS]" if (s and abs(s/s_expect-1) < 0.1) else "  [OFF]"))


DT = 0.033
# S21 extrinsic ~ diag(1,-1,-1) per the project memory (body->camera, near-180 flip)
R_BC = np.diag([1.0, -1.0, -1.0])
# a representative non-trivial body orientation (phone tilted/held): yaw 40deg + we keep
# world->body as a rotation so the frames genuinely differ.
R_GTOI = rot_z(np.deg2rad(40.0))

print("=== ScaleEstimatorVI offline proof (run ~3 m/s) ===")
# RUN: accelerate 0->3 m/s over ~1s then cruise (gives observability via the accel phase)
run_speeds = list(np.linspace(0.2, 3.0, 30)) + [3.0] * 60   # 90 pairs ~ 3s
s_run = np.mean(run_speeds) * DT
for mode in ['body_correct', 'camera_bug', 'camera_fixed']:
    pairs, _ = gen_pairs(run_speeds, DT, R_GTOI, R_BC, mode)
    report(mode, pairs, s_run)

print("\n=== constant-velocity (degeneracy check), body_correct inputs ===")
const_speeds = [3.0] * 90
pairs, _ = gen_pairs(const_speeds, DT, R_GTOI, R_BC, 'body_correct')
report('const-vel body_correct', pairs, np.mean(const_speeds) * DT)

print("\n=== R_bc = identity (no extrinsic) — camera_bug should then equal body_correct ===")
pairs, _ = gen_pairs(run_speeds, DT, R_GTOI, np.eye(3), 'camera_bug')
report('camera_bug (R_bc=I)', pairs, s_run)


# ── ROBUSTNESS: does the FIX hold under recoverPose direction noise + varied
#    body orientations, and does it pass the relsigma<0.25 accept gate (so a real
#    run would actually ACCEPT vi_speed)? The frame fix is exact in expectation;
#    this checks the residual variance is small enough on noisy real-ish data.
def gen_pairs_noisy(speeds, dt, R_GtoI, R_bc, t_vis_mode, dir_noise_deg, accel_noise, rng):
    R_w_b = R_GtoI.T
    R_GtoC = R_bc @ R_GtoI
    fwd_world = np.array([0.0, 1.0, 0.0])
    pairs, v_prev = [], None
    for sp in speeds:
        v_world = sp * fwd_world
        a_world = (v_world - (v_prev if v_prev is not None else np.zeros(3))) / dt
        v_prev = v_world
        f_body = R_GtoI @ (a_world - G) + rng.normal(0, accel_noise, 3)  # accel sensor noise
        delta_v_body = f_body * dt
        delta_p_body = 0.5 * f_body * (dt ** 2)
        disp_world = v_world * dt + 0.5 * a_world * (dt ** 2)
        s_true = np.linalg.norm(disp_world)
        dir_world = disp_world / s_true if s_true > 1e-9 else fwd_world
        t_vis_cam = R_GtoC @ dir_world
        # recoverPose direction noise: perturb the unit direction by a small random rotation
        ax = rng.normal(0, 1, 3); ax /= (np.linalg.norm(ax) + 1e-9)
        ang = np.deg2rad(rng.normal(0, dir_noise_deg))
        K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
        Rn = np.eye(3) + np.sin(ang) * K + (1 - np.cos(ang)) * (K @ K)
        t_vis_cam = Rn @ t_vis_cam
        if t_vis_mode == 'camera_bug':
            t_vis_body = t_vis_cam
        else:  # camera_fixed
            t_vis_body = R_bc.T @ t_vis_cam
        pairs.append(dict(R_w_b=R_w_b, t_vis_body=t_vis_body,
                          delta_p_body=delta_p_body, delta_v_body=delta_v_body, dt=dt))
    return pairs

def gen_pairs_baseline(total_secs, pair_dt, R_GtoI, R_bc, accel_err, dir_noise_deg, rng):
    """Run: accelerate 0->3 m/s over the first ~1.2s, then cruise. Build pairs of span
    pair_dt (sub-sampling: bigger dt -> bigger per-pair displacement -> better noise SNR).
    accel_err = effective per-axis accel error stdev (white + gravity-leak-equivalent)."""
    R_w_b = R_GtoI.T
    R_GtoC = R_bc @ R_GtoI
    fwd = np.array([0.0, 1.0, 0.0])
    n = int(round(total_secs / pair_dt))
    pairs, v_prev = [], 0.0
    for k in range(n):
        t = k * pair_dt
        sp = min(3.0, 0.2 + 2.8 * (t / 1.2)) if t < 1.2 else 3.0   # ramp then cruise
        v_world = sp * fwd
        a_world = (v_world - v_prev * fwd) / pair_dt
        v_prev = sp
        f_body = R_GtoI @ (a_world - G) + rng.normal(0, accel_err, 3)
        delta_v_body = f_body * pair_dt
        delta_p_body = 0.5 * f_body * (pair_dt ** 2)
        disp = v_world * pair_dt + 0.5 * a_world * (pair_dt ** 2)
        s_true = np.linalg.norm(disp)
        d = disp / s_true if s_true > 1e-9 else fwd
        t_cam = R_GtoC @ d
        ax = rng.normal(0, 1, 3); ax /= (np.linalg.norm(ax) + 1e-9)
        ang = np.deg2rad(rng.normal(0, dir_noise_deg))
        K = np.array([[0,-ax[2],ax[1]],[ax[2],0,-ax[0]],[-ax[1],ax[0],0]])
        Rn = np.eye(3) + np.sin(ang)*K + (1-np.cos(ang))*(K@K)
        t_cam = Rn @ t_cam
        t_vis_body = R_bc.T @ t_cam                                   # FIX applied
        pairs.append(dict(R_w_b=R_w_b, t_vis_body=t_vis_body,
                          delta_p_body=delta_p_body, delta_v_body=delta_v_body, dt=pair_dt))
    return pairs

print("\n=== DECISIVE: noise x pair-baseline sweep (fix applied, yaw=40, 20 trials each) ===")
print("    accel_err: 5e-4=white only; 0.02/0.05/0.1 = gravity-leak-equivalent (attitude err)")
rng = np.random.default_rng(7)
for pair_dt, label in [(0.033, 'per-frame (~90 pairs/3s)'), (0.33, 'sub-sampled 10x (~9 pairs/3s)')]:
    print(f"  -- baseline {label} --")
    for ae in (5e-4, 0.02, 0.05, 0.1):
        for dn in (1.0, 3.0):
            ratios, accepts = [], 0
            for _ in range(20):
                pairs = gen_pairs_baseline(3.0, pair_dt, rot_z(np.deg2rad(40)), R_BC, ae, dn, rng)
                # mean per-pair s_true for ratio (depends on baseline)
                strue = np.mean([np.linalg.norm(p['delta_p_body']) for p in pairs])  # placeholder
                ok, s, var_s, _ = solve_scale_vi(pairs)
                relsig = (np.sqrt(var_s)/s) if (s and s > 1e-9) else 9.9
                # expected s = mean per-pair displacement = ~mean_speed*pair_dt
                exp = 2.6 * pair_dt
                if s and s > 1e-9:
                    ratios.append(s / exp)
                if ok and relsig < 0.25:
                    accepts += 1
            mr = np.median(ratios) if ratios else float('nan')
            print(f"     accel_err={ae:<6} dir_noise={dn}deg  accept={accepts:2d}/20  median s/s_true={mr:.2f}")


def gen_pairs_profile(speed_fn, total_secs, pair_dt, R_GtoI, R_bc, accel_err, dir_noise_deg, rng):
    R_w_b = R_GtoI.T; R_GtoC = R_bc @ R_GtoI; fwd = np.array([0.0, 1.0, 0.0])
    n = int(round(total_secs / pair_dt)); pairs = []; v_prev = speed_fn(0.0)
    exp_disp = []
    for k in range(n):
        t = k * pair_dt; sp = speed_fn(t)
        v_world = sp * fwd; a_world = (v_world - v_prev * fwd) / pair_dt; v_prev = sp
        f_body = R_GtoI @ (a_world - G) + rng.normal(0, accel_err, 3)
        delta_v_body = f_body * pair_dt; delta_p_body = 0.5 * f_body * (pair_dt ** 2)
        disp = v_world * pair_dt + 0.5 * a_world * (pair_dt ** 2)
        s_true = np.linalg.norm(disp); d = disp / s_true if s_true > 1e-9 else fwd
        exp_disp.append(s_true)
        t_cam = R_GtoC @ d
        ax = rng.normal(0, 1, 3); ax /= (np.linalg.norm(ax) + 1e-9)
        ang = np.deg2rad(rng.normal(0, dir_noise_deg))
        Kx = np.array([[0,-ax[2],ax[1]],[ax[2],0,-ax[0]],[-ax[1],ax[0],0]])
        Rn = np.eye(3) + np.sin(ang)*Kx + (1-np.cos(ang))*(Kx@Kx)
        t_cam = Rn @ t_cam
        pairs.append(dict(R_w_b=R_w_b, t_vis_body=R_bc.T @ t_cam,
                          delta_p_body=delta_p_body, delta_v_body=delta_v_body, dt=pair_dt))
    return pairs, np.mean(exp_disp)

print("\n=== TLS vs OLS under noise (scooter profile, fix applied) — does TLS beat the dilution? ===")
scooter0 = lambda t: 5.0 + 3.0 * np.sin(2 * np.pi * t / 1.5)
rng = np.random.default_rng(99)
for pair_dt, label in [(0.033, 'per-frame'), (0.2, 'sub-6x')]:
    for ae, dn in [(5e-4, 1.0), (0.05, 2.0)]:
        ols_r, tls_r = [], []
        for _ in range(20):
            pairs, exp = gen_pairs_profile(scooter0, 4.0, pair_dt, rot_z(np.deg2rad(40)), R_BC, ae, dn, rng)
            _, s_ols, _, _ = solve_scale_vi(pairs)
            _, s_tls, _ = solve_scale_tls(pairs)
            if s_ols and s_ols > 1e-9: ols_r.append(s_ols / exp)
            if s_tls and s_tls > 1e-9: tls_r.append(s_tls / exp)
        mo = np.median(ols_r) if ols_r else float('nan')
        mt = np.median(tls_r) if tls_r else float('nan')
        print(f"  {label:9s} ae={ae:<6} dir={dn}deg  OLS s/s_true={mo:.2f}   TLS s/s_true={mt:.2f}")

print("\n=== STRONG-EXCITATION scooter profile (hard accel/brake 2<->8 m/s, fix applied) ===")
# scooter: oscillate speed 2..8 m/s with ~1.5s period => strong, sustained acceleration
scooter = lambda t: 5.0 + 3.0 * np.sin(2 * np.pi * t / 1.5)
rng = np.random.default_rng(11)
for pair_dt, label in [(0.033, 'per-frame'), (0.2, 'sub-6x'), (0.5, 'sub-15x')]:
    for ae, dn in [(5e-4, 1.0), (0.05, 2.0), (0.1, 2.0)]:
        ratios, accepts = [], 0
        for _ in range(20):
            pairs, exp = gen_pairs_profile(scooter, 4.0, pair_dt, rot_z(np.deg2rad(40)), R_BC, ae, dn, rng)
            ok, s, var_s, _ = solve_scale_vi(pairs)
            relsig = (np.sqrt(var_s)/s) if (s and s > 1e-9) else 9.9
            if s and s > 1e-9: ratios.append(s / exp)
            if ok and relsig < 0.25: accepts += 1
        mr = np.median(ratios) if ratios else float('nan')
        print(f"  {label:9s} accel_err={ae:<6} dir={dn}deg  accept={accepts:2d}/20  median s/s_true={mr:.2f}")
