"""Trajectories: TUM files, matching to dataset frames, pose maths.

Convention used everywhere in mapping_v2: a trajectory is a list of 4x4 world-from-camera
poses of the colour camera OPTICAL frame (x right, y down, z forward), one per dataset frame
(NaN when the engine gave no pose), with the first posed frame at the identity.
"""
import numpy as np
from scipy.spatial.transform import Rotation as Rot


def read_tum(path):
    """-> stamps [N], poses [N,4,4] (world-from-body as written in the file)."""
    rows = []
    for line in open(path):
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        v = line.split()
        if len(v) < 8:
            continue
        rows.append([float(x) for x in v[:8]])
    if not rows:
        return np.zeros(0), np.zeros((0, 4, 4))
    a = np.array(rows)
    a = a[np.argsort(a[:, 0], kind="stable")]
    T = np.tile(np.eye(4), (len(a), 1, 1))
    T[:, :3, :3] = Rot.from_quat(a[:, 4:8]).as_matrix()
    T[:, :3, 3] = a[:, 1:4]
    return a[:, 0], T


def write_tum(path, stamps, poses, header="timestamp tx ty tz qx qy qz qw"):
    with open(path, "w") as fh:
        fh.write(f"# {header}\n")
        for t, T in zip(stamps, poses):
            if not np.all(np.isfinite(T)):
                continue
            q = Rot.from_matrix(T[:3, :3]).as_quat()
            fh.write(f"{t:.6f} {T[0, 3]:.6f} {T[1, 3]:.6f} {T[2, 3]:.6f} "
                     f"{q[0]:.9f} {q[1]:.9f} {q[2]:.9f} {q[3]:.9f}\n")


def match_to_frames(frame_stamps, stamps, poses, tol_s=0.002):
    """Pose per dataset frame (NaN where none within tol). Returns poses [F,4,4], n_matched."""
    out = np.full((len(frame_stamps), 4, 4), np.nan)
    if len(stamps) == 0:
        return out, 0
    idx = np.searchsorted(stamps, frame_stamps)
    n = 0
    for i, t in enumerate(frame_stamps):
        best = None
        for j in (idx[i] - 1, idx[i]):
            if 0 <= j < len(stamps) and abs(stamps[j] - t) <= tol_s and (best is None or abs(stamps[j] - t) < abs(stamps[best] - t)):
                best = j
        if best is not None:
            out[i] = poses[best]
            n += 1
    return out, n


def valid_mask(poses):
    return np.all(np.isfinite(poses.reshape(len(poses), -1)), axis=1)


def anchor_first(poses):
    """Express all poses relative to the first valid one (first posed frame = identity)."""
    m = valid_mask(poses)
    if not m.any():
        return poses
    T0inv = np.linalg.inv(poses[np.argmax(m)])
    out = poses.copy()
    out[m] = T0inv @ poses[m]
    return out


def rot_angle_deg(R):
    c = (np.trace(R) - 1.0) / 2.0
    return float(np.degrees(np.arccos(np.clip(c, -1.0, 1.0))))


def umeyama(src, dst, with_scale=False):
    """Least-squares T (4x4) with dst ~ T @ src for point sets [N,3]."""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    xs, xd = src - mu_s, dst - mu_d
    U, S, Vt = np.linalg.svd(xd.T @ xs / len(src))
    D = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        D[2, 2] = -1
    R = U @ D @ Vt
    s = (S * np.diag(D)).sum() / xs.var(0).sum() if with_scale else 1.0
    T = np.eye(4)
    T[:3, :3] = s * R
    T[:3, 3] = mu_d - s * R @ mu_s
    return T


def interp_rotations(t_query, t_poses, poses):
    """Slerp of rotations (world-from-camera) at t_query; NaN outside / across invalid poses."""
    from scipy.spatial.transform import Slerp
    m = valid_mask(poses)
    tp, Pm = t_poses[m], poses[m]
    out = np.full((len(t_query), 3, 3), np.nan)
    if len(tp) < 2:
        return out
    inside = (t_query >= tp[0]) & (t_query <= tp[-1])
    sl = Slerp(tp, Rot.from_matrix(Pm[:, :3, :3]))
    out[inside] = sl(t_query[inside]).as_matrix()
    return out
