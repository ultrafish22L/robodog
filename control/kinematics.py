"""Per-leg 3-DOF kinematics for robodog — axes/anchors GENERATED from dog13.py's CAD.

Each leg has its own FK/IK in the BODY frame. The reference (front-left) joints rotate about
world X (splay/roll @ SP), Y (pitch @ HP), Y (knee @ K); the other three corners use the SAME
world axes at their tf()-transformed anchor points + tf-transformed zero-pose foot, so a Leg's
IK returns the URDF joint values directly — the gait commands every leg with body-frame foot
targets and no per-leg sign bookkeeping.

    foot_body = R(s, X @ SP_L) · R(p, Y @ HP_L) · R(f, Y @ K_L) · foot0_L
"""
import os
import numpy as np

# --- CAD constants (dog13.py: HPx=106, DX=6, KZ=-75, YSHIFT=7.2) ; reference (FL) leg ---
SP = np.array([0.0,  27.2,  12.0])
HP = np.array([100.0, 0.0,  10.0])
K  = np.array([100.0, 0.0, -75.0])
AX = {"splay": np.array([1.0, 0, 0]), "pitch": np.array([0, 1.0, 0]), "knee": np.array([0, 1.0, 0])}

CORNERS = {"fl": (1, 1), "fr": (1, -1), "rl": (-1, 1), "rr": (-1, -1)}

# CAD physical limits are for the FL leg; the mirror corners flip sense, so the URDF/IK use
# generous symmetric limits and the collision validator (poses.py) enforces real feasibility.
LIM_PHYS_FL = {"splay": (-45.0, 45.0), "pitch": (-90.0, 90.0), "knee": (-120.0, 10.0)}
LIM = {"splay": (-55.0, 55.0), "pitch": (-100.0, 100.0), "knee": (-125.0, 125.0)}

def _rot_about(axis, deg, point):
    a = np.asarray(axis, float); a = a / np.linalg.norm(a)
    th = np.radians(deg); c, s = np.cos(th), np.sin(th); x, y, z = a
    R = np.array([[c + x*x*(1-c),   x*y*(1-c) - z*s, x*z*(1-c) + y*s],
                  [y*x*(1-c) + z*s, c + y*y*(1-c),   y*z*(1-c) - x*s],
                  [z*x*(1-c) - y*s, z*y*(1-c) + x*s, c + z*z*(1-c)]])
    T = np.eye(4); T[:3, :3] = R; p = np.asarray(point, float); T[:3, 3] = p - R @ p
    return T

def tf_point(pt, sx, sy):
    """dog13 tf() on a point: mirror-Y (chiral corners) then 180-Z (rear)."""
    q = np.array(pt, float)
    if (sx > 0 and sy < 0) or (sx < 0 and sy > 0):
        q = np.array([q[0], -q[1], q[2]])
    if sx < 0:
        q = np.array([-q[0], -q[1], q[2]])
    return q

def _clip(q):
    return np.array([np.clip(q[0], *LIM["splay"]), np.clip(q[1], *LIM["pitch"]), np.clip(q[2], *LIM["knee"])])

class Leg:
    def __init__(self, name, sx, sy, foot0_fl):
        self.name, self.sx, self.sy = name, sx, sy
        self.SP = tf_point(SP, sx, sy); self.HP = tf_point(HP, sx, sy); self.K = tf_point(K, sx, sy)
        self.foot0 = tf_point(foot0_fl, sx, sy)

    def fk(self, s, p, f):
        T = _rot_about(AX["splay"], s, self.SP) @ _rot_about(AX["pitch"], p, self.HP) @ _rot_about(AX["knee"], f, self.K)
        return (T @ np.append(self.foot0, 1.0))[:3]

    def _jac(self, q, eps=1e-4):
        base = self.fk(*q); J = np.zeros((3, 3))
        for i, d in enumerate(np.eye(3) * eps):
            J[:, i] = (self.fk(*(q + d)) - base) / eps
        return J

    def _descend(self, seed, target, iters, tol, lam):
        q = _clip(np.array(seed, float)); e = np.linalg.norm(target - self.fk(*q))
        for _ in range(iters):
            if e < tol:
                break
            err = target - self.fk(*q); step = np.linalg.solve(self._jac(q).T @ self._jac(q) + lam * np.eye(3), self._jac(q).T @ err)
            a = 1.0
            for _bt in range(24):
                qn = _clip(q + a * step); en = np.linalg.norm(target - self.fk(*qn))
                if en < e:
                    q, e = qn, en; break
                a *= 0.5
            else:
                break
        return q, e

    def ik(self, target, seed=(0.0, 0.0, 0.0), iters=200, tol=1e-2, lam=1e-2, warm=False):
        """Body-frame foot target (mm) -> URDF joint angles (deg). Backtracking DLS.

        warm=True: descend ONLY from `seed` (stay on the current solution branch — required for gait
        continuity so a leg never teleports between IK branches). warm=False: multi-restart for the
        globally-best pose (for static poses / IK from cold)."""
        target = np.asarray(target, float)
        if warm:
            return self._descend(seed, target, iters, tol, lam)
        seeds = [seed, (0, 0, 0), (0, 40, -70), (0, -40, -30), (-20, 20, -90), (20, 60, -40)]
        best = None
        for s in seeds:
            q, e = self._descend(s, target, iters, tol, lam)
            if best is None or e < best[1]:
                best = (q, e)
            if e < tol:
                break
        return best[0], float(best[1])

def default_foot0():
    """Zero-pose FL foot contact point (mm) = lowest point of the boot STL."""
    try:
        import trimesh
        m = trimesh.load(os.path.join(os.path.dirname(__file__), "..", "stl", "sm3sg90_boot_TPU.stl"))
        return m.vertices[np.argmin(m.vertices[:, 2])].astype(float)
    except Exception:
        return np.array([92.2, 55.2, -183.8])

def make_legs(foot0_fl=None):
    f0 = default_foot0() if foot0_fl is None else foot0_fl
    return {n: Leg(n, sx, sy, f0) for n, (sx, sy) in CORNERS.items()}

if __name__ == "__main__":
    legs = make_legs()
    print("zero-pose feet (mm):")
    for n, lg in legs.items():
        print(f"  {n}: {np.round(lg.fk(0, 0, 0), 1)}")
    fl = legs["fl"]
    tgt = fl.fk(-6, 35, -63)
    q, r = fl.ik(tgt)
    print("FL IK round-trip:", np.round(q, 1), "residual mm:", round(r, 3))
