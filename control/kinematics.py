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

# --- CAD constants GENERATED from dog14.py (v9 @ 200mm) ; reference (FL) leg, BODY datum ---
# 2026-07-27: regenerated from the v9 direct-drive leg (legflat_v9: XC=106, HIPZ=10, KZ=-60, femur 70,
# tibia ball foot) placed by dog14 (yout -DX=6, wheelbase XS=-17.35, YSHIFT=7.2, seat DY=2.1, hip lift
# HPDZ=0.3984). Hip/knee sit at body x = 106-DX+XS = 82.65; hip z = 10+HPDZ = 10.3984; knee z = -60+HPDZ
# = -59.6016 (femur K-HP = 70). Splay axis (world X through y27.2,z10.3984) is unchanged from dog13.
# T_COXA / T_LEG map the RAW-datum exported STLs (coxablock / legflat_v9 export in the leg's own frame,
# NOT yout'd) into this body datum -- gen_urdf applies them so the meshes land on the joint points.
# To regenerate after a CAD change: rerun scratchpad/dump_v9_axes.py (SKIP_GATES dog14 dump).
SP = np.array([0.0,  27.2,  10.3984])
HP = np.array([82.65, 0.0,  10.3984])
K  = np.array([82.65, 0.0, -59.6016])
T_COXA = np.array([-23.35, 7.2, 0.0])   # coxa STL (raw) -> body datum, FL corner
T_LEG  = np.array([-23.35, 9.3, 0.0])   # femur/tibia STL (raw, HPDZ already in mesh z) -> body datum, FL
AX = {"splay": np.array([1.0, 0, 0]), "pitch": np.array([0, 1.0, 0]), "knee": np.array([0, 1.0, 0])}

CORNERS = {"fl": (1, 1), "fr": (1, -1), "rl": (-1, 1), "rr": (-1, -1)}

# CAD physical limits are for the FL leg; the mirror corners flip sense, so the URDF/IK use
# generous symmetric limits and the collision validator (poses.py) enforces real feasibility.
# v9 knee flex is one-directional POSITIVE (f>0 folds the foot rear+up about K); usable travel ~0-70 (MG90S).
LIM_PHYS_FL = {"splay": (-45.0, 45.0), "pitch": (-90.0, 90.0), "knee": (0.0, 70.0)}
# URDF/IK limits = physical servo travel, kept SYMMETRIC where the mirror corners flip sign (the one-way knee
# stop and the coupled frame-clearance envelope below are enforced by in_working_envelope + poses.validate(),
# not by these box limits). Tightened from the old +/-55/100/95 which let the sim command impossible travel.
LIM = {"splay": (-45.0, 45.0), "pitch": (-90.0, 90.0), "knee": (-70.0, 70.0)}
# Collision-free envelope, verified by dog14's FreeCAD leg-vs-frame gate (2026-07-28): the frame clears the leg
# for the full gait box AND for static toe-in, but NOT for large negative splay combined with pitch (worst
# 708 mm3 at splay-25/pitch+45/knee70). The constraint is COUPLED (neg-splay only safe near neutral pitch), so
# it is a rule, not a box -- gait/explore must keep commands inside it, with poses.validate() (the frame
# collision mesh) as the geometric backstop.
# The collision-free envelope is COUPLED, not a box: the frame clips the leg only when POSITIVE pitch meets
# toed-in (negative) splay (sp-25/ph+20 already ~20mm3; sp-25/ph+45 = 677). With pitch<=0 the leg swings AWAY
# from the frame, so splay +/-25, pitch down to -58 and knee to 70 are all clear -- verified 0.00 over the full
# default-gait envelope (dog14 + scratchpad/gate_gaitbox.py, 2026-07-28). The axis bounds below are the outer
# box; in_working_envelope() adds the pitch x splay coupling.
WORKING_ENVELOPE = {
    "gait_box":     {"splay": (-25.0, 25.0), "pitch": (-58.0, 30.0), "knee": (0.0, 70.0)},   # + coupling below
    "toein_static": {"splay": (-45.0, 15.0), "pitch": (-10.0, 10.0), "knee": (0.0, 25.0)},   # toe-in stance only
}

def in_working_envelope(q, box="gait_box"):
    """True if (splay,pitch,knee) deg lies inside a verified collision-free envelope (see WORKING_ENVELOPE).
    gait_box additionally forbids positive pitch while toed-in past splay -15 (that combination clips the frame)."""
    s, pit, k = q
    e = WORKING_ENVELOPE[box]
    if not all(e[j][0] - 1e-6 <= a <= e[j][1] + 1e-6 for j, a in zip(("splay", "pitch", "knee"), (s, pit, k))):
        return False
    if box == "gait_box" and pit > 0.0 and s < -15.0:   # +pitch + toe-in -> leg drives into the frame
        return False
    return True

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
        seeds = [seed, (0, 0, 0), (0, 40, 40), (0, -40, 20), (-20, 20, 55), (20, 60, 30)]
        best = None
        for s in seeds:
            q, e = self._descend(s, target, iters, tol, lam)
            if best is None or e < best[1]:
                best = (q, e)
            if e < tol:
                break
        return best[0], float(best[1])

def default_foot0():
    """Zero-pose FL foot contact point (mm), BODY datum = lowest vertex of the v9 tibia (ball foot
    integral) read in its RAW export datum, shifted into the body datum by T_LEG."""
    try:
        import trimesh
        m = trimesh.load(os.path.join(os.path.dirname(__file__), "..", "stl", "sm3sg90_v9_tibia.stl"))
        raw = m.vertices[np.argmin(m.vertices[:, 2])].astype(float)   # raw-datum toe (ball bottom)
        return raw + T_LEG
    except Exception:
        return np.array([82.65, 41.5, -150.6016])

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
