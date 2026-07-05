"""Named static poses for robodog, defined by body-frame foot targets + per-leg IK, with a
PyBullet self-collision / limit validator. A Pose is {leg: (splay,pitch,knee) deg}.

Body frame (mm): X forward, Y left, Z up; base_link origin at the frame centre.
"""
import os
import numpy as np
import pybullet as p
import kinematics as K

HERE = os.path.dirname(os.path.abspath(__file__))
URDF = os.path.join(HERE, "robodog.urdf")

def targets_to_pose(targets, legs, seed=(0, 30, -70)):
    """{leg: foot_xyz(mm)} -> {leg:(s,p,f)}, and the worst IK residual (mm)."""
    pose, worst = {}, 0.0
    for n, lg in legs.items():
        q, r = lg.ik(targets[n], seed=seed); pose[n] = tuple(np.round(q, 2)); worst = max(worst, r)
    return pose, worst

def _sign(lg):
    return (1 if lg.sx > 0 else -1, 1 if lg.sy > 0 else -1)

# --- pose builders: return {leg: foot target (mm, body frame)} ---
def stand_targets(legs, height=150.0, x_reach=12.0, width=58.0):
    t = {}
    for n, lg in legs.items():
        sx, sy = _sign(lg)
        t[n] = np.array([sx * (abs(lg.HP[0]) + x_reach), sy * width, -height])
    return t

def sit_targets(legs, front_h=150.0, rear_h=70.0, width=58.0):
    """rear crouched + tucked under, front standing tall -> haunch-down sit."""
    t = {}
    for n, lg in legs.items():
        sx, sy = _sign(lg)
        if sx > 0:   # front legs stand
            t[n] = np.array([sx * (abs(lg.HP[0]) + 12), sy * width, -front_h])
        else:        # rear legs fold under, body low at the back
            t[n] = np.array([sx * (abs(lg.HP[0]) - 55), sy * width, -rear_h])
    return t

def lie_targets(legs, height=88.0, width=66.0):
    """belly-down: body low, feet pulled in under the hips (kept within the leg's reach)."""
    t = {}
    for n, lg in legs.items():
        sx, sy = _sign(lg)
        t[n] = np.array([sx * (abs(lg.HP[0]) - 8), sy * width, -height])
    return t

def paw_targets(legs, height=150.0, width=58.0, paw_leg="fl"):
    """stand on three legs, front-left lifted forward+up in a wave."""
    t = stand_targets(legs, height=height, width=width)
    lg = legs[paw_leg]; sx, sy = _sign(lg)
    t[paw_leg] = np.array([sx * (abs(lg.HP[0]) + 70), sy * (width + 10), -height + 95])
    return t

POSES = {"stand": stand_targets, "sit": sit_targets, "lie": lie_targets, "paw": paw_targets}

# --- validation ---
def load(self_collision=True):
    p.connect(p.DIRECT)
    flags = p.URDF_USE_INERTIA_FROM_FILE | p.URDF_MAINTAIN_LINK_ORDER
    if self_collision:
        flags |= p.URDF_USE_SELF_COLLISION
    rid = p.loadURDF(URDF, useFixedBase=True, flags=flags)
    jn = {p.getJointInfo(rid, i)[1].decode(): i for i in range(p.getNumJoints(rid))}
    return rid, jn

def apply(rid, jn, pose):
    for n, (s, pit, f) in pose.items():
        for j, a in zip(("splay", "pitch", "knee"), (s, pit, f)):
            p.resetJointState(rid, jn[f"{n}_{j}"], np.radians(a))

def validate(rid, jn, pose, penetration_tol=0.5):
    """Return (ok, min_gap_mm, limit_violations). min_gap<0 => self-penetration."""
    apply(rid, jn, pose)
    p.performCollisionDetection()
    pts = p.getContactPoints(rid, rid)
    min_gap = min([c[8] for c in pts], default=1.0) * 1000.0  # contactDistance (m) -> mm
    viol = []
    for n, (s, pit, f) in pose.items():
        for j, a in zip(("splay", "pitch", "knee"), (s, pit, f)):
            lo, hi = K.LIM_PHYS_FL[j]  # report against the *physical* FL range as a soft flag
            if not (min(lo, -hi) - 1 <= abs(a) or lo - 1 <= a <= hi + 1):
                viol.append(f"{n}.{j}={a}")
    ok = min_gap > -penetration_tol
    return ok, round(min_gap, 2), viol

def render(rid, tag):
    view = p.computeViewMatrixFromYawPitchRoll([0, 0, -0.08], 0.55, 40, -25, 0, 2)
    proj = p.computeProjectionMatrixFOV(50, 1.0, 0.05, 3)
    img = p.getCameraImage(560, 560, view, proj, renderer=p.ER_TINY_RENDERER, lightDirection=[1, 1, 2], shadow=1)[2]
    return np.reshape(img, (560, 560, 4))[:, :, :3].astype("uint8")

if __name__ == "__main__":
    legs = K.make_legs()
    rid, jn = load(self_collision=True)
    tiles = []
    from PIL import Image, ImageDraw
    print("pose            IK_resid   self-collision(min gap mm)   verdict")
    for name, builder in POSES.items():
        pose, resid = targets_to_pose(builder(legs), legs)
        ok, gap, viol = validate(rid, jn, pose)
        print(f"  {name:12s}  {resid:6.2f}mm   {gap:8.1f}                  {'OK' if ok and resid<2 else 'CHECK'}")
        im = Image.fromarray(render(rid, name)); d = ImageDraw.Draw(im); d.text((6, 6), name, fill=(0, 0, 0))
        tiles.append(im)
    sheet = Image.new("RGB", (560 * 2, 560 * 2), (245, 245, 245))
    for i, im in enumerate(tiles):
        sheet.paste(im, ((i % 2) * 560, (i // 2) * 560))
    sheet.save(os.path.join(HERE, "_poses.png")); print("\nmontage -> _poses.png")
