"""Load robodog.urdf in PyBullet (headless), verify per-leg FK against kinematics, render poses."""
import os
import numpy as np
import pybullet as p
import kinematics as K

HERE = os.path.dirname(os.path.abspath(__file__))
URDF = os.path.join(HERE, "robodog.urdf")

def connect():
    p.connect(p.DIRECT)
    rid = p.loadURDF(URDF, useFixedBase=True, flags=p.URDF_USE_INERTIA_FROM_FILE | p.URDF_MAINTAIN_LINK_ORDER)
    jn = {p.getJointInfo(rid, i)[1].decode(): i for i in range(p.getNumJoints(rid))}
    return rid, jn

def set_leg(rid, jn, leg, s, pit, f):
    for j, ang in zip(("splay", "pitch", "knee"), (s, pit, f)):
        p.resetJointState(rid, jn[f"{leg}_{j}"], np.radians(ang))

def foot_world(rid, jn, lg):
    ls = p.getLinkState(rid, jn[f"{lg.name}_knee"], computeForwardKinematics=True)
    pos, orn = ls[4], ls[5]
    Rm = np.array(p.getMatrixFromQuaternion(orn)).reshape(3, 3)
    return (np.array(pos) + Rm @ ((lg.foot0 - lg.K) * 0.001)) * 1000.0

def render(rid, tag):
    view = p.computeViewMatrixFromYawPitchRoll([0, 0, -0.08], 0.55, 40, -25, 0, 2)
    proj = p.computeProjectionMatrixFOV(50, 1.0, 0.05, 3)
    img = p.getCameraImage(720, 720, view, proj, renderer=p.ER_TINY_RENDERER, lightDirection=[1, 1, 2], shadow=1)[2]
    from PIL import Image
    Image.fromarray(np.reshape(img, (720, 720, 4))[:, :, :3].astype("uint8")).save(os.path.join(HERE, f"_viz_{tag}.png"))
    return f"_viz_{tag}.png"

if __name__ == "__main__":
    legs = K.make_legs()
    rid, jn = connect()
    print("per-leg FK check (pybullet vs kinematics.Leg.fk):")
    worst = 0.0
    for n, lg in legs.items():
        for ang in [(0, 0, 0), (-6, 35, -63), (20, -30, -90), (-40, 60, -20)]:
            set_leg(rid, jn, n, *ang)
            err = np.linalg.norm(foot_world(rid, jn, lg) - lg.fk(*ang)); worst = max(worst, err)
        set_leg(rid, jn, n, 0, 0, 0)
    print(f"  worst FK error across all 4 legs x 4 angles: {worst:.4f} mm")

    print("\nzero-pose render ->", render(rid, "zero"))

    # a STAND pose, defined by body-frame foot targets: each foot pulled up + under its hip.
    print("\nstand via per-leg IK (body-frame targets):")
    for n, lg in legs.items():
        tgt = lg.foot0 + np.array([0, 0, 55.0])   # raise the foot 55mm -> bend the knee, body stands
        tgt[0] = lg.HP[0]                          # foot under the hip fore-aft
        q, r = lg.ik(tgt, seed=(0, 30, -70))
        set_leg(rid, jn, n, *q)
        print(f"  {n}: q={np.round(q,1)} residual={r:.2f}mm foot={np.round(foot_world(rid,jn,lg),1)}")
    print("stand render ->", render(rid, "stand"))
