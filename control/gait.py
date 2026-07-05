"""Static CRAWL gait for robodog + a PyBullet dynamics runner (gravity, ground, contacts).

One foot swings at a time (duty>=0.75) while the body sways laterally so the CoM stays over the
triangle of the three planted feet. Foot trajectories are body-frame; per-leg IK -> joint targets
streamed with POSITION_CONTROL. run() measures real forward travel and whether it stays upright.
"""
import os
import numpy as np
import pybullet as p
import pybullet_data
import kinematics as K

HERE = os.path.dirname(os.path.abspath(__file__))
URDF = os.path.join(HERE, "robodog.urdf")

class Crawl:
    def __init__(self, legs, height=120.0, width=72.0, x_reach=10.0,
                 stride=40.0, lift=28.0, duty=0.8, sway=26.0):
        self.legs = legs
        self.height, self.stride, self.lift, self.duty, self.sway = height, stride, lift, duty, sway
        self.p0 = {}
        for n, lg in legs.items():
            sx = 1 if lg.sx > 0 else -1; sy = 1 if lg.sy > 0 else -1
            self.p0[n] = np.array([sx * (abs(lg.HP[0]) + x_reach), sy * width, -height])
        # lateral-alternating swing order L,R,L,R -> a clean cosine body sway
        self.offset = {"fl": 0.0, "fr": 0.25, "rl": 0.5, "rr": 0.75}

    def targets(self, phase):
        sway_y = self.sway * np.cos(4 * np.pi * phase)   # lean away from the swinging side
        out = {}
        for n in self.legs:
            base = self.p0[n].copy(); ph = (phase - self.offset[n]) % 1.0
            if ph < self.duty:                            # stance: push foot back -> body forward
                fr = ph / self.duty; xo = self.stride * (0.5 - fr); zo = 0.0
            else:                                         # swing: lift + carry forward
                fr = (ph - self.duty) / (1 - self.duty); xo = self.stride * (-0.5 + fr); zo = self.lift * np.sin(np.pi * fr)
            out[n] = base + np.array([xo, sway_y, zo])
        return out

    def joints(self, phase, seeds, warm=True):
        tg = self.targets(phase); js = {}
        for n, lg in self.legs.items():
            q, _ = lg.ik(tg[n], seed=seeds.get(n, (0, 30, -70)), warm=warm); js[n] = q
        return js

def _cam_png(path):
    view = p.computeViewMatrixFromYawPitchRoll([0.2, 0, 0.05], 0.8, 50, -20, 0, 2)
    proj = p.computeProjectionMatrixFOV(50, 1.0, 0.05, 4)
    img = p.getCameraImage(440, 440, view, proj, renderer=p.ER_TINY_RENDERER, lightDirection=[1, 1, 2], shadow=1)[2]
    return np.reshape(img, (440, 440, 4))[:, :, :3].astype("uint8")

def run(params=None, cycles=4, filmstrip=None, walk=True):
    params = dict(params or {}); period = params.pop("period", 2.4)
    legs = K.make_legs(); gait = Crawl(legs, **params)
    p.connect(p.DIRECT); p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81); p.setTimeStep(1 / 240.0)
    plane = p.loadURDF("plane.urdf"); p.changeDynamics(plane, -1, lateralFriction=1.2)
    z0 = gait.height / 1000.0 + 0.006
    rid = p.loadURDF(URDF, [0, 0, z0], flags=p.URDF_USE_INERTIA_FROM_FILE | p.URDF_MAINTAIN_LINK_ORDER)
    jn = {p.getJointInfo(rid, i)[1].decode(): i for i in range(p.getNumJoints(rid))}
    for i in range(p.getNumJoints(rid)):
        p.changeDynamics(rid, i, lateralFriction=1.4)

    def command(js, force=0.6):
        for n, q in js.items():
            for j, a in zip(("splay", "pitch", "knee"), q):
                p.setJointMotorControl2(rid, jn[f"{n}_{j}"], p.POSITION_CONTROL, np.radians(a), force=force, maxVelocity=9)

    seeds = {n: (0, 40, -85) for n in legs}
    stand = gait.joints(0.0, seeds, warm=False); seeds = stand  # cold solve once -> a stable branch
    for n in legs:  # snap to stand, then settle under gravity
        for j, a in zip(("splay", "pitch", "knee"), stand[n]):
            p.resetJointState(rid, jn[f"{n}_{j}"], np.radians(a))
    for _ in range(240):
        command(stand); p.stepSimulation()
    x_start = p.getBasePositionAndOrientation(rid)[0][0]

    steps = int(cycles * period * 240); shots = []
    do_shots = filmstrip is not None
    shot_at = set(int(k * steps / 5) for k in range(6)) if do_shots else set()
    fell = False
    for s in range(steps):
        phase = ((s / 240.0 / period) % 1.0) if walk else 0.0
        js = gait.joints(phase, seeds, warm=True); seeds = js; command(js); p.stepSimulation()
        pos, orn = p.getBasePositionAndOrientation(rid)
        roll, pitch, _ = p.getEulerFromQuaternion(orn)
        if pos[2] < 0.05 or abs(roll) > 1.1 or abs(pitch) > 1.1:
            fell = True
        if s in shot_at:
            shots.append(_cam_png(None))
    pos, orn = p.getBasePositionAndOrientation(rid)
    travel = (pos[0] - x_start) * 1000.0
    roll, pitch, yaw = np.degrees(p.getEulerFromQuaternion(orn))
    if filmstrip and shots:
        from PIL import Image
        sheet = Image.new("RGB", (440 * len(shots), 440), (245, 245, 245))
        for i, a in enumerate(shots):
            sheet.paste(Image.fromarray(a), (i * 440, 0))
        sheet.save(os.path.join(HERE, filmstrip))
    p.disconnect()
    return {"travel_mm": round(travel, 1), "final_z_mm": round(pos[2] * 1000, 1),
            "roll_deg": round(roll, 1), "pitch_deg": round(pitch, 1), "yaw_deg": round(yaw, 1), "fell": fell}

if __name__ == "__main__":
    print("STAND-only stability (no gait):")
    print(" ", run(cycles=3, filmstrip="_stand_dyn.png", walk=False))
    print("CRAWL walk (default params):")
    print(" ", run(cycles=5, filmstrip="_walk.png", walk=True))
