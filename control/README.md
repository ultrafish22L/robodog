# robodog — control software

All-Python control + simulation for the SG90 quadruped, **generated from the CAD** (`../dog13.py`).
Runs headless in [PyBullet]; the same kinematics + gait code is intended to run on a Raspberry Pi +
PCA9685 driving the 12 SG90 servos (sim → real by design — only the bottom driver swaps).

## Setup
```
py -3.13 -m venv .venv
.venv\Scripts\python -m pip install pybullet numpy trimesh pillow
```

## Modules
| file | what |
|---|---|
| `kinematics.py` | per-leg 3-DOF FK/IK in the body frame; anchors (SP/HP/K) come from the CAD. `Leg.ik(target, warm=)` |
| `gen_urdf.py` | writes `robodog.urdf` from the axis constants + the 17 exported STLs + trimesh inertials |
| `poses.py` | named static poses (stand/sit/lie/paw) as body-frame foot targets + a self-collision validator |
| `gait.py` | static **crawl** gait + a PyBullet dynamics runner (`run()`) that measures forward travel + uprightness |
| `explore.py` | random → hill-climb search over gait params, scored by sim rollout; writes `best_gait.json` |
| `viz.py` | load the URDF, verify FK vs kinematics, render poses |

## Quick start
```
.venv\Scripts\python gen_urdf.py     # build robodog.urdf
.venv\Scripts\python viz.py          # FK check + zero/stand renders
.venv\Scripts\python poses.py        # pose library + validation montage
.venv\Scripts\python gait.py         # crawl walk in sim (filmstrip)
.venv\Scripts\python explore.py      # search for a better gait
```

## Verified (2026-07-04)
- Per-leg FK matches PyBullet to 0.00 mm on all 4 legs.
- Poses solve at ~0 mm IK residual; collision validator catches forced self-collisions.
- Crawl gait walks **forward, upright, without falling** under gravity.

## Roadmap
- [x] 1 model + FK/IK  [x] 2 poses + validator  [x] 3 crawl gait  [~] 4 gait search
- [ ] 5 glTF/three.js web viewer  [ ] 6 `hardware.py` — Pi/PCA9685 PWM + servo calibration
