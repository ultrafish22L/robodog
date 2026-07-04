# Robodog — custom 3D-printable SG90 quadruped

A parametric, 3D-printable quadruped robot dog — a **completely custom design** (Boston Dynamics
Spot and the open-source SM3 Spot-Micro are visual **references only**; no external mesh or part
is used), driven by **12 SG90 micro-servos** (3 per leg). Geometry is generated
in **FreeCAD 1.1** through the FreeCAD MCP (`mcp__freecad__execute_code`) — there is no saved
`.FCStd`; running the scripts rebuilds everything.

## Build
Exec `dog.py` inside FreeCAD (via the MCP). It chains `leg7 → leg5 → leg4`, assembles the
4-leg dog, and writes the print-ready parts to `stl/sm3sg90_*.stl`. White-background reference
renders are written to `ref/iter/dog_{side,front,top,iso}.png`.

## Deliverable
Six print-ready STLs for a **Bambu Lab A1** (256³ build volume): `coxa, femur, tibia, boot,
body, head`. Print the **boot** in flexible **TPU**; everything else rigid. A full dog =
4× each leg part + 1 body + 1 head.

## Key files
- `dog.py` — current full-dog assembly (main deliverable).
- `leg4/5/7.py` — the verified leg + joint chain `dog.py` builds on.
- `robodog.md` — the design bible **and restart doc**: read it to resume a session (visual target, joint mechanism, rules, gotchas, session history).
- `ref/` — visual-target photos + reference meshes (large scan meshes are gitignored).
