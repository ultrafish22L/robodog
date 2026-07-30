"""Generate robodog.urdf from the CAD joint axes (kinematics.py) + the exported STLs.

Runs in the sim venv (numpy + trimesh) — no FreeCAD. The 17 watertight STLs + the SP/HP/K
axis constants are the entire interface between the mechanical design and the simulator.

Construction: every link frame is world-aligned (identity orientation) with its origin at that
joint's rotation point, corner-transformed by dog13's tf() (mirror-Y for the two chiral corners,
180-Z spin for the rear pair). Chiral corners use the *_mir STL, placed by the proper remainder of
the reflection. Joint axes are world X (splay) / Y (pitch,knee); per-leg direction sign is carried
in kinematics.LEG_SIGN, so all axis *lines* are exact and only the sense flips per corner.
Inertials come from trimesh mass-properties on the watertight meshes (density tunable for infill).
"""
import os
import numpy as np
import trimesh
import kinematics as K

HERE = os.path.dirname(os.path.abspath(__file__))
STL = os.path.abspath(os.path.join(HERE, "..", "stl"))
OUT = os.path.join(HERE, "robodog.urdf")
MM = 0.001                       # mm -> m
DENSITY = 400.0                  # kg/m^3 effective (ABS ~1050 x ~0.35 infill; tune to a real weigh-in)
INERTIA_FLOOR = 1e-9             # kg*m^2 isotropic floor: regularizes the near-planar bones + un-modelled hardware

LEGS = [("fl", 1, 1), ("fr", 1, -1), ("rl", -1, 1), ("rr", -1, -1)]

def T(t=(0, 0, 0), R=np.eye(3)):
    M = np.eye(4); M[:3, :3] = R; M[:3, 3] = t; return M

RZ180 = np.diag([-1.0, -1.0, 1.0])

def corner_pt(p, sx, sy):
    """Apply dog13 tf() to a point: mirror-Y then 180-Z (matching the code's order)."""
    q = np.array(p, float)
    if (sx > 0 and sy < 0) or (sx < 0 and sy > 0):
        q = np.array([q[0], -q[1], q[2]])
    if sx < 0:
        q = RZ180[:3, :3] @ q
    return q

def mat_to_xyzrpy(M):
    """4x4 -> (xyz, rpy) with URDF fixed-axis roll-pitch-yaw (R = Rz(y)Ry(p)Rx(r))."""
    t = M[:3, 3]; R = M[:3, :3]
    pitch = np.arctan2(-R[2, 0], np.hypot(R[0, 0], R[1, 0]))
    if abs(np.cos(pitch)) > 1e-8:
        roll = np.arctan2(R[2, 1], R[2, 2]); yaw = np.arctan2(R[1, 0], R[0, 0])
    else:
        roll = 0.0; yaw = np.arctan2(-R[0, 1], R[1, 1])
    return t, np.array([roll, pitch, yaw])

def mesh_world_placement(sx, sy, mirror_plane_m, T_body):
    """Proper 4x4 that places the (base or _mir) STL at its v9 zero-pose BODY spot (meters).

    The v9 STLs are exported in the leg's own RAW datum (coxablock/legflat_v9 do NOT apply dog14's
    yout+wheelbase), so T_body = the FL export->body translation (m) is applied here. Non-chiral
    corners (FL, RR) use the base STL translated by T_body; chiral corners (FR, RL) use the _mir STL
    (a Y-reflection of the base about plane y=P) placed by (Tx, -2P - Ty, Tz); rear corners (RL, RR)
    then get the 180-Z spin. P (mirror_plane_m) is derived from the base+_mir bbox centres so it is
    correct whether the export mirrored a part about its OWN centre (coxablock) or about the shared
    femur+tibia centre (legflat_v9). This exactly reproduces dog14's tf() o PX placement of each part."""
    tx, ty, tz = T_body
    is_mir = (sx > 0 and sy < 0) or (sx < 0 and sy > 0)
    if is_mir:
        M = T((tx, -2 * mirror_plane_m - ty, tz))
    else:
        M = T((tx, ty, tz))
    if sx < 0:
        M = T(R=RZ180) @ M
    return M, is_mir

def mirror_plane_y_m(stl_base):
    """Y of the plane the _mir STL was reflected about (m), convention-agnostic: the reflection maps
    the base bbox centre to the _mir bbox centre, so the plane is their midpoint."""
    return 0.5 * (bbox_center_y_m(stl_base) + bbox_center_y_m(stl_base + "_mir"))

def inertial_xml(mesh_link):
    """<inertial> from a trimesh already transformed into the link frame (meters).

    Full-precision (scientific) output: the flat 8mm bones are near-planar, so their principal moments
    nearly obey the lamina identity Iyy=Ixx+Izz; 7-decimal rounding collapsed the real thickness margin
    to equality and PyBullet then rejected the tensor ("Bad inertia tensor"). A tiny isotropic floor
    (INERTIA_FLOOR) also stands in for un-modelled hardware (servo horn, M2 screws, foot pad) and keeps
    every principal axis strictly positive."""
    mesh_link.density = DENSITY
    m = max(mesh_link.mass, 1e-4)
    c = mesh_link.center_mass
    I = mesh_link.moment_inertia + np.eye(3) * INERTIA_FLOOR
    return (f'    <inertial>\n'
            f'      <origin xyz="{c[0]:.6f} {c[1]:.6f} {c[2]:.6f}"/>\n'
            f'      <mass value="{m:.6e}"/>\n'
            f'      <inertia ixx="{I[0,0]:.6e}" ixy="{I[0,1]:.6e}" ixz="{I[0,2]:.6e}"'
            f' iyy="{I[1,1]:.6e}" iyz="{I[1,2]:.6e}" izz="{I[2,2]:.6e}"/>\n'
            f'    </inertial>\n')

def geom_xml(stl_name, origin_M, rgba):
    xyz, rpy = mat_to_xyzrpy(origin_M)
    o = f'<origin xyz="{xyz[0]:.6f} {xyz[1]:.6f} {xyz[2]:.6f}" rpy="{rpy[0]:.6f} {rpy[1]:.6f} {rpy[2]:.6f}"/>'
    mesh = f'<mesh filename="{STL}/{stl_name}.stl" scale="{MM} {MM} {MM}"/>'.replace("\\", "/")
    v = f'    <visual>\n      {o}\n      <geometry>{mesh}</geometry>\n      <material name="{stl_name}_m"><color rgba="{rgba}"/></material>\n    </visual>\n'
    c = f'    <collision>\n      {o}\n      <geometry>{mesh}</geometry>\n    </collision>\n'
    return v + c

def link_xml(link_name, parts, link_origin_world_M):
    """parts = [(stl_name, mesh_world_M, rgba)]. link_origin_world_M = link frame in world (meters)."""
    inv_link = np.linalg.inv(link_origin_world_M)
    body = ""
    merged = None
    for stl_name, mesh_world_M, rgba in parts:
        origin_in_link = inv_link @ mesh_world_M
        body += geom_xml(stl_name, origin_in_link, rgba)
        tm = trimesh.load(f"{STL}/{stl_name}.stl"); tm.apply_scale(MM); tm.apply_transform(origin_in_link)
        merged = tm if merged is None else (merged + tm)
    return f'  <link name="{link_name}">\n{body}{inertial_xml(merged)}  </link>\n'

def joint_xml(name, parent, child, origin_xyz_m, axis, lim_deg):
    lo, hi = np.radians(lim_deg[0]), np.radians(lim_deg[1])
    return (f'  <joint name="{name}" type="revolute">\n'
            f'    <parent link="{parent}"/>\n    <child link="{child}"/>\n'
            f'    <origin xyz="{origin_xyz_m[0]:.6f} {origin_xyz_m[1]:.6f} {origin_xyz_m[2]:.6f}"/>\n'
            f'    <axis xyz="{axis[0]} {axis[1]} {axis[2]}"/>\n'
            f'    <limit lower="{lo:.4f}" upper="{hi:.4f}" effort="2.0" velocity="8.0"/>\n'
            f'  </joint>\n')

def bbox_center_y_m(stl_name):
    tm = trimesh.load(f"{STL}/{stl_name}.stl")
    return float(tm.bounds.mean(axis=0)[1]) * MM

def build():
    x = ['<?xml version="1.0"?>\n<robot name="robodog">\n']
    # base = frame, world-aligned at origin (frame STL is already in the body datum)
    x.append(link_xml("base_link", [("sm3sg90_v9_frame", np.eye(4), "0.35 0.37 0.40 1")], np.eye(4)))
    Tc = K.T_COXA * MM   # coxa/cover export->body (m)
    Tl = K.T_LEG * MM    # femur/tibia export->body (m)
    for name, sx, sy in LEGS:
        mir = "_mir" if ((sx > 0 and sy < 0) or (sx < 0 and sy > 0)) else ""
        SPc, HPc, Kc = (corner_pt(p, sx, sy) * MM for p in (K.SP, K.HP, K.K))
        tib_stl = "sm3sg90_v9_tibia" + mir           # v9: one tibia (ball foot integral), no front/rear split, no boot
        Mcoxa, _ = mesh_world_placement(sx, sy, mirror_plane_y_m("sm3sg90_coxa"), Tc)
        Mcover, _ = mesh_world_placement(sx, sy, mirror_plane_y_m("sm3sg90_coxa_cover"), Tc)
        Mfem, _ = mesh_world_placement(sx, sy, mirror_plane_y_m("sm3sg90_v9_femur"), Tl)
        Mtib, _ = mesh_world_placement(sx, sy, mirror_plane_y_m("sm3sg90_v9_tibia"), Tl)
        # link frames (world-aligned, origin at joint point)
        Lcoxa, Lfem, Ltib = T(SPc), T(HPc), T(Kc)
        x.append(link_xml(f"{name}_coxa",
                          [("sm3sg90_coxa" + mir, Mcoxa, "0.55 0.57 0.60 1"),
                           ("sm3sg90_coxa_cover" + mir, Mcover, "0.66 0.54 0.38 1")], Lcoxa))
        x.append(link_xml(f"{name}_femur", [("sm3sg90_v9_femur" + mir, Mfem, "0.90 0.78 0.20 1")], Lfem))
        x.append(link_xml(f"{name}_tibia", [(tib_stl, Mtib, "0.28 0.30 0.34 1")], Ltib))
        x.append(joint_xml(f"{name}_splay", "base_link", f"{name}_coxa", SPc, (1, 0, 0), K.LIM["splay"]))
        x.append(joint_xml(f"{name}_pitch", f"{name}_coxa", f"{name}_femur", HPc - SPc, (0, 1, 0), K.LIM["pitch"]))
        x.append(joint_xml(f"{name}_knee", f"{name}_femur", f"{name}_tibia", Kc - HPc, (0, 1, 0), K.LIM["knee"]))
    x.append("</robot>\n")
    open(OUT, "w").write("".join(x))
    print("wrote", OUT)
    print("links: 1 base + 4 legs x 3 = 13 ; joints: 12 revolute")

if __name__ == "__main__":
    build()
