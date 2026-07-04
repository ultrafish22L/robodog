import os, math, Mesh, Part, FreeCAD as App, FreeCADGui as Gui
# SM3 leg v4: ROUNDED-KNUCKLE knee. The femur knee-end is a CYLINDRICAL cover
# (pocket of radius RIN concentric with the hinge); the tibia clevis has a rounded
# top that nests inside it. Concentric rounding => rotation is collision-free at ANY
# fold angle (a circle rotating about its own centre sweeps itself). Fold target ~125
# deg (lie-down, per Unitree Go2 ~108deg calf travel) not the old 165.
# Hinge at origin, axis Y. Femur extends -X (knee->hip), tibia extends +X (knee->foot).
SM3 = r"C:\ultrafish\Arduino-Projects-main\Nova-SM3\STL Files\All Files"
S = 0.66; T = 18.0
R = 11.0; CLR = 0.6; RIN = R + CLR          # knuckle radius / fold clearance / cover bore
SH = 10.0                                    # shank thickness (centred blade), Y[-5,5]
v = App.Vector; X, Y, Z, O = v(1, 0, 0), v(0, 1, 0), v(0, 0, 1), v(0, 0, 0)
# --- canted knee (standing-referenced) -------------------------------------------------
# KNEE_CANT tilts the Y hinge toward +X; the cant's zero-offset is set AT the standing knee
# fold (KSTAND) so the STANDING foot stays centered and the shin swings clear only as it
# folds past standing. KNEE_CANT=0 => original Y-hinge model (every STL byte-identical).
KNEE_CANT = 30.0     # deg; canted knee SHIPPED (flat-fold clearance). set 0 for the original Y-hinge model.
KSTAND    = 45.0     # standing knee fold (deg) = cant zero-crossing (must match dog.py KNEE_FOLD)
def _caxis():        # canted hinge axis = Y tilted toward +X by KNEE_CANT
    a = math.radians(KNEE_CANT); return v(math.sin(a), math.cos(a), 0.0)
def _cant(s):        # tilt a knee-coupling feature from the Y hinge onto the canted axis (about Z thru knee O)
    return rot(s, -KNEE_CANT, O, Z) if KNEE_CANT else s

def load(name):
    m = Mesh.Mesh(os.path.join(SM3, name + ".stl")); mat = App.Matrix(); mat.scale(S, S, S)
    m.transform(mat); return m
def _sm(a, w=1):
    n = len(a)
    return [sum(a[max(0, i - w):min(n, i + w + 1)]) / (min(n, i + w + 1) - max(0, i - w)) for i in range(n)]
def envelope(meshes, nbin=30):
    bb = App.BoundBox()
    for m in meshes: bb.add(m.BoundBox)
    x0, x1 = bb.XMin, bb.XMax; span = (x1 - x0) or 1.0; bins = {}
    for m in meshes:
        for p in m.Topology[0]:
            k = min(max(int((p.x - x0) / span * nbin), 0), nbin)
            lo, hi = bins.get(k, (1e9, -1e9)); bins[k] = (min(lo, p.z), max(hi, p.z))
    ks = [k for k in range(nbin + 1) if k in bins]
    xs = [x0 + (k + 0.5) * span / nbin for k in ks]
    zb = _sm([bins[k][0] for k in ks]); zt = _sm([bins[k][1] for k in ks])
    return list(zip(xs, zb, zt))
def kneeZ(env, distal=True):
    e = sorted(env, key=lambda t: t[0], reverse=distal)[:3]
    return sum((zb + zt) / 2 for _, zb, zt in e) / len(e)
def plate(env, thick, dx, dz, y0):
    P = sorted((x + dx, zb + dz, zt + dz) for x, zb, zt in env)
    bot = [v(x, y0, zb) for x, zb, zt in P]; top = [v(x, y0, zt) for x, zb, zt in reversed(P)]
    return Part.Face(Part.makePolygon(bot + top + [bot[0]])).extrude(v(0, thick, 0))
def cyl(r, h, base, d=Y): return Part.makeCylinder(r, h, base, d)
def box(a, b, c, base): return Part.makeBox(a, b, c, base)
def rot(s, deg, ctr=O, ax=Y): s = s.copy(); s.rotate(ctr, ax, deg); return s
def _sect(cx, cy, cz, hy, hz, rc, asym, npt=32):     # asymmetric rounded-rect ring in X=cx plane (femur)
    rc = max(0.5, min(rc, hy * 0.9, hz * 0.9)); per = npt // 4
    cs = [(hy - rc, hz - rc, 0.0), (-(hy - rc), hz - rc, math.pi / 2),
          (-(hy - rc), -(hz - rc), math.pi), (hy - rc, -(hz - rc), 3 * math.pi / 2)]
    pts = []
    for gy, gz, s in cs:
        for k in range(per):
            a = s + (math.pi / 2) * k / per
            yy = gy + rc * math.cos(a); zz = gz + rc * math.sin(a)
            yy = yy * (1.0 + asym) if yy > 0 else yy * (1.0 - 0.4 * asym)   # +Y outboard dome, -Y cupped
            pts.append(v(cx, cy + yy, cz + zz))
    pts.append(pts[0]); return Part.makePolygon(pts)
def _ell(cx, cy, cz, hy, hz, npt=28):                # ellipse ring tall-Z/narrow-Y (tibia airfoil)
    pts = [v(cx, cy + hy * math.cos(2 * math.pi * i / npt), cz + hz * math.sin(2 * math.pi * i / npt)) for i in range(npt)]
    pts.append(pts[0]); return Part.makePolygon(pts)

# CUSTOM leg envelopes -- SM3 is a proportional REFERENCE only, captured here as plain constants so
# NO external mesh is loaded. (x, z_bottom, z_top) silhouette sections; these drive FDX/FDZ/TDX/TDZ
# (below), HZ + RPROX (leg5) and the femur/tibia spines. Edit these numbers to reshape the leg.
FENV = [(-50.0183, 9.0994, 25.2067), (-46.5689, 7.6393, 25.703), (-43.1194, 5.526, 26.5197), (-39.67, 2.9828, 26.4092), (-36.2206, 1.412, 24.9346), (-32.7712, 0.4318, 23.0677), (-29.3218, 0.0028, 20.7744), (-25.8723, 0.0007, 19.7221), (-22.4229, 0.0001, 18.8984), (-18.9735, 0.0001, 18.8408), (-15.5241, 0.0, 18.7655), (-12.0746, 0.0004, 18.9055), (-8.6252, 0.0009, 18.9798), (-5.1758, 0.0022, 18.4318), (-1.7264, 0.0031, 17.2683), (1.7231, 0.0046, 17.1236), (5.1725, 0.0035, 17.3267), (8.6219, 0.0392, 18.8871), (12.0713, 0.9547, 19.4903), (15.5208, 1.9981, 20.4076), (18.9702, 2.9308, 20.5744), (22.4196, 3.0703, 20.7165), (25.869, 3.1073, 20.8177), (29.3185, 3.2828, 20.8784), (32.7679, 3.4606, 20.9955), (36.2173, 2.9474, 21.2674), (39.6667, 3.1128, 21.5424), (43.1162, 2.5144, 20.9136), (46.5656, 4.5707, 19.1875), (50.015, 7.1869, 16.1594), (53.4644, 9.8261, 14.5926)]
TENV = [(-41.0339, 0.0156, 15.5178), (-38.1883, 0.0107, 17.3735), (-35.3426, 0.0026, 22.5257), (-32.497, 0.0005, 23.9609), (-29.6513, 0.0013, 25.3984), (-26.8056, 0.0059, 23.9642), (-23.96, 0.0104, 22.5186), (-21.1143, 0.0133, 21.0842), (-18.2686, 0.0086, 21.073), (-15.423, 0.0059, 21.0737), (-12.5773, 0.0021, 21.0634), (-9.7316, 0.0023, 18.4462), (-6.886, 0.0034, 15.8196), (-4.0403, 0.0305, 13.1517), (-1.1946, 0.0878, 13.0898), (1.651, 0.236, 12.9686), (4.4967, 0.4232, 12.7996), (7.3424, 0.6863, 12.5674), (10.188, 1.0333, 12.2909), (13.0337, 1.4639, 11.9587), (15.8794, 1.8004, 11.5381), (18.725, 1.8668, 11.3954), (21.5707, 1.236, 11.9595), (24.4164, 0.5922, 12.6865), (27.262, 0.0429, 13.2471), (30.1077, 0.0374, 13.246), (32.9534, 0.0242, 13.2438), (35.799, 0.0093, 13.2397), (38.6447, 0.0008, 13.2328), (41.4903, 3.7452, 12.5639), (44.336, 5.6174, 12.2287)]
FDX, FDZ = -max(x for x, _, _ in FENV), -kneeZ(FENV, True)              # femur knee -> origin
TDX, TDZ = 7 - min(x for x, _, _ in TENV), -kneeZ(TENV, False)         # tibia shank starts X=7

def _interp(P, xi):                                  # linear-interp (zb,zt) at xi over sorted (x,zb,zt) list
    for j in range(len(P) - 1):
        x0, z0b, z0t = P[j]; x1, z1b, z1t = P[j + 1]
        if x0 <= xi <= x1:
            f = (xi - x0) / ((x1 - x0) or 1.0); return z0b + f * (z1b - z0b), z0t + f * (z1t - z0t)
    return (P[-1][1], P[-1][2]) if xi >= P[-1][0] else (P[0][1], P[0][2])

def femur_loft():                                    # Spot-styled lofted thigh (replaces the flat plate base)
    PF = sorted((x + FDX, zb + FDZ, zt + FDZ) for x, zb, zt in FENV)
    xh = PF[0][0]; N = 26; wires = []
    for i in range(N + 1):
        xi = xh + (0 - xh) * i / N
        zb, zt = _interp(PF, xi); zc = (zb + zt) / 2.0
        hz = max((zt - zb) / 2.0, 6.5)                               # half-height from the SM3 silhouette (smooth taper)
        t = (xi - xh) / (-xh or 1.0)                                 # 0 at hip -> 1 at knee
        wy = 18.0 + 2.0 * t                                         # STRAIGHT taper, hip 18 (clears coxa) -> knee 20
        wires.append(_sect(xi, 0.0, zc, wy / 2.0, hz, 3.5, 0.0))    # smooth symmetric rounded-rect, flatish faces
    try:
        sh = Part.makeLoft(wires, True, True).removeSplitter()           # ruled: no B-spline overshoot
        sol = sh.Solids; sh = max(sol, key=lambda s: s.Volume) if len(sol) > 1 else sh
        if sh.isValid() and len(sh.Solids) == 1: return sh
    except Exception as e:
        print("  femur_loft FALLBACK:", e)
    return plate(FENV, T, FDX, FDZ, -T / 2)

def tibia_blade():                                   # Spot-styled lofted shank (replaces the flat plate base)
    PT = sorted((x + TDX, zb + TDZ, zt + TDZ) for x, zb, zt in TENV)
    xtip = PT[-1][0]; X0 = 12.0; N = 18; wires = []
    for i in range(N + 1):
        xi = X0 + (xtip - X0) * i / N
        zb, zt = _interp(PT, xi); zc = (zb + zt) / 2.0
        hz = max((zt - zb) / 2.0, 4.5)                              # half-height from the SM3 silhouette (smooth taper)
        t = (xi - X0) / (xtip - X0)                                 # 0 at knee -> 1 at ankle
        wy = 5.5 * (1 - t) + 4.0 * t                               # SLIM flat blade 11mm knee -> 8mm ankle (real BD-Spot shin; ref/n2,n3)
        wires.append(_sect(xi, 0.0, zc, wy, hz, min(2.6, wy * 0.8), 0.0))   # tall-Z (silhouette) thin flat-faced blade

    try:
        bl = Part.makeLoft(wires, True, True).removeSplitter()           # ruled: no B-spline overshoot
        sol = bl.Solids; bl = max(sol, key=lambda s: s.Volume) if len(sol) > 1 else bl
        if bl.isValid() and len(bl.Solids) == 1: return bl
    except Exception as e:
        print("  tibia_blade FALLBACK:", e)
    return plate(TENV, SH, TDX, TDZ, -SH / 2)

def servo():                                  # SG90, body centre-Y, output +Y at hinge
    return box(23, 22.5, 12.1, v(-17, -13.5, -6)).fuse(cyl(2.7, 4.5, v(0, 9, 0)))

def tibia(fold=0.0):
    shank = tibia_blade()                                               # Spot-styled lofted blade (was flat plate)
    rootcap = cyl(R, SH, v(0, -SH / 2, 0)).common(box(R + 1, SH, R + 2, v(0, -SH / 2, -2)))  # rounded TOP (Z>=-2)
    hubarm = box(8, 6, 14, v(0, 9, -7)); hub = cyl(7, 6, v(0, 9, 0)).cut(cyl(2.5, 8, v(0, 8, 0)))  # thicker heat-set hub wall (audit)
    pegarm = box(8, 6, 14, v(0, -19, -7)); peg = cyl(3, 6, v(0, -19, 0))   # idle peg, fits femur boss hole
    yoke = box(5, 34, 14, v(7, -19, -7))                               # joins prongs to shank, X[7,12]
    tib = shank.fuse(rootcap).fuse(hubarm).fuse(hub).fuse(pegarm).fuse(peg).fuse(yoke).removeSplitter()
    return rot(tib, fold)

def femur():
    sh = femur_loft()                                                   # Spot-styled lofted blade (was flat plate)
    cradle = _cant(box(24, 22.5, T, v(-18, -13.5, -9))); cav = _cant(box(24, 24, 14, v(-17.5, -14, -7)))   # knee-servo pocket, canted
    pegb = _cant(cyl(6, 7, v(0, -19, 0)).cut(cyl(3.4, 9, v(0, -20, 0))))     # hole accepts idle peg (r3); canted to hinge
    fem = sh.fuse(cradle).cut(cav).fuse(pegb)
    # ROUNDED COVER = cylindrical pocket about the hinge, centre-Y band, upper half only
    # (leaves Z<-2 femur as floor -> stays 1 solid; tibia rounded top nests + rotates inside). _cant tilts it onto the canted axis.
    cover = _cant(cyl(RIN, 11, v(0, -5.5, 0)).common(box(40, 11, 40, v(-20, -5.5, -2))))   # band 11 -> 3.5mm side walls (audit)
    # upper-edge relief: bigger-radius scoop (Z>3) where the folded shank lands -> extends clean fold to ~120 deg
    relief = _cant(cyl(20, 11, v(0, -5.5, 0)).common(box(70, 11, 70, v(-35, -5.5, 3))))
    # lower-front fold relief: concentric scoop about the hinge so the round shank folds into a clean rounded channel
    relief2 = _cant(cyl(13.0, T + 2, v(0, -T/2 - 1, 0)).common(box(40, T + 2, 40, v(-2, -T/2 - 1, -40))))
    fem = fem.cut(cover).cut(relief).cut(relief2).removeSplitter()
    fs = fem.Solids
    return max(fs, key=lambda s: s.Volume) if len(fs) > 1 else fem

def peg(): return cyl(1.5, 32, v(0, -21, 0))

def report(n, s):
    try:
        b = s.BoundBox
        print("  %-11s X=%6.1f Y=%5.1f Z=%6.1f vol=%8.0f solids=%d" %
              (n, b.XLength, b.YLength, b.ZLength, s.Volume, len(s.Solids)))
    except Exception as e: print("  %-11s ERR %s" % (n, e))

def main():
    fem, sv, tib0 = femur(), servo(), tibia(0)
    print("=== ROUNDED-KNUCKLE KNEE (SM3 x0.66, SG90, R=%.0f) ===" % R)
    report("femur", fem); report("tibia(0)", tib0); report("servo", sv)
    print("  femur solids vol:", [round(s.Volume) for s in fem.Solids])
    print("=== KNEE FOLD (tibia vs rounded-cover femur) ===")
    for a in (0, 30, 60, 90, 110, 125, 140):
        print("  fold %3d deg  interf=%8.1f mm^3" % (a, fem.common(tibia(-a)).Volume))

    nm = "sm3leg4"
    if nm in list(App.listDocuments()): App.closeDocument(nm)
    doc = App.newDocument(nm)
    foot = load("SM3_Foot"); fb = foot.BoundBox
    foot.translate((7 + (max(x for x, _, _ in TENV) - min(x for x, _, _ in TENV))) - (fb.XMin + fb.XLength / 2),
                   -(fb.YMin + fb.YLength / 2), TDZ - (fb.ZMin + fb.ZLength / 2))
    for n, s, c in [("femur", fem, (0.55, 0.6, 0.55)), ("servo", sv, (0.15, 0.15, 0.17)),
                    ("tibia_ext", tib0, (0.3, 0.6, 0.9)), ("tibia_fold", tibia(-120), (0.9, 0.4, 0.4)),
                    ("hinge_peg", peg(), (0.95, 0.6, 0.1))]:
        o = doc.addObject("Part::Feature", n); o.Shape = s; o.ViewObject.ShapeColor = c
        if n == "tibia_fold": o.ViewObject.Transparency = 60
    doc.addObject("Mesh::Feature", "foot").Mesh = foot
    doc.recompute()
    Gui.activeDocument().activeView().viewFront(); Gui.SendMsgToActiveView("ViewFit")

main()
