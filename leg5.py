import FreeCAD as App, FreeCADGui as Gui, Mesh, os
# Full SG90 SM3 leg = leg4 knee (rounded-knuckle) + HIP-PITCH coupling + COXA, per the
# vetted workflow specs. Hip axis is Y-parallel, so the hip joint is a RELOCATED COPY of
# the solved knee: coxa plays the femur role (holds hip-pitch servo-2), the femur PROXIMAL
# plays the tibia role (driven clevis: +Y spline hub, -Y idle journal, rounded knuckle).
_c = open(r"C:/ultrafish/3dprint/sm3-sg90/leg4.py").read()
exec(_c[:_c.rfind("\nmain()")])                      # leg4 defs/globals, WITHOUT running its main()
_femur4 = femur                                       # keep leg4's knee femur

# ---- hip-pitch frame (computed from FENV, not hard-coded) ----
HX = min(x for x, _, _ in FENV) + FDX + 3.0           # axis 3mm inboard of proximal tip
HZ = kneeZ(FENV, distal=False)                        # proximal silhouette centreline
_pb = sorted(FENV, key=lambda t: t[0])[:3]
RPROX = sum((zt - zb) for _, zb, zt in _pb) / len(_pb) / 2.0   # proximal half-height (like R=11 at knee)
HP = v(HX, 0.0, HZ)
def at(p): return HP.add(p)

def servo2():                                         # hip-pitch servo: output +Y at HP, body -X into coxa
    return box(23, 22.5, 12.1, at(v(-17, -13.5, -6))).fuse(cyl(2.7, 4.5, at(v(0, 9, 0))))

def femur():                                          # OVERRIDE: leg4 knee femur + proximal driven clevis
    fem = _femur4()
    hub = cyl(7, 6, at(v(0, 9, 0)))                                    # +Y spline hub (bore cut AFTER the fuse -- the arm plate re-filled it)
    harm = box(9, 6, 14, at(v(0, 9, -7)))                              # +Y arm -> plate
    peg = cyl(3, 5, at(v(0, -19, 0)))                                  # -Y idle journal stub (1mm shorter: the real SG90 body reaches y-13.5)
    parm = box(9, 5, 14, at(v(0, -19, -7)))
    yoke = box(6, 34, 14, at(v(7, -19, -7)))                          # joins prongs, fuses into femur plate
    cap = cyl(RPROX, T, at(v(0, -T / 2, 0))).common(box(RPROX + 2, T, 2 * RPROX + 4, at(v(0, -T / 2, -(RPROX + 2)))))  # +X full rounded knuckle
    fem = fem.fuse(hub).fuse(harm).fuse(peg).fuse(parm).fuse(yoke).fuse(cap)
    fem = fem.cut(cyl(2.5, 4.7, at(v(0, 8.9, 0)))).cut(cyl(1.2, 8.0, at(v(0, 8.0, 0))))   # spline pocket + M2 clamp-screw shoulder through hub AND arm (audit #22)
    # round the proximal knuckle corners to the hip arc over X[HX-40, HX+6] (the coxa-overlap zone; spares the yoke at X>HX+6)
    proxcut = box(46, T + 2, 60, at(v(-40, -T / 2 - 1, -30))).cut(cyl(RPROX, T + 2, at(v(0, -T / 2 - 1, 0))))
    fem = fem.cut(proxcut).removeSplitter()
    fs = fem.Solids
    return max(fs, key=lambda s: s.Volume) if len(fs) > 1 else fem

def coxa():                                           # femur-role at HP + hip-roll barrel (-X)
    cradle = box(24, 22.5, T, at(v(-18, -13.5, -9)))
    cav = box(24, 24, 14, at(v(-17.5, -14, -7)))
    pegb = cyl(6, 7, at(v(0, -19, 0))).cut(cyl(3.4, 9, at(v(0, -20, 0))))   # idle boss receives femur peg
    cx = cradle.cut(cav).fuse(pegb)
    cover = cyl(RPROX + CLR, 20, at(v(0, -10, 0))).common(box(40, 20, 120, at(v(-2, -10, -60))))  # knuckle cover pocket (+X, full Z arc)
    cx = cx.cut(cover)
    barrel = cyl(8, 16, at(v(-14, 0, 0)), v(-1, 0, 0))                # o16 hip-roll cup, axis -X
    barrel = barrel.cut(cyl(2.5, 9, at(v(-22, 0, 0)), v(-1, 0, 0)))   # spline bore -X (servo-1)
    arm = box(8, 16, 16, at(v(-16, -8, -8)))                         # bridge cradle -> barrel
    cx = cx.fuse(arm).fuse(barrel).removeSplitter()
    fs = cx.Solids
    return max(fs, key=lambda s: s.Volume) if len(fs) > 1 else cx

def main5():
    cx, s2, fem, s3, tib0 = coxa(), servo2(), femur(), servo(), tibia(0)
    print("=== FULL LEG: knee(rounded-knuckle) + hip-pitch + coxa  (SM3 x0.66, SG90) ===")
    print("  HX=%.1f HZ=%.1f RPROX=%.1f" % (HX, HZ, RPROX))
    for n, s in [("coxa", cx), ("servo2", s2), ("femur", fem), ("servo3", s3), ("tibia(0)", tib0)]:
        report(n, s)
    print("=== HIP-PITCH FOLD (coxa vs femur about HP) ===")
    for a in (-45, -30, -15, 0, 15, 30, 45):
        print("  pitch %+4d deg  interf=%8.1f mm^3" % (a, cx.common(rot(fem, a, HP, Y)).Volume))
    print("=== KNEE FOLD (unchanged) ===")
    for a in (0, 60, 110):
        print("  fold %4d deg  interf=%8.1f mm^3" % (a, fem.common(tibia(-a)).Volume))

    nm = "sm3leg5"
    if nm in list(App.listDocuments()): App.closeDocument(nm)
    doc = App.newDocument(nm)
    foot = load("SM3_Foot"); fb = foot.BoundBox
    foot.translate((7 + (max(x for x, _, _ in TENV) - min(x for x, _, _ in TENV))) - (fb.XMin + fb.XLength / 2),
                   -(fb.YMin + fb.YLength / 2), TDZ - (fb.ZMin + fb.ZLength / 2))
    for n, s, c in [("coxa", cx, (0.6, 0.55, 0.5)), ("servo2", s2, (0.15, 0.15, 0.17)),
                    ("femur", fem, (0.55, 0.6, 0.55)), ("servo3", s3, (0.15, 0.15, 0.17)),
                    ("tibia", tib0, (0.3, 0.6, 0.9))]:
        o = doc.addObject("Part::Feature", n); o.Shape = s; o.ViewObject.ShapeColor = c
    doc.addObject("Mesh::Feature", "foot").Mesh = foot
    doc.recompute()
    Gui.activeDocument().activeView().viewAxonometric(); Gui.SendMsgToActiveView("ViewFit")

main5()
