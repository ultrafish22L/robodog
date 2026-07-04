import FreeCAD as App, FreeCADGui as Gui, Mesh, os, math, Part
# v10 mini-Spot leg: RATIO-CORRECT (the real fix). Measured real-Spot thigh:tibia = 1:1.7
# (tibia is the LONG bone); the old code was ~1:0.78 (femur ~103, tibia ~80). Fix = shorten
# the femur to 46mm (tibia stays ~78) -> 1:1.7. Reuses leg5's VERIFIED joints verbatim
# (rounded-knuckle DIRECT-DRIVE knee, hip-pitch=relocated-knee, coxa barrel). Only the
# skeleton lengths + the two loft section shapes change (skin-on-skeleton). TH=65 (torque-gated).
_c = open(r"C:/ultrafish/3dprint/sm3-sg90/leg5.py").read()
exec(_c[:_c.rfind("\nmain5()")])              # leg5 (=> leg4) defs/globals, WITHOUT main5()

FEMUR_LEN = 42.0                               # short+fat thigh vs long+slim tibia -> ratio ~1:1.9 (max Spot contrast)

# --- re-lay the hip joint for the shortened femur. HZ/RPROX describe the PROXIMAL
#     cross-section (preserved), so only the POSITION HX moves inboard. ---
HX = -FEMUR_LEN + 3.0
HP = v(HX, 0.0, HZ)
def at(p): return HP.add(p)

def femur_loft():        # fat ~1:1 TEARDROP thigh; ENDS frozen narrow so the verified joint cuts clear
    PF = sorted((x + FDX, zb + FDZ, zt + FDZ) for x, zb, zt in FENV)
    xh0 = PF[0][0]; N = 20; wires = []               # B-spline loft -> genuinely smooth surface (few sections needed)
    KF, HF = -13.0, -36.0                            # knee/hip end-freeze regions
    for i in range(N + 1):
        f = i / N                                    # 0 hip -> 1 knee
        xi = -FEMUR_LEN * (1.0 - f)                  # new X: -L -> 0
        xs = xh0 * (1.0 - f)                         # sample the original silhouette proportionally
        zb, zt = _interp(PF, xs); zc = (zb + zt) / 2.0
        if HF < xi < KF:                             # teardrop bulge ONLY in mid window; smooth cosine bump -> no B-spline overshoot
            mid = (HF + KF) / 2.0; half = (KF - HF) / 2.0
            w = 0.5 * (1.0 + math.cos(math.pi * (xi - mid) / half))
        else:
            w = 0.0
        hz = 11.0 + 4.0 * w                          # smooth teardrop depth (no silhouette wobble): 11 ends -> 15 mid
        hy = 8.0 + 10.0 * w                          # narrow joint-safe ends -> 18 FAT mid (swallows the cradle/clevis)
        wires.append(_ell(xi, 0.0, zc, hy, hz, 44))  # smooth round teardrop tube
    try:
        sh = Part.makeLoft(wires, True, False).removeSplitter()
        sol = sh.Solids; sh = max(sol, key=lambda s: s.Volume) if len(sol) > 1 else sh
        if sh.isValid() and len(sh.Solids) == 1: return sh
    except Exception as e:
        print("  femur_loft FALLBACK:", e)
    return plate(FENV, T, FDX, FDZ, -T / 2)

def tibia_blade():       # slim flat BLADE: SLIM at the knee waist -> mild widen toward the foot (Spot)
    PT = sorted((x + TDX, zb + TDZ, zt + TDZ) for x, zb, zt in TENV)
    xtip = PT[-1][0]; X0 = 12.0; N = 20; wires = []   # B-spline loft -> smooth curved blade, no ribs
    for i in range(N + 1):
        xi = X0 + (xtip - X0) * i / N
        zb, zt = _interp(PT, xi); zc = (zb + zt) / 2.0
        hz = max((zt - zb) / 2.0 * 1.1, 7.0)         # moderately tall fore-aft -> flat blade on edge
        t = (xi - X0) / (xtip - X0)                  # 0 knee -> 1 foot
        hy = 2.8 + 1.2 * t                           # slim long blade: 5.6mm knee-waist -> 8mm foot
        zc += -2.0 * math.sin(math.pi * t)           # gentle forward bow -> Spot sweep (not dead-straight)
        wires.append(_sect(xi, 0.0, zc, hy, hz, min(2.6, hy * 0.8), 0.0, 48))
    try:
        bl = Part.makeLoft(wires, True, False).removeSplitter()
        sol = bl.Solids; bl = max(sol, key=lambda s: s.Volume) if len(sol) > 1 else bl
        if bl.isValid() and len(bl.Solids) == 1: return bl
    except Exception as e:
        print("  tibia_blade FALLBACK:", e)
    return plate(TENV, SH, TDX, TDZ, -SH / 2)

def main7():
    cx, s2, fem, s3, tib0 = coxa(), servo2(), femur(), servo(), tibia(0)
    L = []
    def pr(s): L.append(s); print(s)
    pr("=== v10 LEG  femur=%.0f tibia~78  direct-drive knee ===" % FEMUR_LEN)
    pr("  HX=%.1f HZ=%.1f RPROX=%.1f" % (HX, HZ, RPROX))
    for n, s in [("coxa", cx), ("femur", fem), ("tibia(0)", tib0)]:
        b = s.BoundBox
        pr("  %-9s X=%6.1f Y=%5.1f Z=%6.1f vol=%8.0f solids=%d" % (n, b.XLength, b.YLength, b.ZLength, s.Volume, len(s.Solids)))
    fl = fem.BoundBox.XLength; tl = tib0.BoundBox.XLength
    pr("  femur X-len=%.0f tibia X-len=%.0f -> thigh:tibia = 1:%.2f" % (fl, tl, tl / fl))
    pr("=== KNEE FOLD (must be ~0 at rest 0deg) ===")
    for a in (0, 60, 110):                            # reuse prebuilt tib0 (rot, not rebuild) -> fast
        pr("  fold %4d  interf=%8.1f" % (a, fem.common(rot(tib0, -a)).Volume))
    pr("=== HIP-PITCH (low at neutral) ===")
    for a in (-45, 0, 45):
        pr("  pitch %+4d  interf=%8.1f" % (a, cx.common(rot(fem, a, HP, Y)).Volume))
    try: open(r"C:/ultrafish/3dprint/sm3-sg90/ref/iter/gates.txt", "w").write("\n".join(L) + "\n")
    except Exception: pass

    RDIR = r"C:/ultrafish/3dprint/sm3-sg90/ref/iter"; os.makedirs(RDIR, exist_ok=True)
    YEL, DRK, GRY, REA = (.92, .72, .10), (.18, .18, .20), (.55, .52, .48), (.85, .70, .25)
    VIEWS = [("side", "viewFront"), ("top", "viewTop"), ("front", "viewRight"), ("iso", "viewAxonometric")]
    def shot(docname, parts, fname, views=VIEWS):
        if docname in list(App.listDocuments()): App.closeDocument(docname)
        doc = App.newDocument(docname)
        for n, s, c in parts:
            if isinstance(s, Mesh.Mesh):
                o = doc.addObject("Mesh::Feature", n); o.Mesh = s
            else:
                o = doc.addObject("Part::Feature", n); o.Shape = s
            try: o.ViewObject.ShapeColor = c
            except Exception: pass
            try: o.ViewObject.Deviation = 0.03       # finer tessellation -> smoother render
            except Exception: pass
        doc.recompute()
        gv = Gui.activeDocument().activeView()
        for vn, vf in views:
            getattr(gv, vf)(); Gui.SendMsgToActiveView("ViewFit")
            gv.saveImage(RDIR + "/" + fname + "_" + vn + ".png", 1100, 850, "White")
    # MY leg, extended, 4 ortho views (side = length+height profile; top/front = lateral width; iso)
    shot("sm3leg7x", [("coxa", cx, GRY), ("femur", fem, YEL), ("tibia", tib0, DRK)], "my")
    # MY leg, posed stance (side + iso)
    HPI, KF2 = 26.0, 55.0
    femP = rot(fem, HPI, HP, Y); tibP = rot(tibia(-KF2), HPI, HP, Y)
    shot("sm3leg7p", [("coxa", cx, GRY), ("femur", femP, YEL), ("tibia", tibP, DRK)], "pose",
         [("side", "viewFront"), ("iso", "viewAxonometric")])
    # (real-leg mesh comparison dropped: the generative mesh has one wonky leg; use the n3_1 photo instead)
    print("  renders saved to", RDIR)

main7()
