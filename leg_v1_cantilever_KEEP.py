# SM3-SG90 "mini Nova" — parametric leg + (SM3-derived) body, FreeCAD 1.1 Part workbench
#
# Redesign brief: SM3 rounded body + proportions @0.6 scale, SG90 servos, snap-together,
#   tuned for a 0.4 mm nozzle / 0.08 mm layers.  Two structural fixes vs SM3:
#     (1) CLOSED 6-sided servo pockets (SM3 leaves a big face open -> weak)  -> box + snap cap
#     (2) OFFSET CANTILEVER knee so the tibia folds nearly flat past the femur (lie-down pose)
#
# Run:  exec(open(r"C:/ultrafish/3dprint/sm3-sg90/build.py").read())
#
# Leg local frame: femur along +X (hip-pitch axis Y at X=0, knee axis Y at X=Lf),
#   Z = up (sagittal plane = XZ).  Joint axes = Y, so fold happens in the XZ plane.
#   Servo canonical: output shaft +Z at origin, top face at Z=0, body hangs -Z, gear end +X.

import FreeCAD as App, Part, math
from FreeCAD import Vector as v

OUT = "C:/ultrafish/3dprint/sm3-sg90"
X, Y, Z, O = v(1,0,0), v(0,1,0), v(0,0,1), v(0,0,0)
S = 0.6  # global scale of SM3 link proportions (servos are real-size, do NOT scale)

P = dict(
    # ---- SG90 envelope (real, fixed) ----
    bL=22.7, bW=12.1, bH=22.5, shaft_off=6.0, hub_d=5.4, hub_h=4.0,
    tab_span=32.0, tab_th=2.4, tab_below=4.5, wire_w=6.0, wire_h=4.5, wire_len=6.5,
    # ---- print / fit (0.4 nozzle, 0.08 layer) ----
    clr=0.25,            # servo pocket clearance per side
    wall=2.0,            # 5 perimeters @0.4 -> closed-pocket walls
    capgap=0.20,         # snap-cap fit gap
    # ---- spline coupling (heat-set metal spline + M2 screw; only hardware) ----
    insert_d=5.0, insert_h=4.5, boss_d=12.0, boss_h=6.0,
    screw_d=2.2, screw_head_d=4.4, screw_head_h=2.0,
    # ---- link lengths (~0.6 x SM3 proportions, femur:tibia ~ 1:1.08) ----
    coxa_len=30.0, femur_len=64.0, tibia_len=70.0,
    leg_y=12.0,          # slim in-plane thickness (Y) of leg shafts
    knee_gap=1.2,        # extra Y clearance, folded tibia vs femur
)


# ---------------------------------------------------------------- primitives
def sg90_cutter(clear=0.0):
    """Solid SG90 envelope; output axis at origin, shaft +Z, top face Z=0, body -Z."""
    c = clear
    L, W, H = P['bL']+2*c, P['bW']+2*c, P['bH']+2*c
    x0 = -(P['bL']-P['shaft_off']) - c
    body = Part.makeBox(L, W, H, v(x0, -W/2, -H))
    hub  = Part.makeCylinder(P['hub_d']/2+c, P['hub_h']+c, O, Z)            # output hub
    bc   = P['shaft_off'] - P['bL']/2
    Tl   = P['tab_span']+2*c
    tz   = -P['tab_below']
    tabs = Part.makeBox(Tl, W, P['tab_th']+2*c, v(bc-Tl/2, -W/2, tz-P['tab_th']-c))
    wire = Part.makeBox(P['wire_len']+c, P['wire_w']+2*c, P['wire_h']+2*c,
                        v(x0-P['wire_len'], -(P['wire_w']/2+c), -P['wire_h']/2-c))
    return body.fuse(hub).fuse(tabs).fuse(wire).removeSplitter()

def boss(at, axis, h=None):
    """Driven coupling boss (spline-insert bore + counterbored M2), base at `at`, growing along axis."""
    h = h or P['boss_h']
    b = Part.makeCylinder(P['boss_d']/2, h, at, axis)
    ins = Part.makeCylinder(P['insert_d']/2, P['insert_h']+0.01, at-axis*0.005, axis)
    shk = Part.makeCylinder(P['screw_d']/2,  h+0.02, at-axis*0.01, axis)
    hd  = Part.makeCylinder(P['screw_head_d']/2, P['screw_head_h']+0.01,
                            at+axis*(h-P['screw_head_h']), axis)
    return b.cut(ins).cut(shk).cut(hd)

def rot(s, axis, deg, ctr=O):
    s = s.copy(); s.rotate(ctr, axis, deg); return s

def bb_box(bb, wall):
    return Part.makeBox(bb.XLength+2*wall, bb.YLength+2*wall, bb.ZLength+2*wall,
                        v(bb.XMin-wall, bb.YMin-wall, bb.ZMin-wall))

def safe_fillet(s, r, minlen=None):
    minlen = minlen or 2.2*r
    try:
        es = [e for e in s.Edges if e.Length > minlen]
        return s.makeFillet(r, es) if es else s
    except Exception:
        return s


# ---------------------------------------------------------------- closed pocket
def place_knee_servo(Lf, clear=0.0):
    """Knee servo (servo 3) at the femur distal end: shaft +Y (outboard), body -Y (inboard)."""
    c = rot(sg90_cutter(clear), X, -90)      # shaft +Z -> +Y, body -Z -> -Y
    c.translate(v(Lf, 0, 0))                 # shaft axis through knee point (Lf,0,0)
    return c

def closed_pocket(Lf):
    """Return (housing_solid, cavity_cutter, cap_solid) for the knee servo.
    Housing fully encloses the servo (all 6 faces walled) EXCEPT a drop-in opening on +Z,
    which a snap cap closes -> the SM3 'open side' is gone.  Shaft exits a hole in +Y wall."""
    sv  = place_knee_servo(Lf, 0.0)
    cav = place_knee_servo(Lf, P['clr'])
    bb  = sv.BoundBox
    w   = P['wall']
    house = bb_box(bb, w)
    # drop-in opening on +Z (top): remove the top wall over the cavity footprint
    topcut = Part.makeBox(bb.XLength+P['clr']*2, bb.YLength+P['clr']*2, w+0.2,
                          v(bb.XMin-P['clr'], bb.YMin-P['clr'], bb.ZMax))
    # boss clearance hole through +Y wall (driven boss passes through to seat on servo top face)
    hub = Part.makeCylinder(P['boss_d']/2+P['clr'], w+1.0, v(Lf,0,0), Y).translate(v(0,bb.YMax-0.5,0))
    house = house.cut(cav).cut(topcut).cut(hub)
    # snap cap: plate that drops into the +Z opening, with a lip
    cap = Part.makeBox(bb.XLength+P['clr']*2-2*P['capgap'], bb.YLength+P['clr']*2-2*P['capgap'], w,
                       v(bb.XMin-P['clr']+P['capgap'], bb.YMin-P['clr']+P['capgap'], bb.ZMax-w))
    lip = Part.makeBox(bb.XLength+2*w-0.4, bb.YLength+2*w-0.4, w*0.6,
                       v(bb.XMin-w+0.2, bb.YMin-w+0.2, bb.ZMax))
    cap = cap.fuse(lip).removeSplitter()
    return house, cav, cap


# ---------------------------------------------------------------- leg parts
def make_femur(side=1):
    """Flat-ish femur in XZ plane: proximal coupling boss (to hip-pitch servo 2) at X=0,
    slim shaft, knee servo housing at distal X=Lf.  side=+1 left, -1 right (mirror Y)."""
    Lf = P['femur_len']
    house, cav, cap = closed_pocket(Lf)
    bb = place_knee_servo(Lf, 0.0).BoundBox
    wo = bb.YMax + P['wall']            # femur +Y wall outer face
    # slim shaft beam from proximal to the knee housing; +Y face kept inboard of the tibia plane
    bh = P['bH'] + 2*P['wall']          # ~ knee housing height in Z
    y_in = -(P['leg_y']/2 + P['wall'])  # inboard face (toward servo body)
    beam = Part.makeBox(Lf+6, wo - y_in, bh*0.78, v(-3, y_in, -bh*0.30))
    beam = safe_fillet(beam, 3.0)
    fem = house.fuse(beam).removeSplitter().cut(cav)
    # clear the +Y knee coupling zone so the tibia boss seats on the servo top face (Y=0)
    bc = Part.makeCylinder(P['boss_d']/2+P['clr'], wo+2, v(Lf, -1, 0), Y)
    fem = fem.cut(bc)
    # proximal coupling boss: mounts on coxa's hip-pitch servo shaft (+Y from coxa) -> boss faces -Y
    pb = boss(v(0, -(P['leg_y']/2+P['wall']), 0), Y*-1)
    fem = fem.fuse(pb).removeSplitter()
    if side < 0:
        fem = fem.mirror(O, Y); cap = cap.mirror(O, Y)
    return fem, cap

def make_coxa(side=1):
    """Coxa (hip): closed pocket for the hip-pitch servo (servo 2, shaft +Y -> drives femur),
    plus a hip-roll coupling boss (axis -X) that mounts on the body's hip-roll servo (servo 1).
    Returned translated so the hip-pitch servo sits at the origin (= femur proximal)."""
    Lc = P['coxa_len']
    house, cav, cap = closed_pocket(Lc)
    bb = place_knee_servo(Lc, 0.0).BoundBox
    wo = bb.YMax + P['wall']
    bh = P['bH'] + 2*P['wall']
    y_in = -(P['leg_y']/2 + P['wall'])
    beam = Part.makeBox(Lc+6, wo - y_in, bh*0.78, v(-3, y_in, -bh*0.30))
    beam = safe_fillet(beam, 3.0)
    coxa = house.fuse(beam).removeSplitter().cut(cav)
    bc = Part.makeCylinder(P['boss_d']/2+P['clr'], wo+2, v(Lc, -1, 0), Y)
    coxa = coxa.cut(bc)
    rb = boss(v(0, 0, 0), X*-1, h=P['boss_h']+2)        # hip-roll coupling to body servo 1
    coxa = coxa.fuse(rb).removeSplitter()
    coxa.translate(v(-Lc, 0, 0)); cap.translate(v(-Lc, 0, 0))
    if side < 0:
        coxa = coxa.mirror(O, Y); cap = cap.mirror(O, Y)
    return coxa, cap

def make_tibia(side=1, fold_deg=0.0):
    """Cantilever tibia OFFSET to +Y (outboard) so it folds flat past the femur.
    Couples to knee servo 3 shaft at the knee; fold_deg rotates it about the knee (Y) axis."""
    Lf, Lt = P['femur_len'], P['tibia_len']
    wo = place_knee_servo(Lf, 0.0).BoundBox.YMax + P['wall']   # femur +Y wall outer face
    yo = wo + P['knee_gap'] + P['leg_y']/2                     # tibia plate center (outboard of femur)
    # slim tapered tibia plate from knee (X=Lf) toward the foot (+X), narrowing, in its own +Y plane
    pts = [v(Lf-8, 0, -10), v(Lf+Lt, 0, -4.5), v(Lf+Lt, 0, 4.5), v(Lf-8, 0, 11), v(Lf-8, 0, -10)]
    prof = Part.Face(Part.makePolygon(pts))
    tib = prof.extrude(v(0, P['leg_y'], 0))
    tib.translate(v(0, yo - P['leg_y']/2, 0))
    tib = safe_fillet(tib, 3.0)
    # coupling boss: base on servo top face (Y=0), grows +Y through the wall hole into the plate
    cb = boss(v(Lf, 0, 0), Y, h=yo - P['leg_y']/2 + 2.5)
    tib = tib.fuse(cb).removeSplitter()
    # foot ball-stud at distal (snap mount for the TPU boot)
    foot = Part.makeSphere(5.0, v(Lf+Lt, yo, 0))
    tib = tib.fuse(foot).removeSplitter()
    tib = rot(tib, Y, fold_deg, v(Lf, 0, 0))             # fold about knee axis
    if side < 0:
        tib = tib.mirror(O, Y)
    return tib


# ---------------------------------------------------------------- build + fold test
def report(name, s):
    try:
        bb = s.BoundBox
        print("  %-10s ok  X=%6.1f Y=%6.1f Z=%6.1f vol=%8.0f solids=%d" %
              (name, bb.XLength, bb.YLength, bb.ZLength, s.Volume, len(s.Solids)))
    except Exception as e:
        print("  %-10s FAIL %s" % (name, e))

def main():
    doc = App.newDocument("sm3leg") if "sm3leg" not in App.listDocuments() else App.getDocument("sm3leg")
    for o in list(doc.Objects): doc.removeObject(o.Name)

    coxa, capc = make_coxa(1)
    fem, cap = make_femur(1)
    tib0   = make_tibia(1, 0.0)        # extended
    print("=== PARTS ===")
    report("coxa", coxa); report("cap_coxa", capc)
    report("femur", fem); report("cap", cap); report("tibia(0)", tib0)

    print("=== KNEE FOLD (tibia vs femur interference; ~0 = clears) ===")
    for ang in (0, 90, 140, 160, 170):
        t = make_tibia(1, ang)
        iv = fem.common(t).Volume
        print("  fold %3d deg  interf=%7.1f mm^3" % (ang, iv))

    # show extended + folded(165) overlay
    folded = make_tibia(1, 165)
    for nm, sh, col in [("coxa", coxa, (0.55,0.6,0.55)),
                        ("cap_coxa", capc, (0.9,0.7,0.2)),
                        ("femur", fem, (0.6,0.6,0.65)),
                        ("cap", cap, (0.9,0.7,0.2)),
                        ("tibia_ext", tib0, (0.3,0.6,0.9)),
                        ("tibia_fold", folded, (0.9,0.4,0.4))]:
        ob = doc.addObject("Part::Feature", nm); ob.Shape = sh
        try: ob.ViewObject.ShapeColor = col
        except Exception: pass
    doc.recompute()
    import FreeCADGui as Gui
    Gui.activeDocument().activeView().viewFront()
    Gui.SendMsgToActiveView("ViewFit")

main()
