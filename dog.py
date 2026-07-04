import FreeCAD as App, FreeCADGui as Gui, Part, Mesh, os, math
# ROBODOG -- a COMPLETELY CUSTOM 3D-printable SG90 quadruped. This is our own mechanism and body;
# SM3 (Spot-Micro) and BD Spot are only visual/proportional REFERENCES -- no external mesh or part
# is loaded or printed. FULL DOG assembly: the verified leg mechanism x4 (mirror L/R) on a narrow
# custom body + forward sensor head, posed in a natural stance.
#
# Two upgrades vs leg7:
#   (1) SMOOTH femur skin: analytic-ellipse loft rings (kills leg7's makePolygon "corncob"),
#       same section schedule as leg7 so the verified knee gate still clears. Gate-checked at
#       run time; auto-reverts to leg7's femur if it overshoots into the knee.
#   (2) Whole-leg STANDING pose: pitch the femur ~down about the hip-Y axis, fold the knee,
#       so the leg hangs Spot-like (thigh steep, shin raked, foot under the body).
#
# Run:  exec(open(r"C:/ultrafish/3dprint/sm3-sg90/dog.py").read())

_c = open(r"C:/ultrafish/3dprint/sm3-sg90/leg7.py").read()
exec(_c[:_c.rfind("\nmain7()")])              # leg7 (=>leg5=>leg4) defs/globals, NO main7()
# available now: v,X,Y,Z,O, coxa(), femur(), tibia(fold), servo(), rot(), box(), cyl(),
#                HP,HX,HZ,RPROX, FENV,TENV,FDX,FDZ,TDX,TDZ, R,SH,T, _interp, FEMUR_LEN

OUT  = r"C:/ultrafish/3dprint/sm3-sg90"
RDIR = OUT + "/ref/iter"; os.makedirs(RDIR, exist_ok=True)

# ---------- tunables (the look) -----------------------------------------------
PITCH      = 66.0     # hip-pitch: thigh leans forward (knee ahead of hip), Spot-like
KNEE_FOLD  = 45.0     # sharper knee bend -> shin rakes back so the foot plants under the hip
KNEE_BACK  = True     # all 4 knees point toward the TAIL (SM3/Spot); head stays +X with a normal root — legs flip in X, not the head
KNEE_CANT  = 0.0      # OVERRIDE leg4's 30deg: the knee SERVO shaft must be PARALLEL to the hip-pitch shaft (both +Y) so femur+tibia rotate in the SAME (sagittal) plane. A canted knee axis is mechanically wrong for a single-servo hinge.
BL, BW, BH = 185.0, 72.0, 33.0     # body beam: ~10% longer than 168 (L:W~2.6:1 before rump); H=33 = SM3 x0.66
BR         = 9.0                    # body corner round
RUMP       = 14.0                   # extra body length added at the REAR only (-X) -> fuller hindquarters behind the rear hips (front face stays under the head)
HIP_INSET  = 22.0                   # hips inset from the body ends (smaller -> hips/feet toward the corners)
HIP_OUT    = 6.0                    # coxa sits this far outside the body wall (wider track = planted Spot stance)
HIP_DZ     = 11.0                  # shoulders at TOP of frame (deck level): body hangs below the hips -> max belly freeboard (legs reach below the belly when standing; fold UP clear of it when lying down)
YEL, DRK, GRY = (.95, .74, .10), (.14, .14, .16), (.45, .45, .48)
STL_EXPORT = True     # export the unique printable parts to stl/ (print ONE leg first, then commit)

# --- SM3 leg proportions: measured SM3_Frame femur:tibia = 138.5:129.3 = 1:0.93 (femur LONGER).
#     Our old 54/long-tibia was ~1:1.86 (tibia nearly 2x). Fix = long femur + short tibia -> ~1:0.93. ---
FEMUR_LEN   = 68.0     # hip->knee bone (functional = |HX| = 65); was 54
TIBIA_SCALE = 0.60     # slightly LONGER shank (user) -> femur:tibia ~1:1.1; round full calf with slim knee-neck
# hip-pitch axis AT the femur's top-centre (joints rotate about their geometric centre).
# HX -> proximal tip (no 3mm inboard offset); HZ -> the ROUND femur's proximal centreline
# (leg5's kneeZ(FENV) is in the UNSHIFTED frame -> sat ~11mm above the FDZ-shifted thigh centre).
HX = -FEMUR_LEN
_PFh = sorted((x + FDX, zb + FDZ, zt + FDZ) for x, zb, zt in FENV)
HZ = sum(_interp(_PFh, _PFh[0][0])) / 2.0
HP = v(HX, 0.0, HZ)

# ---------- (1) smooth femur skin --------------------------------------------
def _round_tube(stations):                        # ROUND limb: loft 48-gon rings along a clean spine; station = (x,z,r) round OR (x,z,hy,hz) ellipse
    wires = []
    for st in stations:
        if len(st) == 4: x, z, hy, hz = st
        else: x, z, r = st; hy = hz = r           # hy=hz -> round; 48-gon = smooth look + robust boolean cuts (true B-spline circles make the hip proxcut invalid)
        wires.append(_ell(x, 0.0, z, hy, hz, 48))
    sh = Part.makeLoft(wires, True, False).removeSplitter()       # B-spline loft along length -> smooth taper, no twist
    sol = sh.Solids
    return max(sol, key=lambda s: s.Volume) if len(sol) > 1 else sh

def _smoothstep(t): return t * t * (3.0 - 2.0 * t)

def femur_loft_smooth():                          # ROUND thigh: circular sections on a smooth straight spine + faint muscle swell
    PF = sorted((x + FDX, zb + FDZ, zt + FDZ) for x, zb, zt in FENV)
    xh0 = PF[0][0]
    zb, zt = _interp(PF, 0.0);  zk = (zb + zt) / 2.0           # silhouette centre at knee (~0) -- ENDPOINTS ONLY, no per-station noise
    zb, zt = _interp(PF, xh0);  zh = (zb + zt) / 2.0           # silhouette centre at hip
    N = 22; st = []
    for i in range(N + 1):
        t = i / N; s = _smoothstep(t)                          # 0 knee -> 1 hip
        x = -FEMUR_LEN * t
        z = zk + (zh - zk) * s                                 # CLEAN 2-point spine: zero envelope ripple
        r = 12.6 + 0.8 * s + 0.7 * math.sin(math.pi * t)       # round: 12.6 knee (walls RIN 11.6) -> 13.4 hip, single smooth belly
        st.append((x, z, r))
    return _round_tube(st)

_leg7_femur_loft = femur_loft                     # keep the verified fallback

# ---------- foot + tibia-with-foot -------------------------------------------
# --- CUSTOM foot end: rounded paw toe + lateral boot-snap hole, built from primitives. NO mesh loaded.
#     SM3 was only a proportional reference; this printed toe is entirely our own geometry.
FOOT_PITCH = 17.0          # tilt the foot fwd about the lateral (hinge) axis so the rounded sole sits tangent to the ground in stance (tune for tangency)

def _make_foot_end():
    PT = sorted((x + TDX, zb + TDZ, zt + TDZ) for x, zb, zt in TENV)
    xo = PT[-1][0]; zf = sum(_interp(PT, xo)) / 2.0   # tibia foot tip (max X); ankle centreline Z
    x0 = 12.0; xtip = x0 + (xo - x0) * TIBIA_SCALE     # OUR compressed ankle X (join plane) -- unchanged, so shank length holds
    L, W, H = 22.0, 13.2, 15.0                          # toe length / width / height
    pad = box(L, W, H, v(xtip - 2.0, -W / 2.0, zf - H / 2.0))   # rounded pad on the ankle line, extends +X (toe)
    try:    pad = pad.makeFillet(4.0, pad.Edges)        # soften the box into a paw
    except Exception:
        try: pad = pad.makeFillet(2.0, pad.Edges)
        except Exception: pass
    pad = pad.cut(cyl(2.4, W + 2.0, v(xtip + L - 7.0, -W / 2.0 - 1.0, zf - H * 0.15), Y))   # lateral boot-snap hole near the toe
    foot = rot(pad, FOOT_PITCH, v(xtip, 0.0, zf), Y)   # pitch fwd about the lateral axis -> sole rakes toward ground tangent
    return foot.removeSplitter(), xtip, zf
_FOOT_END, _FOOT_XTIP, _FOOT_ZF = _make_foot_end()
def foot_end(): return _FOOT_END.copy()               # custom foot end, fully placed (at ankle, pitched)

def boot():                                           # CUSTOM snap-on TPU boot: rounded cap, open at the ankle side so it slips onto the toe. PRINT IN FLEXIBLE TPU, separately.
    L, W, H = 20.0, 15.0, 16.0
    outer = box(L, W, H, v(0.0, -W / 2.0, -H / 2.0))
    try: outer = outer.makeFillet(4.0, outer.Edges)
    except Exception: pass
    inner = box(L, W - 3.5, H - 3.5, v(-3.0, -(W - 3.5) / 2.0, -(H - 3.5) / 2.0))   # hollow it, open at the back (-X)
    return outer.cut(inner).removeSplitter()

def tibia_shank_round(xj, hyj, hzj, zcj):          # ROUND shin -> over the last 30% morph the section to the foot-join ellipse (hyj,hzj,@zcj) for a seamless blend
    PT = sorted((x + TDX, zb + TDZ, zt + TDZ) for x, zb, zt in TENV)
    x0 = 12.0
    zb, zt = _interp(PT, x0);  zk = (zb + zt) / 2.0
    N = 24; st = []
    for i in range(N + 1):
        t = i / N; s = _smoothstep(t)                          # 0 knee -> 1 foot join
        x = x0 + (xj - x0) * t
        z = zk + (zcj - zk) * s - 3.0 * math.sin(math.pi * t)  # CLEAN spine, single fwd bow; ends exactly at the foot-join centre
        r = 4.5 + 5.0 * (1.0 - s)                              # round shin: WIDE 9.5 knee -> slim ankle
        b = _smoothstep(max(0.0, (t - 0.7) / 0.3))             # round -> foot ellipse over the last 30%
        st.append((x, z, (1 - b) * r + b * hyj, (1 - b) * r + b * hzj))
    return _round_tube(st)

_TIBF_BASE = None                                 # fold-independent tibia solid; coplanar knee -> tibiaF(fold)=rot(base,fold)
def tibiaF(fold=0.0):
    global _TIBF_BASE
    if _TIBF_BASE is None:                          # build the (fold-independent) shank+foot+clevis ONCE, then just rotate
        foot = foot_end(); xj = _FOOT_XTIP
        foot = foot.common(box(80, 90, 90, v(xj, -45, -45)))   # cut the foot flush at the ankle plane (keep toe side); shank fills x<xj
        slab = foot.common(box(0.8, 90, 90, v(xj, -45, -45))); sb = slab.BoundBox   # foot cross-section AT the join
        hyj = max(sb.YLength / 2.0, 3.0); hzj = max(sb.ZLength / 2.0, 3.0); zcj = sb.Center.z
        shank = tibia_shank_round(xj + 0.8, hyj, hzj, zcj)     # shank ends with the exact foot-join ellipse, 0.8mm into the foot
        rootcap = cyl(R, SH, v(0, -SH/2, 0)).common(box(R+1, SH, R+2, v(0, -SH/2, -2)))   # clevis cap (hinge at origin)
        # AUDIT: the KNEE SG90 (femur cradle) was never placed; the rotating rootcap sweeps THROUGH its body
        # (760..2171 mm3 at every fold). Carve the servo's swept envelope out of the cap -- the knee bearing is
        # the +Y hub + -Y idle peg (as in SM3); the cap survives as a partial nesting arc.
        for q in _pitch_servo_sweep():
            rootcap = rootcap.cut(q)
        rcs = rootcap.Solids                       # keep only the largest surviving cap arc (severed slivers are unsupported)
        rootcap = max(rcs, key=lambda s: s.Volume) if len(rcs) > 1 else (rootcap if rcs else None)
        hubarm = box(8, 6, 14, v(0, 9, -7))
        hub = cyl(7, 6, v(0, 9, 0))                # bore cut AFTER the fuse: the hubarm plate used to re-fill half the bore
        pegarm = box(8, 5, 14, v(0, -19, -7)); peg = cyl(3, 5, v(0, -19, 0))     # -Y idle peg (real SG90 body reaches y-13.5)
        yoke = box(5, 34, 14, v(7, -19, -7))                                     # shank stays in-plane (doglegged clevis)
        for q in _pitch_servo_sweep():             # the knee servo's +X tab & lead stub sweep through the yoke annulus -> slot it
            yoke = yoke.cut(q)
        yks = yoke.Solids
        yoke = max(yks, key=lambda s: s.Volume) if len(yks) > 1 else (yoke if yks else None)
        # one seamless tibia solid: round shin morphs into the real SM3 foot end (with boot-snap hole)
        tib = shank
        for pc in (rootcap, hubarm, hub, pegarm, peg, yoke, foot):
            if pc is not None: tib = tib.fuse(pc)
        tib = tib.cut(cyl(2.5, 4.7, v(0, 8.9, 0))).cut(cyl(1.2, 8.0, v(0, 8.0, 0)))  # spline pocket + M2 shoulder through hub AND arm plate
        try: tib = tib.removeSplitter()
        except Exception: pass
        sols = tib.Solids
        _TIBF_BASE = max(sols, key=lambda q: q.Volume) if len(sols) > 1 else tib
    return rot(_TIBF_BASE, fold)                    # coplanar knee: fold is a plain in-plane rotation of the cached base

# ---- true SG90 (audit #8/#15): the old dummy was tabless with a CENTERED shaft; the real servo has the
# shaft 5.5mm off the length-center, mounting tabs (32.2mm overall, screw holes ~27.6 apart) and a lead stub.
SG_L, SG_W, SG_H, SG_OFF = 22.8, 12.2, 22.5, 5.5   # body length / width / height(bottom->top face) / shaft offset
SG_TE, SG_TT, SG_TH1 = 4.7, 2.5, 7.1               # tab extension per end / plate thickness / plate-bottom below top face

def _sg90_roll(bx, hdir):                         # hip-roll SG90 lying on its side: shaft along hdir*X at (bx,0,HZ);
    zc = HZ - SG_OFF                              # body hangs BELOW the roll axis (shaft is 5.9mm from the +Z front end)
    zlo, zhi = zc - SG_L / 2, zc + SG_L / 2
    x0 = bx - (SG_H if hdir > 0 else 0.0)
    body = box(SG_H, SG_W, SG_L, v(x0, -SG_W / 2, zlo))
    tx0 = min(bx - hdir * SG_TH1, bx - hdir * (SG_TH1 - SG_TT))       # tab plate near the top(horn) face, extends +-Z
    s = body.fuse(box(SG_TT, SG_W, SG_L + 2 * SG_TE, v(tx0, -SG_W / 2, zlo - SG_TE)))
    for tz in (zhi + SG_TE - 2.3, zlo - SG_TE + 2.3):                 # tab screw holes (axis X, 2.3 from the tips)
        s = s.cut(cyl(1.05, SG_TT + 2.0, v(tx0 - 1.0, 0, tz), X))
    s = s.fuse(box(4.0, 6.0, 4.0, v(x0 + (1.0 if hdir > 0 else SG_H - 5.0), -3.0, zhi)))   # lead stub: shaft-side end, case bottom
    s = s.fuse(cyl(2.4, 8.0, v(bx, 0, HZ), v(hdir, 0.0, 0.0))).removeSplitter()            # spline shaft into the barrel bore
    return rot(s, 180.0, v(0.0, 0.0, HZ), X)          # FLIP vertically: body swings ABOVE the roll axis -> femur-fold clearance below

def servo1(): return _sg90_roll(HX - 31.0, 1.0)   # front-type: shaft +X (rear legs after the KNEE_BACK X-flip)

def _sg90_pitch():                                # true SG90 for the Y-axis joints (hip-pitch / knee): shaft +Y at the origin
    body = box(SG_L, SG_H, SG_W, v(-(SG_L - 5.9), -13.5, -SG_W / 2))   # length along X (shaft 5.9 from +X face), height Y, width Z
    s = body.fuse(box(SG_L + 2 * SG_TE, SG_TT, SG_W, v(-(SG_L - 5.9) - SG_TE, 1.9, -SG_W / 2)))   # mounting tab plate y[1.9,4.4]
    for tx in (5.9 + SG_TE - 2.3, -(SG_L - 5.9) - SG_TE + 2.3):
        s = s.cut(cyl(1.05, SG_TT + 2.0, v(tx, 1.4, 0)))               # tab screw holes (axis Y)
    s = s.fuse(box(4.0, 4.0, 6.0, v(5.9, -12.5, -3.0)))                # lead exits the shaft-side face near the case bottom
    return s.fuse(cyl(2.4, 4.5, v(0, 9.0, 0))).removeSplitter()        # output spline r2.4 (was modeled r2.7 -- audit #22)

def _sg90_pitch_clear():                          # envelope + clearance for pocket/tab-slot cuts (slide-in along -X from the knuckle side)
    c = box(SG_L + .6, SG_H + .6, SG_W + .6, v(-(SG_L - 5.9) - .3, -13.8, -SG_W / 2 - .3))
    c = c.fuse(box(SG_L + 2 * SG_TE + 13.2, SG_TT + .6, SG_W + .6, v(-(SG_L - 5.9) - SG_TE - .3, 1.6, -SG_W / 2 - .3)))
    c = c.fuse(box(4.6, 4.6, 6.6, v(5.6, -12.8, -3.3)))
    return c.removeSplitter()

def servo2():                                     # hip-pitch SG90 (true geom) in the coxa cradle: spline +Y at HP
    s = _sg90_pitch(); s.translate(v(HP.x, 0.0, HP.z)); return s

def servoK():                                     # knee SG90 in the femur cradle, on the knee hinge (coplanar: shaft +Y at origin)
    return _sg90_pitch()

_PSPR = None
def _pitch_servo_sweep():                         # knee-servo swept envelope in TIBIA frame, cached (16 fold steps -65..160)
    global _PSPR
    if _PSPR is None:
        m = App.Matrix(); m.A11 = m.A22 = m.A33 = 1.06
        p = servoK().transformGeometry(m).common(box(26.0, 25.0, 26.0, v(-13.0, -14.5, -13.0)))   # inflate ~5%, knuckle zone only
        _PSPR = [rot(p, -float(f)) for f in range(-65, 165, 15)]   # coplanar knee: sweep = in-plane rotation about the hinge
    return _PSPR

_femur5 = femur
_FEM_BASE = None                                  # clean pre-carve femur, kept for the coxa sweep-clear proxy: transformGeometry
def femur():                                      # on the carved femur fails silently -> proxy empty -> NO clearance (989 regression)
    global _FEM_BASE
    _FEM_BASE = _femur5()
    fem = _FEM_BASE.cut(_sg90_pitch_clear())                          # KNEE servo pocket + tab slots (true tabs span 32.2 > old 24 cavity)
    # AUDIT: the hip-pitch SG90 was never placed; the proximal knuckle sweeps THROUGH its body (1670..2009
    # mm3 at every pitch). Carve the servo's swept envelope out of the knuckle -- bearing = +Y hub + -Y peg.
    m = App.Matrix(); m.A11 = m.A22 = m.A33 = 1.05
    p = servo2().copy(); p.translate(v(-HP.x, -HP.y, -HP.z)); p = p.transformGeometry(m); p.translate(v(HP.x, HP.y, HP.z))
    p = p.common(box(26.0, 25.0, 26.0, at(v(-13.0, -14.5, -13.0))))    # knuckle zone near HP only
    for a in (-45.0, -32.0, -19.0, -6.0, 7.0, 20.0, 33.0, 46.0, 59.0, 72.0, 85.0, 98.0):
        fem = fem.cut(rot(p, -a, HP, Y))
    # AUDIT #17: the tibia's -Y clevis arm passes through the knee idle-boss wall (solid 252 mm3 at standing,
    # blocks assembly) -- open the boss to a partial arc over the arm's swing sector (bore keeps the r3 peg).
    pa = box(8.0, 5.0, 14.0, v(0, -19.0, -7.0)).fuse(cyl(3.0, 5.0, v(0, -19.0, 0)))
    pa = pa.transformGeometry(m).common(box(26.0, 6.4, 26.0, v(-13.0, -19.8, -13.0)))   # inflate ~5%, boss band only
    for f in range(-65, 180, 20):                                     # coplanar knee: idle-boss arc = in-plane sweep
        fem = fem.cut(rot(pa, float(f)))
    fs = fem.Solids                                # the sweep SEVERS unsupported knuckle-cap arcs -- drop them,
    fem = max(fs, key=lambda q: q.Volume) if len(fs) > 1 else fem   # keep the structural femur (hub+arm+yoke+peg survive)
    try: fem = fem.removeSplitter()
    except Exception: pass                         # OCC chokes on some multi-shell results; the raw solid is fine
    return fem

def servo1_bracket(s=None):                       # printed cradle: open-top drop-in pocket, bottom-tab slot + M2 boss (servo
    if s is None: s = servo1()                    # retention, audit #19), roll-barrel bore, frame mount foot. Fits either variant.
    bb = s.BoundBox; w = 2.0
    vlo = s.common(box(1.5, 60.0, 60.0, v(bb.XMin, -30.0, HZ - 30.0))).Volume
    vhi = s.common(box(1.5, 60.0, 60.0, v(bb.XMax - 1.5, -30.0, HZ - 30.0))).Volume
    hx0, hdir = ((bb.XMin, -1.0) if vlo < vhi else (bb.XMax, 1.0))     # horn end = the thin end slab; hdir = direction the horn POINTS
    bx = hx0 - hdir * 8.0                                              # output face (horn cyl is 8 long)
    x0 = bx - (SG_H if hdir > 0 else 0.0)                              # body extents (same layout as _sg90_roll)
    zc = HZ - SG_OFF; zlo = zc - SG_L / 2
    shell = box(SG_H + .5 + 2 * w, SG_W + .5 + 2 * w, SG_L + .5 + w, v(x0 - .25 - w, -SG_W / 2 - .25 - w, zlo - .25 - w))
    cav   = box(SG_H + .5, SG_W + .5, SG_L + 20.0, v(x0 - .25, -SG_W / 2 - .25, zlo - .25))   # pocket, open upward (drop-in; top tab + lead exit above the rim)
    out = shell.cut(cav)
    tx0 = min(bx - hdir * SG_TH1, bx - hdir * (SG_TH1 - SG_TT))
    out = out.cut(box(SG_TT + .6, SG_W + .6, 8.0, v(tx0 - .3, -SG_W / 2 - .3, zlo - 6.0)))    # bottom-tab slot through the floor (locates the servo in X)
    bsx = (tx0 - 5.3) if hdir > 0 else (tx0 + SG_TT + .3)                                     # M2 boss under the floor, tail side of the tab:
    out = out.fuse(box(5.0, 8.0, 6.25, v(bsx, -4.0, zlo - 6.5)))                              # self-tap through the protruding tab hole = servo can't lift out
    out = out.cut(cyl(0.8, 5.6, v(bsx + (5.0 if hdir > 0 else 0.0), 0, zlo - SG_TE + 2.3), v(-hdir, 0.0, 0.0)))
    # HORN-END + ROLL-BARREL CLEARANCE (audit #16/#26/#36): the coxa's r8 roll barrel passes ~9mm INTO the
    # bracket at the horn end (and the front-variant end wall sat across the horn). Bore it out r8.6 so the
    # printed parts never interpenetrate (bracket^coxa was 503/215 mm3).
    out = out.cut(cyl(8.6, 12.0, v(bx + hdir * 4.0, 0.0, HZ), v(-hdir, 0.0, 0.0)))
    out = rot(out, 180.0, v(0.0, 0.0, HZ), X)         # FLIP the cradle to match the flipped servo (barrel bore is coaxial -> invariant)
    # MOUNT FOOT (extend-the-bracket): inboard flange resting on the frame deck top (placed Z145 = leg-local Z9), bolted down with 2x M2.
    fy0 = -SG_W / 2 - .25 - w - 14.0                                                        # 14 mm inboard from the inner wall toward the body/frame (-Y; mirror sends it inboard on both sides)
    out = out.fuse(box(bb.XLength, 14.0, 3.0, v(bb.XMin, fy0, 9.0)))                        # 3 mm plate sitting on the deck top (leg-local Z9..12)
    for hx in (bx + hdir * 0.5, bx + hdir * 0.5 - hdir * 15.0):                             # 2x M2 clearance holes keyed to the OUTPUT FACE (fixed vs the hip) -> lands EXACTLY on the frame pilots
        out = out.cut(cyl(1.1, 8.0, v(hx, -18.0, 7.0), Z))                                  # (placed |x|25/40, |y|24; bb.XMin-keying was 0.5mm off on the rear-type variant)
    return out.removeSplitter()

def coxa_rear():                                  # REAR leg: hip-roll barrel flipped to +X so the rear servo tucks INBOARD
    cradle = box(24, 22.5, T, at(v(-18, -13.5, -9)))         # femur-coupling half: identical to leg5 coxa()
    cav    = box(24, 24, 14, at(v(-17.5, -14, -7)))
    pegb   = cyl(6, 7, at(v(0, -19, 0))).cut(cyl(3.4, 9, at(v(0, -20, 0))))
    cx = cradle.cut(cav).fuse(pegb)
    cover  = cyl(RPROX + CLR, 20, at(v(0, -10, 0))).common(box(40, 20, 120, at(v(-2, -10, -60))))
    cx = cx.cut(cover)
    dz = 0.0                                                  # hip-roll cup on +X (inboard), on the femur centreline HZ
    barrel = cyl(8, 16, at(v(14, 0, dz)), v(1, 0, 0))
    barrel = barrel.cut(cyl(2.5, 9, at(v(22, 0, dz)), v(1, 0, 0)))    # spline bore opens +X (servo enters from inboard)
    arm    = box(16, 16, 20, at(v(4, -8, -9)))                        # bridge cradle -> +X barrel
    cx = cx.fuse(arm).fuse(barrel).removeSplitter()
    fs = cx.Solids
    return max(fs, key=lambda s: s.Volume) if len(fs) > 1 else cx

def servo1_rear():                                # rear-type hip-roll SG90: shaft -X into the +X barrel; body tucks toward centre
    return _sg90_roll(HX + 31.0, -1.0)            # same axis height as the front type (barrel on the femur centreline)

def _femur_sweep_proxy(fem):                      # AUDIT FIX #1 helper: the thigh inflated ~5% about the hip (~0.7mm clearance),
    s = fem.copy(); s.translate(v(-HP.x, -HP.y, -HP.z))          # journal/yoke (u<4) MASKED OFF so the cradle bearing (CLR=0.6) is untouched
    m = App.Matrix(); m.A11 = m.A22 = m.A33 = 1.05
    s = s.transformGeometry(m); s.translate(v(HP.x, HP.y, HP.z))
    return s.common(box(120.0, 80.0, 80.0, v(HX + 4.0, -40.0, HZ - 40.0)))

def _sweep_clear(cxs, prox, angles):              # AUDIT FIX #1: carve the thigh's SWEPT envelope (hip-pitch range) out of a coxa so the
    # rigid parts can never interpenetrate at any covered pitch angle. prox (= _femur_sweep_proxy(fem)) is
    for a in angles: cxs = cxs.cut(rot(prox, float(a), HP, Y))    # hoisted into build_dog -> built ONCE for both coxa variants
    cxs = cxs.removeSplitter()
    fs = cxs.Solids
    return max(fs, key=lambda s: s.Volume) if len(fs) > 1 else cxs

def _horn_screw(cxs, ddir):                       # AUDIT: hip-roll joint had NO axial retention (smooth 5mm bore friction-fit on
    z0 = HZ                                        # the horn; barrel solid behind it blocked the horn screw). Cut an M2 channel:
    cxs = cxs.cut(cyl(1.1, 10.0, v(HX + ddir * 24.0, 0, z0), v(-ddir, 0.0, 0.0)))    # screw clearance through the 2mm head shoulder
    cxs = cxs.cut(cyl(3.0, 32.0, v(HX + ddir * 20.0, 0, z0), v(-ddir, 0.0, 0.0)))    # head counterbore + driver channel out through arm/cradle
    cxs = cxs.removeSplitter()                    # assembly: screw the coxa onto the horn BEFORE the femur enters the cradle
    fs = cxs.Solids                               # (the femur knuckle blocks the driver channel once assembled)
    return max(fs, key=lambda s: s.Volume) if len(fs) > 1 else cxs

# ---------- placement ---------------------------------------------------------
def roundbox(a, b, c, base, r):
    bx = Part.makeBox(a, b, c, base)
    try: bx = bx.makeFillet(r, bx.Edges)
    except Exception: pass
    return bx

def export_stl(shape, name):                      # write a single printable part to stl/
    p = OUT + "/stl/" + name + ".stl"
    try:
        shape.exportStl(p); b = shape.BoundBox
        return "  %-16s %5.1f x %5.1f x %5.1f mm  solids=%d  %dkB" % (
            name, b.XLength, b.YLength, b.ZLength, len(shape.Solids), os.path.getsize(p) // 1024)
    except Exception as e:
        return "  %-16s EXPORT FAIL: %s" % (name, e)

def frame_part():                                 # PROMOTED structural frame = exported part: cage + electronics holder + body-cover snap catches
    bz0 = HIP_DZ - BH + 7                          # -15 (= body bottom)
    fx0, fx1, fy = -(BL/2 - HIP_INSET + 22.0), BL/2 - HIP_INSET + 22.0, 26.0     # X -92.5..92.5  Y -26..26 (pulled in: hip-pitch is about Y so the femur shaft inner edge Y28.2 is pitch-invariant; fy=26 clears it at every sweep angle)
    Lx = fx1 - fx0
    floor = roundbox(Lx, 2*fy, 5.0, v(fx0, -fy, bz0 - 1), 2.0)        # bottom plate  Z -16..-11 (cavity floor)
    railL = box(Lx, 4.0, 14.0, v(fx0, fy - 4, bz0 + 4))              # side rails  Y +-(22..26)  Z -11..3 (leave Z 3..9 open = side reveal band)
    railR = box(Lx, 4.0, 14.0, v(fx0, -fy,    bz0 + 4))
    deck  = roundbox(Lx, 2*fy, 5.0, v(fx0, -fy, bz0 + 24), 2.0)       # top deck  Z 9..14
    fr = floor.fuse(railL).fuse(railR).fuse(deck)
    for sgn in (1.0, -1.0):                                          # posts bridge rail(Z3)->deck(Z9) so it's ONE solid; reveal band stays open between them
        yp = -fy if sgn < 0 else fy - 4
        for cx in (-80.0, -40.0, 0.0, 40.0, 80.0):
            fr = fr.fuse(box(6.0, 4.0, 8.0, v(cx - 3, yp, bz0 + 18)))
    ftop = bz0 + 4                                                   # cavity floor top  Z -11
    # ELECTRONICS HOLDER: 4 board standoffs (M2 pilot) front-central + a rear 1S-LiPo pocket, all inside the cavity
    for bx in (14.0, 50.0):
        for by in (-11.0, 11.0):
            fr = fr.fuse(cyl(3.0, 6.0, v(bx, by, ftop)).cut(cyl(1.0, 9.0, v(bx, by, ftop - 1))))
            fr = fr.cut(cyl(2.0, 8.0, v(bx, by, bz0 + 23), Z))       # DRIVER HOLE through the deck above each standoff (audit: M2s were unreachable under the fused deck)
    fr = (fr.fuse(box(70, 2.5, 9, v(-84, 14.0, ftop)))               # 1S-LiPo pocket: +Y wall
            .fuse(box(70, 2.5, 9, v(-84, -16.5, ftop)))              # -Y wall
            .fuse(box(2.5, 33, 9, v(-84, -16.5, ftop))))             # rear end wall
    # LiPo BELLY HATCH (audit: pocket was battery-first assembly only; swap = full teardown). The floor under
    # the pocket opens through the lower cover's open belly; 2x hook-and-loop straps drop through the slots.
    fr = fr.cut(box(60.0, 24.0, 7.0, v(-78.0, -12.0, bz0 - 2)))      # hatch: floor strips y +-(12..26) + both ends stay -> ONE solid
    for hx in (-62.0, -34.0):                                        # strap slots outboard of the pocket walls
        for sy in (1.0, -1.0):
            fr = fr.cut(box(4.0, 2.5, 7.0, v(hx, sy * 17.5 - (2.5 if sy < 0 else 0.0), bz0 - 2)))
    # COVER CATCH ARMS (audit #3: old edge nubs never engaged -- 0.4mm air gap, no lock). One WIDE arm per side
    # (central deck edge; x +-12..+-47 is occupied by the bracket feet) + two REAR arms pierce matching WINDOWS
    # in the cover's side/rear walls: the wall flexes ~0.8mm over the arm on push-down, then sits pinned = lock.
    for sgn in (1.0, -1.0):
        fr = fr.fuse(box(24.0, 9.4, 3.0, v(-12.0, (25.0 if sgn > 0 else -34.4), bz0 + 25)))     # side arm, Z 10..13, tip at |y|34.4
    for py in (-14.0, 14.0):
        fr = fr.fuse(box(13.5, 8.0, 3.0, v(-105.0, py - 4.0, bz0 + 25)))                        # rear arm: deck rear edge -> through the cover rear wall
    for sx in (1.0, -1.0):                                               # clearance notches at the 4 hip corners: the femur hip-pitch yoke sweeps inboard to ~Y23
        for sgn in (1.0, -1.0):
            x0 = 48.0 if sx > 0 else -93.0
            y0 = 20.0 if sgn > 0 else -33.0
            fr = fr.cut(box(45.0, 13.0, 32.0, v(x0, y0, bz0 - 1)))
    for px in (-50.0, 50.0):                                            # BELLY-PAN pilots: M2 self-tap up into the floor (repurposed
        for py in (-14.0, 14.0):                                        # cover-peg positions; the deck-top holes are gone -- audit #21)
            fr = fr.cut(cyl(0.8, 6.0, v(px, py, bz0 - 2)))              # floor-bottom pilot Z -17..-11
    for py in (-6.0, 6.0):                                              # HEAD-PLUG pilots: M2 down through the deck front into the head's
        fr = fr.cut(cyl(0.8, 8.0, v(87.0, py, bz0 + 23), Z))            # under-deck plug (head mount -- audit #5: head was unmounted)
    for sx in (1.0, -1.0):                                              # HIP-MOUNT pilot holes: the leg's bracket foot bolts down here (M2 self-tap), 2/corner on the deck top
        for sy in (1.0, -1.0):
            for hx in (25.0, 40.0):
                fr = fr.cut(cyl(0.8, 8.0, v(sx * hx, sy * 24.0, bz0 + 23), Z))   # Z 8..16 through the deck
    fr = fr.removeSplitter()
    fs = fr.Solids
    return max(fs, key=lambda s: s.Volume) if len(fs) > 1 else fr

def covers():                                     # SMOOTH CONTIGUOUS SM3-STYLE SHELL in TWO SNAP-ON PIECES
    # (user: body = 2 snap-on parts -- our own split; head like SM3). ONE smooth rounded body, NO pods; the
    # legs + hip-roll servos emerge through 4 leg openings and sit OUTBOARD (SM3 shows its shoulder servos).
    # Split on a horizontal seam -> a TOP and a BOTTOM clamshell that snap together over the frame (seam lip on
    # the top piece nests inside the bottom rim). Head plug pierces the top-piece front. Belly z UNCHANGED so the
    # fold-flat clearance survives. Returns (top, bottom).
    bz0 = HIP_DZ - BH + 7; bx0 = -(BL/2 + RUMP); Lx = BL + RUMP + 4.0; w = 2.2
    bch = BH + 5.0; rtop = 8.0                                        # FULL-HEIGHT sides (only the top edges rounded), TALLER to cover the hip tops
    zsp = bz0 + 18.0                                                   # horizontal snap seam (z=3): top>=zsp, bottom<zsp
    outer = roundbox(Lx, BW, bch, v(bx0, -BW/2, bz0), rtop)            # fuller body, full-height flat sides + rounded top
    inner = roundbox(Lx - 2*w, BW - 2*w, bch - 2*w, v(bx0 + w, -BW/2 + w, bz0 + w), max(rtop - w, 1.0))
    sh = outer.cut(inner)
    notches = []                                                      # 4 LEG NOTCHES (SM3 shoulder-cutout style): STEPPED, sized from MEASURED smooth-femur hip hardware.
    zr = bz0 + 36.0                                                   # roof-step z (~21): coxa top Z22 + hip-roll servo top Z21.6 poke the roof; the inboard bracket
    for sx in (1.0, -1.0):                                            # (top Z17.8) sits BELOW it -> keep the roof over the inboard-low strip, open it only over the gear.
        x0 = 13.0 if sx > 0 else -91.0                               # base (side-wall) cut x[13,91] front / [-91,-13] rear -- inboard edge set by the hip-roll servo (X17)/bracket (X14.8)
        xr = 16.0 if sx > 0 else -91.0                               # roof cut inset 3mm inboard (bracket zone keeps its roof); femur knuckle + coxa poke within this zone
        for sgn in (1.0, -1.0):
            y0 = 18.0 if sgn > 0 else -60.0                          # base y[18,60]: inboard 18 clears the bracket foot (Y19.6); outboard 60 spans past the wall
            yr = 22.0 if sgn > 0 else -60.0                          # roof cut inboard edge 22 (coxa Y28.5 / servo Y35.9 / femur Y23 poke here; the y18-22 bracket strip keeps its roof)
            base = box(78.0, 42.0, zr - (bz0 - 4.0), v(x0, y0, bz0 - 4.0))   # z[-19,~21]: side wall + everything below the roof
            roof = box(75.0, 38.0, 26.0 - zr, v(xr, yr, zr))                # z[~21,26]: through the roof only over the coxa/servo/femur-knuckle zone
            notches += [base, roof]; sh = sh.cut(base); sh = sh.cut(roof)
    sh = sh.cut(box(4.6, 20.6, 6.0, v(BL/2 + 1.5, -10.3, zsp - 1.0)))  # front head-plug opening (in the TOP piece)
    sh = sh.removeSplitter()
    hi = box(Lx + 8, BW + 8, BH + 4, v(bx0 - 4, -BW/2 - 4, zsp))       # split plane
    top = sh.common(hi); bot = sh.cut(hi)
    lip = box(Lx - 2*(w+0.6), BW - 2*(w+0.6), 4.5, v(bx0 + w+0.6, -BW/2 + w+0.6, zsp - 4.0))   # rectangular collar (always 1 solid); z[-1,3.5] overlaps the top piece by 0.5
    lip = lip.cut(box(Lx - 2*(2*w+0.6), BW - 2*(2*w+0.6), 6.0, v(bx0 + 2*w+0.6, -BW/2 + 2*w+0.6, zsp - 5.0)))
    top = top.fuse(lip)                                               # seam snap lip: nests into the bottom rim
    for nb in notches: top = top.cut(nb)                              # re-notch the TOP: the lip re-added material at the hips (coxa/femur crossed it) -> clear it back out
    def _one(s):
        s = s.removeSplitter(); return max(s.Solids, key=lambda x: x.Volume) if len(s.Solids) > 1 else s
    return _one(top), _one(bot)

def belly_pan():                                  # screwed belly skin: flat plate under the frame floor; LiPo hatch stays open
    bz0 = HIP_DZ - BH + 7
    p = roundbox(164.0, 48.0, 1.6, v(-82.0, -24.0, bz0 - 2.6), 1.5)     # z -17.6..-16, y +-24 (inboard of the femur band y28+)
    for px in (-50.0, 50.0):
        for py in (-14.0, 14.0):
            p = p.cut(cyl(1.1, 4.0, v(px, py, bz0 - 3.0), Z))           # M2 clearance -> the frame floor pilots
    p = p.cut(box(56.0, 20.0, 4.0, v(-76.0, -10.0, bz0 - 3.0)))         # hatch window (matches the frame hatch; straps pass through)
    return p.removeSplitter()

def place(parts, sx, sy, corner):                 # mirror L/R (and F/B), then move hip HP -> corner
    xmir = KNEE_BACK                                             # flip leg in X: all knees point tailward
    hx = HP.x * (-1 if xmir else 1)
    hy = HP.y * (-1 if sy < 0 else 1)
    T = v(corner.x - hx, corner.y - hy, corner.z - HP.z)
    out = []
    for n, s, c in parts:
        s2 = s
        if sy < 0:  s2 = s2.mirror(O, Y)                          # left/right handedness
        if xmir:    s2 = s2.mirror(O, X)                          # fore-aft flip = knee direction; head/body stay +X (normal root)
        s2 = s2.copy(); s2.translate(T)
        out.append(("%s_%+d%+d" % (n, sx, sy), s2, c))
    return out

def build_dog():
    # smooth femur first, with a knee-gate safety check
    global femur_loft
    femur_loft = femur_loft_smooth
    try:
        fem = femur()
        ok = fem.isValid() and len(fem.Solids) == 1
    except Exception as e:
        ok = False; print("smooth femur EXC:", e)
    tib0 = tibiaF(0.0)                                    # coplanar knee: extended shank in-plane (clean print/report)
    kR = fem.common(tib0).Volume if ok else 1e9                 # rest-fold graze (round femur is fatter -> larger)
    kP = fem.common(tibiaF(KNEE_FOLD)).Volume if ok else 1e9   # graze at the ACTUALLY-RENDERED fold
    smooth = ok and kP < 600.0                                  # display model: round knee grazes the folded shin at the bend (cosmetic, knees touch when bent)
    if not smooth:
        print("  smooth femur rejected (valid=%s restknee=%.1f posedknee=%.1f) -> leg7 femur" % (ok, kR, kP))
        femur_loft = _leg7_femur_loft; fem = femur()
    # AUDIT FIX #1: femur-sweep clearance. Hip-pitch demand: walking 41..91, crouch 78, foldflat 0 (foldflat
    # excluded on front hips -- at 0deg the thigh is coaxial with the +X barrel, geometrically unresolvable).
    s2c = _sg90_pitch_clear(); s2c.translate(v(HP.x, 0.0, HP.z))        # hip-pitch servo pocket/tab-slot envelope at HP: the coxa
    s2 = servo2(); sk = servoK()                                        # bridge ARM was fused straight through the servo body (1722/387 mm3)
    femP0 = _FEM_BASE if _FEM_BASE is not None else fem                 # sweep proxies from the CLEAN femur (superset = conservative clearance)
    prox0 = _femur_sweep_proxy(femP0)                                   # build the sweep proxy ONCE; both coxa variants reuse it
    cx = _horn_screw(_sweep_clear(coxa().cut(s2c), prox0, (70.0, 77.0, 84.0, 91.0, 98.0)), -1.0)   # rear legs: thigh rubs the cradle top only at deep pitch (was 314..674 @75..91)

    # posed leg in leg-local frame (coxa static; femur+tibia pitched about HP; knee servo rides the femur)
    femP = rot(fem, PITCH, HP, Y)
    tibP = rot(tibiaF(KNEE_FOLD), PITCH, HP, Y)        # tibia now includes the fused SM3 foot end (boot-snap toe)
    skP = rot(sk, PITCH, HP, Y)
    leg = [("coxa", cx, YEL), ("femur", femP, YEL), ("tibia", tibP, DRK),
           ("servo1", servo1(), GRY), ("s1br", servo1_bracket(), (.30, .32, .36)),
           ("servo2", s2, GRY), ("servoK", skP, GRY)]                   # ALL 3 leg servos placed (audit #10/#39: 8 of 12 were absent)
    # rear leg: same femur/tibia (knee forward), but coxa_rear flips the hip-roll barrel to +X so the servo tucks inboard
    cxr = _horn_screw(_sweep_clear(coxa_rear().cut(s2c), prox0, (38.0, 44.0, 50.0, 56.0, 62.0, 68.0, 74.0, 80.0, 86.0, 92.0, 98.0)), 1.0)   # front legs: +X barrel/arm interpenetrated the thigh at ALL angles (audit #1)
    s1r = servo1_rear()
    leg_rear = [("coxa", cxr, YEL), ("femur", femP, YEL), ("tibia", tibP, DRK),
                ("servo1", s1r, GRY), ("s1br", servo1_bracket(s1r), (.30, .32, .36)),
                ("servo2", s2, GRY), ("servoK", skP, GRY)]

    # gates -> file (knee fold + hip pitch + servo fits; tibias built once and reused across gates)
    k0 = 0.0                                           # coplanar knee: standing reference fold is 0
    tibs = {f: (tib0 if abs(f - k0) < 0.1 else tibiaF(float(f))) for f in (k0 - 110, k0 - 60, k0, k0 + 55, k0 + 100, k0 + 115)}
    L = ["=== DOG  femur=%.0f  smooth_femur=%s  pitch=%.0f knee_fold=%.0f ==="
         % (FEMUR_LEN, smooth, PITCH, KNEE_FOLD)]
    L.append("  knee:  " + "  ".join("%d=%.0f" % (a, fem.common(tibs[k0 - a]).Volume) for a in (0, 60, 110)))
    L.append("  kneeD: " + "  ".join("+%d=%.0f" % (a, fem.common(tibs[k0 + a]).Volume) for a in (55, 100, 115)))   # feet-DOWN folds (crouch~100, foldflat~115) -- the direction the poses actually use (audit #27: old gate swept foot-up only)
    L.append("  hip :  " + "  ".join("%+d=%.0f" % (a, cx.common(rot(fem, a, HP, Y)).Volume) for a in (-45, 0, 45)))
    L.append("  hipR:  " + "  ".join("%+d=%.0f" % (a, cxr.common(rot(fem, a, HP, Y)).Volume) for a in (-45, 0, 45))
             + "  solids=%d" % len(cxr.Solids))   # rear coxa (+X barrel) vs femur -> shows the +pitch range cost
    L.append("  svc2:  " + "  ".join("%+d=%.0f" % (a, s2.common(rot(fem, float(a), HP, Y)).Volume) for a in (0, 45, 66, 91))
             + "  s2^cx=%.0f s2^cxr=%.0f" % (cx.common(s2).Volume, cxr.common(s2).Volume))   # hip-pitch servo vs pitched femur + coxa pockets
    L.append("  svcK:  " + "  ".join("%+d=%.0f" % (int(f - k0), sk.common(tibs[f]).Volume) for f in (k0 - 110, k0, k0 + 55, k0 + 115))
             + "  sk^fem=%.0f" % fem.common(sk).Volume)                                      # knee servo vs folding tibia + its own cradle
    for n, s in (("coxa", cx), ("femur", fem), ("tibia(0)", tib0)):
        b = s.BoundBox; L.append("  %-9s X=%6.1f Y=%5.1f Z=%6.1f vol=%8.0f solids=%d"
                                 % (n, b.XLength, b.YLength, b.ZLength, s.Volume, len(s.Solids)))
    fem_fl = abs(HX); tib_fl = tib0.BoundBox.XMax                  # femur=hip->knee, tibia=knee->foot tip
    L.append("  RATIO: femur(hip->knee)=%.0f  tibia(knee->foot)=%.0f  femur:tibia=1:%.2f  (SM3 frame=1:%.2f)"
             % (fem_fl, tib_fl, tib_fl / fem_fl, 129.3 / 138.5))

    # SM3-STYLE sensor head (user: head like SM3): compact ROUNDED BOX with a dominant flat front face, not a long
    # snout. MOUNTED via an under-deck tongue + 2x M2; root clears the cover front.
    bz0 = HIP_DZ - BH + 7                        # body bottom (deck-level shoulders; body hangs below the hips)
    hL, hW, hH = 38.0, 0.56 * BW, 34.0        # SM3-style head: BIGGER rounded box + strong forward-sloping "visor" top + big angled sensor face
    hx0 = BL/2 + 4.2; hbz = bz0 + 1.0; htop = hbz + hH
    head = roundbox(hL, hW, hH, v(hx0, -hW/2, hbz), 7.0)               # root x96.7 clears the cover front
    head = head.cut(rot(box(hL + 16, hW + 6, 26.0, v(hx0 - 8, -hW/2 - 3, htop)), 24.0, v(hx0, 0.0, htop), Y))  # steeper visor: slope the top down toward the front
    head = head.fuse(box(13.1, 20.0, 5.0, v(83.8, -10.0, 3.9)))       # UNDER-DECK MOUNT TONGUE: slides beneath the frame deck front
    for py in (-6.0, 6.0):                                            # 2x M2 down through the deck pilots
        head = head.cut(cyl(0.8, 5.0, v(87.0, py, 3.8), Z))
    head = head.removeSplitter()
    cu_top, cu_bot = covers()                                          # BODY = two snap-on cosmetic pieces (top + bottom clamshell)
    parts = [("cover_top", cu_top, YEL), ("cover_bot", cu_bot, (.90, .70, .10)), ("head", head, YEL)]

    # ---- export the unique printable parts (unposed local frame) ----
    # CHIRALITY (audit #29): place() X-mirrors ALL legs and Y-mirrors the sy<0 side; net effect: as-exported
    # geometry = the sy=-1 (RIGHT) legs (rotZ180 = same handedness); the sy=+1 (LEFT) legs are TRUE MIRRORS.
    # Print one set as-is + one set slicer-mirrored. The canted tibia + SM3-LeftTibia foot + TPU boot are chiral.
    if STL_EXPORT:
        os.makedirs(OUT + "/stl", exist_ok=True)
        EL = ["=== STL EXPORT (stl/sm3sg90_*.stl) ==="]
        for s, nm in ((cx, "sm3sg90_coxa"), (fem, "sm3sg90_femur"), (tib0, "sm3sg90_tibia"),   # coxa = REAR legs (outboard barrel), x2 + slicer-mirror
                      (cxr, "sm3sg90_coxa_front"),   # FRONT legs (inboard/+X barrel, femur-sweep-cleared), x2 + slicer-mirror -- was MISSING from exports (audit)
                      (boot(), "sm3sg90_boot"),   # flexible TPU boot (RIGHT feet -- matches the as-exported chiral tibia), x2
                      (boot().mirror(O, Y), "sm3sg90_boot_L"),   # mirrored boot for the LEFT feet, x2 (audit #32: single boot was wrong-handed for one side)
                      (servo1_bracket(), "sm3sg90_hipbracket"),   # hip-roll servo cradle + frame mount foot (bolts the leg to the deck)
                      (frame_part(), "sm3sg90_frame"),   # PROMOTED frame: electronics holder + body-cover snap catches
                      (cu_top, "sm3sg90_cover_top"), (cu_bot, "sm3sg90_cover_bottom"),   # body = 2 snap-on cosmetic pieces
                      (head, "sm3sg90_head")):
            EL.append(export_stl(s, nm))
        L += EL; print("\n".join(EL))

    # dark sensor 'face' panel dominating the sloped head front -> the SM3/Spot-Micro sensor-head cue (cosmetic)
    fc = roundbox(3.5, 0.84 * hW, 0.60 * hH, v(hx0 + hL - 2.0, -0.42 * hW, hbz + 0.08 * hH), 4.0)
    parts.append(("face", fc, DRK))

    # ---- internal structural frame: floor + side rails + top spine (panel mount) + rear-central electronics bay ----
    hxc = BL/2 - HIP_INSET                                             # hip fore-aft position; frame must span the placed cradles
    fx0, fx1, fy = -(hxc + 22.0), hxc + 22.0, BW/2 - 5                 # frame follows hips in X; floor half-width tracks BW (just inside the walls)
    frame = frame_part()                                                 # PROMOTED: cage + electronics holder + body-cover snap catches
    elec  = box(58, 32, 8, v(3, -16, bz0 + 10))                        # Pico/PCA9685 stack IN the frame cavity ON the standoffs (x14/x50; was clipping the LiPo pocket walls)
    parts += [("frame", frame, (.32, .34, .38)), ("elec", elec, (.10, .10, .12))]

    # 4 legs at the body corners
    hipX = BL/2 - HIP_INSET; hipY = BW/2 + HIP_OUT; hipZ = HIP_DZ
    for sx in (1, -1):
        for sy in (1, -1):
            # hip-roll barrel must tuck the servo INBOARD; the knee-back X-flip swaps which coxa goes front/rear
            L4 = (leg_rear if sx > 0 else leg) if KNEE_BACK else (leg if sx > 0 else leg_rear)
            parts += place(L4, sx, sy, v(sx * hipX, sy * hipY, hipZ))

    # drop everything so the lowest foot sits on z=0 (clean stance render)
    zmin = min(s.BoundBox.ZMin for _, s, _ in parts)
    for _, s, _ in parts: s.translate(v(0, 0, -zmin))
    L.append("  PROPORTION: shell %.0fx%.0fx%.0f (BL=%.0f+rump%.0f)  L:W=%.2f:1  track=%.0f  wheelbase=%.0f  rear/front overhang=%.0f/%.0f"
             % (BL + RUMP, BW, BH, BL, RUMP, (BL + RUMP) / BW, BW + 2 * HIP_OUT, BL - 2 * HIP_INSET, HIP_INSET + RUMP, HIP_INSET))
    L.append("  stance height (belly bottom) = %.1f mm  | parts=%d" % (bz0 - zmin, len(parts)))
    open(RDIR + "/dog_gates.txt", "w").write("\n".join(L) + "\n")
    print("\n".join(L))
    return parts

def render(parts):
    nm = "dog"
    if nm in list(App.listDocuments()): App.closeDocument(nm)
    doc = App.newDocument(nm)
    for n, s, c in parts:
        o = doc.addObject("Part::Feature", n); o.Shape = s
        try: o.ViewObject.ShapeColor = c
        except Exception: pass
        try: o.ViewObject.Transparency = 72 if n in ("cover_top", "cover_bot", "head", "face") else 0   # see the servos/bay inside
        except Exception: pass
        try: o.ViewObject.Deviation = 0.02
        except Exception: pass
    doc.recompute()
    gv = Gui.activeDocument().activeView()
    for vn, vf in (("side", "viewFront"), ("front", "viewRight"), ("top", "viewTop"), ("iso", "viewAxonometric")):
        getattr(gv, vf)(); Gui.SendMsgToActiveView("ViewFit")
        gv.saveImage(RDIR + "/dog_" + vn + ".png", 1200, 900, "White")
    gv.viewAxonometric(); Gui.SendMsgToActiveView("ViewFit")
    print("  renders -> dog_side/front/top/iso.png")

PARTS = build_dog()                               # kept module-global: iteration scripts reuse PARTS instead of rebuilding
render(PARTS)
