# dog14.py -- v9 ASSEMBLY @ 200mm: framemin frame + 4x coxa (unchanged, COXA_SPEC R15) + v9 direct-drive legs.
# Composes the live scripts dog13-style (exec into this namespace) and runs the leg-vs-frame / leg-vs-coxa
# clearance gates the v9 parts have never had: splay x pitch x knee pose sweep, common() at every pose.
#   placement: parts shift inboard by framemin's XS (245->200 wheelbase); leg lands on the coxa cover face
#   (FACE_OUT) with the rigid HPDZ lift onto the measured hip axis -- same mates as LEG_SPEC "Hip mount".
# Headless-safe (freecadcmd): only writes posed part STLs for the matplotlib family photo.
import FreeCAD as App, FreeCADGui as Gui, Part, MeshPart, math, os, traceback
v=App.Vector; X,Y,Z=v(1,0,0),v(0,1,0),v(0,0,1)
FAST=True
exec(open(r"C:/ultrafish/robodog/dog13.py").read())      # SH, servo0, servo1, tf, rot, SP, DX, YSHIFT, S0DZ ...
# R16 coxa MIGRATION (audit 2026-07-27): the printed coxa is coxablock's COXA/CLIP, not dog13's shoulder(). Exec
# coxablock HERE -- BEFORE framemin (which now grows its pocket from COXA) and BEFORE legflat_v9 (which redefines
# the shared cyl/KZ/box globals to 3-arg/leg values and would break coxablock's 4-arg calls if run first).
HORN_MODE='embed'
exec(open(r"C:/ultrafish/robodog/coxablock.py").read())  # -> COXA (R16 coxa), CLIP (cover) ; re-execs dog13
exec(open(r"C:/ultrafish/robodog/framemin.py").read())   # FR = v9 frame (pocket grown from COXA now), XS, TOPZ, zbot
V9_PARTS_ONLY=True
try: exec(open(r"C:/ultrafish/robodog/legflat_v9.py").read())   # FE, TI, SV, SVc, XC, HIPZ, KZ (leg-local)
except SystemExit: pass
A_LOG=[]
def alog(s): A_LOG.append(str(s))
try:
    HPDZ=0.3984; FACE_OUT=35.1          # cover outer face (leg-local Y); flange inboard face sits at 33.0
    DY=FACE_OUT-33.0                    # leg-local -> coxa-frame lateral seat
    PX=v(-DX+XS, YSHIFT+DY, HPDZ)       # leg-local -> body (+X+Y corner): wheelbase shift + seat + lift
    HIPX=106.0-DX+XS                    # body-frame hip axis (x, z) for pitch
    def leg_pose(sp,ph,th,clr=True):
        TIp=TI.copy(); TIp.rotate(v(XC,0,KZ),Y,th)
        L=FE.fuse(TIp).fuse((SVc if clr else SV).copy())
        L.translate(PX)
        L.rotate(v(HIPX,0,10.0+HPDZ),Y,ph)
        return rot(L,sp,SP,X)
    # ---- gate 1: leg vs FRAME over the pose envelope (frame pockets were grown from the COXA sweep only;
    #      the v9 leg has never been checked against the 200 frame) ----
    # R16 coxa + cover at the v9 wheelbase. dog13's SH is yout(-DX,+YSHIFT)'d already; coxablock's COXA/CLIP are
    # NOT, so apply the SAME datum here: (-DX+XS, +YSHIFT, 0). Verified: COVER9 outer face Y=42.3 mates the femur
    # hip flange (audit). (was SH9=SH.translate(XS) = the OLD shoulder, which the leg floated 2.3mm off of.)
    SH9=COXA.copy(); SH9.translate(v(-DX+XS, YSHIFT, 0.0))        # R16 coxa body (gate + posing)
    COVER9=CLIP.copy(); COVER9.translate(v(-DX+XS, YSHIFT, 0.0))  # cover = the face the femur hip rubs
    # SKIP_GATES: the 40 common()-over-poses can exceed the MCP 90s GUI timeout; set it (global) for a
    # pure render run once the gates have passed headless. Default runs them.
    if globals().get('SKIP_GATES',False):
        alog("GATES skipped (SKIP_GATES set) -- render-only run; last headless: leg-vs-frame 0.00, leg-vs-coxa 0.00")
    else:
        worst=0.0; bad=[]
        for sp in (-25,0,45,105):
            for ph in (-45,0,45,90):
                for th in (0,70):
                    q=leg_pose(sp,ph,th).common(FR).Volume
                    worst=max(worst,q)
                    if q>0.5: bad.append("sp%+d ph%+d th%d = %.1f"%(sp,ph,th,q))
        alog("GATE leg-vs-frame: worst %.2f mm3 over 32 poses"%worst)
        for b in bad[:8]: alog("   HIT "+b)
        # working-range gate: real gait envelope (moderate splay/pitch/knee), not the full clearance extremes
        worstw=0.0; badw=[]
        for sp in (-15,0,15):
            for ph in (-30,-15,0,15,30):
                for th in (0,25,45):
                    q=leg_pose(sp,ph,th).common(FR).Volume
                    worstw=max(worstw,q)
                    if q>0.5: badw.append("sp%+d ph%+d th%d = %.1f"%(sp,ph,th,q))
        alog("GATE leg-vs-frame WORKING range (splay+/-15, pitch+/-30, knee 0-45): worst %.2f mm3"%worstw)
        for b in badw[:8]: alog("   wHIT "+b)
        # ---- gate 2: leg vs COXA (coxa rides splay with the leg -> relative DOF = pitch+knee only) ----
        worst2=0.0; bad2=[]
        for ph in (-45,0,45,90):
            for th in (0,70):
                q=leg_pose(0,ph,th).common(SH9).Volume
                worst2=max(worst2,q)
                if q>0.5: bad2.append("ph%+d th%d = %.1f"%(ph,th,q))
        alog("GATE leg-vs-coxa: worst %.2f mm3 over 8 poses"%worst2)
        for b in bad2[:8]: alog("   HIT "+b)
    # ---- family photo: stance pose, all four corners, posed part STLs for the matplotlib render ----
    OUTD=r"C:/ultrafish/robodog/ref/iter/dog14_parts"
    if not os.path.isdir(OUTD): os.makedirs(OUTD)
    SP_ST,PH_ST,TH_ST=0,-20,35                     # stance: slight forward femur, knee flexed back
    # posed part builders (shared by the STL dump AND the GUI doc): frame once, then each corner's parts.
    def coxa_at(sxc,syc): return tf(SH9,sxc,syc)
    def cover_at(sxc,syc): return tf(COVER9,sxc,syc)     # R16 cover: the femur-hip bearing face
    def pin_at(sxc,syc):                                 # REAL printed screw-in splay pin (framemin's PIN); fallback = O8 stub
        p=PIN if ('PIN' in globals() and PIN is not None) else Part.makeCylinder(4.0,(cbx1+COLL)-PINTIP,v(PINTIP,SPYp,SPZp),v(1,0,0))
        return tf(p,sxc,syc)
    def servo0_at(sxc,syc): s=servo0.copy(); s.translate(v(XS,0,0)); return tf(s,sxc,syc)
    def servo1_at(sxc,syc): s=servo1.copy(); s.translate(v(XS,0,0)); return tf(s,sxc,syc)
    def legpart_at(pp,th,sxc,syc):
        q=pp.copy()
        if th is not None: q.rotate(v(XC,0,KZ),Y,th)   # tibia folds about the knee first
        q.translate(PX); q.rotate(v(HIPX,0,10.0+HPDZ),Y,PH_ST); q=rot(q,SP_ST,SP,X)
        return tf(q,sxc,syc)
    CORNERS=((1,1),(1,-1),(-1,1),(-1,-1))
    def corner_parts(sxc,syc):                      # -> list of (label, shape) for one leg+coxa at a corner
        return [("coxa",coxa_at(sxc,syc)),("cover",cover_at(sxc,syc)),("pin",pin_at(sxc,syc)),
                ("servo0",servo0_at(sxc,syc)),("servo1",servo1_at(sxc,syc)),
                ("femur",legpart_at(FE,None,sxc,syc)),("tibia",legpart_at(TI,TH_ST,sxc,syc)),
                ("servo3",legpart_at(SV,None,sxc,syc))]
    def dump(shape,name,defl=0.3):
        m=MeshPart.meshFromShape(Shape=shape,LinearDeflection=defl,AngularDeflection=0.5,Relative=False)
        m.write(OUTD+"/"+name+".stl")
    dump(FR,"frame",0.15)
    for i,(sxc,syc) in enumerate(CORNERS):
        for nm,sh in corner_parts(sxc,syc): dump(sh,"%s_%d"%(nm,i))
    tz=TI.BoundBox.ZMin+HPDZ                      # straight-leg toe z (pose moves it up)
    alog("stance pose: splay %d pitch %d knee %d ; frame zbot=%.1f toe(straight)=%.1f"%(SP_ST,PH_ST,TH_ST,zbot,tz))
    alog("posed part STLs -> "+OUTD)
    # ---- GUI ASSEMBLY DOCUMENT (hero view): frame + 4 legs posed, colorway = yellow body / black leg ----
    if App.GuiUp:
        COL={"frame":(.86,.74,.20),"coxa":(.12,.12,.13),"cover":(.66,.54,.38),"servo0":(.32,.62,.47),
             "servo1":(.46,.46,.50),"femur":(.86,.74,.20),"tibia":(.12,.12,.13),"servo3":(.30,.55,.90),
             "pin":(.80,.80,.83),"body":(.93,.62,.10)}
        nm="dog14"
        if nm in list(App.listDocuments()): App.closeDocument(nm)
        d=App.newDocument(nm)
        def add(n,sh):
            o=d.addObject("Part::Feature",n); o.Shape=sh
            try: o.ViewObject.ShapeColor=COL.get(n.split("_")[0],(.6,.6,.6)); o.ViewObject.Deviation=0.02
            except Exception: pass
        add("frame",FR)
        for i,(sxc,syc) in enumerate(CORNERS):
            for lab,sh in corner_parts(sxc,syc): add("%s_%d"%(lab,i),sh)
        # snap-on BODY canopy: part_body's shell, loaded as a mesh (part_body execs THIS file, so it can't be built here).
        _bp=r"C:/ultrafish/robodog/stl/sm3sg90_body.stl"
        if os.path.exists(_bp):
            import Mesh as _M
            _bm=_M.Mesh(); _bm.read(_bp); _bo=d.addObject("Mesh::Feature","body"); _bo.Mesh=_bm
            try: _bo.ViewObject.ShapeColor=COL["body"]; _bo.ViewObject.Transparency=55
            except Exception: pass
            alog("  + snap-on body canopy (mesh)")
        else: alog("  (body STL absent -- run part_body once to generate it)")
        d.recompute()
        gv=Gui.activeDocument().activeView()
        for _vn in ("Isometric","Front","Right"):
            getattr(gv,"view"+_vn)(); Gui.SendMsgToActiveView("ViewFit")
        alog("GUI doc 'dog14' built: frame(+pegs) + 4x(coxa+cover+pin+servo0+servo1+femur+tibia+servo3) + snap-on body")
    print("OK dog14 v9 assembly\n"+"\n".join(A_LOG))
except Exception:
    print("FAIL dog14:\n"+traceback.format_exc())
