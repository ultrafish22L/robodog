# legflat_v9.py == DIRECT-DRIVE knee (no linkage), scaled ~0.82x: femur 70 / tibia 80. Lightest-possible bones.
#   servo3 lies flat along the LOWER FEMUR (long axis down the thigh), output spline pokes laterally INBOARD (-Y) at
#   the knee K (servo FLIPPED 2026-07-22; body stays outboard); the tibia bolts straight to its horn. 1:1. Drops v8's
#   crank/pushrod/kneecap-lever/688/O8-pin.
#   Knee load carried by the servo output shaft, SINGLE-SHEAR -> MG90S (metal gears + output bushing) is MANDATORY
#   at the knee, not optional (user decision 2026-07-27). A printed inboard double-shear stub was built and dropped:
#   the audit flagged the PLA-on-PLA journal + thin bridges as a fresh failure point, and single-shear on MG90S is
#   the lighter, simpler load path. The tibia bolts to the MG90S horn; the M2 centre screw carries pull-off.
#   Y-LAYERS (inboard->outboard): tibia 28.7-35.7 | servo3 spline 34-36 on the knee | femur plate 36-43 |
#     servo3 tab flats -> Y45 | servo3 body hangs outboard to ~64. Servo body drops THROUGH a plate slot,
#     the two mounting tabs seat on offset flats (M2 self-tap), output spline pokes INBOARD to the tibia.
import FreeCAD as App, FreeCADGui as Gui, Mesh, Part, MeshPart, math, traceback
v=App.Vector; X,Y,Z=v(1,0,0),v(0,1,0),v(0,0,1)
def box(a,b,c,p): return Part.makeBox(a,b,c,p)
def cyl(r,h,p): return Part.makeCylinder(r,h,p,Y)
def disc(r,p,y0,yl): return Part.makeCylinder(r,yl,v(p[0],y0,p[1]),Y)
def plate(pts,y0,yl):
    w=Part.makePolygon([v(x,y0,z) for (x,z) in pts]+[v(pts[0][0],y0,pts[0][1])])
    return Part.Face(w).extrude(v(0,yl,0))
def biggest(s): q=s.Solids; return max(q,key=lambda k:k.Volume) if q else s
LOG=[]
def log(s): LOG.append(str(s))
try:
    XC=106.0; HIPZ=10.0; KZ=-60.0; TOEZ=-140.0        # femur 70, tibia 80
    BONE=7.0
    FIY0,FIY1=36.0,43.0            # femur inboard plate (7mm), hip flange sits inboard of this
    K=(XC,KZ)
    # ---- servo horn RECEIVER: 3 modes (mesh / round / arm) via horns.py (user 2026-07-28) ----
    if 'HORN_MODE' not in globals():
        import sys as _sys
        HORN_MODE='embed'
        for _a in _sys.argv[1:]:
            if _a.startswith('--horn='): HORN_MODE=_a.split('=',1)[1].strip().lower()
    exec(open(r"C:/ultrafish/robodog/horns.py").read())            # -> MODES, resolve, horn_void, ...  (exec, not import: no stale cache)
    CANON='arm'                                                    # leg canonical (LEG_SPEC: the STRONG joint); keeps the sm3sg90_v9_* names
    PRIMARY=resolve(HORN_MODE,CANON)
    RND_BONE=14.0                                                  # round-disc OD for the bones -- fits the O20 boss with a roof (coxa keeps O20)
    def slot(cx,z1,z2,r,y0,yl):                   # racetrack lightening tool (z1>z2), full bone thickness
        return box(2*r,yl,z1-z2,v(cx-r,y0,z2)).fuse(disc(r,(cx,z1),y0,yl)).fuse(disc(r,(cx,z2),y0,yl))
    # ---- servo3 mesh: long axis DOWN the thigh (Z), output spline INBOARD (-Y) at the knee (body stays outboard) ----
    def servo_base(clr):
        m=Mesh.Mesh(); m.read(r"C:/ultrafish/robodog/SG90-Servo.stl"); c=m.BoundBox.Center
        m.translate(-c.x,-c.y,-c.z)
        # anisotropic clearance = add exactly 2*clr to EACH native axis (0.5/side). Divide by the axis's OWN
        # length (not hardcoded body dims -- the mesh's native axes are {tabspan32.2, tall29.8, thin12.5}, so a
        # fixed {22.8,12.5,22.5} set landed the width divisor on the tall axis and under-clearanced the thin
        # slot axis to 0.28/side -- audit 2026-07-27). Self-correcting per axis.
        bb=m.BoundBox
        g=App.Matrix(); g.A11=1+2*clr/bb.XLength; g.A22=1+2*clr/bb.YLength; g.A33=1+2*clr/bb.ZLength
        m.transform(g); s=Part.Shape(); s.makeShapeFromMesh(m.Topology,0.05); return Part.makeSolid(s)
    # servo3 FLIPPED (user 2026-07-22): OUTPUT points INBOARD (-Y) so the tibia tucks on the INSIDE of the femur,
    # against its inner face; only the servo body sticks out outboard. Boss lands on the knee axis (drivable).
    BOSS_DZ=-5.62    # output-boss Z-offset from the servo centre in the flipped frame (measured off SG90-Servo.stl)
    def place_servo(clr):
        s=servo_base(clr)
        s.rotate(v(0,0,0),Y,90.0)          # output axis -> Y (boss on +Y), long axis -> Z (down thigh), thin -> X
        s.rotate(v(0,0,0),Z,180.0)         # FLIP: boss to -Y (inboard, tibia side); long axis stays down Z
        b=s.BoundBox
        # spline tip (bbox YMin) lands 2mm inboard of the femur inner face (FIY0) -> the spline reaches just inside
        # the femur and the tibia grips it there; boss (Z-offset BOSS_DZ from centre) lands ON the knee axis KZ.
        s.translate(v(XC-b.Center.x, (FIY0-2.0)-b.YMin, (KZ-BOSS_DZ)-b.Center.z))
        return s
    SVc=place_servo(0.5); SV=place_servo(0.0); sb=SV.BoundBox
    log("servo3 X[%.1f,%.1f] Y[%.1f,%.1f] Z[%.1f,%.1f]  (output -Y INBOARD; thin=%.1f tall=%.1f long=%.1f)"%(
        sb.XMin,sb.XMax,sb.YMin,sb.YMax,sb.ZMin,sb.ZMax,sb.XLength,sb.YLength,sb.ZLength))
    TIY1=FIY0-0.3; TIY0=TIY1-BONE                     # tibia INBOARD, outboard face 0.3 clr inside the femur inner face (max spline grip)
    log("tibia Y[%.1f,%.1f] (inboard, close to femur inner face %.1f)  servo body outboard to %.1f  leg width %.1f"%(
        TIY0,TIY1,FIY0,sb.YMax,sb.YMax-TIY0))
    # ============================ FEMUR (inboard plate, hip -> knee + servo3 SLIDE-IN SLOT) ============================
    def servo_mount(fe):
        # servo3 CLOSED POCKET, CAPPED TOP (user 2026-07-28): the housing wraps the servo BODY on +/-X, outboard +Y,
        # bottom -Z AND the top +Z (now CAPPED). The ONLY opening is the INBOARD -Y (spline/tibia) face: the servo
        # slides in LATERALLY from there, its output spline pokes out -Y to the tibia, and the tibia + M2 centre screw
        # cap the -Y opening = retention. Above the servo top the femur reverts to SOLID plate up to the hip. Walls
        # 2.0mm (W-CLR). (Was an open-+Z slide-DOWN channel; the audit reinforced the walls, this re-aims the insert.)
        b=SVc.BoundBox; x0,x1,y0,y1,z0,z1=b.XMin,b.XMax,b.YMin,b.YMax,b.ZMin,b.ZMax
        W=2.6; CLR=0.6; ZTOP=z1+1.0; HTOP=ZTOP+2.0     # cavity top +1mm over servo; +2.0mm top CAP; -Y open full height
        fe=fe.fuse(box((x1-x0)+2*W, (y1+W)-FIY0, HTOP-(z0-W), v(x0-W, FIY0, z0-W))); fe=biggest(fe)      # HOUSING (capped +Z, open -Y)
        # body cavity OPEN on -Y across the FULL servo height z0..ZTOP: the servo -- INCLUDING its full-width bottom
        # flange/tab -- slides in LATERALLY from the -Y side; the -Z wall (z0-W..z0) is the floor it rests on. (Was
        # BODZ0=z0+4.2, which left the plate solid below the servo bottom and BLOCKED the bottom tab -- user 2026-07-28.)
        fe=fe.cut(box((x1-x0)+2*CLR, (y1-FIY0)+CLR, ZTOP-z0, v(x0-CLR, FIY0, z0)))                       # body cavity (open -Y, full height)
        fe=fe.cut(box(x1-x0, FIY0-(y0-2.0), 11.0, v(x0, y0-2.0, KZ-5.5)))                                # OUTPUT slot -Y -> tibia (spline)
        for xc in (x0+3.0, x1-3.0):
            fe=fe.cut(cyl(0.9, W+2.0, v(xc, y1-1.0, (z0+z1)/2.0)))                                       # M2 flange pilots thru +Y wall
        return biggest(fe)
    def femur_blank():
        # rails through the servo zone were ~2.15mm (audit 2026-07-28); widen the mid-thigh half-widths so the two
        # plate rails flanking the servo cavity are ~3.4mm (rail = hx - 7.35, the cavity half-width).
        st=[(HIPZ,11.5),(0,11.5),(-15,10.75),(-30,10.75),(-45,10.75),(-55,10.5),(KZ,10.0)]
        front=[(XC+hx,z) for z,hx in st]; back=[(XC-hx,z) for z,hx in reversed(st)]
        dome=[(XC+11.5*math.cos(math.pi-math.pi*i/12),HIPZ+7*math.sin(math.pi-math.pi*i/12)) for i in range(1,12)]
        fe=plate(front+back+dome,FIY0,BONE); fe=biggest(fe)
        # HIP (inboard): O20 flange -- the horn receiver (mesh|round|arm) is applied AFTER, per-mode, in the export loop
        fe=fe.fuse(cyl(10.0,5.0,v(XC,FIY0-3.0,HIPZ))); fe=biggest(fe)
        fe=servo_mount(fe)                                            # servo3 closed pocket (2mm walls, closed box full span)
        # (removed the old z[-8,-22] upper-thigh lightening slot: the servo cavity now supersedes that region and the
        #  cut only nicked the reinforced rail edges -- audit 2026-07-28.)
        return biggest(fe)
    FE0=femur_blank()
    HIP_P=(XC,FIY0-3.0,HIPZ); HIP_D=(0,1,0); HIP_A=(0,0,1)        # hip receiver: mating face inboard, into-part +Y, arm along Z
    KNEE_P=(XC,TIY1,KZ);      KNEE_D=(0,-1,0); KNEE_A=(0,0,1)     # knee receiver: mating face outboard (servo side), into-part -Y, arm along Z
    FE=horn_void(FE0.copy(),HIP_P,HIP_D,HIP_A,PRIMARY,floor=1.0,mesh_len=5.0,m2=2.7,rnd_disk=RND_BONE,slot='down')   # PRIMARY variant for gates/dog14
    log("FEMUR valid=%s solids=%d vol=%.0f Y[%.1f,%.1f]"%(FE.isValid(),len(FE.Solids),FE.Volume,FE.BoundBox.YMin,FE.BoundBox.YMax))
    _fs=FE.common(SVc)
    log("  FEMUR^servo3 = %.1f mm3 (closed pocket, capped top; servo slides in LATERALLY from the -Y spline/tibia side)"%_fs.Volume)
    # ============================ TIBIA (INBOARD blade, on the servo spline) ============================
    def tibia_blank():
        head=disc(10.0,K,TIY0,BONE)                                     # knee head O20 at K=(106,-60), INBOARD of the femur
        # BLADE TOP MUST OVERLAP THE HEAD, not touch it tangentially. The O10 head bottom is z=-70; a blade whose top
        # edge is also at z=-70 meets it at the SINGLE point (106,-70), so head.fuse(plate) yields TWO disjoint solids
        # and biggest() silently DROPS the head -- taking the servo spline socket, the M2 clamp hole AND the knee stub
        # bore with it (audit 2026-07-27: the shipped tibia was a headless blade with no knee interface at all). Start
        # the blade top at z=-64 (a chord of the head, half-width ~9.17) so it shares VOLUME with the head -> one solid.
        zs=[-64.0,-95.0,-120.0,-135.0]; hw=[9.0,7.5,6.0,5.0]
        front=[(XC+w,z) for z,w in zip(zs,hw)]
        toe=[(XC+4.5*math.cos(-math.pi*i/12),TOEZ+4.5*math.sin(-math.pi*i/12)) for i in range(1,12)]
        back=[(XC-w,z) for z,w in zip(reversed(zs),reversed(hw))]
        ti=head.fuse(plate(front+toe+back,TIY0,BONE))
        assert len(ti.Solids)==1, "TIBIA head/plate fused into %d solids -- tangent contact dropped the head"%len(ti.Solids)
        ti=biggest(ti)
        # KNEE: mode-consistent interface on the OUTBOARD face grips servo3's inboard-poking output (embed = OEM
        # cross-horn capture, the STRONG joint; spline = printed 23T female). Socket Y[TIY1-2,TIY1]=[33.7,35.7]
        # over the O4.9 male spline Y[34,36] -> ~1.7mm engagement, the physical max (SG90 spline is only ~2mm).
        # An M2 CENTRE SCREW through the head clamps the tibia axially onto the output. NOTE: SG90 has no
        # factory-tapped output -- it's an M2 self-tapper into the boss cavity (M2.5 would split the O4.9 boss;
        # audit 2026-07-27); MG90S takes an M2 machine screw. O2.7 stays a clearance hole for the M2 shank.
        ti=biggest(ti.cut(disc(1.35,K,TIY0-1,BONE+2)))               # O2.7 M2 centre-screw shank clearance through the head (horn receiver applied AFTER, per-mode)
        # lightening: the shin has the room -- racetrack slots down the neutral axis, ~3.5mm rim kept
        ti=biggest(ti.cut(slot(XC,-72.0,-92.0,3.6,TIY0-1,BONE+2)))
        ti=biggest(ti.cut(slot(XC,-96.0,-116.0,3.2,TIY0-1,BONE+2)))
        # (ball foot removed per user 2026-07-28 -- tibia ends in the plain pointed blade toe again)
        return biggest(ti)
    TI0=tibia_blank()
    TI=horn_void(TI0.copy(),KNEE_P,KNEE_D,KNEE_A,PRIMARY,floor=0.4,mesh_len=2.0,m2=None,rnd_disk=RND_BONE,slot='down')   # PRIMARY variant for gates/dog14
    log("TIBIA valid=%s solids=%d vol=%.0f Y[%.1f,%.1f]"%(TI.isValid(),len(TI.Solids),TI.Volume,TI.BoundBox.YMin,TI.BoundBox.YMax))
    _ts=TI.common(SV)
    log("  TIBIA^servo3 = %.1f mm3 at the knee (the spline receiver engaging the output boss/shaft)"%_ts.Volume)
    # ============================ KNEE FOLD CHECK (tibia vs femur, tibia vs servo) ============================
    log("knee fold (mm3): th  TI^FE  TI^SV")
    worst=0
    for th in range(0,71,10):
        Tr=TI.copy(); Tr.rotate(v(XC,0,KZ),Y,th)
        c=[FE.common(Tr).Volume, SVc.common(Tr).Volume]
        worst=max([worst]+c)
        log("  %3d %6.0f %6.0f"%(th,c[0],c[1]))
    log("  worst 0-70 = %.0f mm3"%worst)
    # ============================ EXPORT + L/R MIRROR ============================
    # THREE print variants per bone (user 2026-07-28): canonical (no suffix) = ARM (LEG_SPEC strong joint; keeps the
    # names gen_urdf/dog14 read) + _mesh (printed spline) + _round alongside. Blanks built ONCE, horn_void applied per
    # mode -> 12 STLs (2 bones x 3 modes x L/R). See horns.py for the void geometry + the print-pause layer per mode.
    if globals().get('V9_PARTS_ONLY',False):
        raise SystemExit(0)         # composition mode (dog14): FE/TI/SV are built, skip export+render
    HPDZ=0.3984                     # rigid lift: build at nominal HIPZ=10.0, ship on the MEASURED hip axis 10.3984 (no bone deforms)
    SUFFIX={'arm':'','mesh':'_mesh','round':'_round'}
    def stl(shape,name):
        m=MeshPart.meshFromShape(Shape=shape,LinearDeflection=0.08,AngularDeflection=0.35,Relative=False)
        m.write(r"C:/ultrafish/robodog/stl/"+name+".stl"); return (m.CountFacets, name)
    def mir(shape,mc):
        mm=App.Matrix(); mm.A22=-1.0; s=shape.transformGeometry(mm); s.translate(v(0,2*mc,0)); return biggest(s)
    for mode in ('arm',):    # arm-only canonical export (user 2026-07-29); run with --horn=mesh/round for a variant
        FEx=horn_void(FE0.copy(),HIP_P,HIP_D,HIP_A,mode,floor=1.0,mesh_len=5.0,m2=2.7,rnd_disk=RND_BONE,slot='down'); FEx.translate(v(0,0,HPDZ))
        TIx=horn_void(TI0.copy(),KNEE_P,KNEE_D,KNEE_A,mode,floor=0.4,mesh_len=2.0,m2=None,rnd_disk=RND_BONE,slot='down'); TIx.translate(v(0,0,HPDZ))
        _bb=FEx.BoundBox.united(TIx.BoundBox); MIRC=(_bb.YMin+_bb.YMax)/2.0   # chiral pair mirrors about their shared centre
        for n,s in {"v9_femur":FEx,"v9_tibia":TIx}.items():
            for lab,sh in (("",s),("_mir",mir(s,MIRC))):
                _sh=stl(sh,"sm3sg90_"+n+SUFFIX[mode]+lab)
                log("  STL %-24s facets=%5d"%(_sh[1],_sh[0]))
    # ============================ RENDER (doc) ============================
    if App.GuiUp:
        nm="legv9"
        if nm in list(App.listDocuments()): App.closeDocument(nm)
        d=App.newDocument(nm)
        for n,s,col in [("femur",FE,(0.85,0.85,0.20)),("tibia",TI,(0.15,0.15,0.15)),("servo3",SV,(0.30,0.55,0.90))]:
            o=d.addObject("Part::Feature",n); o.Shape=s
            try: o.ViewObject.ShapeColor=col
            except Exception: pass
        d.recompute(); Gui.activeDocument().activeView().viewAxonometric(); Gui.SendMsgToActiveView("ViewFit")
    print("OK v9 direct-drive [%s]\n"%HORN_MODE+"\n".join(LOG))
except Exception:
    print("FAIL:\n"+traceback.format_exc())
