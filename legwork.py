
# legwork.py - fast isolated femur+tibia+boot harness for tibia AESTHETIC iteration.
# Joint mechanism is SETTLED. Only the cosmetic skin of the tibia shank/fork changes.
# Renders isolated views + a light fold gate + a ball-vs-knee-middle Y report.
import FreeCAD as App, FreeCADGui as Gui, Part, math, traceback
v=App.Vector; X,Y,Z=v(1,0,0),v(0,1,0),v(0,0,1)
OUT=r"C:/ultrafish/robodog/ref/iter"; RPT=OUT+"/legwork.txt"
def box(a,b,c,p): return Part.makeBox(a,b,c,p)
def cyl(r,h,p,d=Y): return Part.makeCylinder(r,h,p,d)
def rot(s,deg,ctr,ax): s=s.copy(); s.rotate(ctr,ax,deg); return s
def tri(pts,ext):
    w=Part.makePolygon([v(*p) for p in pts]+[v(*pts[0])]); return Part.Face(w).extrude(v(*ext))
L=[]
def flush(): open(RPT,"w").write("\n".join(L)+"\n")
try:
    BOSS_H,SPL_H=1.0,3.0
    HUB_D,HORN_H,PLATE_T,ARM_R,ARM_W=7.18,4.12,1.7,17.0,7.2
    HUB_CYL=HORN_H-PLATE_T; CASE=31.0; BRG0,BRG1=CASE+0.2,CASE+5.2
    F=BRG1+0.2-HUB_CYL; HPx=106.0; HP=v(HPx,0,10.0); SP=v(0,20,12); KW=1.7
    # ---- SCALE-UP (user-approved +25%): fixed joints, longer bones ----
    LEGSCALE=1.25                 # femur/tibia bone-length multiplier
    FEMLEN0=68.0                  # old femur length (hip z=10 -> knee z=-58)
    KZ=10.0-FEMLEN0*LEGSCALE      # new knee z  = 10 - 85 = -75
    DKZ=KZ-(-58.0)                # knee drop = -17  (shift for all knee-tied features)
    TIBSC=1.25                    # tibia shank-length multiplier (about knee)
    K=v(HPx,0,KZ)
    def femz(z):                  # remap a femur ST section z for the longer bone
        if z>=10.0: return z                    # hip dome: unchanged
        if z<=-58.0: return z+DKZ               # knee carrier: rigid shift with knee
        return 10.0+(z-10.0)*LEGSCALE           # shaft: stretch hip->knee
    OBZ=-82.0; BZ=KZ-24.0                       # old / new fork-bridge z (fork depth 24 fixed)
    SHSC=(78.0*TIBSC-24.0)/(78.0-24.0)          # shank stretch so knee->ball = 78*TIBSC
    def tibz(z): return BZ+(z-OBZ)*SHSC         # remap tibia shank/ball z, anchored at bridge
    YEL=(.95,.74,.10); DRK=(.14,.14,.16); STL=(.55,.60,.78); COR=(.85,.35,.19); TPU=(.20,.20,.22)
    def rring(z,hx,wy,rc):
        ri=2.5; y0=F; y1=F+wy; pts=[]
        def arc(cxa,cya,r,a0,a1,n):
            for i in range(n+1):
                a=a0+(a1-a0)*i/float(n); pts.append((cxa+r*math.cos(a),cya+r*math.sin(a)))
        arc(HPx-hx+ri,y0+ri,ri,math.pi,1.5*math.pi,3)
        arc(HPx+hx-ri,y0+ri,ri,1.5*math.pi,2.0*math.pi,3)
        arc(HPx+hx-rc,y1-rc,rc,0.0,0.5*math.pi,8)
        arc(HPx-hx+rc,y1-rc,rc,0.5*math.pi,math.pi,8)
        return Part.makePolygon([v(p[0],p[1],z) for p in pts]+[v(pts[0][0],pts[0][1],z)])
    def femur():
        # top/dome sections taper lateral width (wy) toward the top so the shoulder domes
        # from the top down to the outboard side (rounded, not cylinder); inboard face stays flat.
        ST=[(20.6,3.0,8.0,2.5),(19.4,5.5,14.0,5.0),(17.6,8.0,20.0,7.5),(15,9.8,25,9.0),(11.5,10.9,28,9.0),(10,11.0,28,9.0),
            (2,10.6,28,8.5),(-12,9.6,26,8),(-26,9.2,25.5,7.5),(-40,9.6,26,8),(-50,10.0,28,8),
            (-58,10.0,28,7.5),(-63,9.7,28,7.5),(-66.5,9.2,28,7),(-69.6,8.0,28,5.5),(-71.2,4.0,28,3.4)]
        ST=[(femz(z),hx,wy,rc) for (z,hx,wy,rc) in ST]   # lengthen shaft, keep hip dome + knee carrier
        fe=Part.makeLoft([rring(*s) for s in ST],True,False)
        fe=fe.cut(cyl(7.97,BRG1+0.05-F,v(HPx,F,10),Y)).cut(cyl(3.72,HUB_CYL+0.15,v(HPx,F-0.05,10),Y))
        fe=fe.cut(cyl(3.78,PLATE_T+0.6,v(HPx,F+HUB_CYL,10),Y)).cut(box(ARM_W,PLATE_T+0.6,ARM_R,v(HPx-ARM_W/2,F+HUB_CYL,10-ARM_R)))
        fe=fe.cut(cyl(2.8,28,v(HPx,F+HORN_H+0.4,10),Y)).cut(box(12.9,24.9,23.5,v(99.55,F-0.1,-64.3+DKZ)))
        for tzc in (-69.1,-41.3): fe=fe.cut(box(12.9,KW+7.3,5.4,v(99.55,F-0.1,tzc+DKZ)))
        fe=fe.cut(cyl(0.85,4.5,v(HPx,F+5.4,-66.3+DKZ),Y)).cut(cyl(0.85,4.5,v(HPx,F+5.4,-38.8+DKZ),Y))
        fe=fe.fuse(cyl(2.0,3.2,v(HPx,F+28,KZ),Y).fuse(Part.makeCone(2.0,1.3,1.4,v(HPx,F+31.2,KZ),Y)))
        s=fe.Solids; return max(s,key=lambda q:q.Volume) if len(s)>1 else fe

    YI=F-5.5
    KMID=F+14.0   # knee lateral middle == leg centerline (femur knob mid ~48); shank centers here
    BY=KMID       # ball foot lateral center -> directly under knee middle
    # ===================== TIBIA (cosmetic skin under work) =====================
    # rringT: cross-section centered laterally on cy, flat-ish inboard face (small ri corners),
    # rounded fore/aft/outboard (rc). Centering every section on KMID tilts the inboard face
    # inward as it descends and drops the ball straight under the knee.
    def rringT(z,xc,hx,cy,hy,rc):
        ri=3.0; ri=min(ri,hx-0.5,hy-0.5); rc=min(rc,hx-0.5,hy-0.5); y0=cy-hy; y1=cy+hy; pts=[]
        def arc(cxa,cya,r,a0,a1,n):
            for i in range(n+1):
                a=a0+(a1-a0)*i/float(n); pts.append((cxa+r*math.cos(a),cya+r*math.sin(a)))
        arc(xc-hx+ri,y0+ri,ri,math.pi,1.5*math.pi,4)
        arc(xc+hx-ri,y0+ri,ri,1.5*math.pi,2.0*math.pi,4)
        arc(xc+hx-rc,y1-rc,rc,0.0,0.5*math.pi,8)
        arc(xc-hx+rc,y1-rc,rc,0.5*math.pi,math.pi,8)
        return Part.makePolygon([v(p[0],p[1],z) for p in pts]+[v(pts[0][0],pts[0][1],z)])
    def clev_wire(y0,rr,bx0,bx1):
        pts=[(bx0,BZ),(bx1,BZ)]                  # bridge follows knee (BZ), arc centred on knee (KZ)
        for i in range(29):
            a=math.radians(-14.0+208.0*i/28.0)
            pts.append((HPx+rr*math.cos(a),KZ+rr*math.sin(a)))
        return Part.makePolygon([v(p[0],y0,p[1]) for p in pts]+[v(bx0,y0,BZ)])
    def clev(y0):   # flat prong (inboard = horn side, kept flat for the seat)
        return Part.Face(clev_wire(y0,8.5,97.0,115.0)).extrude(v(0,5.2,0))
    def clevB(y0):  # outboard prong: full inner face -> inset outer face = rounded/beveled lobe
        w1=clev_wire(y0,8.5,97.0,115.0)
        w2=clev_wire(y0+5.2,6.6,99.4,112.6)
        return Part.makeLoft([w1,w2],True,False)
    def tibia():
        armI=clev(YI)
        armO=clevB(F+28.4)
        # (z, xc, hx_foreaft, cy_lateral, hy_lateral, rc_outboard)
        ST=[(-78.0,106.0,8.8,KMID,19.55,8),(-84,107.2,9.7,KMID,15.5,9),(-92,107.8,10.0,KMID,12.5,9),
            (-102,107.4,9.3,KMID,10.25,8),(-108,107.2,8.7,KMID,9.65,7.8),(-116,106.4,8.3,KMID,9.25,7.4),
            (-123,104.9,7.8,KMID,9.1,7.0),(-129,102.8,7.4,KMID,9.0,6.5),(-134,100.2,7.2,KMID,9.0,6.0)]
        ST=[(tibz(z),xc,hx,cy,hy,rc) for (z,xc,hx,cy,hy,rc) in ST]   # stretch shank about the bridge
        body=Part.makeLoft([rringT(*s) for s in ST],True,False)
        p=Part.makeSphere(10.0,v(99.3,BY,tibz(-135.8)))
        t=armI.fuse(armO).fuse(body).fuse(p)
        t=t.cut(cyl(3.72,2.9,v(HPx,F-3.0,KZ),Y)).cut(cyl(3.78,2.0,v(HPx,F-4.7,KZ),Y))
        t=t.cut(box(ARM_W,2.0,ARM_R,v(HPx-ARM_W/2,F-4.7,KZ-ARM_R))).cut(cyl(2.1,1.2,v(HPx,F-5.6,KZ),Y))
        t=t.cut(cyl(4.46,4.5,v(HPx,F+29.1,KZ),Y)).cut(cyl(2.35,1.0,v(HPx,F+28.3,KZ),Y))
        s=t.Solids; return max(s,key=lambda q:q.Volume) if len(s)>1 else t
    def boot():
        C=v(99.3,BY,tibz(-135.8))
        o=Part.makeSphere(11.6,C).cut(Part.makeSphere(10.05,C))
        o=o.cut(box(14.6,20.0,12.5,v(C.x-7.3,BY-10.0,C.z)))     # chimney: shank entry, centered on BY
        o=o.cut(box(30,30,10,v(C.x-15.0,BY-15.0,C.z+4.5)))      # heel/top open
        o=rot(o,28.0,C,Y)
        s=o.Solids; return max(s,key=lambda q:q.Volume) if len(s)>1 else o
    # ===========================================================================

    FE=femur(); TIF=tibia(); BTF=boot()
    L.append("SOLIDS: fem=%d vol=%.0f  tib=%d vol=%.0f  boot=%d vol=%.0f"%(
        len(FE.Solids),FE.Volume,len(TIF.Solids),TIF.Volume,len(BTF.Solids),BTF.Volume))
    # knee lateral middle: fork prongs armI y[YI,YI+5.2], armO y[F+28.4,F+33.6]; femur knob y[F,F+28]
    forkmid=((YI)+(F+28.4+5.2))/2.0
    femmid=F+14.0
    bb=TIF.BoundBox; bbb=BTF.BoundBox
    L.append("Y-CENTER: forkMidY=%.2f  femurKnobMidY=%.2f  ballCtrY=%.2f  (ball vs forkMid=%.2f, vs femMid=%.2f)"%(
        forkmid,femmid,BY,BY-forkmid,BY-femmid))
    L.append("TIB bbox X[%.1f,%.1f] Y[%.1f,%.1f] Z[%.1f,%.1f]"%(bb.XMin,bb.XMax,bb.YMin,bb.YMax,bb.ZMin,bb.ZMax))
    L.append("BOOT bbox X[%.1f,%.1f] Y[%.1f,%.1f] Z[%.1f,%.1f]"%(bbb.XMin,bbb.XMax,bbb.YMin,bbb.YMax,bbb.ZMin,bbb.ZMax))
    # light fold gate (knee fold about K, front sense = negative)
    g=[]
    for a in (0,-30,-60,-90,-110,-115,-120):
        tq=rot(TIF,a,K,Y)
        g.append("%d=%.1f"%(a,FE.common(tq).Volume))
    L.append("FOLD GATE fem^tib: "+"  ".join(g))
    flush()

    nm="legdetail"
    if nm in list(App.listDocuments()): App.closeDocument(nm)
    d=App.newDocument(nm)
    for n,q,c in [("fe",FE,YEL),("ti",TIF,DRK),("bt",BTF,TPU)]:
        o=d.addObject("Part::Feature",n); o.Shape=q
        try:
            o.ViewObject.ShapeColor=c; o.ViewObject.Deviation=0.008; o.ViewObject.AngularDeflection=10.0
        except Exception: pass
    d.recompute()
    gv=Gui.activeDocument().activeView()
    for vn,fn in [("Right","legwork_front"),("Front","legwork_side"),("Axonometric","legwork_iso"),("Bottom","legwork_bottom")]:
        getattr(gv,"view"+vn)(); Gui.SendMsgToActiveView("ViewFit")
        gv.saveImage(OUT+"/"+fn+".png",1000,1000,"White")
    L.append("DONE render")
    print("OK legwork")
except Exception:
    L.append("FAIL: "+traceback.format_exc()); print("FAIL")
flush()
