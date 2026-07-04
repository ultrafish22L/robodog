
import FreeCAD as App, FreeCADGui as Gui, Part, math, traceback
v=App.Vector; X,Y,Z=v(1,0,0),v(0,1,0),v(0,0,1)
OUT=r"C:/ultrafish/robodog/ref/iter"; RPT=OUT+"/leg13_dog13.txt"
def box(a,b,c,p): return Part.makeBox(a,b,c,p)
def cyl(r,h,p,d=Y): return Part.makeCylinder(r,h,p,d)
def rot(s,deg,ctr,ax): s=s.copy(); s.rotate(ctr,ax,deg); return s
def tri(pts,ext):
    w=Part.makePolygon([v(*p) for p in pts]+[v(*pts[0])]); return Part.Face(w).extrude(v(*ext))
def smoothstep(t): return t*t*(3.0-2.0*t)
L=[open(RPT).read().rstrip()]
def flush(): open(RPT,"w").write("\n".join(L)+"\n")
try:
    BOSS_H,SPL_H=1.0,3.0
    HUB_D,HORN_H,PLATE_T,ARM_R,ARM_W=7.18,4.12,1.7,17.0,7.2
    HUB_CYL=HORN_H-PLATE_T; CASE=31.0; BRG0,BRG1=CASE+0.2,CASE+5.2
    YSHIFT=7.2                    # widen stance: legs+hips outboard so cosmetic body reaches ~78mm (SM3 3.8:1); frame 60->74.4
    F=BRG1+0.2-HUB_CYL; HPx=106.0; HP=v(HPx,0,10.0); SP=v(0,20+YSHIFT,12); KW=1.7
    # ---- SCALE-UP (user-approved +25%): fixed joints, longer bones ----
    LEGSCALE=1.25; FEMLEN0=68.0
    KZ=10.0-FEMLEN0*LEGSCALE      # new knee z = 10 - 85 = -75
    DKZ=KZ-(-58.0)                # knee drop = -17 (shift for knee-tied features)
    TIBSC=1.25
    K=v(HPx,0,KZ)
    def femz(z):
        if z>=10.0: return z                    # hip dome unchanged
        if z<=-58.0: return z+DKZ               # knee carrier shifts rigidly
        return 10.0+(z-10.0)*LEGSCALE           # shaft stretches
    OBZ=-82.0; BZ=KZ-24.0                        # old / new fork-bridge z (fork depth 24 fixed)
    SHSC=(78.0*TIBSC-24.0)/(78.0-24.0)           # shank stretch -> knee->ball = 78*TIBSC
    def tibz(z): return BZ+(z-OBZ)*SHSC          # tibia shank/ball z, anchored at bridge
    FRM=(.32,.34,.38); YEL=(.95,.74,.10); DRK=(.14,.14,.16); GRY=(.45,.45,.48); STL=(.55,.60,.78); COR=(.85,.35,.19); TPU=(.20,.20,.22)
    def tf(q,sx,sy):
        q=q.copy()
        if sx>0:
            if sy<0: q=q.mirror(v(0,0,0),Y)
        else:
            if sy>0: q=q.mirror(v(0,0,0),Y)
            q.rotate(v(0,0,0),Z,180)
        return q
    def xmir(q): return q.mirror(v(HPx,0,0),X)
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
        # domed shoulder: top sections taper lateral width toward the top so the hip end rounds
        # smoothly from the top over to the outboard side (not cylinder-like); inboard face stays flat.
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
    KMID=F+14.0   # knee lateral middle == leg centerline; shank sections center here so the
    BY=KMID       # ball foot drops straight under the knee & the flat inboard face tilts in
    # rringT: flat-ish inboard face (small ri), rounded fore/aft/outboard (rc); centered on cy.
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
    def clev(y0):   # flat prong (inboard = horn seat side)
        return Part.Face(clev_wire(y0,8.5,97.0,115.0)).extrude(v(0,5.2,0))
    def clevB(y0):  # outboard prong: inset outer face -> rounded lobe continuing the shank
        return Part.makeLoft([clev_wire(y0,8.5,97.0,115.0),clev_wire(y0+5.2,6.6,99.4,112.6)],True,False)
    def tibia():
        armI=clev(YI)
        armO=clevB(F+28.4)
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
        o=o.cut(box(14.6,20.0,12.5,v(C.x-7.3,BY-10.0,C.z)))
        o=o.cut(box(30,30,10,v(C.x-15.0,BY-15.0,C.z+4.5)))
        o=rot(o,28.0,C,Y)
        s=o.Solids; return max(s,key=lambda q:q.Volume) if len(s)>1 else o
    def shoulder():
        sh=box(24.5,26.8,28.5,v(94,6,-8.5))
        sh=sh.cut(tri([(93.8,6,-0.5),(93.8,6,-8.5),(93.8,14,-8.5)],(25,0,0)))
        sh=sh.cut(tri([(93.8,32.8,-0.5),(93.8,32.8,-8.5),(93.8,24.8,-8.5)],(25,0,0)))
        top=[e for e in sh.Edges if all(abs(vv.Point.z-20.0)<1e-6 for vv in e.Vertexes)]
        try: sh=sh.makeFillet(4.0,top)
        except Exception:
            try: sh=sh.makeFillet(2.5,top)
            except Exception: pass
        sh=sh.cut(cyl(3.72,2.9,v(93.7,20,12),X)).cut(cyl(3.78,2.2,v(96.3,20,12),X))
        sh=sh.cut(box(2.2,ARM_W,ARM_R,v(96.3,20-ARM_W/2,12-ARM_R)))
        sh=sh.cut(cyl(2.7,15.2,v(98.4,20,12),X)).cut(cyl(4.4,2.2,v(111.4,20,12),X)).cut(cyl(7.93,5.4,v(113.2,20,12),X))
        sh=sh.cut(box(12.7,25.25,23.6,v(99.4,5.8,-7.0))).cut(cyl(8.75,2.0,v(HPx,30.9,10),Y))
        s=sh.Solids; return max(s,key=lambda q:q.Volume) if len(s)>1 else sh
    def frame():
        W=52.0+2*YSHIFT; Y0=-26.0-YSHIFT               # widened central body (60 -> 60+2*YSHIFT ~ 67.2 mm)
        TRAYZ=-30.0                                    # electronics tray floor (v21 was -40; raised to -30 for a slimmer SM3-ish body): internal cavity ~34 mm, frame 50 mm
        fr=box(185,W,5,v(-92.5,Y0,TRAYZ)).fuse(box(185,W,5,v(-92.5,Y0,9)))                     # deep bottom + top plates
        fr=fr.fuse(box(185,4,9-(TRAYZ+5),v(-92.5,22+YSHIFT,TRAYZ+5))).fuse(box(185,4,9-(TRAYZ+5),v(-92.5,Y0,TRAYZ+5)))   # tall side rails (tray..top)
        for sx in (1,-1): fr=fr.fuse(tf(box(3,W,14-TRAYZ,v(89.5,Y0,TRAYZ)),sx,1))              # hip end-cap bulkheads (full tray depth)
        # cross-body supports: a raised transverse bulkhead just inboard of each hip, tying left<->right hip mounts
        for sx in (1,-1): fr=fr.fuse(tf(box(6,W,18-TRAYZ,v(83.5,Y0,TRAYZ)),sx,1))              # rib x83.5..89.5, full tray depth (z TRAYZ..18)
        def ys(y): return y+YSHIFT                     # hip-mount features shift outboard with the hip
        adds=[box(39,20,5,v(89.5,ys(10),-16)),box(8,16,31,v(120.5,ys(12),-11)),box(22.5,14,6.3,v(70,ys(13),-11.1)),
              box(19.5,1.4,20,v(70,ys(12.5),-11)),box(19.5,1.4,20,v(70,ys(26.1),-11)),box(2,15,20,v(68,ys(12.5),-11)),
              # raised hip supports: taller inner/outer tray walls + front post that hug servo0 (22.7 tall)
              box(20.5,1.8,25,v(70,ys(11.9),-11)),box(20.5,1.8,25,v(70,ys(26.3),-11)),box(2.2,16.4,25,v(90.3,ys(11.9),-11))]
        cuts=[box(26,18,7,v(67,ys(11),8)),box(5,14.2,25,v(88.5,ys(12.9),-5.7)),box(3.1,14.2,5.4,v(85.05,ys(12.9),-10.0)),
              cyl(0.8,7,v(85.3,ys(20),-7.1),v(-1,0,0)),cyl(1.1,6,v(88.5,ys(20),-7.1),X),cyl(4.15,10,v(119.5,ys(20),12),X),
              box(24.5,6.2,21.5,v(69,ys(20.9),-11.2)),
              box(23.1,12.7,23.3,v(69.7,ys(13.65),-5.0)),box(3.0,12.7,32.7,v(85.1,ys(13.65),-9.7))]   # servo0 body+tab pocket (was overlapping the front hip post ~620mm3)
        for sx in (1,-1):
            for sy in (1,-1):
                for a in adds: fr=fr.fuse(tf(a,sx,sy))
                for c in cuts: fr=fr.cut(tf(c,sx,sy))
        fr=fr.removeSplitter()
        s=fr.Solids; return max(s,key=lambda q:q.Volume) if len(s)>1 else fr

    SH=shoulder(); FE=femur(); TIF=tibia(); TIR=xmir(TIF); BTF=boot(); BTR=xmir(BTF); FR=frame()
    servo0=box(22.5,12.1,22.7,v(70,13.95,-4.7)).fuse(box(2.4,12.1,32.1,v(85.4,13.95,-9.4))).fuse(cyl(2.9,BOSS_H,v(92.5,20,12),X)).fuse(cyl(1.7,BOSS_H+SPL_H,v(92.5,20,12),X))
    servo1=box(12.1,22.5,22.7,v(HPx-6.1,8.5,-6.7)).fuse(cyl(2.9,BOSS_H,v(HPx,31,10),Y)).fuse(cyl(1.7,BOSS_H+SPL_H,v(HPx,31,10),Y))
    servo2=box(12.1,22.5,22.7,v(HPx-6.05,F+KW,-63.9+DKZ)).fuse(cyl(2.9,BOSS_H,v(HPx,F+KW,KZ),v(0,-1,0))).fuse(cyl(1.7,BOSS_H+SPL_H,v(HPx,F+KW,KZ),v(0,-1,0)))
    b688a=cyl(8,5,v(113.4,20,12),X).cut(cyl(4.05,5,v(113.4,20,12),X))
    b688b=cyl(8,BRG1-BRG0,v(HPx,BRG0,10),Y).cut(cyl(4.02,BRG1-BRG0,v(HPx,BRG0,10),Y))
    b684=cyl(4.5,4.0,v(HPx,F+29.15,KZ),Y).cut(cyl(2.01,4.0,v(HPx,F+29.15,KZ),Y))
    hornH=cyl(3.59,HUB_CYL,v(HPx,F,10),Y).fuse(cyl(3.55,PLATE_T,v(HPx,F+HUB_CYL,10),Y))
    hornK=cyl(3.59,HUB_CYL,v(HPx,F-2.7,KZ),v(0,-1,0)).fuse(cyl(3.55,PLATE_T,v(HPx,F-2.7,KZ),v(0,-1,0)))
    pin=cyl(3.9,17.5,v(112.8,20,12),X).fuse(cyl(5.5,1.6,v(130.3,20,12),X))
    # widen: shift the whole leg assembly outboard so the hips sit at the new (wider) body edge.
    # knee sweeps are translation-invariant (FE & TIF move together) so gates stay 0; femur stays outboard of frame.
    def yout(q): q=q.copy(); q.translate(v(0,YSHIFT,0)); return q
    SH,FE,TIF,TIR,BTF,BTR=yout(SH),yout(FE),yout(TIF),yout(TIR),yout(BTF),yout(BTR)
    servo0,servo1,servo2=yout(servo0),yout(servo1),yout(servo2)
    b688a,b688b,b684,hornH,hornK,pin=yout(b688a),yout(b688b),yout(b684),yout(hornH),yout(hornK),yout(pin)

    def T(q,s,p,f,dofold,dopitch):
        if dofold: q=rot(q,f,K,Y)
        if dopitch: q=rot(q,p,HP,Y)
        return rot(q,s,SP,X)

    L.append("--- v22 (tray floor raised z-40->-30, frame 50mm/~34 cavity; stance widened for 3.8:1 body: body 60->%.1f, legs+hips shifted +%.1f) ---"%(60+2*YSHIFT,YSHIFT))
    L.append("SOLIDS v22: femur=%d vol=%.0f tibF=%d vol=%.0f boot=%d  FRAME=%d bbox Y=%.1f Z=%.1f"%(len(FE.Solids),FE.Volume,len(TIF.Solids),TIF.Volume,len(BTF.Solids),len(FR.Solids),FR.BoundBox.YLength,FR.BoundBox.ZLength))
    flush()
    r1=[]
    for a in (0,-15,-30,-45,-60,-75,-90,-105,-110,-115,-120):
        tq=rot(TIF,a,K,Y); bq=rot(BTF,a,K,Y)
        r1.append("%d=%.1f/%.1f/%.1f"%(a,FE.common(tq).Volume,servo2.common(tq).Volume,FE.common(bq).Volume))
    L.append("KNEE SWEEP FRONT v16 (fem^tib/servo2^tib/fem^boot): "+"  ".join(r1)); flush()
    r2=[]
    for a in (15,45,75,90,110,115,120):
        tq=rot(TIR,a,K,Y); bq=rot(BTR,a,K,Y)
        r2.append("%d=%.1f/%.1f/%.1f"%(a,FE.common(tq).Volume,servo2.common(tq).Volume,FE.common(bq).Volume))
    L.append("KNEE SWEEP REAR v16: "+"  ".join(r2)); flush()
    tw=T(TIF,-6,35,-63,1,1); bw=T(BTF,-6,35,-63,1,1)
    L.append("STANCE v16: tibF^frame=%.1f bootF^frame=%.1f  foot x[%.0f,%.0f] zmin=%.1f bootzmin=%.1f"%(FR.common(tw).Volume,FR.common(bw).Volume,tw.BoundBox.XMin,tw.BoundBox.XMax,tw.BoundBox.ZMin,bw.BoundBox.ZMin))
    L.append("SERVO2 seat: servo2^fem=%.1f  (tab screws M2 @ z-66.3/-38.8, pilots r0.85)"%(servo2.common(FE).Volume))
    r3=[]
    for a in range(0,360,30):
        r3.append("%d=%.1f"%(a,rot(FE,a,HP,Y).common(FR).Volume))
    L.append("ROUND femur^FRAME sweep (wider frame + new ribs): "+"  ".join(r3))
    flush()

    def leg(s,p,f,TI,BT):
        return [("sh",T(SH,s,p,f,0,0),DRK,60),("b688a",T(b688a,s,p,f,0,0),STL,0),("s1",T(servo1,s,p,f,0,0),GRY,0),
                ("fe",T(FE,s,p,f,0,1),YEL,0),("b688b",T(b688b,s,p,f,0,1),STL,0),("hH",T(hornH,s,p,f,0,1),COR,0),("s2",T(servo2,s,p,f,0,1),GRY,0),
                ("ti",T(TI,s,p,f,1,1),DRK,0),("hK",T(hornK,s,p,f,1,1),COR,0),("b684",T(b684,s,p,f,1,1),STL,0),("bt",T(BT,s,p,f,1,1),TPU,0)]

    nm="dog13"
    if nm in list(App.listDocuments()): App.closeDocument(nm)
    doc=App.newDocument(nm)
    POSES={"stance":{"F":(-6,35,-63),"R":(-6,-35,63)}}
    tag="stance"
    parts=[("frame",FR,FRM,0)]
    for sx in (1,-1):
        pose=POSES[tag]["F"] if sx>0 else POSES[tag]["R"]
        TI=TIF if sx>0 else TIR; BT=BTF if sx>0 else BTR
        for sy in (1,-1):
            parts.append(("s0_%d%d"%(sx,sy),tf(servo0,sx,sy),GRY,0))
            parts.append(("pin_%d%d"%(sx,sy),tf(pin,sx,sy),STL,0))
            for n,q,c,t in leg(pose[0],pose[1],pose[2],TI,BT):
                parts.append(("%s_%d%d"%(n,sx,sy),tf(q,sx,sy),c,t))
    for n,q,c,t in parts:
        o=doc.addObject("Part::Feature",n); o.Shape=q
        try:
            o.ViewObject.ShapeColor=c; o.ViewObject.Transparency=t
        except Exception: pass
    doc.recompute()
    gv=Gui.activeDocument().activeView()
    gv.viewFront(); Gui.SendMsgToActiveView("ViewFit")
    gv.saveImage(OUT+"/dog13_stance_Front.png",1200,900,"White")
    gv.viewRight(); Gui.SendMsgToActiveView("ViewFit")
    gv.saveImage(OUT+"/dog13_stance_Right.png",1200,900,"White")
    gv.viewAxonometric(); Gui.SendMsgToActiveView("ViewFit")
    gv.saveImage(OUT+"/dog13_stance_Isometric.png",1200,900,"White")
    L.append("DONE v22")
    print("OK dog13 v22")
except Exception:
    L.append("FAIL v22: "+traceback.format_exc())
    print("FAIL: "+traceback.format_exc())
flush()

