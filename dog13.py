
# dog13.py -- ASSEMBLER ONLY (2026-07-19 refactor).
# Every unique part now lives in its own file; this script just loads them, builds the
# parts, places the 4 legs and renders/reports. Geometry is unchanged (verified against
# ref/iter/_refactor_baseline.json: volumes/bboxes/solid-counts identical).
#
#   dogcommon.py     constants + reusable funcs (THE source of truth for the axes)
#   part_femur.py  part_tibia.py  part_boot.py  part_shoulder.py  part_frame.py
#   part_dummies.py  display/gate proxies
#
# dinc() execs each file into THIS namespace (not `import`) on purpose: downstream scripts
# do `exec(open('dog13.py').read())` and expect FR/SH/FE/TIF/parts/servo_cut/tf/box/cyl to
# land flat in their globals -- and `import` would also cache stale modules across a warm
# FreeCAD session, which is exactly the wrong behaviour while iterating on geometry.
_RD=r"C:/ultrafish/robodog"
def dinc(_n):
    exec(compile(open(_RD+"/"+_n).read(),_n,"exec"),globals())

try:
    dinc("dogcommon.py")        # -> v,X,Y,Z,box,cyl,rot,tri, FAST, L/flush, servo_cut, spline_axis,
                               #    wire_slot, HPZ/SPZ/SPY/SP/S0DZ/HP/K/KZ, femz/tibz, tf/xmir/rring
    for _p in ("part_femur.py","part_tibia.py","part_boot.py","part_shoulder.py","part_frame.py"):
        dinc(_p)               # -> femur() tibia() boot() shoulder() frame()

    # ---- build every part, then the display proxies, then shift the whole assembly ----
    SH=shoulder(); FE=femur(); TIF=tibia(); TIR=xmir(TIF); BTF=boot(); BTR=xmir(BTF); FR=frame()
    dinc("part_dummies.py")     # -> servo0/1/2, b688a/b688b/b684, hornH/hornK, pin
    # widen + bed-fit: shift the whole leg assembly outboard (+YSHIFT) AND inboard (-DX) so the hips sit at
    # the widened body edge and the frame fits the X1C bed. Rotations preserve both shifts so gates stay 0.
    def yout(q): q=q.copy(); q.translate(v(-DX,YSHIFT,0)); return q
    SH,FE,TIF,TIR,BTF,BTR=yout(SH),yout(FE),yout(TIF),yout(TIR),yout(BTF),yout(BTR)
    servo0,servo1,servo2=yout(servo0),yout(servo1),yout(servo2)
    b688a,b688b,b684,hornH,hornK,pin=yout(b688a),yout(b688b),yout(b684),yout(hornH),yout(hornK),yout(pin)

    def T(q,s,p,f,dofold,dopitch):
        if dofold: q=rot(q,f,K,Y)
        if dopitch: q=rot(q,p,HP,Y)
        return rot(q,s,SP,X)

    L.append("--- v24 (flat frame side y=%.1f; X1C bed-fit DX=%.1f frame X=%.1fmm; +servo crush ribs + cavity access) ---"%(FR.BoundBox.YMax,DX,FR.BoundBox.XLength))
    L.append("SOLIDS v23: femur=%d vol=%.0f tibF=%d vol=%.0f boot=%d  FRAME=%d bbox X=%.1f Y=%.1f Z=%.1f"%(len(FE.Solids),FE.Volume,len(TIF.Solids),TIF.Volume,len(BTF.Solids),len(FR.Solids),FR.BoundBox.XLength,FR.BoundBox.YLength,FR.BoundBox.ZLength))
    BED=256.0
    L.append("BED-FIT (X1C %g): frame X=%.1f Y=%.1f Z=%.1f -> %s (want all<=%g, ideally<=246 ABS margin)"%(BED,FR.BoundBox.XLength,FR.BoundBox.YLength,FR.BoundBox.ZLength,"OK" if max(FR.BoundBox.XLength,FR.BoundBox.YLength,FR.BoundBox.ZLength)<=BED else "OVER",BED))
    flush()
    r1=[]
    for a in (() if FAST else (0,-15,-30,-45,-60,-75,-90,-105,-110,-115,-120)):
        tq=rot(TIF,a,K,Y); bq=rot(BTF,a,K,Y)
        r1.append("%d=%.1f/%.1f/%.1f"%(a,FE.common(tq).Volume,servo2.common(tq).Volume,FE.common(bq).Volume))
    L.append("KNEE SWEEP FRONT v16 (fem^tib/servo2^tib/fem^boot): "+"  ".join(r1)); flush()
    r2=[]
    for a in (() if FAST else (15,45,75,90,110,115,120)):
        tq=rot(TIR,a,K,Y); bq=rot(BTR,a,K,Y)
        r2.append("%d=%.1f/%.1f/%.1f"%(a,FE.common(tq).Volume,servo2.common(tq).Volume,FE.common(bq).Volume))
    L.append("KNEE SWEEP REAR v16: "+"  ".join(r2)); flush()
    if not FAST:
        tw=T(TIF,-6,35,-63,1,1); bw=T(BTF,-6,35,-63,1,1)
        L.append("STANCE v16: tibF^frame=%.1f bootF^frame=%.1f  foot x[%.0f,%.0f] zmin=%.1f bootzmin=%.1f"%(FR.common(tw).Volume,FR.common(bw).Volume,tw.BoundBox.XMin,tw.BoundBox.XMax,tw.BoundBox.ZMin,bw.BoundBox.ZMin))
        L.append("SERVO2 seat: servo2^fem=%.1f  (tab screws M2 @ z-66.3/-38.8, pilots r0.85)"%(servo2.common(FE).Volume))
    r3=[]
    for a in (() if FAST else range(0,360,30)):
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
    if not FAST:
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
