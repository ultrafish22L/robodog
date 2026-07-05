
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
    FAST=globals().get('FAST',False)    # v28: downstream scripts (bodyview/assembly/testfit) set FAST=True to skip the gate sweeps + the 53-object doc render (they only need the geometry) -> avoids the 90s GUI timeout
    BOSS_H,SPL_H=1.0,3.0
    HUB_D,HORN_H,PLATE_T,ARM_R,ARM_W=7.18,4.12,1.7,17.0,7.2
    HUB_CYL=HORN_H-PLATE_T; CASE=31.0; BRG0,BRG1=CASE+0.2,CASE+5.2
    YSHIFT=7.2                    # widen stance: legs+hips outboard so cosmetic body reaches ~78mm (SM3 3.8:1); frame 60->74.4
    DX=6.0                        # X1C bed-fit (v23): shift each hip/corner inboard -> frame 257->245mm (<=256 bed). HPx stays 106 for BUILD; HP/K (placement+gate centres) use HPx-DX; leg solids + frame corner feats translate -DX. Joints/bones/stance untouched; wheelbase narrows 2*DX.
    F=BRG1+0.2-HUB_CYL; HPx=106.0; HP=v(HPx-DX,0,10.0); SP=v(0,20+YSHIFT,12); KW=1.7
    # ---- SCALE-UP (user-approved +25%): fixed joints, longer bones ----
    LEGSCALE=1.25; FEMLEN0=68.0
    KZ=10.0-FEMLEN0*LEGSCALE      # new knee z = 10 - 85 = -75
    DKZ=KZ-(-58.0)                # knee drop = -17 (shift for knee-tied features)
    TIBSC=1.25
    K=v(HPx-DX,0,KZ)
    def femz(z):
        if z>=10.0: return z                    # hip dome unchanged
        if z<=-58.0: return z+DKZ               # knee carrier shifts rigidly
        return 10.0+(z-10.0)*LEGSCALE           # shaft stretches
    OBZ=-82.0; BZ=KZ-24.0                        # old / new fork-bridge z (fork depth 24 fixed)
    SHSC=(78.0*TIBSC-24.0)/(78.0-24.0)           # shank stretch -> knee->ball = 78*TIBSC
    def tibz(z): return BZ+(z-OBZ)*SHSC          # tibia shank/ball z, anchored at bridge
    FRM=(.32,.34,.38); YEL=(.95,.74,.10); DRK=(.14,.14,.16); GRY=(.45,.45,.48); STL=(.55,.60,.78); COR=(.85,.35,.19); TPU=(.20,.20,.22)
    # v28 SERVO POCKET CUTTER: carve every servo pocket with the REAL SG90 model (not an amalgamation of boxes). Load the
    # mesh, body-centre it, grow 0.25mm/side (per-axis so each face moves out evenly), -> one clean solid used as the cut tool.
    import Mesh
    def _servo_base(clr):
        m=Mesh.Mesh(); m.read(r"C:/ultrafish/robodog/SG90-Servo.stl"); c=m.BoundBox.Center
        m.translate(-c.x,-c.y+3.6,-c.z)                                                   # body centre -> origin (boss skews bbox +3.6 Y)
        g=App.Matrix(); g.A11=1+2*clr/22.8; g.A22=1+2*clr/22.5; g.A33=1+2*clr/12.5         # grow clr/side (len 22.8 / height 22.5 / width 12.5)
        m.transform(g); s=Part.Shape(); s.makeShapeFromMesh(m.Topology,0.05); return Part.makeSolid(s)
    SRVBASE=_servo_base(0.25)
    SRVMAT={"s0":App.Matrix(0,1,0,0,0,0,1,0,1,0,0,0,0,0,0,1),      # local (L=+X,W=+Z,Hout=+Y) -> pocket axes, per servo
            "s1":App.Matrix(0,0,-1,0,0,1,0,0,1,0,0,0,0,0,0,1),
            "s2":App.Matrix(0,0,1,0,0,-1,0,0,1,0,0,0,0,0,0,1)}
    SRVCTR={"s0":(81.25,27.2,6.65),"s1":(105.95,19.75,4.65),"s2":(106.0,46.93,-69.55)}   # body centre in each part's BUILD frame (s0=frame pre-dxin; s1/s2=coxa/femur pre-yout)
    def servo_cut(kind):
        c=SRVBASE.copy(); c.transformShape(SRVMAT[kind]); c.translate(v(*SRVCTR[kind])); return c
    # WIRE SLOT: the SG90 lead exits a NUB on the +X end face, low (local Y~-5), centred in width (local Z). Cut a slot there
    # (8mm wide for the connector) running from just inside the case OUT through the pocket wall, and down the low-Y range so the
    # wire can exit at the nub OR lower on the body. Same per-servo transform as the pocket -> lands right for s0/s1/s2.
    def wire_slot(kind):
        ws=Part.makeBox(34,8,8,v(9,-11,-4))   # local: X9..43 (through wall), Y-11..-3 (nub + lower variation), Z-4..4 (8mm connector)
        ws.transformShape(SRVMAT[kind]); ws.translate(v(*SRVCTR[kind])); return ws
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
        fe=fe.cut(cyl(2.8,28,v(HPx,F+HORN_H+0.4,10),Y)).cut(servo_cut("s2")).cut(wire_slot("s2"))   # v28 servo2 pocket = SG90 model cut + wire slot at the nub
        fe=fe.cut(cyl(0.85,4.5,v(HPx,F+5.4,-66.3+DKZ),Y)).cut(cyl(0.85,4.5,v(HPx,F+5.4,-38.8+DKZ),Y))   # M2 tab-screw pilots into the flange-rest walls
        fe=fe.fuse(cyl(2.0,3.2,v(HPx,F+28,KZ),Y).fuse(Part.makeCone(2.0,1.3,1.4,v(HPx,F+31.2,KZ),Y)))
        # v28 push-fit ridges on both X-walls of the model-cut servo2 pocket (~0.35 proud into the 0.25 gap -> ~0.1 crush)
        fe=fe.fuse(box(0.35,18,14,v(99.5,F,-62+DKZ))).fuse(box(0.35,18,14,v(112.15,F,-62+DKZ)))
        # v26 WIRE ROUTING: groove up the femur inboard (print-bed) face from the servo2 (knee/"servo3") cavity to a mouth just
        # below the hip-PITCH bearing rim (axis x106,z10; 688 lower rim z~2). Femur pitches a FULL CIRCLE about that axis, so the
        # lead exits ~10mm below it (arc r~10mm) and twists rather than winding a 60-90mm arc. Sized 4.2w x 3.0deep so a real SG90
        # 3-wire bundle seats past FDM elephant-foot (verify-panel: 3.5 nominal squishes to ~2.7); mouth recessed to z-3 so the
        # lead clears the 688 OD when the bearing is seated. Prints inboard-face-DOWN (slicer: brim + ~0.15mm e-foot comp).
        fe=fe.cut(box(4.2,3.5,55,v(102.65,F-0.5,-58)))
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
        sh=sh.cut(servo_cut("s1")).cut(wire_slot("s1")).cut(cyl(8.75,2.0,v(HPx,30.9,10),Y))   # v28 servo1 pocket = SG90 model cut + wire slot at the nub
        # v28 push-fit ridges on both X-walls of the model-cut servo1 pocket (~0.35 proud into the 0.25 gap -> ~0.1 crush)
        # (the v27 box tab-slots are gone: the model cut already carves the flange + ears)
        sh=sh.fuse(box(0.35,18,15,v(99.45,10,-3))).fuse(box(0.35,18,15,v(112.1,10,-3)))
        # v26 WIRE ROUTING: splay(yaw)-axis eyelet. The coxa yaws +-45 about SP (a fore-aft/X line at build (20,12)); a bore along
        # -X exiting the inboard face into the frame corner well, offset ~7mm below the yaw axis to clear the O7.5 yaw-bearing bore
        # -> ~7mm arc radius (a small twist, not a wind-up). O5 (not O4) so the servo1+servo2 BUNDLE (~4-5mm) passes after a
        # horizontal FDM bore sags the crown ~0.6mm (verify-panel).
        sh=sh.cut(cyl(2.5,10,v(95,20,5),v(-1,0,0)))
        # v26 WIRE ROUTING "complete the path": (a) RECEIVER scoop just below the pitch-bearing seat (rim z~1.25) that drops the
        # femur/knee lead off its groove mouth into the servo1 pocket to bundle; (b) POCKET->EYELET connector through the ~4.4mm
        # wall between the servo1 pocket (x>=99.4) and the eyelet (x95) so the bundle actually reaches the bore; (c) flared lead-in
        # at the eyelet exit lip so the splay-twisting bundle bears on a radius, not a printed edge.
        sh=sh.cut(box(6,5,6,v(102,28,-5)))                             # receiver: femur-lead crossing -> servo1 pocket
        sh=sh.cut(box(6,4,4,v(94,18,3)))                               # bundle: servo1 pocket -> eyelet mouth
        sh=sh.cut(Part.makeCone(2.5,3.6,1.5,v(93.7,20,5),v(-1,0,0)))   # flared eyelet exit lip
        s=sh.Solids; return max(s,key=lambda q:q.Volume) if len(s)>1 else sh
    def frame():
        W=52.0+2*YSHIFT; Y0=-26.0-YSHIFT               # widened central body (60 -> 60+2*YSHIFT ~ 67.2 mm)
        TRAYZ=-30.0                                    # electronics tray floor (v21 was -40; raised to -30 for a slimmer SM3-ish body): internal cavity ~34 mm, frame 50 mm
        def dxin(s): s=s.copy(); s.translate(v(-DX,0,0)); return s   # bed-fit: pull each corner feature inboard by DX (matches -DX leg shift); central plates/rails stay put
        PL=185-2*DX; PX=-(92.5-DX)   # bed-fit: shorten plates/rails 2*DX so the inboard-shifted coxa/servo1 clear the deck ends (their inboard edge moved to x~88; deck now ends x~86.5)
        SIDE=34.25; TOPZ=18.0; FLR=TRAYZ           # v28: flat outer side (1mm past servo0 y33.25), tub rim, floor
        # v28 ELECTRONICS TUB: a proper open-top tub filling the space between the hips - 2mm floor + 2mm long side walls
        # (outer face = the flat frame side at y=+-SIDE) + the hip bulkheads/cross-rib as short walls; open top for the
        # cover with a 1cm cross-strip mid-span for rigidity. servo0 pockets carve into the corners (leaving a 1mm outer cover).
        fr=box(PL,2*SIDE,2,v(PX,-SIDE,FLR))                                                       # 2mm tub floor
        fr=fr.fuse(box(PL,2,TOPZ-FLR,v(PX,SIDE-2,FLR))).fuse(box(PL,2,TOPZ-FLR,v(PX,-SIDE,FLR)))  # 2mm long side walls (floor->rim)
        fr=fr.fuse(box(10,2*SIDE,2,v(-5,-SIDE,TOPZ-2)))                                            # 1cm top cross-strip (rigidity)
        for sx in (1,-1): fr=fr.fuse(tf(dxin(box(3,2*SIDE,TOPZ-FLR,v(82.4,-SIDE,FLR))),sx,1))     # front/back tub end walls moved IN to the servo0 flange plane (build x82.4-85.4 -> world x76.4-79.4) so the tabs sit against them
        for sx in (1,-1): fr=fr.fuse(tf(dxin(box(6,40,TOPZ-FLR,v(83.5,-20,FLR))),sx,1))            # x83.5 cross-rib, CENTRAL only (y+-20) so it doesn't run through the servo0 corners
        # v23 L/R shoulder girdle: top + bottom crossbars tie the left+right bearing pillars (at the nose & tail) into one
        # rigid axle -> kills the fore-aft fold + gives the O8 pin a 2nd shear support. y+-28 stays under the cover hip openings.
        for sx in (1,-1): fr=fr.fuse(tf(dxin(box(8,56,3.5,v(120.5,-28,16.5))),sx,1))            # top bar ties pillar tops (z20)
        for sx in (1,-1): fr=fr.fuse(tf(dxin(box(8,56,3.5,v(120.5,-28,-16))),sx,1))             # bottom bar ties bridge/pillar bottoms
        def ys(y): return y+YSHIFT                     # hip-mount features shift outboard with the hip
        # v24 FLAT FRAME SIDE: every outboard hip feature pulled to y<=ys(26)=33.2 (= central-plate edge = servo0 outer wall),
        # so the frame side is ONE plane. servo0's outer face is now flush with that side (the bulging outer well wall + outer
        # thin wall are removed); servo0 retention = M2 tab screws + inner wall + seat + closed top. Coxa/femur stay outboard (moving parts).
        adds=[box(39,16,7,v(89.5,ys(10),-18)),box(8,15,31,v(120.5,ys(11.5),-11)),box(22.5,13,6.3,v(70,ys(13),-11.1)),  # v29 splay pillar widened 14->15 (+Y) for socket wall
              box(19.5,1.4,20,v(70,ys(12.5),-11)),box(2,13.5,20,v(68,ys(12.5),-11)),
              box(20.5,1.8,25,v(70,ys(11.9),-11)),box(2.2,14.1,25,v(90.3,ys(11.9),-11)),
              # fore-aft gussets: outer web pulled inboard to the flat side (outer 33.2) + inner web; floor->bridge->pillar, below the coxa splay arc
              box(45,3,19,v(83.5,ys(23.0),-30)),box(45,3,19,v(83.5,ys(18.5),-30)),
              box(22.5,15,1.6,v(70,ys(12),18)),   # v28 servo0 TOP CAP (z18-19.6, 1.6mm above the servo top) -> servo0 covered on top; only the +X hip side stays open
              box(24.5,16,31,v(68,ys(11),-11))]   # v28 FILL block round servo0 so the model cut leaves clean 5-sided walls; its inboard face (dxin x64) = the electronics-bay wall (motor end)
        # v28: the SG90 model cut IS the servo0 pocket now. All the old box-pocket relief cuts are DELETED (one of them,
        # box(24.5,6.2,21.5,v(69,ys(20.9),..)), was carving y28-34 = the whole outboard cover -> that opened the side).
        # v28 servo inserts FROM THE HIP SIDE (+X): the whole region beyond the seated tab must be clear of frame - the tab/body/
        # output/insertion path all live there. Cut everything outboard of the tab plane (world x79.4 -> build x85.4) in the servo0
        # footprint, up to just short of the O8 pillar. Only the tab-seating wall (the moved bulkhead, ending at x79.4) remains.
        cuts=[box(15.6,12,32.1,v(85.4,ys(13.8),-9.4)),                                       # CLEAR everything beyond the tab (hip side)
              cyl(0.8,7,v(85.3,ys(20),-7.1),v(-1,0,0)),cyl(1.1,6,v(88.5,ys(20),-7.1),X),     # servo0 M2 tab-screw pilot + head clearance
              cyl(4.15,10,v(119.5,ys(20),12),X),                                             # O8 splay-pin bore in the pillar (servo0 wire slot is cut via wire_slot("s0") in the loop, at the actual nub)
              box(3.6,9.6,9.6,v(125.2,ys(15.2),7.2))]                                         # v29 square SOCKET for the recessed pin head (world x119.2-122.8) = anti-rotation key, keeps the head inside the frame face  # v26 servo0-lead channel: pass-through in the x83.5 cross-rib EXTENDED up (6->10) to overlap the servo0 pocket floor (z-5), so the static yaw-stator lead drops straight from its pocket into the central bay (no arc needed - servo0 crosses no moving joint)
        cr=[box(15,0.5,15,v(72,ys(13.65),-3)),   # v24 crush rib on servo0's inner pocket wall only (outer face is now the flush frame side)
            box(1.0,7,12,v(127.5,ys(16.5),7))]   # v29 splay-pin RETAINER: vertical snap clip standing in the socket mouth (world x121.5-122.5, flush with the frame face), caps the recessed head; anchored at top in solid pillar above the socket, free lower end flexes outboard into open air to insert/release the pin. Fused in cr (AFTER the socket cut) so it survives.
        for sx in (1,-1):
            for sy in (1,-1):
                for a in adds: fr=fr.fuse(tf(dxin(a),sx,sy))
                fr=fr.cut(tf(dxin(servo_cut("s0")),sx,sy))    # v28 servo0 pocket = SG90 model cut into the filled corner (clean single cut)
                fr=fr.cut(tf(dxin(wire_slot("s0")),sx,sy))     # v28 servo0 wire slot at the nub (8mm connector + variation), through the wall
                for c in cuts: fr=fr.cut(tf(dxin(c),sx,sy))
                for r in cr: fr=fr.fuse(tf(dxin(r),sx,sy))
        # v28 the tub is open-top (no deck) so the old deck windows are gone; keep only the side power/charge/switch port
        fr=fr.cut(box(6,8,8,v(-52,26,-4)))
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
    pin=cyl(3.9,12.7,v(112.8,20,12),X).fuse(box(2.0,9,9,v(125.5,15.5,7.5)))   # v29 SQUARE head (9x9x2), shaft shortened so the head recesses into the pillar socket (keyed=anti-rotation) -> frame stays <=246
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

