# dogcommon.py -- split out of dog13.py (2026-07-19), VERBATIM.
# ALL shared constants + reusable helpers: primitives, report log, servo mesh cutter,
# the MEASURED hip/splay axes (SPZ/HPZ/S0DZ) and every joint constant, femz/tibz, tf/xmir/rring.
# This is the ONE place the axes live -- testfit/legwork/frameblock/kinematics must read them
# from here rather than keeping private copies (those copies are what went stale on 2026-07-19).
# Loaded by dog13.py via dinc(); execs into the CALLER's globals so the long-standing
# `exec(open('dog13.py').read())` contract (bodyview/export/frameview/coxablock/frameblock)
# keeps seeing every name flat, exactly as before.


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

FAST=globals().get('FAST',False)    # v28: downstream scripts (bodyview/assembly/testfit) set FAST=True to skip the gate sweeps + the 53-object doc render (they only need the geometry) -> avoids the 90s GUI timeout
BOSS_H,SPL_H=1.0,3.0
HUB_D,HORN_H,PLATE_T,ARM_R,ARM_W=7.18,4.12,1.7,17.0,7.2
HUB_CYL=HORN_H-PLATE_T; CASE=31.0; BRG0,BRG1=CASE+0.2,CASE+5.2
YSHIFT=7.2                    # widen stance: legs+hips outboard so cosmetic body reaches ~78mm (SM3 3.8:1); frame 60->74.4
DX=6.0                        # X1C bed-fit (v23): shift each hip/corner inboard -> frame 257->245mm (<=256 bed). HPx stays 106 for BUILD; HP/K (placement+gate centres) use HPx-DX; leg solids + frame corner feats translate -DX. Joints/bones/stance untouched; wheelbase narrows 2*DX.
FRM=(.32,.34,.38); YEL=(.95,.74,.10); DRK=(.14,.14,.16); GRY=(.45,.45,.48); STL=(.55,.60,.78); COR=(.85,.35,.19); TPU=(.20,.20,.22)
# v28 SERVO POCKET CUTTER: carve every servo pocket with the REAL SG90 model (not an amalgamation of boxes). Load the
# mesh, body-centre it, grow 0.25mm/side (per-axis so each face moves out evenly), -> one clean solid used as the cut tool.
# (HOISTED above the joint constants 2026-07-19: the hip axes are now MEASURED off these meshes, so HP/KZ depend on them.)
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
# ---- SPLAY AXIS (the fore-aft X hinge the coxa yaws about) -- 2026-07-19 ----
# MEASURED off the scanned servo meshes, NEVER from the idealised boss cylinders further down:
# on the EXI-style case the output spline is NOT centred in the bulky top block, so those
# idealised bosses sit 0.398 LOW. Z is taken from servo1's spline so that the splay axis and
# the hip-pitch axis INTERSECT (a proper 2-DOF hip); at the old z12 they were SKEW by 2mm.
def spline_axis(kind,ax):          # bbox ctr of the outermost slab along +ax = the spline axis
    m=servo_cut(kind); mb=m.BoundBox
    s=(m.common(box(1.5,300,300,v(mb.XMax-1.5,-150,-150))) if ax is X
       else m.common(box(300,1.5,300,v(-150,mb.YMax-1.5,-150)))).BoundBox
    return ((s.XMin+s.XMax)/2.0,(s.YMin+s.YMax)/2.0,(s.ZMin+s.ZMax)/2.0)
# BOTH hip axes are set by the SAME servo1 spline z, so they INTERSECT by construction.
HPZ=spline_axis("s1",Y)[2]              # hip-PITCH axis z = 10.398 (was the round 10.0)
HPDZ=HPZ-10.0                           # +0.398 -> the whole leg hangs 0.398 higher; bone LENGTHS unchanged
SPY,SPZ=20.0,HPZ                        # pre-yout SPLAY axis (y,z) = (20.0, 10.398)
SP=v(0,SPY+YSHIFT,SPZ)                  # splay centre for the swing/gate sweeps (post-yshift y)
# servo0 DRIVES the splay, so its spline must sit ON that axis -> drop the whole servo0 pocket
# (and every frame feature dimensioned off servo0's BODY) by this much. Compute BEFORE mutating.
S0DZ=SPZ-spline_axis("s0",X)[2]         # = -2.000
SRVCTR["s0"]=(SRVCTR["s0"][0],SRVCTR["s0"][1],SRVCTR["s0"][2]+S0DZ)   # moves servo_cut+wire_slot together
F=BRG1+0.2-HUB_CYL; HPx=106.0; HP=v(HPx-DX,0,HPZ); KW=1.7
# ---- SCALE-UP (user-approved +25%): fixed joints, longer bones ----
LEGSCALE=1.25; FEMLEN0=68.0
KZ=HPZ-FEMLEN0*LEGSCALE       # knee z = 10.398 - 85 = -74.602; the femur LENGTH (85) is unchanged,
DKZ=KZ-(-58.0)                # the whole leg just hangs HPDZ higher -> every bone/STL shape identical
TIBSC=1.25
K=v(HPx-DX,0,KZ)
def femz(z):
    # z in = the ORIGINAL design coords (hip at 10.0); out = scaled AND lifted onto the measured hip axis.
    if z>=10.0: return z+HPDZ               # hip dome rides up with the hip axis
    if z<=-58.0: return z+DKZ               # knee carrier shifts rigidly
    return HPZ+(z-10.0)*LEGSCALE            # shaft stretches, anchored at the hip axis
OBZ=-82.0; BZ=KZ-24.0                        # old / new fork-bridge z (fork depth 24 fixed)
SHSC=(78.0*TIBSC-24.0)/(78.0-24.0)           # shank stretch -> knee->ball = 78*TIBSC
def tibz(z): return BZ+(z-OBZ)*SHSC          # tibia shank/ball z, anchored at bridge (follows KZ)
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
