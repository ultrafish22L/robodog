
# frameblock.py - NEW subtractive frame: start from a solid 245x68x40 block, cut down from the top.
#   cut 1: coxa pockets x4 = coxa bbox grown in the swing plane (Y,Z) for free splay, crossing the outboard wall
#   cut 2: servo0 slots x4 = servo body footprint, top-down; tabs (wider) land on the ledge -> servo+coxa push in
# Execs dog13.py (FAST) so SH(coxa)/servo0/servo1/SP/tf are the live canonical parts.
import FreeCAD as App, FreeCADGui as Gui, Part, math, traceback
from PIL import Image, ImageDraw
v=App.Vector; X,Y,Z=v(1,0,0),v(0,1,0),v(0,0,1)
if 'SH' not in globals() or 'servo0' not in globals():   # warm-session reuse: skip the (heavy) dog13 rebuild if parts already live
    FAST=True
    exec(open(r"C:/ultrafish/robodog/dog13.py").read())  # -> SH, servo0, servo1, SP, tf, rot, box, cyl, HPx, F ...
OUT=r"C:/ultrafish/robodog/ref/iter"

# ---------- parameters (all tweakable) ----------
BX,BY,BZ = 245.0, 68.0, 40.0          # block
TOPZ = servo0.BoundBox.ZMax           # block top = servo0 tab top -> tab FLUSH with block top (user); block = Z[TOPZ-BZ, TOPZ]
WALL = 2.0
SPMIN,SPMAX = -25.0, 105.0             # splay range for the free-swing envelope
RUN_CLEAR = False                      # flush-ring clearance gate (settled -> off to keep runs under the 90s GUI timeout)
PCLR = 1.0                             # pocket clearance (Y,Z swing sides)
SCLR = 0.1                             # servo-slot slide-fit clearance (just fits the body)
OUTPAST = 16.0                         # how far the pocket breaks past the outboard wall

def bb6(s): b=s.BoundBox; return [b.XMin,b.XMax,b.YMin,b.YMax,b.ZMin,b.ZMax]
def sweep6(solid,amin,amax,n=25):
    xs=[];ys=[];zs=[]
    for i in range(n):
        a=amin+(amax-amin)*i/(n-1); b=rot(solid,a,SP,X).BoundBox
        xs+=[b.XMin,b.XMax];ys+=[b.YMin,b.YMax];zs+=[b.ZMin,b.ZMax]
    return [min(xs),max(xs),min(ys),max(ys),min(zs),max(zs)]

try:
    z0=TOPZ-BZ
    BLK=box(BX,BY,BZ,v(-BX/2,-BY/2,z0))

    cs=bb6(SH)                                   # coxa static bbox
    ce=sweep6(SH,SPMIN,SPMAX)                     # coxa swept bbox (Y,Z grown by swing)
    # inboard X wall of the coxa pocket moves inboard to the BACK of the servo0 tabs (user):
    # the tab plate is the only servo0 feature above the body top (z18), so a thin slab isolates its back.
    TAB_BACK = servo0.common(box(200,80,4.4,v(-100,-40,18.3))).BoundBox.XMin

    # --- cut 1: coxa pocket (build-frame, front-right) then tf to 4 corners ---
    px0,px1 = TAB_BACK, cs[1]+PCLR               # inboard wall AT the servo-tab back; outboard = coxa extent
    py0     = ce[2]-PCLR                          # Y inboard = swept min
    py1     = BY/2 + OUTPAST                       # Y outboard = past the wall (crosses it)
    pz0     = ce[4]-PCLR                          # Z bottom = swept min
    pz1     = TOPZ + 0.1                          # open to the top
    COXP=box(px1-px0, py1-py0, pz1-pz0, v(px0,py0,pz0))

    # --- cut 2: servo0 slot = box that JUST FITS the servo body (snug X/Y + floor at the body bottom),
    #     so the servo drops in from the top and seats exactly (bottoms out on the floor). (user)
    sbb=box(22.5,12.1,22.7,v(70,13.95,-4.7)); sbb.translate(v(-DX,YSHIFT,0)); sB=sbb.BoundBox   # servo0 BODY only (no tabs/boss/spline)
    sx0,sx1 = sB.XMin-SCLR, sB.XMax+SCLR
    sy0,sy1 = sB.YMin-SCLR, sB.YMax+SCLR
    sz0     = sB.ZMin                             # floor at the body underside -> body bottoms out = exact seat depth
    SLOT=box(sx1-sx0, sy1-sy0, TOPZ+0.1-sz0, v(sx0,sy0,sz0))

    FR=BLK
    corners=[(1,1),(1,-1),(-1,1),(-1,-1)]
    for sxc,syc in corners:
        FR=FR.cut(tf(COXP,sxc,syc)).cut(tf(SLOT,sxc,syc))
    FR=FR.removeSplitter()
    fb=FR.BoundBox
    rpt=("FRAMEBLOCK  block %gx%gx%g  (Z[%g,%g])  wall %g\n"
         "  coxa static  X[%.1f,%.1f] Y[%.1f,%.1f] Z[%.1f,%.1f]\n"
         "  coxa swept   X[%.1f,%.1f] Y[%.1f,%.1f] Z[%.1f,%.1f]   (splay %g..%g)\n"
         "  servo tab back X=%.1f  -> coxa pocket inboard wall\n"
         "  coxa pocket  X[%.1f,%.1f] Y[%.1f,%.1f (past wall %g)] Z[%.1f,%.1f]\n"
         "  servo0 slot  X[%.1f,%.1f] Y[%.1f,%.1f] Z[%.1f,%.1f]\n"
         "  FRAME solids=%d vol=%.0f bbox X=%.1f Y=%.1f Z=%.1f\n"%(
         BX,BY,BZ,z0,TOPZ,WALL, cs[0],cs[1],cs[2],cs[3],cs[4],cs[5],
         ce[0],ce[1],ce[2],ce[3],ce[4],ce[5],SPMIN,SPMAX, TAB_BACK,
         px0,px1,py0,py1,OUTPAST,pz0,pz1, sx0,sx1,sy0,sy1,sz0,TOPZ,
         len(FR.Solids),FR.Volume,fb.XLength,fb.YLength,fb.ZLength))
    open(OUT+"/frameblock.txt","w").write(rpt); print(rpt)

    # ---------- splay-clearance check: does raising the top 20->TOPZ cost swing room? ----------
    # `extra` = ONLY the material the flush added (the ring at z[20,TOPZ] that survives the pockets).
    # If the swung leg intersects `extra` = 0 at every angle, the flush is provably free.
    if RUN_CLEAR and TOPZ>20.001:
        extra=FR.common(box(BX,BY,TOPZ-20.0,v(-BX/2,-BY/2,20.0)))
        inner=SH.fuse(servo1)                                    # rides splay only; lives in the pocket
        outer=FE.fuse(servo2).fuse(TIF)                          # pitches about HP, then splays about SP; mostly outboard
        gl=["SPLAY CLEARANCE vs the +%.1fmm flush ring  (extra ring vol=%.0f; want all 0):"%(TOPZ-20.0,extra.Volume)]
        gl.append("  coxa+servo1: "+"  ".join("s%d=%.1f"%(a,extra.common(rot(inner,a,SP,X)).Volume) for a in (-25,-10,0,45,90,105)))
        for p in (-45,0,45,90):
            gl.append("  femur/leg @pitch%+d: "%p+" ".join("s%d=%.1f"%(a,extra.common(rot(rot(outer,p,HP,Y),a,SP,X)).Volume) for a in (-25,0,45,105)))
        open(OUT+"/frameblock_clear.txt","w").write("\n".join(gl)+"\n"); print("\n".join(gl))

    # ---------- render: block + seated coxa/servo0/servo1 at 4 corners ----------
    nm="frameblock"
    if nm in list(App.listDocuments()): App.closeDocument(nm)
    d=App.newDocument(nm)
    def add(n,s,c,t=0):
        o=d.addObject("Part::Feature",n); o.Shape=s
        try: o.ViewObject.ShapeColor=c; o.ViewObject.Transparency=t; o.ViewObject.Deviation=0.02
        except Exception: pass
    add("frame",FR,(.34,.36,.40),55)
    for i,(sxc,syc) in enumerate(corners):
        add("coxa_%d"%i, tf(SH,sxc,syc), (.20,.55,.90))
        add("servo0_%d"%i, tf(servo0,sxc,syc), (.30,.62,.47))
        add("servo1_%d"%i, tf(servo1,sxc,syc), (.55,.55,.58))
    d.recompute()
    gv=Gui.activeDocument().activeView()
    SZ=760; shots=[]
    for vn in ("Axonometric","Top","Front","Isometric"):
        getattr(gv,"view"+vn)(); Gui.SendMsgToActiveView("ViewFit")
        fp=OUT+"/_fb_%s.png"%vn; gv.saveImage(fp,SZ,SZ,"White"); shots.append((fp,vn))
    # montage 2x2
    ims=[Image.open(p).convert("RGB").resize((SZ,SZ)) for p,_ in shots]
    sheet=Image.new("RGB",(SZ*2,SZ*2),(250,250,250)); dr=ImageDraw.Draw(sheet)
    labs=["iso","top (plan)","front (dog side)","iso-2"]
    for i,im in enumerate(ims):
        r,c=divmod(i,2); sheet.paste(im,(c*SZ,r*SZ))
        dr.rectangle([c*SZ+4,r*SZ+4,c*SZ+150,r*SZ+24],fill=(25,25,25)); dr.text((c*SZ+8,r*SZ+8),labs[i],fill=(255,255,255))
    sheet.save(OUT+"/frameblock_views.png")
    print("OK frameblock")
except Exception:
    print("FAIL: "+traceback.format_exc())
