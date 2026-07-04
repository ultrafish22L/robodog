
# bodyview.py - CONCEPTUAL body-panel pass draped over the v19 frame+legs.
# Execs dog13.py (leaves FR/legs + `parts` + helpers as globals), then builds ONE
# integrated Spot-like body (head + trunk + rump as a single filleted shell) with
# slant-cut end faces, and renders a multi-angle montage.
# User direction: head+rump integrated into the body; head longer on top -> front face
# slants back ~22deg; rump same but ~8deg; similar-not-identical to SM3; nice fillets.
import FreeCAD as App, FreeCADGui as Gui, Part, math, traceback
from PIL import Image, ImageDraw
exec(open(r"C:/ultrafish/robodog/dog13.py").read())        # -> FR, parts[], v, X,Y,Z, box, cyl, rot ...
OUT=r"C:/ultrafish/robodog/ref/iter"; SZ=680
BODY=(.82,.82,.85); FACE=(.09,.09,.11)
BODYT=(.95,.74,.10); BODYB=(.86,.66,.06)   # STANDARD YELLOW (lid) + a hair deeper (tub) so the split still reads
CLIP=(.95,.45,.12)                          # snap posts, bright so they read against the panels

def rrect(x,cz,hy,hz,r):
    # SMOOTH rounded rect in plane X=x, centred (y=0, z=cz), half W=hy half H=hz, corner r.
    # Built from arcs+lines (a real Part.Wire), NOT a makePolygon -- faceted polygon rings loft
    # into corncob stripes (see robodog.md); smooth arc rings loft clean.
    a=hy-r; b=hz-r
    def P(y,z): return v(x,y,z)
    def arc(cy,cc,a0,a1):
        f=lambda t:(cy+r*math.cos(t),cc+r*math.sin(t)); s=f(a0); m=f((a0+a1)/2.0); e=f(a1)
        return Part.Arc(P(*s),P(*m),P(*e)).toShape()
    E=[Part.LineSegment(P(-a,cz-hz),P(a,cz-hz)).toShape(), arc(a,cz-b,-0.5*math.pi,0.0),
       Part.LineSegment(P(hy,cz-b),P(hy,cz+b)).toShape(),  arc(a,cz+b,0.0,0.5*math.pi),
       Part.LineSegment(P(a,cz+hz),P(-a,cz+hz)).toShape(), arc(-a,cz+b,0.5*math.pi,math.pi),
       Part.LineSegment(P(-hy,cz+b),P(-hy,cz-b)).toShape(),arc(-a,cz-b,math.pi,1.5*math.pi)]
    return Part.Wire(E)

def loft(sts):
    return Part.makeLoft([rrect(*s) for s in sts], True, False)

try:
    # --- ONE integrated body: single loft rump->trunk->head; CONSTANT width (no head/rump
    #     width taper) -- head & rump defined only by height + slant faces, sides stay parallel ---
    WALL=2.2; fpv=v(151,0,19.6); rpv=v(-147,0,15.2)
    #  Section tables (x, cz, hy, hz for MAIN; x, zbot for skirt/bump). Skirt floor dropped to
    #  -34.5 (was -31) to leave room for a shell wall beneath the frame floor (z-30).
    MAIN=[(-150,15.2,39,9.5),(-138,15.4,39,10),(-122,15.5,39,11),(-90,16,39,12.5),(-45,16.3,39,13),
          (0,16.3,39,13),(45,16.5,39,13),(80,16.8,39,12.5),(100,18,39,12),(122,19.3,39,12.5),(140,19.6,39,12),(156,19.6,39,11.5)]
    SK=[(-120,7),(-110,-18),(-98,-33.5),(-75,-34.5),(-30,-34.5),(15,-34.5),(55,-34.5),(78,-34.5),(100,-34),(120,-32),(140,-30),(152,-27)]
    BM=[(-75,-30),(-52,-38),(-26,-41),(2,-41),(30,-40),(54,-37),(78,-30)]
    def sec(ztop,zbot,hy):
        cz=(ztop+zbot)/2.0; hz=(ztop-zbot)/2.0
        return (cz,hy,hz,min(10.0,max(0.6,hz-0.6),hy-0.6))
    def loftr(SS): return Part.makeLoft([rrect(*s) for s in SS],True,True)   # RULED (no bspline overshoot)
    def build_body(IN):
        #  the whole shell, inset by IN (0 = outer surface, WALL = inner cavity). Main loft = smooth
        #  bspline of arc rings; skirt/bump = ruled. Slant cuts shifted outward by IN/cos(angle).
        b=loft([(x,cz,hy-IN,hz-IN,max(0.6,min(10.0,hz-IN-0.6))) for (x,cz,hy,hz) in MAIN]).common(box(360,80-2*IN,160,v(-190,-40+IN,-40)))
        b=b.fuse(loftr([(x,)+sec(9.0,zb+IN,39.0-IN) for x,zb in SK])).removeSplitter()      # skirt
        b=b.fuse(loftr([(x,)+sec(-27.0,zb+IN,30.0-IN) for x,zb in BM])).removeSplitter()    # belly bump
        dF=IN/math.cos(math.radians(22)); dR=IN/math.cos(math.radians(8))
        b=b.cut(rot(box(90,90,130,v(151-dF,-45,-50)),22,v(151-dF,0,19.6),Y))                # front 22deg
        b=b.cut(rot(box(90,90,130,v(-237+dR,-45,-50)),-8,v(-147+dR,0,15.2),Y))              # rear 8deg
        return b
    body=build_body(0.0)     # solid OUTER shape (for the silhouette + bbox)
    # fillet the LONG seam/chine edges (all-edge fillet chokes on the tiny arc-junction slivers)
    filleted=False
    for L,rad in ((50,1.2),(55,1.0),(45,0.8),(60,1.5),(50,0.6)):
        try:
            edg=[e for e in body.Edges if e.Length>L]; body2=body.makeFillet(rad,edg)
            if body2.isValid() and len(body2.Solids)==1: body=body2; filleted="%d@%.1f"%(len(edg),rad); break
        except Exception: pass
    body=body.removeSplitter()
    # HOLLOW into a thin-wall printable cover: subtract the inset inner cavity, then subtract the
    # FRAME for guaranteed clearance (the skin grazes the frame all over, ~2mm; cut(FR) clears it in
    # one robust boolean instead of chasing each spot). makeThickness/makeOffsetShape both FAIL on
    # this filleted b-rep, so the cavity is built parametrically.
    shell=body.cut(build_body(WALL)).cut(FR.copy()).removeSplitter()
    # HIP OPENINGS: leg-shoulder cutouts at the 4 hips (x=+-106) so the coxa + hip servos +
    # femur-top pass through (Spot-like) and the hip-roll motion clears. Through the cover side.
    hipcut=None
    for sx in (1,-1):
        for sy in (1,-1):
            hb=box(64,17,38,v(sx*106-32, 29.0 if sy>0 else -46.0, -13))
            hipcut=hb if hipcut is None else hipcut.fuse(hb)
    shell=shell.cut(hipcut)
    for _n,_q,_c,_t in parts:            # form-fit the openings to the actual shoulder parts -> 0 collision in stance
        if _n.split('_')[0] in ('sh','s0','s1','fe'): shell=shell.cut(_q)
    shell=shell.removeSplitter()
    # dark sensor face on the slanted head front
    face=box(7.0,52,40,v(146,-26,1)); face=rot(face,22,fpv,Y); face=face.common(body)

    # --- SNAP-ON PANEL SPLIT: top lid + bottom tub, parted by a clean single "long Z" reveal gap
    #     on the long sides (rather than a tight flush seam). Real-Z parting terrain z=zpart(x):
    #     a long HIGH top line along the FRONT (head/chest), one diagonal sloping BACK and DOWN,
    #     a low bottom line along the belly to the rear. The raised tail sits ABOVE the low line so
    #     the whole rump comes off as ONE clean piece on the lid (no sliver). Offset +-GAP/2. ---
    ZIG=[(-160,-5),(-55,-5),(25,13),(160,13)]     # low line z-5 (rear) -> diagonal up -> top line z13 (front)
    GAP=2.5
    def region(dz,cap):
        line=[(x,zz+dz) for (x,zz) in ZIG]; poly=[(ZIG[0][0],cap)]+line+[(ZIG[-1][0],cap)]
        w=Part.makePolygon([v(x,-50,z) for (x,z) in poly]+[v(poly[0][0],-50,poly[0][1])])
        return Part.Face(w).extrude(v(0,100,0))
    topp=shell.common(region(+GAP/2,70)); botp=shell.common(region(-GAP/2,-70))

    # --- CANTILEVER LATCHES (toolless assemble + disassemble): each tub-side finger is a thin
    #     tongue anchored to the rim by a gusset; it rises past the parting and its hook pokes into a
    #     RELEASE WINDOW in the lid wall. Press the lid down -> the hook lead-in flexes the finger
    #     inboard, then it springs into the window and its flat underside catches the window's bottom
    #     edge (holds the lid down). To open: push the hook in through the window, lift. Stations
    #     x=-55/0/55 are inboard of the hip features (frame +-33.2) so the finger clears the frame. ---
    Z1=App.Vector(0,0,1)
    def zpart(x):
        for i in range(len(ZIG)-1):
            (x0,z0),(x1,z1)=ZIG[i],ZIG[i+1]
            if x0<=x<=x1: return z0+(z1-z0)*(x-x0)/(x1-x0)
        return ZIG[0][1] if x<=ZIG[0][0] else ZIG[-1][1]
    def yb(a,b,sgn): return (a if sgn>0 else -b), (b-a)      # (y0,dy) for a +-y band
    fingers=None; windows=None; nclip=0
    for px in (-55,0,55):
        zp=zpart(px)
        for sgn in (1,-1):
            gy,gd=yb(34.8,39.0,sgn); gus=box(7,gd,4.5,v(px-3.5,gy,zp-4.5))       # gusset -> anchors finger to tub rim
            fy,fd=yb(34.8,36.3,sgn); fin=box(6,fd,13.0,v(px-3,fy,zp-4))          # 1.5mm-thin cantilever, flexes inboard
            hy,hd=yb(36.3,37.5,sgn); hk =box(6,hd,3.0,v(px-3,hy,zp+5))           # hook (0.7mm catch, needs 0.7mm flex)
            f=gus.fuse(fin).fuse(hk)
            wy,wd=yb(36.0,40.0,sgn); win=box(8,wd,5.0,v(px-4,wy,zp+4))           # lid release window (finger access)
            fingers=f if fingers is None else fingers.fuse(f)
            windows=win if windows is None else windows.fuse(win); nclip+=1
    botp=botp.fuse(fingers).removeSplitter()                # cantilever fingers -> tub
    topp=topp.cut(windows).removeSplitter()                 # release windows -> lid wall
    def big(s): return max(s.Solids,key=lambda q:q.Volume) if len(s.Solids)>1 else s
    topp=big(topp); botp=big(botp)              # drop any tiny chip a cut may shed -> one solid each

    # --- FRAME LOCATING KEYS (panels latch to each other; the frame is LOCATED + SANDWICHED, not
    #     clipped): a CRADLE on the tub floor that the frame floor-plate (x+-92.5, y+-33.2) nests into
    #     -> locates X+Y and rests it at z-30; + LID CLAMP PADS that bear on the frame top-plate top
    #     (z14) when the latches pull the lid down -> vertical sandwich, no rattle. 0.5mm nest clr. ---
    cradle=box(192,72,4.5,v(-96,-36,-30)).cut(box(186,67.4,7,v(-93,-33.7,-31)))       # OPEN-CENTRE ring wall
    for cxr in (1,-1):                                                                 # (belly cavity stays connected)
        for cyr in (1,-1):
            cradle=cradle.fuse(box(12,10,2.3,v(cxr*88-6,cyr*31-5,-32.3)))              # 4 corner pads: rest frame floor @ z-30
    botp=botp.fuse(cradle).removeSplitter()
    lidpads=None
    for cx in (-45,0,45):                        # central: clear of the hip servos (x70+)
        pd=box(16,36,14,v(cx-8,-18,14))          # bears on the frame top-plate (z14), rises to fuse into the lid roof
        lidpads=pd if lidpads is None else lidpads.fuse(pd)
    topp=topp.fuse(lidpads).removeSplitter()
    topp=big(topp); botp=big(botp)

    bb=body.BoundBox; hollow=100*(1-shell.Volume/body.Volume)
    frOL=topp.common(FR).Volume+botp.common(FR).Volume
    open(OUT+"/body.txt","w").write(
        "BODY(integrated head+trunk+rump) bbox X[%.1f,%.1f]=%.1f Y[%.1f,%.1f]=%.1f Z[%.1f,%.1f]=%.1f\n"
        "  outer solids=%d  fillet=%s  face solids=%d  SHELL wall=%.1fmm hollow=%.0f%%  frame^cover=%.0f (0=clear)\n"
        "  SPLIT: lid solids=%d vol=%.0f  tub solids=%d vol=%.0f  (long-Z reveal gap %.1fmm, %d cantilever latches)\n"%(
        bb.XMin,bb.XMax,bb.XLength,bb.YMin,bb.YMax,bb.YLength,bb.ZMin,bb.ZMax,bb.ZLength,
        len(body.Solids),filleted,len(face.Solids),WALL,hollow,frOL,
        len(topp.Solids),topp.Volume,len(botp.Solids),botp.Volume,GAP,nclip))

    nm="bodyview"
    if nm in list(App.listDocuments()): App.closeDocument(nm)
    d=App.newDocument(nm)
    shown=[]
    for n,q,c,t in parts:
        shown.append((n,q,c,max(t,55)))          # fade frame+legs so the body silhouette reads
    shown+=[("top",topp,BODYT,20),("bottom",botp,BODYB,20),("clips",fingers,CLIP,0),("face",face,FACE,0)]
    for n,q,c,t in shown:
        o=d.addObject("Part::Feature",n); o.Shape=q
        try: o.ViewObject.ShapeColor=c; o.ViewObject.Transparency=t; o.ViewObject.Deviation=0.01
        except Exception: pass
    d.recompute()
    gv=Gui.activeDocument().activeView()
    pl=[]
    for vn,lab in [("Front","side"),("Axonometric","iso"),("Right","front (dog)"),("Top","top (plan)")]:
        getattr(gv,"view"+vn)(); Gui.SendMsgToActiveView("ViewFit")
        fp=OUT+"/_bd_%s.png"%vn; gv.saveImage(fp,SZ,SZ,"White"); pl.append((fp,lab))
    # EXPLODED iso: opaque panels only (frame/legs hidden), lid lifted +34mm to show the two parts
    for o in d.Objects:
        if o.Name in ("top","bottom","face","clips"): o.ViewObject.Transparency=0
        else: o.ViewObject.Visibility=False
    for o in d.Objects:
        if o.Name in ("top","face"): o.Placement=App.Placement(App.Vector(0,0,34),App.Rotation())
    d.recompute(); gv.viewAxonometric(); Gui.SendMsgToActiveView("ViewFit")
    fp=OUT+"/_bd_sil.png"; gv.saveImage(fp,SZ,SZ,"White"); pl.append((fp,"exploded (lid lifted)"))

    cols=3; rows=(len(pl)+cols-1)//cols; W=cols*SZ; H=rows*SZ+24
    sheet=Image.new("RGB",(W,H),(245,245,245)); dr=ImageDraw.Draw(sheet); dr.text((6,6),"body_concept v15 (HOLLOW covers: cantilever latches, toolless assemble/disassemble)",fill=(0,0,0))
    for i,(p,lab) in enumerate(pl):
        im=Image.open(p).convert("RGB").resize((SZ,SZ)); dd=ImageDraw.Draw(im)
        dd.rectangle([0,0,150,20],fill=(30,30,30)); dd.text((4,4),lab,fill=(255,255,255))
        r,c=divmod(i,cols); sheet.paste(im,(c*SZ,24+r*SZ))
    sheet.save(OUT+"/body_concept.png")
    App.closeDocument(nm)
    print("OK bodyview")
except Exception:
    print("FAIL: "+traceback.format_exc())
