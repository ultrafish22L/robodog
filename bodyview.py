
# bodyview.py - CONCEPTUAL body-panel pass draped over the v19 frame+legs.
# Execs dog13.py (leaves FR/legs + `parts` + helpers as globals), then builds ONE
# integrated Spot-like body (head + trunk + rump as a single filleted shell) with
# slant-cut end faces, and renders a multi-angle montage.
# User direction: head+rump integrated into the body; head longer on top -> front face
# slants back ~22deg; rump same but ~8deg; similar-not-identical to SM3; nice fillets.
import FreeCAD as App, FreeCADGui as Gui, Part, math, traceback
from PIL import Image, ImageDraw
FAST=True   # v28: dog13 skips its gate sweeps + 53-object doc render (we only need the geometry) -> avoids the 90s GUI timeout
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
    # HOLLOW into a thin-wall printable cover: subtract the inset inner cavity, then subtract the frame's
    # OUTER ENVELOPE for clearance. (v28: cut a simple box hull of the frame body, NOT the full FR - FR now
    # carries ~18k mesh-derived faces from the model-cut servo pockets, and cut(FR) blows past the 90s GUI
    # limit. The internal pockets don't affect the cover, so the outer tub box is all the clearance we need.)
    FRHULL=box(256,71,53,v(-128,-35.5,-31))    # frame OUTER envelope over its full X (pillars reach x~122); clears the caps' hip region too
    shell=body.cut(build_body(WALL)).cut(FRHULL).removeSplitter()
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

    # --- FORE-AFT MODULE SPLIT: head (front) + trunk (mid) + rump (rear) as 3 snap-together modules.
    #     Seams at x=+-XS, just inboard of the front/rear hip openings (x74 / -74) so each seam ring is a
    #     clean oval (no hip cutout crosses it) and trunk (~140) + head/rump each fit the 256 bed. Head &
    #     rump plug INTO the trunk on an internal upper-C SPIGOT (male on head/rump, female trunk cavity):
    #     the OUTER surface is a plain butt seam -> reads as ONE continuous surface (smooth flow, no step),
    #     while the hidden spigot laps ~9mm inside and grips ("slightly overfit"). Upper-C (z>ZSP) so it
    #     clears the frame + cradle down on the floor.
    def big(s): return max(s.Solids,key=lambda q:q.Volume) if len(s.Solids)>1 else s
    XS=70.0; LAP=9.0; SPCLR=0.30; SPW=1.5; DET=0.55; ZSP=-16.0
    # NB: OCC common() mis-clips these filleted organic b-reps on a MIDDLE band, and cut()-with-a-slab is
    # unreliable too; the ONLY robust slice is common() with an END-band (a half-space open past the shape).
    # So build a middle band as two sequential end-band commons.
    def keepge(s,lo): return big(s.common(box(2200,1200,1200,v(lo,-600,-600))))        # keep x>=lo
    def keeple(s,hi): return big(s.common(box(2200,1200,1200,v(hi-2200,-600,-600))))    # keep x<=hi
    def bandx(s,lo,hi): return keepge(keeple(s,hi),lo)                                  # middle band = 2 end-bands
    def keepzge(s,zlo): return big(s.common(box(2200,1200,2200,v(-1100,-600,zlo))))     # keep z>=zlo (upper-C)
    head = keepge(shell,XS)
    rump = keeple(shell,-XS)
    trunk= bandx(shell,-XS,XS)
    # GRIP: head/rump are LOCATED by cupping over the frame ends (frame runs x+-86.5, into the caps) and
    # RETAINED by a cantilever snap-finger on each cap that hooks a window in the lid roof (robust box
    # primitives -- the offset-ring spigot defeats OCC booleans on this filleted shell). Built below with
    # the trunk lid/tub latches so both share the finger/window machinery.

    # --- TRUNK lid/tub split (electronics access): a long-Z reveal parting terrain, over the trunk only. ---
    ZIG=[(-160,-5),(-55,-5),(25,13),(160,13)]     # low line z-5 (rear) -> diagonal up -> top line z13 (front)
    GAP=2.5
    def region(dz,cap):
        line=[(x,zz+dz) for (x,zz) in ZIG]; poly=[(ZIG[0][0],cap)]+line+[(ZIG[-1][0],cap)]
        w=Part.makePolygon([v(x,-50,z) for (x,z) in poly]+[v(poly[0][0],-50,poly[0][1])])
        return Part.Face(w).extrude(v(0,100,0))
    topp=trunk.common(region(+GAP/2,70)); botp=trunk.common(region(-GAP/2,-70))

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
    # head/rump -> lid SNAP LATCH (toolless, same cantilever principle): a tongue reaches from the cap seam
    # back under the lid roof (top-centre, above the frame) and its up-hook springs into a lid window; pulling
    # the cap off in X drives the hook back-face into the window's edge -> latched. Robust box primitives.
    hf=None; rf=None; cw=None
    for yc in (-14,14):                                  # TWO fingers per cap (2-point snap) across the flat roof top
        h=box(3,10,6,v(70,yc-5,23)).fuse(box(10,4,2.6,v(60,yc-2,24))).fuse(box(3,4,4,v(60,yc-2,25.5)))     # head finger (reaches -X)
        r=box(3,10,6,v(-73,yc-5,23)).fuse(box(10,4,2.6,v(-70,yc-2,24))).fuse(box(3,4,4,v(-63,yc-2,25.5)))  # rump finger (reaches +X)
        hf=h if hf is None else hf.fuse(h); rf=r if rf is None else rf.fuse(r)
        w=box(6,6,5,v(58,yc-3,25)).fuse(box(6,6,5,v(-64,yc-3,25)))                                          # lid windows
        cw=w if cw is None else cw.fuse(w)
    head=big(head.fuse(hf).removeSplitter()); rump=big(rump.fuse(rf).removeSplitter())
    topp=topp.cut(cw).removeSplitter()                   # 4 lid windows (2 per cap) for the hooks
    def big(s): return max(s.Solids,key=lambda q:q.Volume) if len(s.Solids)>1 else s
    topp=big(topp); botp=big(botp)              # drop any tiny chip a cut may shed -> one solid each

    # --- FRAME LOCATING KEYS (panels latch to each other; the frame is LOCATED + SANDWICHED, not
    #     clipped): a CRADLE on the tub floor that the bed-fit frame floor-plate (x+-86.5, y+-33.2) nests
    #     into -> locates Y + rests it at z-30 (frame X is held by the head/rump caps gripping the ends);
    #     + LID CLAMP PADS that bear on the frame top-plate top (z14) -> vertical sandwich, no rattle. ---
    cradle=box(132,2.0,4.8,v(-66,34.5,-30)).fuse(box(132,2.0,4.8,v(-66,-36.5,-30)))   # v28 Y-locating rails re-fit just outboard of the WIDER tub floor (y+-34.25)
    for cxr in (1,-1):
        for cyr in (1,-1):
            cradle=cradle.fuse(box(12,10,2.3,v(cxr*60-6,cyr*29-5,-32.3)))              # 4 pads rest the frame floor at z-30
    botp=botp.fuse(cradle).removeSplitter()
    # v28 LID CLAMP: the frame is an open tub now (no deck). A central pad bears on the 1cm cross-strip (z18) to
    # sandwich the frame down; two more sit over the side-wall rims (z18) at the trunk sides.
    lidpads=box(12,70,10,v(-6,-35,18))                                                 # central pad on the cross-strip
    for cyr in (1,-1): lidpads=lidpads.fuse(box(80,3,10,v(-40,cyr*33.25-1.5,18)))       # side-rim rails (bear on the tub side walls z18)
    topp=topp.fuse(lidpads).removeSplitter()
    topp=bandx(topp,-XS,XS); botp=bandx(botp,-XS,XS)   # hard-clip lid/tub to the trunk band + drop stray wall slivers -> deterministic

    bb=body.BoundBox; hollow=100*(1-shell.Volume/body.Volume); BED=256.0
    frOL=topp.common(FR).Volume+botp.common(FR).Volume+head.common(FR).Volume+rump.common(FR).Volume
    def fitline(nm2,s):
        b=s.BoundBox; fit="OK" if max(b.XLength,b.YLength,b.ZLength)<=BED else "OVER-BED!"
        return "  %-10s solids=%d vol=%7.0f  %3.0f x %3.0f x %3.0f mm  %s"%(nm2,len(s.Solids),s.Volume,b.XLength,b.YLength,b.ZLength,fit)
    open(OUT+"/body.txt","w").write(
        "BODY(head+trunk+rump split) outer bbox X[%.1f,%.1f]=%.1f Y[%.1f,%.1f]=%.1f Z[%.1f,%.1f]=%.1f  fillet=%s hollow=%.0f%% wall=%.1f\n"%(
        bb.XMin,bb.XMax,bb.XLength,bb.YMin,bb.YMax,bb.YLength,bb.ZMin,bb.ZMax,bb.ZLength,filleted,hollow,WALL)
        +"  frame^cover(all 4 summed) = %.0f (want ~0)   seams x=+-%.0f  reveal gap %.1f  %d trunk latches\n"%(frOL,XS,GAP,nclip)
        +"\n".join([fitline("trunk_lid",topp),fitline("trunk_tub",botp),fitline("head",head),fitline("rump",rump)])+"\n"
        +"  modules: head XMin=%.1f  rump XMax=%.1f  (butt seams at x=+-%.0f; grip = frame-cup + cap snap-latch)\n"%(
        head.BoundBox.XMin,rump.BoundBox.XMax,XS))
    # --- export the 4 cover STLs to stl/ (mechanism parts come from export.py) ---
    import Mesh, MeshPart
    for _s,_n in [(topp,"sm3sg90_body_top_lid"),(botp,"sm3sg90_body_bottom_tub"),(head,"sm3sg90_body_head"),(rump,"sm3sg90_body_rump")]:
        # v29 PRE-PRINT REPAIR verified ON DISK (split organic shells tessellate with self-intersections +
        # non-manifold seam edges; repair, write, RE-READ, repeat until the on-disk mesh is provably clean).
        _m=MeshPart.meshFromShape(Shape=_s,LinearDeflection=0.15,AngularDeflection=0.6,Relative=False)
        _m.removeDuplicatedPoints(); _m.removeDuplicatedFacets(); _p=r"C:/ultrafish/robodog/stl/"+_n+".stl"; _st="DEFECT!"
        for _it in range(6):
            _k=0
            while _m.hasSelfIntersections() and _k<4: _m.fixSelfIntersections(); _k+=1
            if _m.hasNonManifolds(): _m.removeNonManifolds()
            _m.harmonizeNormals(); _m.write(_p)
            _r=Mesh.Mesh(); _r.read(_p)
            if not (_r.hasSelfIntersections() or _r.hasNonManifolds()): _st="clean"; break
            _m=_r
        print("  cover %s: tris=%d %s"%(_n,_m.CountFacets,_st))

    nm="bodyview"
    if nm in list(App.listDocuments()): App.closeDocument(nm)
    d=App.newDocument(nm)
    shown=[]
    for n,q,c,t in parts:
        shown.append((n,q,c,max(t,55)))          # fade frame+legs so the body silhouette reads
    shown+=[("top",topp,BODYT,20),("bottom",botp,BODYB,20),("head",head,(.92,.60,.10),20),("rump",rump,(.92,.60,.10),20),("clips",fingers,CLIP,0),("face",face,FACE,0)]
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
    # EXPLODED iso: opaque panels only (frame/legs hidden); modules slid apart to show the 3-way split +
    # the trunk lid lifted -> head forward, rump back, lid up.
    for o in d.Objects:
        if o.Name in ("top","bottom","face","clips","head","rump"): o.ViewObject.Transparency=0
        else: o.ViewObject.Visibility=False
    for o in d.Objects:
        if o.Name=="head": o.Placement=App.Placement(App.Vector(60,0,0),App.Rotation())
        elif o.Name=="rump": o.Placement=App.Placement(App.Vector(-60,0,0),App.Rotation())
        elif o.Name in ("top","face"): o.Placement=App.Placement(App.Vector(0,0,40),App.Rotation())
    d.recompute(); gv.viewAxonometric(); Gui.SendMsgToActiveView("ViewFit")
    fp=OUT+"/_bd_sil.png"; gv.saveImage(fp,SZ,SZ,"White"); pl.append((fp,"exploded (head fwd, rump back, lid up)"))

    cols=3; rows=(len(pl)+cols-1)//cols; W=cols*SZ; H=rows*SZ+24
    sheet=Image.new("RGB",(W,H),(245,245,245)); dr=ImageDraw.Draw(sheet); dr.text((6,6),"body_concept v16 (head+trunk+rump split: spigot lap, smooth butt seam, toolless; trunk lid/tub)",fill=(0,0,0))
    for i,(p,lab) in enumerate(pl):
        im=Image.open(p).convert("RGB").resize((SZ,SZ)); dd=ImageDraw.Draw(im)
        dd.rectangle([0,0,150,20],fill=(30,30,30)); dd.text((4,4),lab,fill=(255,255,255))
        r,c=divmod(i,cols); sheet.paste(im,(c*SZ,24+r*SZ))
    sheet.save(OUT+"/body_concept.png")
    App.closeDocument(nm)
    print("OK bodyview")
except Exception:
    print("FAIL: "+traceback.format_exc())
