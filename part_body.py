# part_body.py -- ONE-PIECE cosmetic dog-body SHELL (canopy) that snaps onto the frame TOP.
#   Reuses the bodyview.py silhouette (rrect rounded-rect rings lofted along X), SCALED to the current
#   ~206mm frame (SX in X; width + height profile kept so the roof domes ~8mm above the frame top z20.7).
#   Built as a SINGLE hollow shell with an OPEN BELLY: a dome that wraps the frame top + upper sides down
#   to a skirt line, open below. Hip openings at the 4 corners for the legs. Snaps onto the rim cap.
#   Build headless:  freecadcmd -c "exec(open('part_body.py').read())"
import FreeCAD as App, Part, math, traceback
if 'leg_pose' not in globals():
    FAST=True; SKIP_GATES=True
    exec(open(r"C:/ultrafish/robodog/dog14.py").read())   # -> FR + leg_pose,SH9,COVER9,SP,tf,rot,CORNERS,cbx1 + frame geom

# ---- silhouette (smooth rounded-rect ring; arcs+lines, NOT a faceted polygon -> lofts clean) ----
def rrect(x,cz,hy,hz,r):
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
def loft(sts): return Part.makeLoft([rrect(*s) for s in sts], True, False)

# ---- SINGLE-FEATURE body (issue 1, user 2026-07-29): the old build = dome + straight skirt + keel-chamfer, which
#      read as TWO stacked rounded lobes on head/rump (a horizontal groove where the dome met the skirt/chamfer).
#      Now the WHOLE body is ONE tall loft: each rounded-rect ring spans crown->keel in a single section, so every
#      cross-section is one smooth rounded form (dome top -> vertical side -> rounded keel). The trunk's lower half
#      is removed by the belly-open cut below, so the tall rings survive only as the wrap-down head/rump end caps --
#      each now a single continuous feature. No separate skirt loft, no keel chamfer -> no groove.
#      CROWN[x_old,z_top] = the old roof-top height (old MAIN0 cz+hz); ring: cz=(top+KEEL)/2, hz=(top-KEEL)/2.
WALL=2.2; SX=0.84; BELLYZ=4.0; HY=39.5; KEEL=-26.0; RAD=11.0
CROWN=[(-150,24.7),(-138,25.4),(-122,26.5),(-90,28.5),(-45,29.3),(0,29.3),
       (45,29.5),(80,29.3),(100,30.0),(122,31.8),(140,31.6),(156,31.1)]
MAIN=[(xo*SX,(ct+KEEL)/2.0,HY,(ct-KEEL)/2.0) for (xo,ct) in CROWN]   # (x, cz, hy, hz); KEEL wrap-down clears frame floor (zbot=-22)
FX=156*SX; RX=-147*SX                                   # slant-cut pivots (head front / rump rear)

def build(IN):
    # single tall loft, inset by IN (0=outer, WALL=inner cavity). Y-clip to +/-(40-IN); slant-cut the head/rump ends.
    b=loft([(x,cz,hy-IN,hz-IN,max(0.6,min(RAD,hz-IN-0.6))) for (x,cz,hy,hz) in MAIN]).common(box(400,80-2*IN,340,v(-200,-40+IN,-60)))
    dF=IN/math.cos(math.radians(22)); dR=IN/math.cos(math.radians(8))
    b=b.cut(rot(box(90,90,190,v(FX-dF,-45,-62)),22,v(FX-dF,0,19.6),Y))                  # head front 22deg slant
    b=b.cut(rot(box(90,90,190,v(RX-90+dR,-45,-62)),-8,v(RX+dR,0,15.2),Y))               # rump rear 8deg slant
    return b

try:
    body=build(0.0)
    # fillet the long chine/seam edges (all-edge fillet chokes on the tiny arc-junction slivers)
    fil=False
    # the skirt/keel added edges that break a whole-body fillet; try LONGER-edge subsets (the main chines) with
    # smaller radii first -- fewer, cleaner edges are more likely to fillet without StdFail_NotDone.
    for L,rad in ((60,0.6),(70,0.6),(80,0.5),(95,0.5),(110,0.4),(55,0.6),(65,0.5),(75,0.5),(90,0.4),(120,0.3),(45,0.5)):
        try:
            edg=[e for e in body.Edges if e.Length>L]
            if not edg: continue
            b2=body.makeFillet(rad,edg)
            if b2.isValid() and len(b2.Solids)==1: body=b2; fil="%d@%.1f"%(len(edg),rad); break
        except Exception: pass
    body=body.removeSplitter()
    # HOLLOW to a thin wall (no keel chamfer any more -- the single tall loft's own rounded keel forms the wrap-down;
    # the old chamfer was what split each cap into two lobes). Then OPEN the belly.
    shell=body.cut(build(WALL)).removeSplitter()
    # OPEN THE TRUNK BELLY ONLY (x in [-98.4,98.4], between + incl the hips): the head + rump keep the wrapped-down
    # keel; the trunk stays an open-belly canopy for the legs + belly electronics.
    shell=shell.cut(box(196.8,140,BELLYZ+80,v(-98.4,-70,BELLYZ-80)))
    shell=shell.removeSplitter()
    if len(shell.Solids)>1: shell=max(shell.Solids,key=lambda q:q.Volume)

    # ---- FRAME CLEARANCE: trim any shell material that dips into the frame outer envelope (+0.6/side gap). Height
    #      capped at the frame top (z20.7) so it does NOT eat the roof crown above it (that was severing the shell). ----
    shell=shell.cut(box(207.2,72.6,21.2,v(-103.6,-36.3,-0.5)))
    shell=shell.removeSplitter()
    # ---- COLLAR CLEARANCE + END-GRIP (user 2026-07-29 "snaps on the bottom edges where head/rump meet the frame"):
    #      the caps are noses BEYOND the frame -- the collars (O16, x100-103) are the only frame-end feature reaching
    #      into each cap. Clear the collar (O16.8 pocket) but leave a GRIP RING (O15.6 inner, 2mm, at the collar tip)
    #      so each cap nose lightly snaps onto its 2 collars = the caps grip the frame end. Ring clears the O13 pin
    #      head (passes over it to reach the collar). ----
    _col=Part.makeCylinder(COLR+0.4, COLL+7.0, v(cbx1-3.0, SPYp, SPZp), X)      # O16.8 clearance pocket
    for sxc,syc in CORNERS: shell=shell.cut(tf(_col,sxc,syc))
    _grip=Part.makeCylinder(COLR+3.0, 2.0, v(cbx1+COLL-2.5, SPYp, SPZp), X).cut(
          Part.makeCylinder(COLR-0.2, 2.6, v(cbx1+COLL-2.8, SPYp, SPZp), X))    # grip ring O15.6, 2mm, at the collar tip
    for sxc,syc in CORNERS: shell=shell.fuse(tf(_grip,sxc,syc))
    shell=shell.removeSplitter()

    # ---- HIP OPENINGS: the leg passes through the shell ONLY in the band z>=BELLYZ (the swinging femur/tibia hang in
    #      the open belly below). Take the bbox of (leg INTERSECT z>=BELLYZ) over a slightly-beyond-working sweep at
    #      the +X+Y corner -- bounded near the hip, since the swinging shaft is below BELLYZ -- then cut a clearance
    #      BOX at each of the 4 corners. Bounded (roof spine Y<env / z>env stays) so the shell remains ONE piece. ----
    CLR=1.8; FTOP=20.7
    cx=SH9.fuse(COVER9)                       # coxa + cover (cover face reaches ~Y42.3, outboard of the shell wall)
    def acc(bb,e):
        if not bb.isValid() or bb.XLength<=0: return e
        return bb if e is None else App.BoundBox(min(e.XMin,bb.XMin),min(e.YMin,bb.YMin),min(e.ZMin,bb.ZMin),
                                                  max(e.XMax,bb.XMax),max(e.YMax,bb.YMax),max(e.ZMax,bb.ZMax))
    def envof(zb):
        e=None
        for sp in (-15,0,15): e=acc(rot(cx,sp,SP,X).common(zb).BoundBox,e)
        for sp in (-15,15):
            for ph in (-30,30):
                for th in (0,45): e=acc(leg_pose(sp,ph,th).common(zb).BoundBox,e)
        return e
    # TWO-BAND opening: at the hip the leg fills nearly the whole cross-section, so a single box would sever the shell
    # or clip the leg. Split at the frame top: a WIDE low cut (BELLYZ..FTOP, clears the low/wide leg) + a NARROW high
    # cut (>FTOP, clears the high leg but keeps a wide crown spine |Y|<(eH.YMin-CLR) as the bridge tying head/trunk/rump).
    eL=envof(box(500,80,FTOP-(BELLYZ-1),v(-250,-40,BELLYZ-1)))   # |Y|<=40, BELLYZ-1..FTOP
    eH=envof(box(500,80,100,v(-250,-40,FTOP)))                    # |Y|<=40, > FTOP
    def obox(e,z0,z1): return box(e.XMax-e.XMin+2*CLR, e.YMax-e.YMin+2*CLR, z1-z0, v(e.XMin-CLR, e.YMin-CLR, z0))
    hips=[obox(eL,KEEL-1.0,FTOP)]   # open the low hip all the way down to the keel: the single tall loft wraps the
    if eH is not None: hips.append(obox(eH,FTOP-0.5,eH.ZMax+CLR))
    for sxc,syc in CORNERS:
        for hb in hips: shell=shell.cut(tf(hb,sxc,syc))
    shell=shell.removeSplitter()
    print("HIP low Y[%.0f,%.0f] ; high Y[%.0f,%.0f]z<=%.0f -> crown spine |Y|<%.0f ; solids=%d"%(
        eL.YMin,eL.YMax,(eH.YMin if eH else 0),(eH.YMax if eH else 0),(eH.ZMax if eH else FTOP),
        (eH.YMin-CLR if eH else 40),len(shell.Solids)))

    # ---- CAP LEG RELIEF: the single tall loft wraps the caps DOWN into where the front/rear leg swings below the
    #      trunk belly (the hip openings only clear the near-hip z>=BELLYZ footprint; the old keel chamfer used to
    #      pull the belly inboard so the swinging leg passed OUTSIDE it). Cut the leg's swept envelope where it enters
    #      the cap lower region, BOUNDED to the cap X-band (x>=70) so the far-swinging toe can't eat the shell, +0.8
    #      gap, mirrored to all 4 corners. Keeps the OUTER cap smooth (this is an inner/lower wheel-well relief). ----
    _capbox=box(80.0,140.0,(BELLYZ+2)-(KEEL-2), v(70.0,-70.0,KEEL-2))     # +X cap lower region x[70,150], z[KEEL-2,BELLYZ+2]
    _legenv=None
    for sp in (-15,0,15):
        for ph in (-30,0,30):
            for th in (0,25,45):
                _seg=leg_pose(sp,ph,th).common(_capbox)
                if _seg.Volume>1: _legenv=_seg if _legenv is None else _legenv.fuse(_seg)
    if _legenv is not None:
        _legenv=_legenv.removeSplitter()
        for sxc,syc in CORNERS: shell=shell.cut(tf(_legenv,sxc,syc))
        shell=shell.removeSplitter()
        if len(shell.Solids)>1: shell=max(shell.Solids,key=lambda q:q.Volume)
        print("CAP LEG RELIEF cut (env vol %.0f)"%_legenv.Volume)

    # ---- SPINE STIFFENER RIB (issue 2, user 2026-07-29): the two-band hip cuts leave only a thin crown-spine strip
    #      (|Y|<~6) tying each cap to the trunk -- flimsy. Add a continuous rib on the roof UNDERSIDE along the spine
    #      (Y~0), rump->head, turning each flat strip into a T-beam (flange=strip, web=rib). rib = a Y-narrow column
    #      INTERSECTED with the outer body (so its top rides the roof outer, unchanged) then fused into the hollow
    #      shell -> a solid spine beam from the roof inner down to RIBZ. At Y~0 the frame centre bay is open (max
    #      z~4), so RIBZ can hang well below the frame top with clearance; kept modest for weight. ----
    RIBHY=2.5; RIBZ=21.5     # RIBZ just above the frame top (20.7): the corner blocks are near-full-width at their
    rib=box(FX-RX+10.0, 2*RIBHY, 60.0, v(RX-5.0, -RIBHY, RIBZ)).common(body)   # tops (coxa swings inboard), so a
    shell=shell.fuse(rib).removeSplitter()                                     # deeper rib would hit them; 21.5 clears

    # ---- issue-3 RECEIVERS (user 2026-07-29): the frame carries lateral detent NUBS + vertical alignment POSTS
    #      (built in framemin, positions in the shared globals NUB/NUB_X/SNAP_Y/POST/POST_Z0). Here we cut the body-
    #      side receivers: a wall POCKET over each nub (the seated detent sits relaxed; the wall above/below springs
    #      ~0.5mm for push-on / pull-off retention) and a CUP boss with a clearance blind hole over each peg (locates
    #      X-Y-theta as the canopy drops on). Nubs/pockets are on the trunk sides only (the hip openings gap the wall
    #      beyond x+/-60 and the caps sit beyond the frame) -- the stiffened spine rib carries that hold to the caps. ----
    _po=SNAP_Y+NUB['proud']+0.5                              # pocket outboard face, just past the nub tip
    _pk_out=_po-(SNAP_Y-0.3)
    for xc in NUB_X:
        pk=box(NUB['w']+2.0, _pk_out, 2*NUB['hh']+1.0, v(xc-(NUB['w']+2.0)/2.0, SNAP_Y-0.3, NUB['zc']-NUB['hh']-0.5))
        shell=shell.cut(pk).cut(tf(pk,1,-1))
    for (px,py,pd,ph) in POST:
        for sy in (1,-1):
            qy=py*sy
            boss=Part.makeCylinder((pd+3.0)/2.0, 70.0, v(px,qy,POST_Z0+0.3), Z).common(body)   # +0.3 clears frame top; clipped to roof
            shell=shell.fuse(boss)
            shell=shell.cut(Part.makeCylinder((pd+0.5)/2.0, ph+1.2, v(px,qy,POST_Z0-0.4), Z))   # clearance blind bore for the peg
    shell=shell.removeSplitter()
    # confirm the cup bores clear the frame pegs + the roof (peg tip must enter the bore, bore must stay blind):
    def _in(s,x,y,z):
        try: return s.isInside(App.Vector(x,y,z),0.05,True)
        except Exception: return False
    for (px,py,pd,ph) in POST:
        print("PROBE post x%+.0f y%+.0f: roofInner z"%(px,py)+"".join(
            " %d:%s"%(zz,"S" if _in(body,px,py,zz) else "-") for zz in (22,24,26,28,30)))

    # ---- verify shell clears frame + legs over the working range ----
    _frc=shell.common(FR); frov=_frc.Volume
    if frov>1:
        for _p in sorted(_frc.Solids,key=lambda q:-q.Volume)[:8]:
            _b=_p.BoundBox; print("  FR-OV X[%.0f,%.0f] Y[%.1f,%.1f] Z[%.1f,%.1f] v%.0f"%(_b.XMin,_b.XMax,_b.YMin,_b.YMax,_b.ZMin,_b.ZMax,_p.Volume))
    legworst=0.0; _lw=None
    for sp in (-15,0,15):
        for ph in (-30,0,30):
            for th in (0,25,45):
                _c=shell.common(leg_pose(sp,ph,th))
                if _c.Volume>legworst: legworst=_c.Volume; _lw=("leg sp%+d ph%+d th%d"%(sp,ph,th),_c)
    for sxc,syc in ((1,1),(1,-1),(-1,1),(-1,-1)):
        _c=shell.common(tf(cx,sxc,syc))
        if _c.Volume>legworst: legworst=_c.Volume; _lw=("coxa %d,%d"%(sxc,syc),_c)
    print("CLEAR shell^frame=%.1f  shell^leg(working sweep, worst)=%.1f mm3 (want ~0)"%(frov,legworst))
    if _lw and _lw[1].Volume>1:
        for _p in sorted(_lw[1].Solids,key=lambda q:-q.Volume)[:4]:
            _b=_p.BoundBox; print("  LEG-OV [%s] X[%.0f,%.0f] Y[%.1f,%.1f] Z[%.1f,%.1f] v%.0f"%(_lw[0],_b.XMin,_b.XMax,_b.YMin,_b.YMax,_b.ZMin,_b.ZMax,_p.Volume))

    bb=shell.BoundBox; ov=frov
    print("SHELL bbox X[%.1f,%.1f]=%.1f Y[%.1f,%.1f]=%.1f Z[%.1f,%.1f]=%.1f  fillet=%s wall=%.1f vol=%.0f"%(
        bb.XMin,bb.XMax,bb.XLength,bb.YMin,bb.YMax,bb.YLength,bb.ZMin,bb.ZMax,bb.ZLength,fil,WALL,shell.Volume))
    print("SHELL^FRAME overlap=%.1f mm3 (want ~0; shell inner Y%.1f clears frame Y35.7)"%(ov, 39.0-WALL))

    # ---- export STL + on-disk watertight repair (organic shells tessellate with self-int/nonman seams) ----
    import Mesh, MeshPart
    _m=MeshPart.meshFromShape(Shape=shell,LinearDeflection=0.12,AngularDeflection=0.5,Relative=False)
    _m.removeDuplicatedPoints(); _m.removeDuplicatedFacets(); _p=r"C:/ultrafish/robodog/stl/sm3sg90_body.stl"; _st="DEFECT!"
    for _it in range(6):
        _k=0
        while _m.hasSelfIntersections() and _k<4: _m.fixSelfIntersections(); _k+=1
        if _m.hasNonManifolds(): _m.removeNonManifolds()
        _m.harmonizeNormals(); _m.write(_p); _r=Mesh.Mesh(); _r.read(_p)
        if not (_r.hasSelfIntersections() or _r.hasNonManifolds()): _st="clean"; break
        _m=_r
    print("SHELL stl tris=%d %s"%(_m.CountFacets,_st))
    print("OK part_body")
except Exception:
    print("FAIL: "+traceback.format_exc())
