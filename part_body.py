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

# ---- scaled section table. Old body was sized to a ~245mm frame; new frame is 206 (x[-103,103]).
#      SX scales X only; hy=39 (width 78 over the 71.4 frame -> ~3mm skirt clearance/side); cz/hz kept
#      so the dome roof sits ~8mm above the frame top (z20.7) like before. ----
WALL=2.2; SX=0.84; BELLYZ=4.0
MAIN0=[(-150,15.2,39,9.5),(-138,15.4,39,10),(-122,15.5,39,11),(-90,16,39,12.5),(-45,16.3,39,13),
       (0,16.3,39,13),(45,16.5,39,13),(80,16.8,39,12.5),(100,18,39,12),(122,19.3,39,12.5),(140,19.6,39,12),(156,19.6,39,11.5)]
MAIN=[(x*SX,cz,hy+0.5,hz) for (x,cz,hy,hz) in MAIN0]   # hy 39->39.5: skirt inner ~37.3 clears the frame snap-bead tip (Y36.7)
FX=156*SX; RX=-147*SX                                   # slant-cut pivots (head front / rump rear)
SKIRTZ=-25.0                                            # shell belly wraps UNDER the frame keel (frame floor bottom zbot=-22);
                                                        # -25 keeps the 2.2mm floor's inner surface (~-22.8) below the frame floor -> clears it. Head+rump only (trunk reopened)
def sec(ztop,zbot,hy):
    cz=(ztop+zbot)/2.0; hz=(ztop-zbot)/2.0
    return (cz,hy,hz,min(10.0,max(0.6,hz-0.6),hy-0.6))

def build(IN):
    b=loft([(x,cz,hy-IN,hz-IN,max(0.6,min(10.0,hz-IN-0.6))) for (x,cz,hy,hz) in MAIN]).common(box(400,80-2*IN,200,v(-200,-40+IN,-40)))
    # SKIRT: extend the body straight DOWN (z7 -> SKIRTZ) so head+rump can drape over the frame sides+bottom (user
    # 2026-07-29). RULED loft, same x-stations as MAIN, overlaps the MAIN bottom (~z3-7) to fuse into one body.
    sk=Part.makeLoft([rrect(x,*sec(7.0,SKIRTZ+IN,hy-IN)) for (x,cz,hy,hz) in MAIN],True,True)
    b=b.fuse(sk).removeSplitter()
    dF=IN/math.cos(math.radians(22)); dR=IN/math.cos(math.radians(8))
    b=b.cut(rot(box(90,90,150,v(FX-dF,-45,-55)),22,v(FX-dF,0,19.6),Y))                  # head front 22deg slant
    b=b.cut(rot(box(90,90,150,v(RX-90+dR,-45,-55)),-8,v(RX+dR,0,15.2),Y))               # rump rear 8deg slant
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
    # HOLLOW to a thin wall, then OPEN the belly (remove everything below the skirt line) -> a canopy/dome.
    shell=body.cut(build(WALL)).removeSplitter()
    # KEEL CHAMFER: taper the shell's lower-outer edges to match the frame belly chamfer, offset OUT+down for ~2mm
    # clearance, so the head/rump belly hugs the frame keel. Frame chamfer (Y35.7,z-11)->(Y23.7,z-22); shell
    # (Y39.5,z-9)->(Y24.5,z-24). Applies to head/rump; the trunk skirt below is reopened next so it's unaffected.
    # chamfer line = the frame belly chamfer shifted ~3.5mm OUTward so even the 2.2mm-wall inner surface clears the
    # frame chamfer by ~1mm. Top (Y39.5,z-12.25) meets the straight skirt wall; slope dY/dz=1.09 (frame's); keel to (20.2,-30).
    chp=[v(-210,y,z) for (y,z) in [(39.5,-12.25),(50.0,-12.25),(50.0,-30.0),(20.2,-30.0)]]
    ch=Part.Face(Part.makePolygon(chp+[chp[0]])).extrude(v(420,0,0))
    shell=shell.cut(ch).cut(tf(ch,1,-1)).removeSplitter()
    # OPEN THE TRUNK BELLY ONLY (x in [-98.4,98.4], between + incl the hips): the head + rump keep the wrapped-down
    # skirt+keel; the trunk stays an open-belly canopy for the legs + belly electronics.
    shell=shell.cut(box(196.8,140,BELLYZ+80,v(-98.4,-70,BELLYZ-80)))
    shell=shell.removeSplitter()
    if len(shell.Solids)>1: shell=max(shell.Solids,key=lambda q:q.Volume)

    # ---- FRAME CLEARANCE: trim any shell material that dips into the frame outer envelope (+0.6/side gap). Height
    #      capped at the frame top (z20.7) so it does NOT eat the roof crown above it (that was severing the shell). ----
    shell=shell.cut(box(207.2,72.6,21.2,v(-103.6,-36.3,-0.5)))
    shell=shell.removeSplitter()
    # clear the 4 splay-pin COLLARS (O16, ~3mm proud of the frame outer at cbx1~100) + dowel stubs, which now sit
    # inside the head/rump drape and poke past the shell wall.
    _col=Part.makeCylinder(COLR+1.6, COLL+7.0, v(cbx1-3.0, SPYp, SPZp), X)
    for sxc,syc in CORNERS: shell=shell.cut(tf(_col,sxc,syc))
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
    hips=[obox(eL,BELLYZ-2.0,FTOP)]
    if eH is not None: hips.append(obox(eH,FTOP-0.5,eH.ZMax+CLR))
    for sxc,syc in CORNERS:
        for hb in hips: shell=shell.cut(tf(hb,sxc,syc))
    shell=shell.removeSplitter()
    print("HIP low Y[%.0f,%.0f] ; high Y[%.0f,%.0f]z<=%.0f -> crown spine |Y|<%.0f ; solids=%d"%(
        eL.YMin,eL.YMax,(eH.YMin if eH else 0),(eH.YMax if eH else 0),(eH.ZMax if eH else FTOP),
        (eH.YMin-CLR if eH else 40),len(shell.Solids)))

    # ---- SNAP FINGERS: 6 leaf-spring fingers (3 per rail at x=0,+-45) clip the shell onto the frame rim-cap bead
    #      (framemin, x[-75,75]). Each = a U-slot isolating a horizontal leaf (anchored at its -X end) + an inward NUB
    #      whose flat top hooks under the bead undershelf (z16.2-16.55) and whose sloped bottom cams over the bead on
    #      insertion. Print belly-up -> the leaf bends along X in the layer plane (strong). ----
    for xc in (-45.0,0.0,45.0):
        for sl in [box(14,6,1.4,v(xc-6,35.8,17.8)),        # top gap
                   box(14,6,1.4,v(xc-6,35.8,11.8)),        # bottom gap
                   box(1.4,6,6.0,v(xc+6,35.8,11.8))]:      # free-end gap (leaf anchored at its -X end)
            shell=shell.cut(sl).cut(tf(sl,1,-1))
        prof=[(37.4,16.3),(36.0,16.3),(36.0,15.3),(37.4,13.9)]     # Y-Z: flat catch top (z16.3, to Y36.0) + sloped lead-in bottom
        nub=Part.Face(Part.makePolygon([v(xc+1,y,z) for (y,z) in prof]+[v(xc+1,prof[0][0],prof[0][1])])).extrude(v(4.0,0,0))
        shell=shell.fuse(nub).fuse(tf(nub,1,-1))
    shell=shell.removeSplitter()

    # ---- verify shell clears frame + legs over the working range ----
    _frc=shell.common(FR); frov=_frc.Volume
    if frov>1:
        for _p in sorted(_frc.Solids,key=lambda q:-q.Volume)[:8]:
            _b=_p.BoundBox; print("  FR-OV X[%.0f,%.0f] Y[%.1f,%.1f] Z[%.1f,%.1f] v%.0f"%(_b.XMin,_b.XMax,_b.YMin,_b.YMax,_b.ZMin,_b.ZMax,_p.Volume))
    legworst=0.0
    for sp in (-15,0,15):
        for ph in (-30,0,30):
            for th in (0,25,45):
                legworst=max(legworst, shell.common(leg_pose(sp,ph,th)).Volume)
    for sxc,syc in ((1,1),(1,-1),(-1,1),(-1,-1)):
        legworst=max(legworst, shell.common(tf(cx,sxc,syc)).Volume)
    print("CLEAR shell^frame=%.1f  shell^leg(working sweep, worst)=%.1f mm3 (want ~0)"%(frov,legworst))

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
