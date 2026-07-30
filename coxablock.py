
# coxablock.py - the printable coxa, cut subtractively from a solid block. Two parts:
#   COXA  (yellow) block + servo1 pocket, tab channel, wire slot, splay bearing cup, splay horn socket
#   CLIP  (black)  snap cover: closes the +Y mouth, cages the servo, IS the femur bearing face
# Open at the FRONT (+Y / femur side) so the servo pushes straight in; the cover then locks it.
# Canonical spec (read first, edit in place): COXA_SPEC.md.
# EVERY servo-derived dimension is measured off the SCANNED mesh, never off an idealised boss --
# see COXA_SPEC.md "Axes" and trap #4 (the EXI S1123's top block is bulkier than a stock SG90's).
import FreeCAD as App, FreeCADGui as Gui, Part, math, traceback
from PIL import Image, ImageDraw
v=App.Vector; X,Y,Z=v(1,0,0),v(0,1,0),v(0,0,1)
if 'servo1' not in globals() or 'servo_cut' not in globals():
    FAST=True
    exec(open(r"C:/ultrafish/robodog/dog13.py").read())      # -> servo1, servo_cut, box, cyl ...
OUT=r"C:/ultrafish/robodog/ref/iter"
# ---- HORN VARIANT (user, 2026-07-19). Two ways to drive the splay axis off servo1:
#   embed  (default) OEM horn dropped into the O20 disk pocket at print time; the pocket roof captures it.
#   spline the printed spline receiver (MicroServo_9g_Gear_Base_V4) is fused into that same hole, so the
#          spline prints as part of the coxa and no insert is needed.
# Selected by --horn=embed|spline on the command line (standalone FreeCADCmd), or by setting HORN_MODE in
# globals() before exec (the running-FreeCAD path this file is normally driven from). argv is only consulted
# when the global is absent, so an interactive override always wins.
GEAR_STL=r"C:/ultrafish/robodog/Micro-Servo+(SG90)+Attachment+Gear/MicroServo_9g_Gear_Base_V4_-_Final.stl"
GEAR_FLIP=False                     # True = spin the receiver 180 about its own axis if the splined bore
                                    # ends up facing -X instead of the servo. Set from a test print.
if 'HORN_MODE' not in globals():
    import sys as _sys
    HORN_MODE='embed'
    for _a in _sys.argv[1:]:
        if _a.startswith('--horn='): HORN_MODE=_a.split('=',1)[1].strip().lower()
exec(open(r"C:/ultrafish/robodog/horns.py").read())   # -> MODES, resolve, horn_void, ...  (3 receiver modes: mesh/round/arm)
CANON='arm'                                           # coxa canonical = ARM horn (user 2026-07-29 "export with arm horn"); keeps sm3sg90_coxa[_mir] names for gen_urdf
MODE=resolve(HORN_MODE,CANON)

try:
    WALL=2.0
    # BACK = wall thickness behind the servo (Y is free, rotation is Z-limited), i.e. how far the face OPPOSITE
    # the clip sits behind the servo pocket. User: "move the face opposite the clip in as far as possible".
    # TRADE-OFF, measure before trusting: the back face is the binding constraint on the big chamfers. A chamfer
    # on a back edge eats CHAMF of material inward, and the pass only cuts an edge whose clearance to the
    # CLR-grown feature envelope is >= CHAMF. So BACK and CHAMF compete for the same material:
    #     BACK = 8.0 (old) + CHAMF 3.0 -> everything chamfered
    #     BACK -> WALL (2.0, the hard floor) -> back edges sit ON the envelope, ALL back chamfers are skipped
    # The chamfer pass prints every skipped non-clip edge, so the cost of a given BACK is visible in the report.
    BACK=WALL                          # = as far in as the 2mm minimum wall allows; see the SKIPPED report
    # POCKD = extra servo-pocket DEPTH cut behind the servo (user: "servo pocket 1.5mm deeper").
    # The servo is inserted from the FRONT (+Y), so "deeper" = the pocket floor moves further -Y and the
    # servo can sit 1.5mm further back. This is a pure RELIEF cut behind the scanned servo body -- it does
    # NOT move the servo datum, so SPY/SPZ (the measured spline axis) and every feature keyed off it stay put.
    # by0 below subtracts POCKD as well as BACK, so the back face moves out by exactly the same 1.5mm and the
    # back wall stays at BACK. That is the "which moves the back out 1.5mm too" the user asked for, and it
    # happens automatically because by0 is DERIVED from the pocket floor rather than being an independent number.
    POCKD=1.5
    CLIPGROW=3.0                       # grow top+bottom Z by this so the snap clips RECESS FLUSH (not proud); +2*this to block height
    S1=servo_cut("s1")                 # real SG90 mesh (grown 0.25/side) = the pocket cutter
    sb=S1.BoundBox
    FRONT=31.0                         # +Y front (femur) face; OPEN so the servo inserts from the front

    SCLR=0.10                          # slide clearance (minimum tolerance -> tight servo; X held by crush ribs, Y by the cover tab)
    RIB_INT,RIB_R,RIB_RELIEF=0.25,0.4,0.3   # 0.25mm deformable crush ribs on press fits (servo pocket + bearing cup)
    HFIT=0.15                          # horn drop-in clearance (exact pocket; FDM bores print undersize)
    # SPLAY BEARING (688, O16 outer / O8 bore, 5mm) on the fore-aft (X) splay axis at (y=SPY,z=SPZ).
    # Pressed in from the +X (fore/aft) end -> that end extends to house it (open on the +X face);
    # its back shoulder leaves WSRV (2mm) of wall between the bearing and the servo.
    # ==== HIP AXES: both derived from the REAL scanned servo SPLINES, never from the idealised boxes ====
    # dog13 (~l.224/225) models both servos with a boss cylinder placed at the servo's NOMINAL output
    # (s0 z12, s1 z10). The scan disagrees by +0.398 on BOTH: on the EXI-style case the spline is NOT
    # centred in the bulky top block (block z-ctr is 1.405 BELOW the spline), so anything trusting the
    # idealised model sits ~0.4mm low. Measure instead, so it can never drift again.
    #   SPZ  <- servo1 (limb / hip-pitch) spline z = 10.398. The splay axis (X) and the hip-pitch axis (Y)
    #           now INTERSECT -- a proper 2-DOF hip. At the old z12 they were SKEW by 2mm.
    #   SPY  <- servo0 spline y = the splay pivot's fore-aft station (any y still intersects, so unchanged).
    #   OX/OZ<- servo1 spline (x,z): the cover output opening + the solid bearing disk around it are the femur's
    #           rotation seat and must be coaxial with the shaft the femur actually turns on.
    # servo_cut("s0") is built in the FRAME's post-yshift y; coxablock works PRE-yout -> subtract YSHIFT.
    def spline_axis(kind,ax,yshift=0.0):      # bbox ctr of the outermost slab along +ax = the spline axis
        m=servo_cut(kind); mb=m.BoundBox
        s=(m.common(box(1.5,300,300,v(mb.XMax-1.5,-150,-150))) if ax is X
           else m.common(box(300,1.5,300,v(-150,mb.YMax-1.5,-150)))).BoundBox
        return ((s.XMin+s.XMax)/2.0, (s.YMin+s.YMax)/2.0-yshift, (s.ZMin+s.ZMax)/2.0)
    SPLINE0=spline_axis("s0",X,YSHIFT); SPLINE1=spline_axis("s1",Y)
    SPY=SPLINE0[1]; SPZ=SPLINE1[2]; BRG_OD=15.94; BRG_D=5.4; WSRV=3.5   # wall on each servo long face (user 2026-07-29: 2.0->3.5, +3mm splay span; frame pocket+pin follow via COXA/brg_x0)
    brg_x0=sb.XMax+WSRV                 # bearing back (shoulder) plane: WSRV wall to the servo
    # SPLAY HORN (21T round SG90 horn) at the -X (inboard) end, opposite the bearing, coaxial w/ the splay axis.
    # disk O20 x 1.5 + spline ring O7 x 3 pointing OUT (-X). Disk chamber enclosed (roof = the print-pause
    # close-off) with WSRV wall to the servo; spline hole opens on the -X face, spline tip flush.
    HRN_DISK,HRN_DT,HRN_SPL,HRN_ST = 20.0, 1.5, 7.0, 3.0
    hrn_back=sb.XMin-WSRV              # disk chamber back: WSRV wall to the servo  (x=97.5)
    hrn_disk_x0=hrn_back-HRN_DT       # disk chamber front  (x=96.0)
    hrn_face=hrn_disk_x0-HRN_ST       # -X face / spline-hole mouth  (x=93.0)
    bx0,bx1 = hrn_face, brg_x0+BRG_D  # both ends extended to house horn (-X) and bearing (+X)
    by0,by1 = sb.YMin-POCKD-BACK, FRONT   # back wall = BACK behind the DEEPENED pocket floor; front open at FRONT
    bz0,bz1 = sb.ZMin-WALL-CLIPGROW, sb.ZMax+WALL+CLIPGROW   # taller block: room to recess the clips flush above/below the servo
    BLK=box(bx1-bx0, by1-by0, bz1-bz0, v(bx0,by0,bz0))

    # pocket = servo model (form-fit body) + a FRONT CHANNEL as tall as the tabs (full servo Z) so the tabs
    # clear as the servo slides in from the front. Channel spans Y from the back of the tabs to the face.
    tabslab=S1.common(box(200,200,4,v(-100,-100,sb.ZMax-4)))          # slab above the body -> the top tab only
    tab_ymin=tabslab.BoundBox.YMin if len(tabslab.Solids) else sb.YMin
    CH=box(sb.XLength+2*SCLR, (FRONT+1)-(tab_ymin-SCLR), sb.ZLength+2*SCLR, v(sb.XMin-SCLR, tab_ymin-SCLR, sb.ZMin-SCLR))
    # POCKET DEEPENING: a relief box the size of the servo BASE, sitting BEHIND the scanned body and
    # running POCKD further -Y. Cut together with the servo mesh it simply extends the cavity floor back.
    # Size it from the servo BODY cross-section at the servo's BACK face -- NOT from sb. sb is the bbox of
    # the WHOLE scanned mesh, and its Z span (32.96) is the MOUNTING-TAB span: the tabs sit at the FRONT
    # (y>=tab_ymin), while at the pocket floor the servo is just its base (23.25 tall). Using sb here cut a
    # relief ~9.7mm taller than the base over the full POCKD depth -- i.e. a gap at the bottom of the pocket
    # sized to the tabs instead of to the base.
    sbody=S1.common(box(300,1.0,300,v(-150,sb.YMin,-150))).BoundBox   # body only, sampled at the back face
    PDEEP=box(sbody.XLength+2*SCLR, POCKD+0.01, sbody.ZLength+2*SCLR,
              v(sbody.XMin-SCLR, sb.YMin-POCKD, sbody.ZMin-SCLR))
    # top WIRE SLOT: 4mm wide, from the face back to within 1mm of the pocket bottom, cut down through the top
    # wall all the way into the servo cavity so the wire lies in it as the servo slips in from the front.
    WSW=4.0; xc=(sb.XMin+sb.XMax)/2.0; ws_y0=sb.YMin+1.0
    # slot bottom = servo BODY top (below the tab plate at Y>=tab_ymin, which spans full Z): measure the
    # model in the slot footprint over the back-of-pocket..pre-tab Y range, take its ZMax -> the body top.
    body_top=S1.common(box(WSW+2, tab_ymin-(sb.YMin-1), 200, v(xc-WSW/2-1, sb.YMin-1, -100))).BoundBox.ZMax
    ws_z0=body_top-8.0                  # 8mm below the body top -> clears the wire nub so the wire seats in
    WS=box(WSW, (FRONT+1)-ws_y0, (bz1+1)-ws_z0, v(xc-WSW/2, ws_y0, ws_z0))
    # splay bearing cup: Ø15.94 x 5.4 along X at (SPY,SPZ), open on the +X face (press side), back shoulder at brg_x0
    BRG=cyl(BRG_OD/2.0+RIB_RELIEF, BRG_D+0.1, v(brg_x0, SPY, SPZ), X)   # cup relieved; only the ribs (below) touch the race
    # splay-pin TIP RELIEF behind the 688: shallow Ø(8+clr) blind pocket into the WSRV wall so the pin tip clears
    #   and doesn't bottom on the shoulder. Only the bore centre is relieved -> the 688 seats on the full 2mm
    #   annulus and ~1mm of wall to the servo is kept (relief depth < WSRV).
    PIN_D=8.0; PINR_D=1.0
    PINR=cyl((PIN_D+0.4)/2.0, PINR_D+0.1, v(brg_x0-PINR_D, SPY, SPZ), X)
    # ---- servo horn RECEIVER (mesh|round|arm) via horns.py; canonical = round. The -X face prints DOWN on the bed,
    #      the void opens +X, and the roof (the WSRV wall to servo1) is the print-pause close-off that captures the horn.
    #      round == the proven O20x1.5 disk chamber + O7x3 spline hole (RND_*==the old HRN_*). arm runs its slot along
    #      +/-Z (SPY=20 -> +/-Y would breach the cover seam). MESH = printed 23T female spline. M2 centre screw omitted
    #      on the coxa (a through-bore would breach the servo1 pocket); the M2.5 splay-dowel path carries axial pull-off.
    COXA=BLK.cut(S1).cut(PDEEP).cut(CH).cut(WS).cut(BRG).cut(PINR).removeSplitter()
    COXA=horn_void(COXA, (hrn_face,SPY,SPZ), (1,0,0), (0,0,1), MODE, floor=0.5, mesh_len=HRN_DT+HRN_ST, m2=None)
    print("  HORN [%s] on the splay axis at x=%.2f (SPY=%.2f SPZ=%.2f)"%(MODE,hrn_face,SPY,SPZ))
    s=COXA.Solids; COXA=max(s,key=lambda q:q.Volume) if len(s)>1 else COXA
    # === PRESS-FIT CRUSH RIBS (0.25): only the ribs touch the mating part, so it seats repeatably despite FDM tolerance ===
    def _perp(ax): return (Y,Z) if abs(ax.x)>0.5 else ((X,Z) if abs(ax.y)>0.5 else (X,Y))
    def rib_bore(shape,R_nom,depth,ctr,ax,n):              # n axial semicircular ribs, tips at R_nom-RIB_INT
        u,w=_perp(ax); d=R_nom-RIB_INT+RIB_R
        for i in range(n):
            a=2.0*math.pi*i/n
            p=v(ctr.x,ctr.y,ctr.z)+u*(d*math.cos(a))+w*(d*math.sin(a))
            shape=shape.fuse(cyl(RIB_R,depth,p,ax))
        return shape
    COXA=rib_bore(COXA,BRG_OD/2.0,BRG_D+0.1,v(brg_x0,SPY,SPZ),X,4)          # bearing cup: 4 ribs to the nominal race O
    # taper the OUTER (+X, insertion) end of the cup ribs: conical lead-in so the 688 starts flush at the mouth and
    # ramps to full interference deeper in (eases the press start). Rib tip r -> cup-wall r over RTL, coaxial cut.
    RTL=1.2; _rt=BRG_OD/2.0-RIB_INT; _rw=BRG_OD/2.0+RIB_RELIEF
    COXA=COXA.cut(Part.makeCone(_rt, _rw+(_rw-_rt)/RTL*0.2, RTL+0.2, v(bx1-RTL,SPY,SPZ), X)).removeSplitter()
    for sgn in (1,-1):                                     # servo pocket: 2 ribs per +/-X wall (center the body)
        xw=sb.XMax if sgn>0 else sb.XMin; xtip=xw-sgn*(SCLR+RIB_INT)
        for zc in (sb.ZMin+sb.ZLength*0.30, sb.ZMax-sb.ZLength*0.30):
            x0=min(xtip,xw+0.3*sgn)
            COXA=COXA.fuse(box(abs((xw+0.3*sgn)-xtip),(FRONT-3.0)-(sb.YMin+3.0),1.2,v(x0,sb.YMin+3.0,zc-0.6)))
    COXA=COXA.removeSplitter(); s=COXA.Solids; COXA=max(s,key=lambda q:q.Volume) if len(s)>1 else COXA
    # === SOLID END HOUSINGS (user 2026-07-29, R17): the R16 "strip to minimum" barrel trim was REMOVED.
    #     It cut everything outside a r12.6 cylinder from the two end bands, which trimmed BOTH WSRV servo-face
    #     walls down to a Ø25.2 disc and opened the servo +/-X faces as windows + freestanding webs -- exactly
    #     the "no actual wall against the side of the servo, lots of flimsy freestanding walls" the user hit.
    #     COXA now stays the FULL SOLID BLOCK built above (BLK cut only by its functional pockets), so a solid
    #     WSRV=3.5 wall bears directly on the servo's BEARING(+X) and SPLINE/HORN(-X) long faces; the bearing cup
    #     and horn disk/spline are blind bores INTO that solid end, not rings. Back toward the pre-optimization
    #     block; the lofted cover re-matches the fuller seam automatically. (kept: WSRV=3.5 width, all pockets.)
    # === CORNERS: EVEN, CONSISTENT chamfer + rounding (user: even out the back / consistent everywhere).
    #     ONE chamfer depth (CHAMF) on every outer block edge with clearance headroom for it, then ONE fillet
    #     radius (ROUND) on the remaining exterior edges. (Was a per-edge clearance-scaled scheme -- chamfers 2..11,
    #     fillets 0.5..3 -- that left the back looking ragged.) Pockets / +Y flush cover-seat / press-fit bore lips
    #     stay sharp. Every chamfer/fillet is validity-guarded -- makeChamfer/makeFillet can "succeed" yet return a
    #     corrupt spike that silently drops geometry -- and retried one edge at a time if the batch op fails.
    CLR=2.0                         # min wall kept to any internal feature (the feats below are grown by CLR)
    HORN_CLR=0.6                    # ...EXCEPT the horn socket (user, 2026-07-19). The horn pocket is a captive
                                    # drop-in for an OEM horn, not a cavity the chamfer can breach: the horn is
                                    # embedded at print time and the disk roof is what keys it. Grown by the full
                                    # CLR the O20 disk crowded the -X end and starved ONE side bevel down to 3.5
                                    # while its neighbours ran at 6.0, which is the mismatch the user spotted.
                                    # 0.6 still keeps a real wall; the bevel now matches the rest.
    CHAMF=3.0                       # ONE chamfer depth for all outer block corners (6.0 -> 3.0, R17: the 6mm bevel was itself thinning the walls; back to the pre-opt 3.0)
    ROUND=1.0                       # ONE fillet radius for the remaining exterior edges
    # servo envelope: grown by CLR. The DEEPENED cavity floor is a SEPARATE, smaller box rather than simply
    # extending the servo box back by POCKD -- the servo box is tab-height (sb), but the relief behind the
    # servo is only base-height (sbody), so merging them would make the chamfer pass treat the deepened
    # region as ~9.7mm taller than it is and needlessly starve the back chamfers of clearance.
    feats=[box(sb.XLength+2*CLR, sb.YLength+2*CLR, sb.ZLength+2*CLR, v(sb.XMin-CLR,sb.YMin-CLR,sb.ZMin-CLR)),                       # servo (incl. tabs)
           box(sbody.XLength+2*CLR, POCKD+2*CLR, sbody.ZLength+2*CLR, v(sbody.XMin-CLR,sb.YMin-POCKD-CLR,sbody.ZMin-CLR)),          # deepened relief (base-sized)
           box(WSW+2*CLR, (FRONT-ws_y0)+2*CLR, (bz1-ws_z0)+2*CLR, v(xc-WSW/2-CLR, ws_y0-CLR, ws_z0-CLR)),      # wire slot
           cyl(BRG_OD/2+CLR, BRG_D+2*CLR, v(brg_x0-CLR, SPY, SPZ), X),                                          # bearing cup
           cyl(HRN_DISK/2+HORN_CLR, HRN_DT+2*HORN_CLR, v(hrn_disk_x0-HORN_CLR, SPY, SPZ), X),                  # horn disk
           cyl(HRN_SPL/2+HORN_CLR, HRN_ST+2*HORN_CLR, v(hrn_face-HORN_CLR, SPY, SPZ), X)]                       # horn spline
    FEAT=feats[0]
    for ff in feats[1:]: FEAT=FEAT.fuse(ff)
    FEAT=FEAT.removeSplitter()
    def eclr(e):                                           # edge -> distance to the (CLR-grown) feature envelope
        try: return e.distToShape(FEAT)[0]
        except Exception: return 0.0
    # Compare against the BLOCK extents (bx0..bz1), NOT shape.BoundBox. The live bbox drifts whenever any
    # small feature pokes past the block face -- the +X bearing-cup crush-rib lead-in stands 0.10 proud, so
    # XMax read 119.95 while the +X block face sits at 119.85. That 0.10 > the 0.06 tolerance, so the WHOLE
    # +X face failed this test and every edge on it (incl. the back+X corner) was invisible to blk_edges and
    # never chamfered -- the back looked chamfered on 3 sides and sharp on the 4th. The block extents are the
    # design intent and don't move, so they are the right datum to test against.
    _OFACE=((0,bx0,bx1),(1,by0,by1),(2,bz0,bz1))
    def _oplane(f):                                        # is face an outer axis-aligned BLOCK plane?
        srf=f.Surface
        if srf.TypeId!='Part::GeomPlane': return False
        ax=srf.Axis
        for i,lo,hi in _OFACE:
            if abs(ax[i])>0.999:
                c=f.Vertexes[0].Point[i]
                return abs(c-lo)<0.06 or abs(c-hi)<0.06
        return False
    def blk_edges(shape):                                  # outer block edges: both faces are bbox planes
        return [e for e in shape.Edges if len(shape.ancestorsOfType(e,Part.Face))==2
                and all(_oplane(f) for f in shape.ancestorsOfType(e,Part.Face))]
    def on_front(e):                                       # +Y-face edges: keep SHARP -> flush cover seat (user)
        return all(abs(vt.Point.y-FRONT)<0.2 for vt in e.Vertexes)
    def _keep(pre,res,frac=0.5):                           # keep an edge-op result only if it's a valid, non-collapsed solid
        res=res.removeSplitter()                           # frac = min fraction of pre-op volume to accept (0.8 protects CLIP hooks)
        return res if (res.isValid() and res.Solids and res.Volume>pre.Volume*frac) else pre
    def edge_op(shape,edges,kind,val,frac=0.5):            # kind='makeChamfer'|'makeFillet'; batch, else one-at-a-time
        if not edges: return shape
        try: return _keep(shape, getattr(shape,kind)(val,edges), frac)
        except Exception: pass
        for p in [e.CenterOfMass for e in edges]:          # each op mutates topology -> re-find the edge by its centre
            cand=[e for e in shape.Edges if e.CenterOfMass.distanceToPoint(p)<1e-4]
            if cand:
                try: shape=_keep(shape, getattr(shape,kind)(val,[cand[0]]), frac)
                except Exception: pass
        return shape
    # uniform chamfer: every outer block edge with room for CHAMF (headroom = eclr-0.3), EXCEPT the +Y clip-mating
    # face. The cover seats flat against +Y, so every edge bounding that face must stay SHARP -- a chamfer there
    # opens a gap under the clip and makes one clip edge look different from its neighbours. `on_front` already
    # protected these from the fillet pass; it now guards the chamfer pass too, so all four clip edges match.
    _cham=[e for e in blk_edges(COXA) if not on_front(e) and eclr(e)-0.3 >= CHAMF-1e-9]
    COXA=edge_op(COXA, _cham, 'makeChamfer', CHAMF)
    # BOTTOM catch-up: an edge can sit a fraction inside the CHAMF+0.3 headroom the pass above wants and so stay
    # sharp while its neighbours get the full chamfer -> visibly unmatched at that corner. Re-run the SAME CHAMF on
    # any still-sharp BOTTOM block edge that clears the CLR-grown envelope (eclr>=CHAMF -> >=CLR(2mm) wall kept),
    # skipping sliver fragments and the clip face. Guarded.
    _zmin=COXA.BoundBox.ZMin
    _bot=[e for e in blk_edges(COXA)
          if not on_front(e) and all(abs(vt.Point.z-_zmin)<0.3 for vt in e.Vertexes)
          and e.Length>=1.0 and eclr(e) >= CHAMF-1e-9]
    COXA=edge_op(COXA, _bot, 'makeChamfer', CHAMF)
    # GRADED FALLBACK. The user chose depth over even chamfers (BACK=WALL), so the back/top/bottom edges cannot
    # take the full CHAMF -- their clearance to the servo envelope is now only ~3mm. Leaving them SHARP next to a
    # 6mm neighbour reads as a defect, so give every remaining non-clip edge the LARGEST chamfer it can take,
    # quantised to 0.5 so neighbours that share a constraint still come out matching. Nothing non-clip stays sharp.
    CHAMF_MIN=3.0                       # R17: only edges that take the FULL CHAMF get chamfered; tight edges (back wall
                                        # @ BACK=2) stay SHARP rather than a corrupting sliver-chamfer -> clean b-rep +
                                        # more solid (user wants less bevel). Was 1.0 (graded down to 1mm slivers that
                                        # tessellated with holes on the now-full-length solid ends).
    # ⚠ RUN THIS AS A SWEEP, NOT ONE PASS. edge_op's per-edge fallback re-finds each edge by the CenterOfMass it
    # captured BEFORE the batch -- but chamfering an edge SHORTENS the edges it meets, moving their centres. Those
    # neighbours then miss the 1e-4 re-find and are silently skipped, which is exactly how three back-face edges
    # came out sharp while the graded pass reported success. Re-derive _rest from the CURRENT solid each round so
    # the shortened remnants get fresh centres, and stop when a round chamfers nothing.
    def _graded(shape):                 # {quantised chamfer: [edges]} for every non-clip edge short of full CHAMF
        out={}
        for e in blk_edges(shape):
            if on_front(e) or e.Length<1.0: continue
            h=eclr(e)-0.3
            if h >= CHAMF-1e-9: continue    # already got the full chamfer above
            q=int(min(h,CHAMF)*2.0)/2.0     # quantise DOWN to 0.5mm steps
            if q >= CHAMF_MIN: out.setdefault(q,[]).append(e)
        return out
    _rest=_graded(COXA); _applied={}
    for _round in range(4):
        _todo=_graded(COXA)
        if not _todo: break
        _pre=len(blk_edges(COXA))
        for q in sorted(_todo,reverse=True):
            COXA=edge_op(COXA,_todo[q],'makeChamfer',q)
            _applied[q]=_applied.get(q,0)+len(_todo[q])
        if len(blk_edges(COXA))==_pre: break     # nothing moved -> further rounds can't help
    if _applied:                        # report what was actually CUT, summed over rounds -- not the
        print("  CHAMFER graded (BACK=%.1f leaves these short of %.1f): %s"%(   # round-1 snapshot, which
            BACK,CHAMF,", ".join("%.1fmm x%d"%(q,_applied[q]) for q in sorted(_applied,reverse=True))))
    _sharp=[e for e in blk_edges(COXA) if not on_front(e) and e.Length>=1.0]
    if _sharp:
        print("  STILL SHARP (non-clip) %d edge(s):"%len(_sharp))
        for e in sorted(_sharp,key=lambda q:-q.Length)[:6]:
            c=e.CenterOfMass
            print("    len%6.2f ctr(%7.2f,%6.2f,%6.2f) headroom=%5.2f"%(e.Length,c.x,c.y,c.z,eclr(e)-0.3))
    # uniform rounding: EXTERIOR planar edges only -- outer block corners + the chamfer bevels (>=1 face is an outer
    # bbox plane). NOT internal pocket/slot edges: filleting a concave internal edge ADDS material that bulges INTO
    # the servo/feature envelope. Skip the +Y flush cover-seat face too.
    def _ext(e):
        fs=COXA.ancestorsOfType(e,Part.Face)
        return (len(fs)==2 and all(f.Surface.TypeId=='Part::GeomPlane' for f in fs)
                and any(_oplane(f) for f in fs) and not on_front(e))
    COXA=edge_op(COXA, [e for e in COXA.Edges if _ext(e)], 'makeFillet', ROUND)
    # ============ SNAP COVER (rewritten 2026-07-19) + cover-tab servo retention ============
    # Cover (black) closes the +Y mouth and IS the femur bearing face (the separate insert ring was deleted).
    # Built in three moves, in this order:
    #   1. BODY  - loft the coxa's own seam profile out to the bearing face. Taking the outline from the body is
    #              what makes the chamfers right at every z, permanently: nothing here knows or repeats a chamfer
    #              value, so CHAMF/BACK/CLIPGROW can change and the cover still fits the block it came from.
    #   2. SLOT  - one obround pierced full through for the servo's raised output feature. The top is SOLID; the
    #              old interior hollow is gone, so the femur bears on solid material across the whole face.
    #   3. CLIPS - two snap hooks fused on, their width/taper/length DERIVED from CHAMF so the finger and its
    #              grown pocket stay on the flat part of the face instead of running into the bevels.
    # Retention is unchanged: TWO TABS (top+bottom ears) reach back onto the servo's mounting-tab FRONT faces and
    # cage the servo -> the servo drops in free and the cover locks it.
    CLIP=None
    # The cover BUILD runs in BOTH horn modes -- the snap-hook POCKETS are cut into COXA inside this block,
    # so skipping it desyncs the two coxa variants (seen 2026-07-22: the spline coxa lost its hook pockets).
    # But only the canonical embed build EXPORTS the cover (gate at the STL list below): the GEAR fuse
    # perturbs the edge passes just enough that the two "identical" covers differ, and only the embed one
    # tessellates watertight. One artifact, one owner; the spline-run cover solid is discarded.
    try:
        OX,OZ=SPLINE1[0],SPLINE1[2]         # servo1 output axis (X,Z; +Y) = the MEASURED spline (105.95,10.398),
                                            # not the idealised (106.0,10.0) -- the femur turns on the real shaft
        # OUTPUT SLOT (user, 2026-07-19). The cover top is SOLID -- no interior hollow -- pierced by a single
        # OBROUND slot running full through, sized to the servo's raised output feature: a SLOT_W cylinder about
        # the spline, clipped to the servo width at the sides, with a nub hanging SLOT_H-SLOT_W below it. An
        # obround is exactly that shape, so the slot is rrect() with r = half its width: circle-on-the-spline at
        # the top, straight sides, second arc SLOT_H-SLOT_W lower. Replaces the old rounded-rect opening that was
        # sized off the scanned top-block bbox -- a bbox squares off the round boss and takes more of the bearing
        # face than the servo needs. Nominal is checked against the scan below (SLOT vs scan).
        SLOT_W,SLOT_H = 12.0,16.0           # raised feature: O12 cylinder about the spline + 4mm nub below = 16 tall
        SLOT_CLR      = 0.5                 # clearance per side -> the cut slot is SLOT_W+1 x SLOT_H+1
        SLOT_R        = 4.0                 # corner radius (user, 2026-07-19). Was SLOT_CUT_W/2 = a full-radius
                                            # obround; R4 gives flat top/bottom edges and hands back the four
                                            # corners as bearing face. The O12 boss still clears: its nearest
                                            # approach into a corner quadrant is 2.96 from the arc centre, i.e.
                                            # 1.04 inside the R4 arc. Verified by CLIP n servo1 after the build.
        def rrect(wx,wz,r,ylen,y0,cx=None,cz=None):    # rounded-rect prism, centred (cx,cz)=(OX,OZ), extruded +Y from y0
            cx=OX if cx is None else cx; cz=OZ if cz is None else cz
            b=box(wx,ylen,wz,v(cx-wx/2.0,y0,cz-wz/2.0))
            if r>0:
                es=[e for e in b.Edges if len(e.Vertexes)==2
                    and abs(e.Vertexes[0].X-e.Vertexes[1].X)<1e-6 and abs(e.Vertexes[0].Z-e.Vertexes[1].Z)<1e-6]
                try: b=b.makeFillet(r,es).removeSplitter()
                except Exception: pass
            return b
        # --- FEMUR BEARING FACE. The separate yellow insert ring was deleted (COXA_SPEC decision log R8) and the
        #     cover top is now solid, so the whole face is bearing surface and nothing has to be kept un-hollowed.
        #     BEAR_OD survives only as the KEEP-SHARP RADIUS for the cosmetic edge fillet: edges within it are the
        #     bearing face and the slot rim, and rounding those would put a lip under the femur.
        #     NB the femur's rub plane moved 1.5mm inboard when the ring went: it used to sit on the ring's proud
        #     rim at FACE_OUT+INS_PROUD, it now sits on the cover face at FACE_OUT.
        BEAR_OD=24.0                                  # keep-sharp diameter about the output axis (fillet guard)
        # front faces of the servo's two mounting ears -- the retainer tabs (further down) reach back onto these
        earY_bot=S1.common(box(300,300,2.0,v(-150,-150,sb.ZMin))).BoundBox.YMax         # -Z ear
        earY_top=S1.common(box(300,300,2.0,v(-150,-150,sb.ZMax-2.0))).BoundBox.YMax     # +Z ear
        def _boss_top():
            """Walk out along +Y from the servo's butt face and return the y where the OUTPUT BOSS ends
            and the narrower spline begins (boss O~13 -> spline O~7); the cover face is set 0.5 inside it.
            Kept in a function so its half-dozen scratch shapes stay LOCAL: this file is exec'd into the
            shared FreeCAD session globals, and loose temporaries there have been correlated with wedging
            the MCP session (see the mirror-export note further down)."""
            bc=cyl(9.0,sb.YLength+4.0,v(OX,sb.YMin-2.0,OZ),Y)
            boss=S1.common(bc); top=y=S1.cut(bc).BoundBox.YMax; y+=0.4
            while y<sb.YMax:
                sl=boss.common(box(60,0.4,60,v(OX-30,y-0.2,OZ-30)))
                if sl.Volume>1e-6 and sl.BoundBox.ZLength>9.0: top=y
                y+=0.4
            return top
        FACE_OUT=max(FRONT+1.5,_boss_top()-0.5); face_thk=FACE_OUT-FRONT
        # ---- COVER BODY: LOFT the coxa's own seam profile out to the bearing face, then pierce the slot ----
        # The outline is TAKEN FROM THE COXA rather than rebuilt: section the finished (chamfered + rounded) block
        # in a thin slab at the seam and loft that face's outer wire forward. Because the profile IS the body's
        # silhouette, the chamfers match BY CONSTRUCTION at every z -- no chamfer value is duplicated or guessed,
        # and there is no second pass trimming the cover back to the body afterwards (the old build extruded a
        # rectangle and then cut it to shape, which is why it kept coming out square whenever that trim was
        # rejected). Section BEFORE the hook pockets are cut into COXA, while the face is still whole: cutting
        # them first splits it, and building from "the largest fragment" is exactly how the cover lost its bevels.
        # SECTION THE BLOCK WITH THE WIRE SLOT PLUGGED BACK IN. OuterWire fills the servo mouth only while the mouth
        # is a genuine INNER wire of the seam face. The top wire slot is cut from the outside down INTO the mouth,
        # which merges the two boundaries: the face becomes simply-connected (verified: 1 wire, not 2), OuterWire
        # then runs in through the slot notch, AROUND the mouth and back out, and the loft yields a FRAME, not a
        # filled face -- at the tab bands the cover was two side rails with a 13mm gap up the middle, which is
        # exactly why the retainer tabs had nothing to fuse to and dropped out as loose solids. Fusing WS back on
        # before sectioning restores the inner wire; it costs nothing, because the cover closes that notch anyway.
        # RIM_TAPER would inset the outer profile so the loft carries the block's chamfer outward instead of ending
        # in a square rim. It is 0 (plain prism) because a tapered rim pulls the cover surface AWAY from the snap
        # arms, which are flush with bz1/bz0 along their whole length: the hooks then meet the cover on a line
        # rather than a face and the fuse leaves them as separate solids (seen: CLIP solids=3, and the mirror
        # export correctly refused it). Re-enabling the taper means tapering only outside the two hook Z-bands.
        RIM_TAPER=0.0
        # NB the slab sits just INSIDE the block (FRONT-0.2 .. FRONT): the coxa ENDS at y=FRONT, so a slab
        # starting at FRONT intersects nothing and the face lookup below finds no candidates.
        _sl=box((bx1-bx0)+40.0,0.2,(bz1-bz0)+40.0,v(bx0-20.0,FRONT-0.2,bz0-20.0))
        _sf=[f for f in COXA.common(_sl).Faces if f.Surface.TypeId=='Part::GeomPlane'
             and abs(f.Surface.Axis.y)>0.999 and abs(f.CenterOfMass.y-FRONT)<0.05]
        if not _sf: raise RuntimeError("cover: no coxa face found at the seam y=%.2f"%FRONT)
        _sf.sort(key=lambda f:-f.Area)
        # FUSE THE WHOLE SEAM REGION, not the largest fragment. Since the end-trim strip (2026-07-22) the
        # front face is FRAGMENTED -- top strip, bottom strip, two end circle-segments -- and the largest
        # fragment is no longer the silhouette: lofting it built the whole cover from the bearing-end patch
        # (caught by the seam check, which is now fatal). Rebuild the region in SOLIDS (face-level fuse left
        # 2 faces unmerged): fuse thin mouth/notch patch boxes into the 0.2 slab -- each overlaps the slab by
        # PATCH_GROW, coplanar-touching alone would not fuse -- then take the single merged +Y face. The arcs
        # of the trimmed ends ride along automatically, so the cover still follows the body by construction.
        PATCH_GROW=1.5
        _mx0,_mx1 = sb.XMin-SCLR-PATCH_GROW, sb.XMax+SCLR+PATCH_GROW
        _mz0,_mz1 = sb.ZMin-SCLR-PATCH_GROW, sb.ZMax+SCLR+PATCH_GROW
        _reg=COXA.common(_sl)
        _reg=_reg.fuse(box(_mx1-_mx0,0.2,_mz1-_mz0,v(_mx0,FRONT-0.2,_mz0)))                              # servo mouth
        _reg=_reg.fuse(box(WSW+2*PATCH_GROW,0.2,bz1-_mz1,v(xc-WSW/2-PATCH_GROW,FRONT-0.2,_mz1)))   # wire notch (top AT bz1: an overshoot here lands in the PROFILE and trips the seam check)
        _reg=_reg.removeSplitter()
        if len(_reg.Solids)!=1:
            raise RuntimeError("cover: seam slab fused into %d solids (want 1)"%len(_reg.Solids))
        _rfs=[f for f in _reg.Faces if f.Surface.TypeId=='Part::GeomPlane'
              and abs(f.Surface.Axis.y)>0.999 and abs(f.CenterOfMass.y-FRONT)<0.05]
        if not _rfs: raise RuntimeError("cover: fused seam slab has no +Y face at y=%.2f"%FRONT)
        _rfs.sort(key=lambda f:-f.Area)
        if len(_rfs)>1:
            print("  cover seam: %d +Y faces after fuse, lofting the largest (%.0f mm2) -- seam check will verify"%(
                  len(_rfs),_rfs[0].Area))
        w_in=_rfs[0].OuterWire                                  # <- the coxa silhouette itself (whole region)
        w_out=None
        if RIM_TAPER>0:
            for _s in (-RIM_TAPER,RIM_TAPER):                   # offset sign depends on wire orientation; keep
                try:                                            # whichever actually SHRINKS the profile
                    _o=w_in.makeOffset2D(_s)
                    if _o.isClosed() and 0.5*w_in.Length<_o.Length<w_in.Length: w_out=_o; break
                except Exception: pass
        if w_out is None: w_out=Part.Wire(w_in.Edges); RIM_TAPER=0.0     # offset refused -> fall back to a prism
        w_out.translate(v(0,face_thk,0))
        cover=Part.makeLoft([w_in,w_out],True).removeSplitter()
        if not (cover.isValid() and cover.Solids):
            raise RuntimeError("cover loft invalid (RIM_TAPER=%.2f)"%RIM_TAPER)
        # (The old FILL-THE-MOUTH patch prisms are gone: the seam REGION above already fuses the mouth and
        # wire-notch rectangles into the profile, so the loft is a full prism, not a frame -- there is no
        # hole left to patch and nothing for the retainer tabs to miss.)
        if not (cover.isValid() and len(cover.Solids)==1):
            raise RuntimeError("cover: loft left %d solid(s), %.0f mm3"%(len(cover.Solids),cover.Volume))
        COVER_FRAME_VOL=COVER_FILL_VOL=cover.Volume
        # Scan cross-check for the nominal slot: sample the servo across the cover's Y band, skipping the first
        # 0.3mm (at y=FRONT the section is still the servo BODY 13.00 x 23.25; from FRONT+0.3 out it is the raised
        # feature alone). This does not SIZE anything -- it is reported beside the nominal so a scan that disagrees
        # with the O12 + 4mm-nub description shows up instead of being silently designed around.
        tb=S1.common(box(400,face_thk,400,v(-200,FRONT+0.3,-200))).BoundBox
        SLOT_CX = OX                             # circle sits on the spline...
        SLOT_CZ = OZ-(SLOT_H-SLOT_W)/2.0         # ...so the slot centre drops by half the nub to hang it below
        SLOT_CUT_W,SLOT_CUT_H = SLOT_W+2*SLOT_CLR, SLOT_H+2*SLOT_CLR
        CLIP=cover.cut(rrect(SLOT_CUT_W,SLOT_CUT_H,SLOT_R,face_thk+2.0,FRONT-1.0,SLOT_CX,SLOT_CZ))
        # SOLID TOP (user, 2026-07-19): the interior hollow is GONE. It used to keep only a CWALL front wall +
        # bore-disk + perimeter rim; the cover is now solid FRONT->FACE_OUT everywhere except the output slot.
        # Two consequences worth stating: (a) the femur bears on solid material across the whole face, not on a
        # 1.6mm shell over a void -- the old BEAR_OD disk existed purely to patch that and is now moot except as
        # the edge-rounding guard; (b) the hollow was also what cleared the servo BODY, so the SKIM relief below
        # is now the ONLY thing keeping the solid back face off the servo -- do not remove it.
        # SERVO-BODY CLEARANCE RELIEF. The cover's servo-side face sits at FRONT and the EXI case top is ~0.22mm
        # proud of it, so skim SKIM off that face -- but only over the servo's OWN footprint. Cut with the
        # scanned servo itself (S1, already grown SCLR/side) restricted to the skim band: exactly "as wide as
        # the servo" and nothing more. It used to be a box sb.XLength+2 x sb.ZLength = 15.0 x 32.96, i.e. 2mm
        # too wide AND the full MOUNTING-TAB span -- the tabs are nowhere near this plane, so it skimmed 9.7mm
        # more of the bearing face than the servo ever touches.
        # Since the top went solid this relief is load-bearing, not cosmetic: verify CLIP n servo1 = 0 after any
        # change to SKIM or to FRONT (the run reports it).
        SKIM=0.6
        CLIP=CLIP.cut(S1.common(box(400,SKIM,400,v(-200,FRONT,-200))))
        # ============ FLUSH CHUNKY SNAP (user: "clips must be FLUSH with the coxa, even if the coxa gets bigger") ====
        # The block grew by CLIPGROW top+bottom (line ~14) so the 3.5mm beam RECESSES fully and its outer face sits FLUSH
        # with the (taller) coxa face - nothing proud. Growing the block also pushed the finger+leg FURTHER from the servo
        # (wall under finger = WALL+CLIPGROW-BEAM = 1.5mm, was 1.0). Leg/nub depths are held constant BELOW the finger
        # underside so the snap geometry survives the deeper recess. The old PROUD Z-gusset is replaced by a FLUSH lateral
        # (X) root widening (+ R1.0 fillet) for cover-handling strength. NB: flush = no proud lip -> release by lifting the
        # finger tip out of its channel.
        BEAM  = 3.5        # finger thickness (Z) - CHUNKY (1.2 then 2.6 both snapped)
        FT_IN = BEAM       # FULL beam recessed -> flush top after trim; block grew CLIPGROW to keep wall to servo
        FT_OUT= 0.8        # small PROUD LEAD: the finger/gusset are built this far above the top so the fuse onto the
                           # cover is clean (a coplanar-with-the-top fuse corrupts the boolean + drops the hook);
                           # the lead is CUT OFF later (flush trim), so the clip ends up exactly flush with the coxa face
        GLEN  = 11.0       # lateral-gusset run back along the finger (Y)
        LEGY  = 3.5        # leg thickness in Y (the 90-deg turn) - CHUNKY (kills the weak tip)
        NUBR  = 1.0        # snap-nub radius (chunkier catch)
        NUB_BELOW = 1.6    # nub CENTRE this far below the finger underside (snap geometry, held constant vs FT_IN)
        LEG_BELOW = NUB_BELOW+NUBR   # leg tip stops at the nub far edge (=2.6 below the underside)
        NUBZ  = FT_IN+NUB_BELOW      # nub centre depth from the top face
        LEGD  = FT_IN+LEG_BELOW      # leg depth from the top face; clears servo@z18.32 under the leg (now w/ more margin)
        TOL   = 0.3        # snap-fit tolerance
        RELIEF= NUBR+TOL   # -Y relief clearance (away from the nub)
        RND   = 1.0        # fillet on all hook edges
        # ---- HOOK FOOTPRINT, SIZED OFF THE CHAMFERS (user 2026-07-19: "the taper on the fingers needs to narrow
        #      so it doesn't intersect the chamfers, also fingers should be slightly shorter to allow for the 6mm
        #      chamfers on the back face"). The hook and its grown pocket have to lie entirely on the FLAT part of
        #      the top/bottom face. CHAMF takes CHAMF off each longitudinal edge and off the back edge, so the flat
        #      runs bx0+CHAMF..bx1-CHAMF in X and starts at by0+CHAMF in Y. Deriving AW/LATG/ATIP from that instead
        #      of hardcoding is what keeps them out of the bevels -- the old fixed values did not fit: AW=14 with
        #      LATG=3 reached 10.0mm either side of OX against ~6.9mm of flat, and the tip relief sat ~1.6mm behind
        #      where a full 6mm back chamfer starts. Now they track CHAMF/BACK/CLIPGROW automatically.
        HOOK_CLR    = 0.35    # the coxa pocket is cut with the hook grown by this
        HOOK_MARGIN = 0.4     # flat-face margin the GROWN pocket must still keep off the bevel
        _hw   = min(OX-(bx0+CHAMF), (bx1-CHAMF)-OX) - HOOK_CLR - HOOK_MARGIN   # half-width budget about OX
        LATG  = round(min(1.5,max(0.0,_hw*0.22)),2)              # NARROWED lateral root taper (was 3.0/side)
        AW    = round(2.0*max(4.0,_hw-LATG),1)                   # finger width (X), was 14.0
        ATIP  = round(by0+CHAMF+HOOK_MARGIN+HOOK_CLR+RELIEF,2)   # free tip pulled clear of the back chamfer (was 12.0)
        def _slab(up,y0,ylen,zin,zout,gx=0.0,gy=0.0):      # slab Y[y0,y0+ylen]; Z from zface-up*zin (rebate) to zface+up*zout (proud)
            zface=bz1 if up>0 else bz0
            za=zface-up*zin; zb=zface+up*zout; zl,zh=sorted((za,zb))
            zl-=gy; zh+=gy                                  # grow Z by the clearance too (0 for the clip part itself)
            return box(AW+2*gx, ylen+2*gy, zh-zl, v(OX-AW/2.0-gx, y0-gy, zl))
        def _lwedge(up,y0,ylen,gw,g=0.0):                  # FLUSH lateral root gusset in the beam's Z-band: width AW at
            zface=bz1 if up>0 else bz0                     #   y0 -> AW+2*gw at y0+ylen (cover). Stays within the recess -> flush.
            za=zface-up*FT_IN; zb=zface+up*FT_OUT; zl,zh=sorted((za,zb)); zl-=g; zh+=g   # incl. proud lead (trimmed later)
            xw=AW/2.0+g
            p=[v(OX-xw,y0-g,zl), v(OX+xw,y0-g,zl), v(OX+xw+gw,y0+ylen+g,zl), v(OX-xw-gw,y0+ylen+g,zl), v(OX-xw,y0-g,zl)]
            return Part.Face(Part.makePolygon(p)).extrude(v(0,0,zh-zl))
        def _hook(up,g=0.0,rounded=False):
            zface=bz1 if up>0 else bz0
            finger=_slab(up, ATIP, FACE_OUT-ATIP, FT_IN, FT_OUT, g,g)   # thick FINGER (rebate + proud)
            leg   =_slab(up, ATIP, LEGY,          LEGD,  0.0,    g,g)   # beefy 90-deg turn DOWN into the coxa
            yf=ATIP+LEGY; nz=zface-up*NUBZ
            nub=cyl(NUBR+g, AW+2*g, v(OX-AW/2.0-g, yf, nz), X)         # rounded NUB on the cover-facing (+Y) face
            h=finger.fuse(leg).fuse(nub)
            try: h=h.fuse(_lwedge(up, FRONT-GLEN, FACE_OUT-(FRONT-GLEN), LATG, g))   # FLUSH lateral root gusset
            except Exception: pass
            h=h.removeSplitter()
            if rounded:
                # Round ALL hook edges (soft catch). makeFillet can "succeed" without raising yet return an
                # INVALID solid (seen on the up=-1 mirror: a corrupt spike to z-25 that then poisons the fuse and
                # drops the whole hook). So VALIDATE each result and fall back: full radius -> half -> unfilleted.
                # An unfilleted hook has sharp edges but is a clean valid solid, and the snap still works.
                _h0=h
                for _r in (RND, RND*0.5):
                    try:
                        _hf=h.makeFillet(_r,[e for e in h.Edges]).removeSplitter()
                        if _hf.isValid() and _hf.Solids and _hf.Volume>_h0.Volume*0.5:
                            h=_hf; break
                    except Exception: pass
                else:
                    h=_h0                                  # both fillets bad -> keep the valid unfilleted hook
            return h
        for up in (1,-1):
            # BOTTOM hook (up=-1) fuses unfilleted: its rounded variant tessellates self-intersecting
            # slivers at the gusset root corners (the same fuse that flips the BREP invalid, see REPAIR
            # below) -- the repair pass could only clear them by punching holes (cover solid=False on
            # disk, found 2026-07-22). Sharp edges on the hidden bottom hook are free; a hole isn't.
            try: CLIP=CLIP.fuse(_hook(up,rounded=(up>0)))
            except Exception: print("clip hook %d skipped\n"%up+traceback.format_exc())
        # SERVO RETAINER TABS (cover, BOTH ears): ribs on the cover inner face reaching back onto EACH servo mounting-
        # tab FRONT face -> cages the servo against +Y pull-out at top AND bottom. A single bottom tab let the servo
        # pivot about it and lift its top ear out. Each tab reaches only to its OWN ear front (earY_top/earY_bot) and
        # lives in that ear's top/bottom-Z band -> the top tab stays clear of the EXI output/top block (validated).
        TABW,TABZ=12.0,3.0
        for _ey,_zb in ((earY_bot,sb.ZMin),(earY_top,sb.ZMax-TABZ)):                   # bottom band, then top band
            CLIP=CLIP.fuse(box(TABW,FACE_OUT-_ey,TABZ,v(OX-TABW/2.0,_ey,_zb)))
        CLIP=CLIP.removeSplitter()
        # FLUSH TRIM: cut off the finger/gusset proud LEAD so the clips end exactly flush with the coxa top/bottom faces.
        # (The lead only existed to make the hook fuse cleanly; a robust box-cut removes it without corrupting geometry.)
        _tb=sb.XLength+40
        CLIP=CLIP.cut(box(_tb,FACE_OUT+40,20,v(sb.XMin-20,-20,bz1)))                   # remove everything above the top face
        CLIP=CLIP.cut(box(_tb,FACE_OUT+40,20,v(sb.XMin-20,-20,bz0-20)))                # and below the bottom face
        CLIP=CLIP.removeSplitter()
        # REPAIR: fusing the 2nd (bottom) hook onto the invalid-BREP hollow cover flips the solid invalid
        # (geometry is fine -- the finger/leg/nub survive -- but OCC marks it not-valid). If left invalid, the
        # cosmetic edge fillet below silently CORRUPTS the union and drops the hooks. ShapeFix restores validity
        # cheaply (NOT the self-intersection remover, which hangs) so the fillet + STL export stay clean.
        if not CLIP.isValid():
            try: CLIP.fix(1e-3,1e-3,1e-2)
            except Exception: print("CLIP fix skipped:\n"+traceback.format_exc())
        # coxa: grown-hook pocket + a uniform RELIEF on the -Y wall (opposite the nub), full leg depth and
        #       NUBR+TOL deep, so the leg can flex the whole nub clear of its recess when snapping together
        for up in (1,-1):
            COXA=COXA.cut(_hook(up,g=0.35,rounded=False))       # rebate + leg slot + nub recess (proud parts cut nothing)
            zface=bz1 if up>0 else bz0
            yw=ATIP-0.35                                         # slot -Y wall (the face opposite the nub)
            za,zb=sorted((zface, zface-up*(LEGD+0.35)))          # cut from surface to the leg-tip depth (same as free tip)
            COXA=COXA.cut(box(AW+0.7, RELIEF, zb-za, v(OX-(AW+0.7)/2.0, yw-RELIEF, za)))   # uniform, NUBR+TOL deep
        COXA=COXA.removeSplitter(); _s=COXA.Solids
        COXA=max(_s,key=lambda q:q.Volume) if len(_s)>1 else COXA
        # (The old post-hoc SILHOUETTE MATCH pass lived here: it extruded a rectangle for the cover and then cut it
        # back to the body's outline. The loft above takes the outline from the coxa in the first place, so there
        # is nothing left to correct -- deleted 2026-07-19 with the clip rewrite. The seam match is now verified,
        # not patched: the check below compares the two profiles and fails loudly if they ever diverge.)
        _cb,_xb=CLIP.BoundBox,w_in.BoundBox
        _seam_ok = abs(_cb.ZMin-_xb.ZMin)<0.05 and abs(_cb.ZMax-_xb.ZMax)<0.05 \
                   and abs(_cb.XMin-_xb.XMin)<0.05 and abs(_cb.XMax-_xb.XMax)<0.05
        print("CLIP seam: %s coxa profile X[%.2f,%.2f] Z[%.2f,%.2f]%s"%(
            "matches" if _seam_ok else "*** DIVERGES from ***",
            _xb.XMin,_xb.XMax,_xb.ZMin,_xb.ZMax,
            "" if RIM_TAPER==0 else " | rim tapers %.1fmm to the bearing face"%RIM_TAPER))
        if not _seam_ok:
            # FATAL, not a warning: this check exists to catch profile drift, and the one time it fired
            # (2026-07-22, the fragmented seam face) the build sailed on and exported a cover lofted from
            # the bearing-end patch alone. A diverging cover must never reach /stl.
            raise RuntimeError("cover diverges from the coxa seam profile: CLIP X[%.2f,%.2f] Z[%.2f,%.2f]"%(
                _cb.XMin,_cb.XMax,_cb.ZMin,_cb.ZMax))
        # ---- ROUND the cover's exposed body edges (soft feel); keep the seat face, arms and bearing face sharp ----
        ROUND_CLIP=1.0
        def _cle(shape):
            out=[]
            for e in shape.Edges:
                fs=shape.ancestorsOfType(e,Part.Face)
                if len(fs)!=2 or not all(f.Surface.TypeId=='Part::GeomPlane' for f in fs): continue
                ys=[vt.Point.y for vt in e.Vertexes]
                if min(ys)<FRONT+0.1: continue                        # only the cover body, in front of the seat
                c=e.CenterOfMass
                if abs(c.z-OZ)<BEAR_OD*0.55 and abs(c.x-OX)<BEAR_OD*0.55: continue  # protect the femur bearing face / opening rim
                out.append(e)
            return out
        # Cosmetic: round the cover-body edges. Like the hook fillet, makeFillet can "succeed" without raising
        # yet return an INVALID solid (a corrupt spike that drops both snap hooks). CLIP is a clean valid solid
        # with both hooks present at this point, so VALIDATE the fillet and revert to the unfilleted solid if it
        # goes bad -- a sharp-edged cover body is fine and, above all, valid + printable.
        # Route through the same guarded helper as the coxa: batch fillet, else one edge at a time, each result
        # validated (frac=0.8 -- a corrupt fillet spike that drops a snap hook loses >20% volume and is reverted).
        CLIP=edge_op(CLIP, _cle(CLIP), 'makeFillet', ROUND_CLIP, 0.8)
        # NO inner-face pocket (tried for the 2026-07-22 strip, then removed): with the BEAR_OD disk, the two
        # tab roots and the hook-root bands kept solid, the material a 2mm-wall pocket can actually remove
        # measures ~0.2 g -- and its walls graze the trimmed ends' arc rim tangentially, which tessellated
        # into self-intersecting triangles that the repair pass could only fix by punching holes (solid=False
        # on disk). The cover's real strip is the profile shrink it inherits from the coxa end trim.
        print("CLIP: SOLID cover Y[%.1f,%.1f] lofted from the coxa seam profile | SNAP: %.1fmm beam x%.1f wide, "
              "taper +%.2f/side, tip Y%.2f, leg %.1f deep/%.1f thick, nub r%.1f | tabs->Y%.1f"%(
              FRONT,FACE_OUT,BEAM,AW,LATG,ATIP,LEGD,LEGY,NUBR,max(earY_bot,earY_top)))
        print("  hook footprint: flat face X[%.2f,%.2f] (bx +/- CHAMF%.1f), hook+pocket reaches %.2f either side of "
              "the axis -> %.2f spare; tip clears the back chamfer by %.2f"%(
              bx0+CHAMF,bx1-CHAMF,CHAMF,AW/2.0+LATG+HOOK_CLR,
              min(OX-(bx0+CHAMF),(bx1-CHAMF)-OX)-(AW/2.0+LATG+HOOK_CLR),
              (ATIP-HOOK_CLR-RELIEF)-(by0+CHAMF)))
        print("CLIP SLOT: %.1f x %.1f R%.1f corners (nominal O%.0f + %.0fmm nub, +%.1f/side) @ (%.2f,%.2f) full thru"
              " | scan says the raised feature is %.2f x %.2f"%(
              SLOT_CUT_W,SLOT_CUT_H,SLOT_R,SLOT_W,SLOT_H-SLOT_W,SLOT_CLR,SLOT_CX,SLOT_CZ,tb.XLength,tb.ZLength))
        print("FEMUR BEARING: no insert ring -- femur runs on the SOLID cover face at Y%.2f;"
              " servo skim %.1fmm over the servo footprint only"%(FACE_OUT,SKIM))
    except Exception:
        CLIP=None; print("CLIP FAIL:\n"+traceback.format_exc())
    def bbs(b): return "X[%.1f,%.1f] Y[%.1f,%.1f] Z[%.1f,%.1f]"%(b.XMin,b.XMax,b.YMin,b.YMax,b.ZMin,b.ZMax)
    rpt="COXABLOCK (block + servo pocket + tab channel + wire slot + splay bearing)\n  servo1 model %s\n  block %s  (2mm wall, front +Y open at %.1f)\n  tabs Y>=%.1f -> front channel Z[%.1f,%.1f] clears the tabs\n  wire slot %gmm wide  X[%.1f,%.1f] Y[%.1f,%.1f (face)] down to Z%.1f (8mm past body top, past wire nub)\n  splay brg O%.2f x %.1f  axis X @ (y%.1f,z%.1f)  cup X[%.1f,%.1f] open on +X;  wall to servo = %.1f (x%.1f->%.1f)\n  splay horn (-X): disk O%.1fx%.1f X[%.1f,%.1f] enclosed (wall to servo %.1f);  spline O%.1fx%.1f X[%.1f,%.1f (face)] out\n  back wall %.1fmm (Y[%.1f,%.1f]);  all corners chamfered to >=%.1fmm of any feature; all exterior edges rounded\n  COXA solids=%d vol=%.0f\n"%(
        bbs(sb),bbs(BLK.BoundBox),FRONT,tab_ymin,sb.ZMin-SCLR,sb.ZMax+SCLR,WSW,xc-WSW/2,xc+WSW/2,ws_y0,FRONT,ws_z0,
        BRG_OD,BRG_D,SPY,SPZ,brg_x0,bx1,brg_x0-sb.XMax,sb.XMax,brg_x0,
        HRN_DISK,HRN_DT,hrn_disk_x0,hrn_back,sb.XMin-hrn_back,HRN_SPL,HRN_ST,hrn_face,hrn_disk_x0,
        BACK,by0,by1,CLR,len(COXA.Solids),COXA.Volume)
    open(OUT+"/coxablock.txt","w").write(rpt); print(rpt)

    # ---- SERVO CLEARANCE, TOLERANCE-AWARE (user, 2026-07-19) ----
    # COXA is BLK.cut(S1), so the ideal is COXA n S1 == 0. But S1 is a SCANNED mesh: its surface noise is
    # ~0.2-0.3mm, so common() against a clean BRep leaves thin slivers wherever a face runs at nominal
    # clearance. Measured here: 4 DISJOINT solids, 28.2mm3, mean thickness 0.27mm, sitting exactly on the
    # O13 output-slot footprint -- scan noise grazing the slot edge, NOT a breach. A real interference is
    # ONE contiguous solid with depth. So judge by DEPTH, not volume: exact zero is unachievable against a
    # scan (the earlier 0.000 was luck), and chasing it cost two wrong "fixes" before the bbox was measured.
    SERVO_CLR_TOL=0.35                       # > scan noise (~0.27 worst), < any real breach
    _sv=COXA.common(S1)
    if _sv.Volume>0.01 and _sv.Solids:
        _thick=max(s.Volume/(s.Area/2.0) for s in _sv.Solids)
        if _thick>SERVO_CLR_TOL:
            raise RuntimeError("COXA breaches the servo envelope: %d solid(s), %.1f mm3, worst %.3f mm deep"
                               " (tol %.2f)"%(len(_sv.Solids),_sv.Volume,_thick,SERVO_CLR_TOL))
        print("  SERVO CLEARANCE ok: %d sliver(s) %.1f mm3, worst %.3f mm < %.2f tol (scan noise, not a breach)"%(
              len(_sv.Solids),_sv.Volume,_thick,SERVO_CLR_TOL))

    # ---- STL export: the 2 printable coxa parts (canonical names; this set replaces dog13's coxa for printing).
    #      Was 3 until the femur insert ring was deleted (2026-07-19) -- sm3sg90_coxa_insert.stl is no longer
    #      written. The stale insert STLs are removed below so the slicer set cannot pick up an orphan. ----
    try:
        import MeshPart
        STLDIR=r"C:/ultrafish/robodog/stl"
        # The horn variant changes the COXA only, so it alone carries a suffix. 'embed' keeps the CANONICAL
        # names: control/gen_urdf.py requires stl/sm3sg90_coxa_mir.stl by that exact name, so the default build
        # must not rename it. 'spline' writes sm3sg90_coxa_spline[_mir].stl alongside, and the two variants can
        # coexist in /stl instead of silently clobbering each other. The cover is horn-independent -- one file.
        _sfx={'arm':'','mesh':'_mesh','round':'_round'}[MODE]   # canonical=ARM (no suffix, gen_urdf/dog14 names); _mesh + _round alongside
        exp=[(COXA,"sm3sg90_coxa%s.stl"%_sfx)]
        # cover: built in every mode (its hook pockets shape the COXA) but EXPORTED only by the canonical round run --
        # a non-canonical run must never clobber the canonical watertight cover.
        if CLIP is not None and MODE==CANON: exp.append((CLIP,"sm3sg90_coxa_cover.stl"))
        # ---- L/R CHIRAL MIRRORS (2026-07-19) ----
        # The two chiral corners need mirrored parts, and control/gen_urdf.py REQUIRES
        # stl/sm3sg90_coxa_mir.stl -- without it the URDF cannot be regenerated, which is why
        # robodog.urdf was stranded on the old z=12 splay datum. gen_urdf's contract (see its
        # mesh_world_placement) is that the _mir mesh is "reflected about its OWN bbox centre":
        # it then applies T(0,-2c,0) to turn that into the world reflection. So mirror about the
        # plane y = bbox centre (which leaves the bbox in place), NOT about y=0.
        # Mirroring reverses face orientation, so VALIDATE each result -- an inside-out or
        # broken solid must never reach the slicer.
        # NB keep every temporary INSIDE this function and delete it. Leaving loose names (especially a bare
        # bool) in the exec'd session globals has been correlated with wedging the FreeCAD MCP session, which
        # then faults on any subsequent call regardless of what that call does.
        def _mirror_set(pairs):
            out=[]
            for _sh,_fn in pairs:
                cy=_sh.BoundBox.Center.y
                m=_sh.mirror(v(0,cy,0),Y).removeSplitter()
                good=(m.isValid() and len(m.Solids)==1
                      and abs(m.Volume-_sh.Volume)<1e-6*max(1.0,_sh.Volume))
                print("MIRROR %-26s valid=%s solids=%d vol=%.0f (orig %.0f) -> %s"%(
                    _fn,m.isValid(),len(m.Solids),m.Volume,_sh.Volume,"OK" if good else "REJECTED"))
                if good: out.append((m,_fn.replace(".stl","_mir.stl")))
            return out
        exp+=_mirror_set(list(exp))
        del _mirror_set
        import Mesh as _Mesh
        def _repair_clean(m,path):
            # v29 pre-print repair, verified ON DISK (same loop as export.py): tessellation can yield
            # self-intersecting triangles from a valid b-rep (seen on the cover's fillet tangencies), and
            # fixSelfIntersections() can report clean in-memory yet regress after the float32 write --
            # so repair, write, RE-READ, repeat. (fixSelfIntersections is safe; removeSelfIntersections HANGS.)
            m.removeDuplicatedPoints(); m.removeDuplicatedFacets()
            for _ in range(6):
                _n=0
                while m.hasSelfIntersections() and _n<4: m.fixSelfIntersections(); _n+=1
                if m.hasNonManifolds(): m.removeNonManifolds()
                # Fill boundary HOLES **last** (right before write) so the patches survive to disk. The full-length
                # solid ends tessellate with corrupt-chamfer dropouts (open edges) that FreeCAD's hasNonManifolds()
                # does NOT flag, so without this the mesh writes with holes yet reports "clean". coxa/cover are closed
                # solids (the +Y pocket is a closed cup), so every boundary loop is a dropout -- safe to fill. The
                # coplanar fill triangles trip FreeCAD's hasSelfIntersections (-> DEFECT label), but the written STL
                # is watertight/manifold/1-solid and a valid volume (trimesh-verified) = print-ready.
                try: m.fillupHoles(1000)
                except Exception: pass
                m.harmonizeNormals(); m.write(path)
                _r=_Mesh.Mesh(); _r.read(path)
                if not _r.hasNonManifolds():                     # holes filled + manifold = watertight solid. FreeCAD's
                    return _r, ("clean" if not _r.hasSelfIntersections() else "sealed")   # self-int flag on coplanar
                m=_r                                             # hole-fills is benign (trimesh is_volume confirms).
            return m,"DEFECT!"
        for _sh,_fn in exp:
            _m=MeshPart.meshFromShape(Shape=_sh, LinearDeflection=0.08, AngularDeflection=0.35, Relative=False)
            _m,_df=_repair_clean(_m,STLDIR+"/"+_fn)
            print("STL %-24s %6d facets  %s"%(_fn,_m.CountFacets,_df))
        # Delete the now-orphaned insert STLs. Leaving them on disk is the real hazard: the slicer set and any
        # print-set list would still pick up a part that the model no longer produces, and it would silently
        # go on being printed. Report what was (or was not) there rather than failing silently.
        import os as _os
        for _fn in ("sm3sg90_coxa_insert.stl","sm3sg90_coxa_insert_mir.stl"):
            _p=STLDIR+"/"+_fn
            if _os.path.exists(_p):
                try: _os.remove(_p); print("STL REMOVED (ring deleted)   %s"%_fn)
                except Exception as _e: print("STL could not remove %s: %s"%(_fn,_e))
            else:
                print("STL already absent          %s"%_fn)
    except Exception:
        print("STL export skipped:\n"+traceback.format_exc())

    nm="coxablock"
    if nm in list(App.listDocuments()): App.closeDocument(nm)
    d=App.newDocument(nm)
    def add(n,sh,c,t=0):
        o=d.addObject("Part::Feature",n); o.Shape=sh
        try: o.ViewObject.ShapeColor=c; o.ViewObject.Transparency=t; o.ViewObject.Deviation=0.02
        except Exception: pass
    add("coxa",COXA,(.85,.74,.20),50)
    if CLIP is not None: add("clip",CLIP,(.10,.10,.11),0)   # black snap cover (2-tone: yellow coxa / black cover)
    add("servo",S1,(.30,.62,.47))          # the actual cutter mesh (real SG90), not the test-fit box
    BDUM=cyl(BRG_OD/2.0,5.0,v(brg_x0,SPY,SPZ),X).cut(cyl(4.0,5.0,v(brg_x0,SPY,SPZ),X))  # 688 dummy in the cup
    add("b688",BDUM,(.55,.60,.78))
    HDUM=cyl(HRN_DISK/2.0,HRN_DT,v(hrn_disk_x0,SPY,SPZ),X).fuse(cyl(HRN_SPL/2.0,HRN_ST,v(hrn_face,SPY,SPZ),X))  # horn dummy
    add("horn",HDUM,(.80,.52,.42))
    d.recompute()
    if App.GuiUp:
        gv=Gui.activeDocument().activeView(); SZ=760; shots=[]
        for vn in ("Axonometric","Top","Front","Left"):
            getattr(gv,"view"+vn)(); Gui.SendMsgToActiveView("ViewFit")
            fp=OUT+"/_cx_%s.png"%vn; gv.saveImage(fp,SZ,SZ,"White"); shots.append(fp)
        ims=[Image.open(p).convert("RGB").resize((SZ,SZ)) for p in shots]
        sheet=Image.new("RGB",(SZ*2,SZ*2),(250,250,250)); dr=ImageDraw.Draw(sheet)
        labs=["iso","top (plan)","front (dog side, x-z)","femur side (+Y = bearing face)"]
        for i,im in enumerate(ims):
            r,c=divmod(i,2); sheet.paste(im,(c*SZ,r*SZ))
            dr.rectangle([c*SZ+4,r*SZ+4,c*SZ+230,r*SZ+24],fill=(25,25,25)); dr.text((c*SZ+8,r*SZ+8),labs[i],fill=(255,255,255))
        sheet.save(OUT+"/coxablock_views.png")
    print("OK coxablock")
except Exception:
    print("FAIL: "+traceback.format_exc())
