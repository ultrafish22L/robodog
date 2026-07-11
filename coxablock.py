
# coxablock.py - MINIMAL: a solid block + the servo1 pocket only, sized for a 2mm wall on the 5 closed
# faces, open at the FRONT (+Y / femur side) so the servo pushes straight in. Nothing else cut.
import FreeCAD as App, FreeCADGui as Gui, Part, math, traceback
from PIL import Image, ImageDraw
v=App.Vector; X,Y,Z=v(1,0,0),v(0,1,0),v(0,0,1)
if 'servo1' not in globals() or 'servo_cut' not in globals():
    FAST=True
    exec(open(r"C:/ultrafish/robodog/dog13.py").read())      # -> servo1, servo_cut, box, cyl ...
OUT=r"C:/ultrafish/robodog/ref/iter"

try:
    WALL=2.0; BACK=8.0                 # BACK = face-to-back wall behind the servo (rotation is Z-limited, Y free)
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
    SPY,SPZ=20.0,12.0; BRG_OD=15.94; BRG_D=5.4; WSRV=2.0
    brg_x0=sb.XMax+WSRV                 # bearing back (shoulder) plane: WSRV wall to the servo
    # SPLAY HORN (21T round SG90 horn) at the -X (inboard) end, opposite the bearing, coaxial w/ the splay axis.
    # disk O20 x 1.5 + spline ring O7 x 3 pointing OUT (-X). Disk chamber enclosed (roof = the print-pause
    # close-off) with WSRV wall to the servo; spline hole opens on the -X face, spline tip flush.
    HRN_DISK,HRN_DT,HRN_SPL,HRN_ST = 20.0, 1.5, 7.0, 3.0
    hrn_back=sb.XMin-WSRV              # disk chamber back: WSRV wall to the servo  (x=97.5)
    hrn_disk_x0=hrn_back-HRN_DT       # disk chamber front  (x=96.0)
    hrn_face=hrn_disk_x0-HRN_ST       # -X face / spline-hole mouth  (x=93.0)
    bx0,bx1 = hrn_face, brg_x0+BRG_D  # both ends extended to house horn (-X) and bearing (+X)
    by0,by1 = sb.YMin-BACK, FRONT      # thick back wall (Y is free); front open at FRONT
    bz0,bz1 = sb.ZMin-WALL-CLIPGROW, sb.ZMax+WALL+CLIPGROW   # taller block: room to recess the clips flush above/below the servo
    BLK=box(bx1-bx0, by1-by0, bz1-bz0, v(bx0,by0,bz0))

    # pocket = servo model (form-fit body) + a FRONT CHANNEL as tall as the tabs (full servo Z) so the tabs
    # clear as the servo slides in from the front. Channel spans Y from the back of the tabs to the face.
    tabslab=S1.common(box(200,200,4,v(-100,-100,sb.ZMax-4)))          # slab above the body -> the top tab only
    tab_ymin=tabslab.BoundBox.YMin if len(tabslab.Solids) else sb.YMin
    CH=box(sb.XLength+2*SCLR, (FRONT+1)-(tab_ymin-SCLR), sb.ZLength+2*SCLR, v(sb.XMin-SCLR, tab_ymin-SCLR, sb.ZMin-SCLR))
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
    # horn: O20x1.5 disk chamber (enclosed, WSRV to servo) + O7x3 spline hole opening on the -X face
    DISKP=cyl(HRN_DISK/2.0+HFIT, HRN_DT, v(hrn_disk_x0, SPY, SPZ), X)   # exact drop-in pocket (roof captures + keys the holed horn)
    SPLP =cyl(HRN_SPL/2.0+HFIT, HRN_ST+0.1, v(hrn_face-0.1, SPY, SPZ), X)
    COXA=BLK.cut(S1).cut(CH).cut(WS).cut(BRG).cut(PINR).cut(DISKP).cut(SPLP).removeSplitter()
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
    # === CORNERS: EVEN, CONSISTENT chamfer + rounding (user: even out the back / consistent everywhere).
    #     ONE chamfer depth (CHAMF) on every outer block edge with clearance headroom for it, then ONE fillet
    #     radius (ROUND) on the remaining exterior edges. (Was a per-edge clearance-scaled scheme -- chamfers 2..11,
    #     fillets 0.5..3 -- that left the back looking ragged.) Pockets / +Y flush cover-seat / press-fit bore lips
    #     stay sharp. Every chamfer/fillet is validity-guarded -- makeChamfer/makeFillet can "succeed" yet return a
    #     corrupt spike that silently drops geometry -- and retried one edge at a time if the batch op fails.
    CLR=2.0                         # min wall kept to any internal feature (the feats below are grown by CLR)
    CHAMF=3.0                       # ONE chamfer depth for all outer block corners
    ROUND=1.0                       # ONE fillet radius for the remaining exterior edges
    feats=[box(sb.XLength+2*CLR, sb.YLength+2*CLR, sb.ZLength+2*CLR, v(sb.XMin-CLR,sb.YMin-CLR,sb.ZMin-CLR)),  # servo
           box(WSW+2*CLR, (FRONT-ws_y0)+2*CLR, (bz1-ws_z0)+2*CLR, v(xc-WSW/2-CLR, ws_y0-CLR, ws_z0-CLR)),      # wire slot
           cyl(BRG_OD/2+CLR, BRG_D+2*CLR, v(brg_x0-CLR, SPY, SPZ), X),                                          # bearing cup
           cyl(HRN_DISK/2+CLR, HRN_DT+2*CLR, v(hrn_disk_x0-CLR, SPY, SPZ), X),                                  # horn disk
           cyl(HRN_SPL/2+CLR, HRN_ST+2*CLR, v(hrn_face-CLR, SPY, SPZ), X)]                                      # horn spline
    FEAT=feats[0]
    for ff in feats[1:]: FEAT=FEAT.fuse(ff)
    FEAT=FEAT.removeSplitter()
    def eclr(e):                                           # edge -> distance to the (CLR-grown) feature envelope
        try: return e.distToShape(FEAT)[0]
        except Exception: return 0.0
    def _oplane(f,shape):                                  # is face an outer axis-aligned bbox plane?
        srf=f.Surface
        if srf.TypeId!='Part::GeomPlane': return False
        ax=srf.Axis; bb=shape.BoundBox
        for i,nm in ((0,'X'),(1,'Y'),(2,'Z')):
            if abs(ax[i])>0.999:
                c=f.Vertexes[0].Point[i]
                return abs(c-getattr(bb,nm+'Min'))<0.06 or abs(c-getattr(bb,nm+'Max'))<0.06
        return False
    def blk_edges(shape):                                  # outer block edges: both faces are bbox planes
        return [e for e in shape.Edges if len(shape.ancestorsOfType(e,Part.Face))==2
                and all(_oplane(f,shape) for f in shape.ancestorsOfType(e,Part.Face))]
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
    # uniform chamfer: every outer block edge with room for CHAMF (headroom = eclr-0.3); this evens out the back
    COXA=edge_op(COXA, [e for e in blk_edges(COXA) if eclr(e)-0.3 >= CHAMF-1e-9], 'makeChamfer', CHAMF)
    # BEARING-side bottom catch-up: the front-bottom edge on the bearing (+X) end sits ~0.12mm inside the CHAMF+0.3
    # headroom the pass above wants, so it stayed sharp while its neighbours (back-bottom, -X bottom) got the 3.0
    # chamfer -> visibly unmatched at that corner. Re-run the SAME CHAMF on any still-sharp BOTTOM block edge that
    # clears the CLR-grown feature envelope (eclr>=CHAMF -> >=CLR(2mm) wall kept), skipping sliver fragments. Guarded.
    _zmin=COXA.BoundBox.ZMin
    _bot=[e for e in blk_edges(COXA)
          if all(abs(vt.Point.z-_zmin)<0.3 for vt in e.Vertexes) and e.Length>=1.0 and eclr(e) >= CHAMF-1e-9]
    COXA=edge_op(COXA, _bot, 'makeChamfer', CHAMF)
    # uniform rounding: EXTERIOR planar edges only -- outer block corners + the chamfer bevels (>=1 face is an outer
    # bbox plane). NOT internal pocket/slot edges: filleting a concave internal edge ADDS material that bulges INTO
    # the servo/feature envelope. Skip the +Y flush cover-seat face too.
    def _ext(e):
        fs=COXA.ancestorsOfType(e,Part.Face)
        return (len(fs)==2 and all(f.Surface.TypeId=='Part::GeomPlane' for f in fs)
                and any(_oplane(f,COXA) for f in fs) and not on_front(e))
    COXA=edge_op(COXA, [e for e in COXA.Edges if _ext(e)], 'makeFillet', ROUND)
    # ============ SNAP COVER (external pry-off clips) + cover-tab servo retention ============
    # Cover (black) closes the +Y mouth and seats the yellow femur insert (ROUNDED bullnose contact). It snaps on via
    # TWO EXTERNAL pry-off arms (inward barb into a blind detent); TWO TABS (top+bottom ears) reach back onto the servo's
    # mounting-tab FRONT faces and cage the servo -> servo drops in free, cover locks it (NO internal coxa ear-lips).
    CLIP=None; RING=None
    try:
        OX,OZ=106.0,10.0                    # servo output axis (X,Z, +Y)
        # The EXI S1123 has a BULKY ~12(X)x16(Z) top/output block (the SG90 tapers to a small boss) that pokes
        # through the cover -> the output opening is a ROUNDED-RECT sized to clear it (SG90 then fits loose), and
        # the femur ring is a slip-fit FRAME around that opening (not a press-fit round washer).
        OPEN_X,OPEN_Z,OPEN_R=15.0,18.0,3.0  # rounded-rect output opening (clears the EXI block + ~1mm/side)
        def rrect(wx,wz,r,ylen,y0,cx=None,cz=None):    # rounded-rect prism, centred (cx,cz)=(OX,OZ), extruded +Y from y0
            cx=OX if cx is None else cx; cz=OZ if cz is None else cz
            b=box(wx,ylen,wz,v(cx-wx/2.0,y0,cz-wz/2.0))
            if r>0:
                es=[e for e in b.Edges if len(e.Vertexes)==2
                    and abs(e.Vertexes[0].X-e.Vertexes[1].X)<1e-6 and abs(e.Vertexes[0].Z-e.Vertexes[1].Z)<1e-6]
                try: b=b.makeFillet(r,es).removeSplitter()
                except Exception: pass
            return b
        # --- FEMUR INSERT (separate yellow part): friction ring that seats in a recess in the (flat) cover.
        #     Bore Ø17; a concentric counterbore on its +Y face is the thrust-washer seat -> FRICTION now
        #     (femur rubs the raised rim), drop a small hobby thrust washer/bearing in the seat later. Swappable = tunable.
        REC_OD,REC_DEP,INS_PROUD,REC_FIT=24.0,2.0,1.5,0.20   # ring seat OD/depth, proud rub, SLIP fit (no press) in the recess (OD<~27 face width)
        earY=max(S1.common(box(300,300,2.0,v(-150,-150,sb.ZMax-2.0))).BoundBox.YMax,   # +Z ear front face
                 S1.common(box(300,300,2.0,v(-150,-150,sb.ZMin))).BoundBox.YMax)       # -Z ear front face
        # cover outer face = butt top - 0.5
        _bc=cyl(9.0,sb.YLength+4.0,v(OX,sb.YMin-2.0,OZ),Y)
        _boss=S1.common(_bc); Y_bf=S1.cut(_bc).BoundBox.YMax; boss_top=Y_bf; _y=Y_bf+0.4
        while _y<sb.YMax:                                       # walk out the butt: boss(O~13) -> spline(O~7)
            _sl=_boss.common(box(60,0.4,60,v(OX-30,_y-0.2,OZ-30)))
            if _sl.Volume>1e-6 and _sl.BoundBox.ZLength>9.0: boss_top=_y
            _y+=0.4
        FACE_OUT=max(FRONT+1.5,boss_top-0.5); face_thk=FACE_OUT-FRONT; CWALL=1.6
        ff=[f for f in COXA.Faces if f.Surface.TypeId=='Part::GeomPlane'
            and abs(f.Surface.Axis.y)>0.999 and abs(f.CenterOfMass.y-FRONT)<0.3]
        ff.sort(key=lambda f:-f.Area)
        cover=Part.Face(ff[0].OuterWire); cover.translate(v(0,FRONT-cover.BoundBox.YMin,0))   # coxa face outline
        cover=cover.extrude(v(0,face_thk,0)); fb=ff[0].BoundBox
        cover=cover.fuse(box((sb.XMax+4.0)-(sb.XMin-4.0),face_thk,fb.ZMax-sb.ZMin,v(sb.XMin-4.0,FRONT,sb.ZMin)))
        CLIP=cover.cut(rrect(OPEN_X,OPEN_Z,OPEN_R,face_thk+2.0,FRONT-1.0))   # rounded-rect output opening (clears the EXI top block)
        # HOLLOW the interior: keep front wall + bore-disk + perimeter rim. This also clears the servo BODY (the
        # solid cover back face otherwise bites the servo ~0.2mm). The hollow leaves an INVALID BREP, but that is
        # harmless now: the snap hooks are valid solids, the fillets are validity-guarded, and .fix() (below, after
        # the trim) repairs the cover to a valid solid before any downstream shaping/export.
        WI=FACE_OUT-CWALL
        _hol=box(fb.XLength-4.0,WI-FRONT,fb.ZLength-4.0,v(fb.XMin+2.0,FRONT,fb.ZMin+2.0))
        _hol=_hol.cut(cyl(REC_OD/2.0+2.0,WI-FRONT+2.0,v(OX,FRONT-1.0,OZ),Y))                    # leave a SOLID disk to host the insert recess
        CLIP=CLIP.cut(_hol)
        # the retained bore-disk's servo-side face sits at FRONT and the EXI top face is ~0.22mm proud of it -> skim
        # 0.6mm off the disk over the servo footprint (recess floor is back at FACE_OUT-REC_DEP, so ~1.5mm backing
        # survives). Clears the servo with a single box cut (no fuse/fillet -> boolean stays clean).
        CLIP=CLIP.cut(box(sb.XLength+2.0,0.6,sb.ZLength,v(sb.XMin-1.0,FRONT,sb.ZMin)))            # servo-top clearance relief
        # FLAT cover face + round RECESS (Ø26) to seat the separate femur ring as a SLIP fit
        CLIP=CLIP.cut(cyl(REC_OD/2.0+REC_FIT/2.0,REC_DEP+0.01,v(OX,FACE_OUT-REC_DEP,OZ),Y))     # ring recess (SLIP, open +Y)
        # build the INSERT (yellow) = a SLIP-FIT thrust FRAME (was a press-fit round washer that wouldn't seat).
        # Ø26 seat disk drops into the recess (no press); a flat rim stands INS_PROUD as the femur rub face; a
        # ROUNDED-RECT hole clears the EXI top block. The femur back holds it seated (swappable, tunable drag).
        _yb=FACE_OUT-REC_DEP; RIM_OD=REC_OD-2.0                          # proud rim slightly inset from the seat
        RING=cyl(REC_OD/2.0,REC_DEP,v(OX,_yb,OZ),Y)                      # seat disk (slip fit, flush to face)
        RING=RING.fuse(cyl(RIM_OD/2.0,INS_PROUD+0.001,v(OX,FACE_OUT,OZ),Y))           # flat-topped proud rub rim
        RING=RING.cut(rrect(OPEN_X+1.0,OPEN_Z+1.0,OPEN_R,REC_DEP+INS_PROUD+2.0,_yb-1.0)).removeSplitter()  # rounded-rect clearance hole
        # CHAMFER the TOP of the rub rim (femur-contact face): bevel ONLY the outer rim top edge (the RIM_OD
        # circle/arcs at the top plane) -> clean lead-in, no sharp lip on the rub face. NOT the inner rrect hole
        # (that's clearance, and chamfering it together with the outer edge is what silently killed the old 0.4).
        # Guard the chamfer (makeChamfer can "succeed" yet return an invalid solid) and revert if it goes bad.
        CHM=0.6; _pre=RING
        try:
            _te=[e for e in RING.Edges
                 if abs(e.CenterOfMass.y-(FACE_OUT+INS_PROUD))<0.05                                  # at the top plane
                 and type(e.Curve).__name__=='Circle' and abs(e.Curve.Radius-RIM_OD/2.0)<0.2]        # outer rim only
            if _te:
                _rc=RING.makeChamfer(CHM,_te).removeSplitter()
                RING=_rc if (_rc.isValid() and _rc.Solids and _rc.Volume>_pre.Volume*0.5) else _pre
        except Exception:
            RING=_pre; print("RING top chamfer skipped:\n"+traceback.format_exc())
        # SIMPLE HOOK CLIP (one flush arm per side, like before, with a hook on the end that snaps into the coxa):
        # a long flush arm runs back from the cover along the coxa top/bottom; at its free tip it bends 90 deg
        # DOWN into the coxa (the hook leg), and a ROUNDED NUB on that leg snaps into a coxa CAVITY. Push the
        # cover -Y: the arm flexes, the leg rides in and the nub clicks into the cavity; +Y pull is held by the
        # leg captured in its slot + the nub in the cavity. Flex the arm out to pop it back off.
        # ============ FLUSH CHUNKY SNAP (user: "clips must be FLUSH with the coxa, even if the coxa gets bigger") ====
        # The block grew by CLIPGROW top+bottom (line ~14) so the 3.5mm beam RECESSES fully and its outer face sits FLUSH
        # with the (taller) coxa face - nothing proud. Growing the block also pushed the finger+leg FURTHER from the servo
        # (wall under finger = WALL+CLIPGROW-BEAM = 1.5mm, was 1.0). Leg/nub depths are held constant BELOW the finger
        # underside so the snap geometry survives the deeper recess. The old PROUD Z-gusset is replaced by a FLUSH lateral
        # (X) root widening (+ R1.0 fillet) for cover-handling strength. NB: flush = no proud lip -> release by lifting the
        # finger tip out of its channel.
        AW    = 14.0       # hook width (X) - CHUNKY
        ATIP  = 12.0       # finger free tip / 90-deg corner (-Y end); longer beam keeps flex strain <1%
        BEAM  = 3.5        # finger thickness (Z) - CHUNKY (1.2 then 2.6 both snapped)
        FT_IN = BEAM       # FULL beam recessed -> flush top after trim; block grew CLIPGROW to keep wall to servo
        FT_OUT= 0.8        # small PROUD LEAD: the finger/gusset are built this far above the top so the fuse onto the
                           # hollow cover is clean (a coplanar-with-the-top fuse corrupts the boolean + drops the hook);
                           # the lead is CUT OFF later (flush trim), so the clip ends up exactly flush with the coxa face
        LATG  = 3.0        # FLUSH lateral (X) root gusset: widen the beam by LATG/side at the cover, taper to 0 over GLEN
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
            try: CLIP=CLIP.fuse(_hook(up,rounded=True))
            except Exception: print("clip hook %d skipped\n"%up+traceback.format_exc())
        # SERVO RETAINER TABS (cover, BOTH ears): ribs on the cover inner face reaching back onto EACH servo mounting-
        # tab FRONT face -> cages the servo against +Y pull-out at top AND bottom. A single bottom tab let the servo
        # pivot about it and lift its top ear out. Each tab reaches only to its OWN ear front (earY_top/earY_bot) and
        # lives in that ear's top/bottom-Z band -> the top tab stays clear of the EXI output/top block (validated).
        TABW,TABZ=12.0,3.0
        earY_bot=S1.common(box(300,300,2.0,v(-150,-150,sb.ZMin))).BoundBox.YMax        # -Z ear front face
        earY_top=S1.common(box(300,300,2.0,v(-150,-150,sb.ZMax-2.0))).BoundBox.YMax    # +Z ear front face
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
        # ---- ROUND the cover's exposed body edges (soft feel); keep the seat face, arms and femur ring sharp ----
        ROUND_CLIP=1.0
        def _cle(shape):
            out=[]
            for e in shape.Edges:
                fs=shape.ancestorsOfType(e,Part.Face)
                if len(fs)!=2 or not all(f.Surface.TypeId=='Part::GeomPlane' for f in fs): continue
                ys=[vt.Point.y for vt in e.Vertexes]
                if min(ys)<FRONT+0.1: continue                        # only the cover body, in front of the seat
                c=e.CenterOfMass
                if abs(c.z-OZ)<REC_OD*0.55 and abs(c.x-OX)<REC_OD*0.55: continue   # protect the femur ring / opening rim
                out.append(e)
            return out
        # Cosmetic: round the cover-body edges. Like the hook fillet, makeFillet can "succeed" without raising
        # yet return an INVALID solid (a corrupt spike that drops both snap hooks). CLIP is a clean valid solid
        # with both hooks present at this point, so VALIDATE the fillet and revert to the unfilleted solid if it
        # goes bad -- a sharp-edged cover body is fine and, above all, valid + printable.
        # Route through the same guarded helper as the coxa: batch fillet, else one edge at a time, each result
        # validated (frac=0.8 -- a corrupt fillet spike that drops a snap hook loses >20% volume and is reverted).
        CLIP=edge_op(CLIP, _cle(CLIP), 'makeFillet', ROUND_CLIP, 0.8)
        print("CLIP: flat cover Y[%.1f,%.1f] | FLUSH SNAP: %.1fmm beam x%.0f RECESSED flush (block grew %.1f/side, wall2servo %.2f) + flush lateral gusset +%.1f/side, leg %.1fmm deep/%.1f thick, nub r%.1f | tabs->Y%.1f"%(
            FRONT,FACE_OUT,BEAM,AW,CLIPGROW,WALL+CLIPGROW-BEAM,LATG,LEGD,LEGY,NUBR,earY))
        if RING is not None:
            print("RING insert: SLIP-FIT frame OD%.0f seat%.1f + flat rim OD%.0f proud%.1f, rounded-rect hole %.0fx%.0f | solids=%d"%(
                REC_OD,REC_DEP,REC_OD-2.0,INS_PROUD,OPEN_X+1.0,OPEN_Z+1.0,len(RING.Solids)))
    except Exception:
        CLIP=None; print("CLIP FAIL:\n"+traceback.format_exc())
    def bbs(b): return "X[%.1f,%.1f] Y[%.1f,%.1f] Z[%.1f,%.1f]"%(b.XMin,b.XMax,b.YMin,b.YMax,b.ZMin,b.ZMax)
    rpt="COXABLOCK (block + servo pocket + tab channel + wire slot + splay bearing)\n  servo1 model %s\n  block %s  (2mm wall, front +Y open at %.1f)\n  tabs Y>=%.1f -> front channel Z[%.1f,%.1f] clears the tabs\n  wire slot %gmm wide  X[%.1f,%.1f] Y[%.1f,%.1f (face)] down to Z%.1f (8mm past body top, past wire nub)\n  splay brg O%.2f x %.1f  axis X @ (y%.1f,z%.1f)  cup X[%.1f,%.1f] open on +X;  wall to servo = %.1f (x%.1f->%.1f)\n  splay horn (-X): disk O%.1fx%.1f X[%.1f,%.1f] enclosed (wall to servo %.1f);  spline O%.1fx%.1f X[%.1f,%.1f (face)] out\n  back wall %.1fmm (Y[%.1f,%.1f]);  all corners chamfered to >=%.1fmm of any feature; all exterior edges rounded\n  COXA solids=%d vol=%.0f\n"%(
        bbs(sb),bbs(BLK.BoundBox),FRONT,tab_ymin,sb.ZMin-SCLR,sb.ZMax+SCLR,WSW,xc-WSW/2,xc+WSW/2,ws_y0,FRONT,ws_z0,
        BRG_OD,BRG_D,SPY,SPZ,brg_x0,bx1,brg_x0-sb.XMax,sb.XMax,brg_x0,
        HRN_DISK,HRN_DT,hrn_disk_x0,hrn_back,sb.XMin-hrn_back,HRN_SPL,HRN_ST,hrn_face,hrn_disk_x0,
        BACK,by0,by1,CLR,len(COXA.Solids),COXA.Volume)
    open(OUT+"/coxablock.txt","w").write(rpt); print(rpt)

    # ---- STL export: the 3 printable coxa parts (canonical names; this set replaces dog13's coxa for printing) ----
    try:
        import MeshPart
        STLDIR=r"C:/ultrafish/robodog/stl"
        exp=[(COXA,"sm3sg90_coxa.stl")]
        if CLIP is not None: exp.append((CLIP,"sm3sg90_coxa_cover.stl"))
        if RING is not None: exp.append((RING,"sm3sg90_coxa_insert.stl"))
        for _sh,_fn in exp:
            _m=MeshPart.meshFromShape(Shape=_sh, LinearDeflection=0.08, AngularDeflection=0.35, Relative=False)
            _m.write(STLDIR+"/"+_fn); print("STL %-24s %6d facets"%(_fn,_m.CountFacets))
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
    if RING is not None: add("ring",RING,(.95,.80,.15),0)   # YELLOW femur insert (friction; thrust-washer ready) -> 3-tone
    add("servo",S1,(.30,.62,.47))          # the actual cutter mesh (real SG90), not the test-fit box
    BDUM=cyl(BRG_OD/2.0,5.0,v(brg_x0,SPY,SPZ),X).cut(cyl(4.0,5.0,v(brg_x0,SPY,SPZ),X))  # 688 dummy in the cup
    add("b688",BDUM,(.55,.60,.78))
    HDUM=cyl(HRN_DISK/2.0,HRN_DT,v(hrn_disk_x0,SPY,SPZ),X).fuse(cyl(HRN_SPL/2.0,HRN_ST,v(hrn_face,SPY,SPZ),X))  # horn dummy
    add("horn",HDUM,(.80,.52,.42))
    d.recompute()
    gv=Gui.activeDocument().activeView(); SZ=760; shots=[]
    for vn in ("Axonometric","Top","Front","Left"):
        getattr(gv,"view"+vn)(); Gui.SendMsgToActiveView("ViewFit")
        fp=OUT+"/_cx_%s.png"%vn; gv.saveImage(fp,SZ,SZ,"White"); shots.append(fp)
    ims=[Image.open(p).convert("RGB").resize((SZ,SZ)) for p in shots]
    sheet=Image.new("RGB",(SZ*2,SZ*2),(250,250,250)); dr=ImageDraw.Draw(sheet)
    labs=["iso","top (plan)","front (dog side, x-z)","femur side (+Y = insert face)"]
    for i,im in enumerate(ims):
        r,c=divmod(i,2); sheet.paste(im,(c*SZ,r*SZ))
        dr.rectangle([c*SZ+4,r*SZ+4,c*SZ+230,r*SZ+24],fill=(25,25,25)); dr.text((c*SZ+8,r*SZ+8),labs[i],fill=(255,255,255))
    sheet.save(OUT+"/coxablock_views.png")
    print("OK coxablock")
except Exception:
    print("FAIL: "+traceback.format_exc())
