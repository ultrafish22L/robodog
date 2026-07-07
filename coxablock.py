
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
    bz0,bz1 = sb.ZMin-WALL, sb.ZMax+WALL
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
    # === CORNERS: chamfer every outer block edge as DEEP as it can while keeping >=CLR to any feature (each cut
    #     wedge lies in a ball of radius d about the edge, so d < clearance => wall stays >=CLR: provably safe).
    #     Then ROUND with VARIABLE radii scaled to local clearance. Pockets stay sharp. ===
    CLR=2.0
    feats=[box(sb.XLength+2*CLR, sb.YLength+2*CLR, sb.ZLength+2*CLR, v(sb.XMin-CLR,sb.YMin-CLR,sb.ZMin-CLR)),  # servo
           box(WSW+2*CLR, (FRONT-ws_y0)+2*CLR, (bz1-ws_z0)+2*CLR, v(xc-WSW/2-CLR, ws_y0-CLR, ws_z0-CLR)),      # wire slot
           cyl(BRG_OD/2+CLR, BRG_D+2*CLR, v(brg_x0-CLR, SPY, SPZ), X),                                          # bearing cup
           cyl(HRN_DISK/2+CLR, HRN_DT+2*CLR, v(hrn_disk_x0-CLR, SPY, SPZ), X),                                  # horn disk
           cyl(HRN_SPL/2+CLR, HRN_ST+2*CLR, v(hrn_face-CLR, SPY, SPZ), X)]                                      # horn spline
    FEAT=feats[0]
    for ff in feats[1:]: FEAT=FEAT.fuse(ff)
    FEAT=FEAT.removeSplitter()
    def eclr(e):
        try: return e.distToShape(FEAT)[0]
        except Exception: return 0.0
    def _oplane(f,shape):
        srf=f.Surface
        if srf.TypeId!='Part::GeomPlane': return False
        ax=srf.Axis; bb=shape.BoundBox
        for i,nm in ((0,'X'),(1,'Y'),(2,'Z')):
            if abs(ax[i])>0.999:
                c=f.Vertexes[0].Point[i]
                return abs(c-getattr(bb,nm+'Min'))<0.06 or abs(c-getattr(bb,nm+'Max'))<0.06
        return False
    def blk_edges(shape):
        return [e for e in shape.Edges if len(shape.ancestorsOfType(e,Part.Face))==2
                and all(_oplane(f,shape) for f in shape.ancestorsOfType(e,Part.Face))]
    for lvl in range(11,1,-1):                 # deep first; chamfered edges leave blk_edges() so no double-cut
        grp=[e for e in blk_edges(COXA) if int(round(max(0.0,eclr(e)-0.3)))==lvl]
        if grp:
            try: COXA=COXA.makeChamfer(float(lvl),grp).removeSplitter()
            except Exception: pass
    # ROUND EVERY EXTERIOR EDGE (user: "all edges need rounding").
    # NOTE: the press-fit bore mouths (bearing cup + horn spline) are LEFT SHARP (user) -> clean seat/register;
    #       the crush ribs and the horn drop-in carry the lead-in, not a bore fillet. So the bore-lip pass is removed.
    # (b) all exterior planar-planar edges: clearance-scaled radius, 0.5 floor -> nothing sharp.
    #     Only truly on-feature edges (servo mesh pocket / bore lips, clearance==0) are left out.
    def rtgt(e):
        c=eclr(e)
        if c<0.05: return 0.0
        if c>=4.0: return 3.0
        if c>=2.5: return 2.0
        if c>=1.3: return 1.0
        return 0.5
    def on_front(e):                                       # +Y face edges stay SHARP -> flush clip seat (user)
        return all(abs(vt.Point.y-FRONT)<0.2 for vt in e.Vertexes)
    def fillet_bucket(shape,r):
        es=[e for e in shape.Edges
            if len(shape.ancestorsOfType(e,Part.Face))==2
            and all(f.Surface.TypeId=='Part::GeomPlane' for f in shape.ancestorsOfType(e,Part.Face))
            and not on_front(e)
            and abs(rtgt(e)-r)<1e-6]
        if not es: return shape
        try: return shape.makeFillet(r,es).removeSplitter()
        except Exception: pass
        for p in [e.CenterOfMass for e in es]:                 # fallback: one at a time, skip failures
            cand=[e for e in shape.Edges if e.CenterOfMass.distanceToPoint(p)<1e-4]
            if cand:
                try: shape=shape.makeFillet(r,[cand[0]]).removeSplitter()
                except Exception: pass
        return shape
    for rr in (3.0,2.0,1.0,0.5):
        COXA=fillet_bucket(COXA,rr)
    # ============ SNAP COVER (external pry-off clips) + cover-tab servo retention ============
    # Cover (black) closes the +Y mouth and seats the yellow femur insert (ROUNDED bullnose contact). It snaps on via
    # TWO EXTERNAL pry-off arms (inward barb into a blind detent); a bottom TAB reaches back onto the servo's mounting-
    # tab FRONT face and retains the servo -> servo drops in free, cover locks it (NO internal coxa ear-lips).
    CLIP=None; RING=None
    try:
        OX,OZ,HOLE=106.0,10.0,17.0          # servo output axis (X,Z, +Y); spline+butt clearance hole
        # --- FEMUR INSERT (separate yellow part): friction ring that seats in a recess in the (flat) cover.
        #     Bore Ø17; a concentric counterbore on its +Y face is the thrust-washer seat -> FRICTION now
        #     (femur rubs the raised rim), drop a small hobby thrust washer/bearing in the seat later. Swappable = tunable.
        REC_OD,REC_DEP,INS_PROUD,INS_FIT=24.0,2.0,1.5,0.10   # insert OD, seat depth in cover, proud of face, friction clearance
        TB_OD,TB_DEP=21.0,1.2                                 # thrust-washer seat (Ø17 bore x Ø21 x ~1.2): empty now, drop-in later
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
        CLIP=cover.cut(cyl(HOLE/2.0,face_thk+2.0,v(OX,FRONT-1.0,OZ),Y))
        WI=FACE_OUT-CWALL                                      # HOLLOW: keep front wall + bore rim + perimeter rim
        _hol=box(fb.XLength-4.0,WI-FRONT,fb.ZLength-4.0,v(fb.XMin+2.0,FRONT,fb.ZMin+2.0))
        _hol=_hol.cut(cyl(REC_OD/2.0+2.0,WI-FRONT+2.0,v(OX,FRONT-1.0,OZ),Y))                    # leave a SOLID disk to host the insert recess
        CLIP=CLIP.cut(_hol)
        # FLAT cover face + circular RECESS to seat the separate femur insert (the contact now lives on the insert)
        CLIP=CLIP.cut(cyl(REC_OD/2.0+INS_FIT,REC_DEP+0.01,v(OX,FACE_OUT-REC_DEP,OZ),Y))         # friction seat (open +Y)
        # build the INSERT (yellow): seat disk (OD REC_OD, seats REC_DEP into the cover) + a BULLNOSE contact ridge
        # (rounded x-section torus, proud of the face) = the rounded femur contact; Ø17 bore (thrust-washer-bore ready
        # -> a flat-seat variant of this swappable insert can take a small hobby thrust washer later).
        _yb=FACE_OUT-REC_DEP
        RCS=1.4; rc=HOLE/2.0+0.6+RCS                                     # ridge minor radius / centreline just outside the bore
        RING=cyl(REC_OD/2.0,REC_DEP,v(OX,_yb,OZ),Y)                      # seat disk, flush to the cover face
        _tor=Part.makeTorus(rc,RCS,v(OX,FACE_OUT,OZ),Y)                  # rounded contact ridge, proud of the face
        _tor=_tor.cut(box(2*rc+8,RCS+2.0,2*rc+8,v(OX-rc-4,FACE_OUT-RCS-2.0,OZ-rc-4)))   # keep the proud (+Y) half
        RING=RING.fuse(_tor).cut(cyl(HOLE/2.0,REC_DEP+RCS+2.0,v(OX,_yb-1.0,OZ),Y)).removeSplitter()   # + Ø17 through bore
        # SIMPLE HOOK CLIP (one flush arm per side, like before, with a hook on the end that snaps into the coxa):
        # a long flush arm runs back from the cover along the coxa top/bottom; at its free tip it bends 90 deg
        # DOWN into the coxa (the hook leg), and a ROUNDED NUB on that leg snaps into a coxa CAVITY. Push the
        # cover -Y: the arm flexes, the leg rides in and the nub clicks into the cavity; +Y pull is held by the
        # leg captured in its slot + the nub in the cavity. Flex the arm out to pop it back off.
        # ============ CANTILEVER SNAP: long FINGER -> 90-deg LEG into coxa -> rounded NUB on cover-facing face ==
        # The finger flexes; as the cover seats, the leg's nub cams past its coxa recess. An ANGLED RELIEF on the
        # coxa wall behind the leg (opposite the nub) gives the leg room to flex AWAY from the cover to clear it.
        AW   = 10.0        # hook width (X)
        ATIP = 14.0        # finger free tip / 90-deg corner (-Y end)
        FT   = 1.2         # finger thickness (Z) - thin so it FLEXES (like the earlier fingers)
        LEGY = 1.6         # leg thickness in Y (the 90-deg turn down into the coxa)
        NUBR = 0.8         # rounded snap nub radius, on the leg's cover-facing (+Y) face
        NUBZ = 2.8         # nub CENTRE depth on the leg
        LEGD = NUBZ+NUBR   # leg STOPS at the far side of the nub (does not extend beyond it)
        TOL  = 0.3         # snap-fit tolerance
        RELIEF = NUBR+TOL  # -Y relief clearance (away from the nub) = nub radius + tol; uniform, full leg depth
        RND  = 0.5         # profile rounding radius
        def _hbox(up,y0,ylen,d0,d1,gx=0.0,gy=0.0,gz=0.0):   # box d0..d1 deep into the coxa from the up/down face
            zface=bz1 if up>0 else bz0
            za,zb=zface-up*d0,zface-up*d1; zl,zh=sorted((za,zb))
            zl-=gz; zh+=gz
            if up>0: zh=min(zh,zface)
            else:    zl=max(zl,zface)
            return box(AW+2*gx, ylen+2*gy, zh-zl, v(OX-AW/2.0-gx, y0-gy, zl))
        def _hook(up,gx=0.0,gy=0.0,gz=0.0,rounded=False):
            zface=bz1 if up>0 else bz0
            finger=_hbox(up, ATIP, FACE_OUT-ATIP, 0.0, FT,   gx,gy,gz)  # long FINGER from the cover (flexes)
            leg   =_hbox(up, ATIP, LEGY,          0.0, LEGD, gx,gy,gz)  # 90-deg turn straight DOWN into the coxa
            yf=ATIP+LEGY                                                 # leg face that looks back toward the cover (+Y)
            nz=zface-up*NUBZ
            nub=cyl(NUBR+gz, AW+2*gx, v(OX-AW/2.0-gx, yf, nz), X)       # rounded NUB on that cover-facing face
            h=finger.fuse(leg).fuse(nub).removeSplitter()
            if rounded:
                try: h=h.makeFillet(RND,[e for e in h.Edges]).removeSplitter()
                except Exception:
                    try: h=h.makeFillet(RND*0.5,[e for e in h.Edges]).removeSplitter()
                    except Exception: pass
            return h
        for up in (1,-1):
            try: CLIP=CLIP.fuse(_hook(up,rounded=True))
            except Exception: print("clip hook %d skipped\n"%up+traceback.format_exc())
        # SERVO RETAINER TAB (cover, bottom): a rib on the cover inner face reaching back onto the servo bottom mounting-
        # tab FRONT face (earY) -> blocks +Y pull-out. The cover retains the servo now (internal coxa ear-lips removed).
        TABW,TABZ=12.0,3.0
        CLIP=CLIP.fuse(box(TABW,FACE_OUT-earY,TABZ,v(OX-TABW/2.0,earY,sb.ZMin)))
        CLIP=CLIP.removeSplitter()
        # coxa: grown-hook pocket + a uniform RELIEF on the -Y wall (opposite the nub), full leg depth and
        #       NUBR+TOL deep, so the leg can flex the whole nub clear of its recess when snapping together
        for up in (1,-1):
            COXA=COXA.cut(_hook(up,gx=0.35,gy=0.35,gz=0.35,rounded=False))
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
                if abs(c.z-OZ)<HOLE*0.7 and abs(c.x-OX)<HOLE*0.7: continue   # protect the femur ring / bore rim
                out.append(e)
            return out
        try:
            es=_cle(CLIP)
            if es:
                try: CLIP=CLIP.makeFillet(ROUND_CLIP,es).removeSplitter()
                except Exception:
                    for p in [e.CenterOfMass for e in es]:
                        cand=[e for e in CLIP.Edges if e.CenterOfMass.distanceToPoint(p)<1e-4]
                        if cand:
                            try: CLIP=CLIP.makeFillet(ROUND_CLIP,[cand[0]]).removeSplitter()
                            except Exception: pass
        except Exception: print("CLIP round skipped:\n"+traceback.format_exc())
        print("CLIP: flat cover Y[%.1f,%.1f] | SNAP: long finger + 90deg leg %.1fmm into coxa + rounded nub r%.1f on cover-facing face | servo tab back to Y%.1f"%(
            FRONT,FACE_OUT,LEGD,NUBR,earY))
        if RING is not None:
            print("RING insert: OD%.0f bore%.0f seat%.1f | ROUNDED bullnose contact ridge rc%.1f x R%.1f | solids=%d"%(
                REC_OD,HOLE,REC_DEP,rc,RCS,len(RING.Solids)))
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
