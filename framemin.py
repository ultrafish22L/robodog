# framemin.py -- v9 MINIMAL frame, 200mm body + two O12 splay-pin collars standing 3mm proud fore/aft = 206mm overall
#   X envelope (still << 256 bed; wheelbase = hip spacing is set by the coxa faces at +/-96.6, unaffected). Architecture:
#   4 CORNER MOUNT BLOCKS  = the proven frameblock drop-in cuts (coxa pocket grown for splay swing +
#                            servo0 top-down slot), wrapped in WALL-thick material -- servo-bound, unchanged.
#   2 WARREN-TRUSS RAILS   = full-height 3mm side rails between corner blocks, alternating triangle cutouts.
#   FLOOR PAN              = 2.4mm pan spanning the bay: electronics tray AND the torsion shear panel.
#   2 BULKHEADS            = low cross walls at bay ends w/ wire notches; zip-tie slots in the floor.
#   SERVO0 SLOT: 2mm walls all round bar the +Z insertion end (user) -- BY grown so the outboard wall is a full
#   2mm (was a 0.65mm sliver); the belly + closeout windows are removed so the floor and inboard wall are solid.
#   LIGHTENING: corner-block cavities (top-bridged), floor 2.0 w/ 6 side windows, warren truss.
#   45deg chamfer along the FULL top edges; "ROBODOG v9" engraved in the belly. Prints floor-down, fits 256 bed.
# Wheelbase note: pockets shift inboard so frame length = 200; dog13's DX must follow when the assembly moves to v9.
import FreeCAD as App, FreeCADGui as Gui, Part, MeshPart, math, traceback
v=App.Vector; X,Y,Z=v(1,0,0),v(0,1,0),v(0,0,1)
if 'SH' not in globals() or 'servo0' not in globals():   # warm-session/composition reuse (dog14 execs us after dog13)
    FAST=True
    exec(open(r"C:/ultrafish/robodog/dog13.py").read())  # -> SH(coxa), servo0, tf, rot, box, S0DZ, DX, YSHIFT ...
LOG=[]
def log(s): LOG.append(str(s))
def plate(pts,y0,yl):
    w=Part.makePolygon([v(x,y0,z) for (x,z) in pts]+[v(pts[0][0],y0,pts[0][1])])
    return Part.Face(w).extrude(v(0,yl,0))
def bb6(s): b=s.BoundBox; return [b.XMin,b.XMax,b.YMin,b.YMax,b.ZMin,b.ZMax]
def sweep6(solid,amin,amax,n=25):
    xs=[];ys=[];zs=[]
    for i in range(n):
        a=amin+(amax-amin)*i/(n-1); b=rot(solid,a,SP,X).BoundBox
        xs+=[b.XMin,b.XMax];ys+=[b.YMin,b.YMax];zs+=[b.ZMin,b.ZMax]
    return [min(xs),max(xs),min(ys),max(ys),min(zs),max(zs)]
try:
    TARGET=200.0; WALL=2.4; RAILT=3.0; FLOORT=1.6; BHH=28.0   # FLOORT 2.0->1.6: floor-top pinned, belly rises 0.4mm (+toe-in), closed shear panel
    SKIN=1.2; KWEB=1.4                                        # tray closure: 1.2mm inner wall skin (truss->ribs-on-skin) + 1.4mm keel-gap web
    PCLR=1.0; SCLR=0.1; OUTPAST=16.0; SPMIN,SPMAX=-25.0,105.0; CH=1.5   # CH 2.0->1.5: wider flat snap-rim landing
    SVWALL=2.0                                  # min wall around the servo0 body (user: 2mm all round bar insertion)
    # Coxa pocket must be grown from the part that is ACTUALLY printed = coxablock's R16 COXA, not dog13's
    # shoulder() (audit 2026-07-27: the R16 barrel is 43mm tall / bigger X, it punched 6.4mm through the old
    # SH-sized pocket floor). Put COXA into the same yout'd datum SH carries (dog13 does SH=yout(SH)); then the
    # bbox/sweep set pz0/px1/py0/pz1 for the real envelope. Falls back to SH only if coxablock wasn't exec'd.
    if 'COXA' in globals():
        CX=COXA.copy(); CX.translate(v(-DX,YSHIFT,0)); log("coxa pocket <- coxablock R16 COXA (yout'd)")
    else:
        CX=SH; log("coxa pocket <- dog13 SH  (WARN: not the R16 coxa -- exec coxablock first)")
    TOPZ=servo0.BoundBox.ZMax                              # corner-block/servo0 top (coxa top pokes above, nothing there)
    cs=bb6(CX); ce=sweep6(CX,SPMIN,SPMAX)
    # servo0 tracks the coxa horn face: thickening the servo long-face walls (coxablock WSRV 2.0->3.5) moved hrn_face
    # outboard (-X), so servo0 + its pocket + the corner block follow by DHORN to keep the splay spline engaged and
    # off the coxa body (user 2026-07-29). Parametric on hrn_face; 92.95 = the WSRV=2.0 baseline horn face.
    DHORN = (92.95 - hrn_face) if 'hrn_face' in globals() else 0.0
    TAB_BACK=servo0.common(box(200,80,4.4,v(-100,-40,18.3))).BoundBox.XMin - DHORN
    # servo0 FORM-FIT POCKET (user 2026-07-27): cut the REAL splay-servo mesh (servo_cut grows 0.25/side) so the
    # body + tabs are captured FORM-FIT, not a loose rectangular box. The servo slides in from the COXA (+X) side
    # (its output already points +X into the coxa pocket) and the COXA then caps the +X mouth = retention. The tabs
    # (top-Z) seat against the mesh tab cavity's -X faces. No more +Z drop-in opening. sx0..sz1 = NATIVE mesh bbox
    # (XS added at cut time, like every other corner feature, so BY/cbx0/trims math is unchanged).
    # servo_cut('s0') carries YSHIFT (post-yshift Y) but NOT dog13's -DX -> add -DX so the pocket lands in the same
    # yout'd datum as servo0 (else the form-fit sits 6mm off and clips the servo: servo0^frame was 700mm3).
    S0=servo_cut('s0'); S0.translate(v(-DX-DHORN,0,0)); s0b=S0.BoundBox   # -DHORN: servo0 pocket follows the horn face outboard
    sx0,sx1,sy0,sy1,sz0,sz1=s0b.XMin,s0b.XMax,s0b.YMin,s0b.YMax,s0b.ZMin,s0b.ZMax
    # BY sized so the servo0 pocket keeps a full SVWALL outboard wall (was 68 -> 0.65mm sliver, unprintable).
    BY=max(68.0, 2.0*(sy1+SVWALL))
    # ---- drop-in cuts, verbatim frameblock math (build corner +X+Y) ----
    px0,px1=TAB_BACK,cs[1]+PCLR
    py0=ce[2]-PCLR; py1=BY/2+OUTPAST
    pz0=ce[4]-PCLR; pz1=TOPZ+0.1
    # ---- wheelbase shrink: put the pocket's outer wall at TARGET/2 ----
    XS=(TARGET/2.0)-WALL-px1
    COXP=box(px1-px0,py1-py0,pz1-pz0,v(px0+XS,py0,pz0))
    # FULL BODY-ONLY POCKET, clean BREP boxes (user 2026-07-27; NOT the raw scanned mesh -- a mesh boolean tessellates
    # non-watertight/unrepairable, fatal for a printed part). Fits JUST the servo BODY (X-profile measured off
    # servo_cut('s0'): body block Y13 x Z[-7,18.3] over 24mm, top+bottom flange EARS at X[+15,+21], +X output boss/spline).
    # The body cavity + two ear channels open on the +X (coxa) side (the coxa caps the +X mouth = retention) and the
    # ear channels' -X walls are the FLATS the tabs seat against. NOTE (audit 2026-07-28): the TOP ear channel also
    # runs up to sz1 (>= TOPZ), i.e. it is intentionally OPEN through the frame top over the ~8mm ear X-zone forward of
    # the coxa -- that clears the SG90 top mounting tab and eases insertion; the servo BODY roof (X[sx0..EX0]) stays
    # solid for retention. So the pocket is closed on -X/+/-Y/-Z, open on +X, and open-top only over the ear strip.
    assert abs((sx1-sx0)-30.4)<1.5 and abs((sz1-sz0)-33.0)<1.5, "servo0 mesh changed -- re-measure the body/ear profile"
    BX1=sx0+24.0; EX0=sx0+15.0                     # body/output split; flange-ear -X flat (tab seat)
    TZ0=sz1-2.83; BZ1=sz0+4.83                     # top-ear underside / bottom-ear topside = body top/bottom
    # guard the ear-split offsets too (the bbox assert above is blind to a shifted tab): they must stay ordered
    # inside the mesh Z-band, else the body/ear channels mis-place on a re-scanned mesh.
    assert sz0 < BZ1 < TZ0 < sz1, "servo0 ear-split Z offsets fell outside the mesh Z-band -- re-measure the tabs"
    CL0=0.3                                         # extra body clearance INBOARD (beyond the 0.25/side mesh grow) for SG90 clone spread; outboard wall untouched
    mouth=px1                                      # coxa-side opening (native x; +XS added below)
    SLOT=box(mouth-sx0, (sy1-sy0)+CL0, TZ0-BZ1, v(sx0+XS, sy0-CL0, BZ1))          # body cavity (+0.3 inboard), open +X
    SLOT=SLOT.fuse(box(mouth-EX0, sy1-sy0, (sz1+0.1)-TZ0, v(EX0+XS, sy0, TZ0)))     # TOP ear channel, -X flat @ EX0
    SLOT=SLOT.fuse(box(mouth-EX0, sy1-sy0, BZ1-(sz0-0.1), v(EX0+XS, sy0, sz0-0.1))) # BOTTOM ear channel, -X flat @ EX0
    SLOT=SLOT.removeSplitter()
    zbot=min(pz0,sz0)-FLOORT
    log("XSHIFT=%.2f  pocket X[%.1f,%.1f] Z[%.1f,%.1f]  slot X[%.1f,%.1f] zbot=%.1f TOPZ=%.1f"%(
        XS,px0+XS,px1+XS,pz0,pz1,sx0+XS,sx1+XS,zbot,TOPZ))
    log("  Y: pocket in=%.2f  slot [%.2f,%.2f]  rail inner=%.1f"%(py0,sy0,sy1,BY/2-RAILT))
    # ---- corner mount block ----
    cbx0=min(px0,sx0)+XS-WALL; cbx1=TARGET/2.0
    cby0=min(py0,sy0)-WALL; cby1=BY/2.0
    CB=box(cbx1-cbx0,cby1-cby0,TOPZ-zbot,v(cbx0,cby0,zbot))
    # ---- SPLAY-PIN SEAT (audit 2026-07-27): the coxa's +X 688 bearing (O15.94 cup / O8 bore) had nothing to
    #      ride on -- the coxa cantilevered off servo0's spline alone. Carry an 8mm STEEL DOWEL through the outer
    #      wall, coaxial with the splay axis; it rides the 688 and its tip lands in the coxa's pin relief. This is a
    #      SINGLE-SHEAR overhung pin (NOT double-shear: the 688 moves WITH the coxa = it is the load, not a 2nd frame
    #      grip) -- fine here, splay bearing stress is only ~2-5MPa. A 3mm collar proud of the outer face + the wall
    #      give ~5.4mm of dowel grip. Inserted from +X AFTER the coxa seats. Built in the master corner; tf -> all 4.
    #      Axial retention = press-fit + threadlock; the bore runs through the collar for +X insertion (add an E-clip
    #      groove or set-screw if it backs out). Dowel is a bought part (like the 688/servos) -- frame provides the seat.
    # DERIVE the axis from coxablock's MEASURED values when it ran (dog14 / standalone-with-coxablock), so a coxa move
    # can't silently mis-place the seat; literals are only a fallback for a dog13-only framemin run. (Hardcoding is
    # exactly how the tibia knee bore drifted onto a dropped head -- audit lesson.)
    if all(k in globals() for k in ('SPY','SPZ','brg_x0')):
        SPYp=SPY+YSHIFT; SPZp=SPZ; PINTIP=(brg_x0-DX+XS)-1.0     # placed y,z; tip 1mm past the 688 inboard face into relief
    else:
        SPYp,SPZp,PINTIP = 27.2, 10.398, 90.3                    # fallback (coxablock not exec'd)
    PINBORE=7.9; COLR=8.0; COLL=3.0                             # O8 steel dowel; O16 collar; 3mm proud
    # O16 (was O12): the dowel is the coxa's ONLY radial support (else it hangs off servo0's compliant spline), so the
    # bore wall must not be a thin cross-layer lip -- O16 gives (16-7.9)/2=4.05mm wall (was 2.05mm). Audit 2026-07-27.
    CB=CB.fuse(cyl(COLR, COLL, v(cbx1, SPYp, SPZp), X))         # O16 collar, 3mm proud of the outer wall (dowel grip + wall)
    CB=CB.cut(cyl(PINBORE/2.0, (cbx1+COLL+0.1)-PINTIP, v(PINTIP, SPYp, SPZp), X))   # O7.9 dowel bore, relief-tip -> through collar
    CB=CB.cut(Part.makeCone(PINBORE/2.0+0.9, PINBORE/2.0, 1.0, v(cbx1+COLL, SPYp, SPZp), v(-1,0,0)))  # +X mouth lead-in chamfer (eases the one-shot dowel insert; audit)
    # RETENTION (user 2026-07-28): COARSE INTERNAL THREAD in the bore so a threaded/self-tapping O8 pin SCREWS in and
    # self-retains -- the horizontal bore's 0.05mm press is unreliable (audit). 2mm pitch x ~0.7 deep = printable +
    # grippy. The O3 set-screw below stays as a secondary lock.
    try:
        THP,THD=2.0,0.7; th0=PINTIP; thl=(cbx1+COLL)-PINTIP
        _hx=Part.makeHelix(THP, thl+THP, PINBORE/2.0)                     # helix along +Z at the bore radius
        _pr=Part.Wire(Part.makePolygon([v(PINBORE/2.0,0,-THP*0.42),v(PINBORE/2.0+THD,0,0),
                                        v(PINBORE/2.0,0,THP*0.42),v(PINBORE/2.0,0,-THP*0.42)]))
        _thr=_hx.makePipeShell([_pr],True,False); _thr.rotate(v(0,0,0),Y,90.0); _thr.translate(v(th0,SPYp,SPZp))
        CB=CB.cut(_thr); log("  splay bore: coarse thread P%.1f d%.1f over %.1fmm (screw-in retention)"%(THP,THD,thl))
    except Exception as _te:
        log("  splay bore thread SKIPPED: %s"%_te)
    CB=CB.cut(cyl(1.5, (TOPZ+1.0)-SPZp, v(cbx1-1.5, SPYp, SPZp), Z))    # O3 set-screw (secondary lock) into the bore, exiting the FLAT block top
    CB=CB.removeSplitter()
    # NOTE (audit): bore axis is X, so floor-down it prints as a HORIZONTAL hole -> crown bridges/sags oversize and the
    # O8/O7.9 0.05mm press is unreliable. Intended assembly = ream/drill the bore to size + retain the dowel with a
    # transverse set-screw or E-clip; press-fit alone is not trustworthy at this diameter/orientation.
    log("  splay-pin seat: O%.1f dowel bore on axis y=%.2f z=%.2f, tip x=%.2f, O%.0f collar to x=%.1f (frame X now +/-%.0f)"%(
        PINBORE,SPYp,SPZp,PINTIP,2*COLR,cbx1+COLL,cbx1+COLL))
    FR=tf(CB,1,1).fuse(tf(CB,1,-1)).fuse(tf(CB,-1,1)).fuse(tf(CB,-1,-1))
    # ---- rails (overlap 3mm into blocks -- coplanar-touching won't fuse) ----
    rx=cbx0+3.0
    RAIL=box(2*rx,RAILT,TOPZ-zbot,v(-rx,BY/2-RAILT,zbot))
    FR=FR.fuse(RAIL).fuse(tf(RAIL,1,-1))
    # ---- floor pan (tray + torsion shear panel) ----
    FLOOR=box(2*rx,BY-2.0,FLOORT,v(-rx,-(BY/2-1.0),zbot))
    FR=FR.fuse(FLOOR)
    # ---- battery corral / floor stiffeners (retention + shear stiffening; inside the solid-floor zone |Y|<23.7) ----
    ft=zbot+FLOORT
    for sy in (-1,1):
        FR=FR.fuse(box(80.0,1.2,4.0,v(-40.0, sy*14.0-0.6, ft)))          # 2 longitudinal battery rails at Y=+/-14
    FR=FR.fuse(box(1.2,28.0,4.0,v(29.4,-14.0,ft)))                        # fore battery end-stop at X=+30
    # ---- bulkheads w/ wire notch ----
    bx=cbx0+2.0
    BH=box(2.4,BY-2.0,BHH,v(bx-2.4,-(BY/2-1.0),zbot))
    BH=BH.cut(box(2.6,16.0,9.0,v(bx-2.5,-8.0,zbot+FLOORT+3.0)))
    FR=FR.fuse(BH).fuse(tf(BH,-1,1))
    FR=FR.removeSplitter()
    # ---- corner-block lightening (built in the +X+Y corner like the drop-in cuts, all guarded by live dims:
    #      every surplus region = material further than WALL from any functional face gets hollowed) ----
    trims=[]
    cavy0=sy1+WALL; cavy1=BY/2-RAILT          # between slot outboard wall and rail (roof bridged 2.5)
    if cavy1-cavy0>0.6:
        trims.append(("slot-rail",box(sx1-sx0,cavy1-cavy0,(TOPZ-2.0)-(zbot+FLOORT),v(sx0+XS,cavy0,zbot+FLOORT))))
    if sy0-WALL-(cby0+WALL)>0.6:              # inboard-Y surplus beside the slot (keep 2.4 hugging each face)
        trims.append(("slot-in",box(sx1-sx0,(sy0-WALL)-(cby0+WALL),(TOPZ-2.0)-(zbot+FLOORT),
                                    v(sx0+XS,cby0+WALL,zbot+FLOORT))))
    if py0-WALL-(cby0+WALL)>0.6:              # inboard-Y surplus beside the pocket (X beyond the slot)
        trims.append(("pock-in",box(px1-max(px0,sx1),(py0-WALL)-(cby0+WALL),(TOPZ-2.0)-(zbot+FLOORT),
                                    v(max(px0,sx1)+XS,cby0+WALL,zbot+FLOORT))))
    # (removed seat-win + closeout-win: user wants 2mm walls all round the servo0 bar the +Z insertion end,
    #  so the floor under the servo and the inboard closeout wall are now solid. The lightening cavities
    #  above stay -- they hollow surplus BEYOND the 2mm servo walls, not the walls themselves.)
    # R18 (user 2026-07-29 "lighten till no more"): under-servo-body column -- solid meat between the servo0 body
    # underside and the floor pan (a side effect of the solid-floor-under-servo choice). Vented -Y (to Y18.3) so it
    # merges with the slot-in hollow below Z-9.5 -> no enclosed cavity. Clears COXP by ~3mm; leg gates stay 0.
    trims.append(("under-servo", box(12.0,13.7,10.6, v(sx0+XS+0.6, 18.3, -20.1))))
    log("  corner trims: "+", ".join("%s %.0fmm3"%(n,t.Volume) for n,t in trims))
    # ---- the drop-in cuts at all 4 corners ----
    for sxc,syc in ((1,1),(1,-1),(-1,1),(-1,-1)):
        FR=FR.cut(tf(COXP,sxc,syc)).cut(tf(SLOT,sxc,syc))
        for _,t in trims: FR=FR.cut(tf(t,sxc,syc))
    # ---- side walls: OPEN TRUSS (user 2026-07-29 "add back cutouts to reduce weight"; the snap-on body will
    #      enclose, so the frame goes back to an open skeleton -- reverses the interim closed-skin tray). THROUGH
    #      windows between 4 vertical rib-posts per side; the solid rim cap (z>tz1, snap-body land) and the keel
    #      band (z<tz0, fed by the keel web) stay as continuous top/bottom rails tying the posts + corner blocks. ----
    span=rx-6.0
    ZTOP,DEEP=-11.0,12.0                                   # the ONE belly chamfer: top Z / inward reach (= toe-in size)
    tz0,tz1=ZTOP-2.0,TOPZ-6.0                              # -13 .. 14.7 : wall band between keel and rim cap
    RIB_W=5.0; CR_IN=3.0                                  # unified rib width (upper posts == lower feet, both centred on rib X) + cross-rib inward lip
    _ribs=(-24.6,-8.2,8.2,24.6); _rw=RIB_W                # 4 vertical rib-posts/side; windows between them
    _edg=[-span]+[x for xc in _ribs for x in (xc-_rw/2.0,xc+_rw/2.0)]+[span]
    for _i in range(0,len(_edg),2):
        _w0,_w1=_edg[_i],_edg[_i+1]
        if _w1-_w0<1.0: continue
        _win=box(_w1-_w0, RAILT+0.6, tz1-tz0, v(_w0, BY/2-RAILT-0.3, tz0))    # full wall thickness -> through window
        FR=FR.cut(_win).cut(tf(_win,1,-1))
    # ---- R18 rim-cap rail-thin (user "till no more"): shave the inboard 1.0mm of the 3.0mm rail over the rim-cap
    #      band (open-bay side, nothing functional there); keeps a continuous 2.0mm outboard land + its top chamfer.
    _rc=box(81.15,1.0,6.0,v(-40.57,32.70,14.70)); FR=FR.cut(_rc).cut(tf(_rc,1,-1))
    # ---- 45deg chamfer along the FULL top outer edges (rails + corner blocks, one continuous line) ----
    chb=box(TARGET+2,CH*2.0,CH*2.0,v(-(TARGET/2+1),-CH,-CH)); chb.rotate(v(0,0,0),X,45.0)
    chb.translate(v(0,BY/2,TOPZ))
    FR=FR.cut(chb).cut(tf(chb,1,-1))
    # ---- SNAP-BODY RETENTION BEAD (user 2026-07-29 "snaps onto frame top"): the one-piece cosmetic canopy
    #      (part_body.py) clips on via 6 leaf-spring fingers that hook UNDER this bead. Continuous triangular bead on
    #      the rim-cap OUTER face (Y=BY/2), x[-75,75] both rails -- ends >=18mm inboard of the coxa (x~100) so the leg
    #      gates are untouched. Cross-section (Y,Z): tip 1.0mm proud at (36.7,16.2); 20deg down-facing undershelf up
    #      to the wall (35.7,16.55); 61deg top lead-in ramp up to (35.7,18.0); stays >=0.7mm below the 45deg register
    #      chamfer (z>=18.7). A 0.4mm relief groove under the undershelf so floor-down bridging droop can't fill the catch.
    _by=BY/2.0
    _bead=Part.Face(Part.makePolygon([v(-75,_by-0.5,16.55),v(-75,_by+1.0,16.2),v(-75,_by-0.5,18.0),v(-75,_by-0.5,16.55)])).extrude(v(150,0,0))
    FR=FR.fuse(_bead).fuse(tf(_bead,1,-1))
    _rel=box(150,1.2,0.4,v(-75,_by,15.75)); FR=FR.cut(_rel).cut(tf(_rel,1,-1))   # anti-droop relief under the undershelf
    FR=FR.removeSplitter()
    # ---- ONE continuous bottom-outer chamfer, FULL length head->rump (user 2026-07-28: "match the chamfer on the head
    #      and rump on the belly, so it's just one larger chamfer"). Replaces the old two-tier belly (shallow mid-span
    #      bevel + deep hip toe-in ramp with a step between). A single 45deg chamfer from the wall at (BY/2, ZTOP) down-
    #      and-in to (BY/2-DEEP, zbot) -- SAME size as the old hip toe-in ramp, so a toed-in leg still clears at the hips
    #      AND the whole belly now reads as one designed keel edge. The truss base (tz0) sits just above ZTOP so this
    #      never touches the mid-span truss; the floor pan stays a continuous (narrowed) bottom = closed tray / hull look.
    # ---- KEEL CLOSURE WEB: the belly chamfer left a full-length OPEN gap between the floor edge and the rail (they
    #      met only at the 4 corner blocks). A KWEB skin laid just inboard of the chamfer plane ties floor->rail the
    #      whole length. Fused BEFORE the chamfer cut so the cut trims its proud outer face flush -> zero exterior /
    #      toe-in change, net ~KWEB closed skin. ----
    def _Yc(z): return BY/2.0 - DEEP*(ZTOP - z)/(ZTOP - zbot)     # chamfer-line Y at height z
    _zt=tz0; _eps=0.5
    _kw=[(_Yc(_zt)+_eps,_zt),(_Yc(zbot)+_eps,zbot),(_Yc(zbot)+_eps-KWEB,zbot),(_Yc(_zt)+_eps-KWEB,_zt)]
    _kwx=cbx0+0.5                                                 # run the web PAST the corner-block inner face (cbx0) so the 4 belly-corner keel slots close
    _kwe=Part.Face(Part.makePolygon([v(-_kwx,y,z) for (y,z) in _kw]+[v(-_kwx,_kw[0][0],_kw[0][1])])).extrude(v(2*_kwx,0,0))
    # ---- side rib-post FEET (user 2026-07-29 "tray side ribs need better attachment to the bottom tray"): the belly
    #      chamfer trims the keel band to a ~0.75mm sliver by z=-13.75, so each mid-span rib-post otherwise reaches the
    #      floor only through the 1.4mm keel web. A trapezoidal buttress per rib -- outer face PROUD of the chamfer
    #      plane (like the keel web, so the chamfer cut below trims it flush -> no protrusion, no sliver), run through
    #      the full floor thickness at the base (overlaps the pan solidly), inboard reach for bending stiffness --
    #      plants each post on the tray. Fused with the keel web BEFORE the chamfer; mid-span only (|X|<=27.6 < cbx0). ----
    _gw,_gtop,_gin=RIB_W,5.0,7.0
    for _xc in _ribs:
        _gp=[(_Yc(tz0)+_eps,tz0),(_Yc(zbot)+_eps,zbot),(_Yc(zbot)+_eps-_gin,zbot),(_Yc(tz0)+_eps-_gtop,tz0)]
        _gf=Part.Face(Part.makePolygon([v(_xc-_gw/2.0,y,z) for (y,z) in _gp]+[v(_xc-_gw/2.0,_gp[0][0],_gp[0][1])])).extrude(v(_gw,0,0))
        FR=FR.fuse(_gf).fuse(tf(_gf,1,-1))
    # SUBSTANTIAL front-to-back cross rib at the upper-post/lower-foot junction (user 2026-07-29: "make the cross rib
    # they meet at more substantial -- that joint is still very fragile"). A solid longitudinal beam corner-to-corner
    # with an inward lip (CR_IN) for beam depth, at z[ZTOP,-6] (sits ABOVE the belly chamfer -> no protrusion). It
    # BRIDGES the side windows so all 4 posts + their feet are tied into one continuous member, instead of isolated
    # thin posts landing on the chamfered keel sliver. Fused before the chamfer.
    _cr=box(2*(cbx0+2.0), RAILT+CR_IN, -6.0-ZTOP, v(-(cbx0+2.0), BY/2-RAILT-CR_IN, ZTOP))
    FR=FR.fuse(_cr).fuse(tf(_cr,1,-1))
    FR=FR.removeSplitter()
    xlo,xhi=-(TARGET/2.0+3.0), TARGET/2.0+3.0
    _poly=[(BY/2.0,ZTOP),(BY/2.0+7.0,ZTOP),(BY/2.0+7.0,zbot-3.0),(BY/2.0-DEEP,zbot-3.0),(BY/2.0-DEEP,zbot)]
    _pts=[v(xlo,y,z) for (y,z) in _poly]
    _ch=Part.Face(Part.makePolygon(_pts+[_pts[0]])).extrude(v(xhi-xlo,0,0))
    FR=FR.cut(_ch).cut(tf(_ch,1,-1))
    # ---- zip-tie slots through the floor (boards fore, battery aft) ----
    for xc in (-30.0,0.0,30.0):
        for yc in (-16.0,16.0):
            FR=FR.cut(box(3.0,8.0,FLOORT+2.0,v(xc-1.5,yc-4.0,zbot-1.0)))
    # ---- floor lightening (open-frame, user 2026-07-29): windows THROUGH the floor pan center band, avoiding the
    #      zip-tie ribs (x=+/-30,0) and the battery-corral ribs (Y=+/-14). Electronics ride the rib grid + straps +
    #      (later) the snap-on body -- the closed shear panel is no longer needed once the body encloses. ----
    for _fx0,_fx1 in ((-54.0,-34.0),(-26.0,-4.0),(4.0,26.0),(34.0,54.0)):    # 4 X-bands between the zip-tie ribs
        FR=FR.cut(box(_fx1-_fx0, 24.0, FLOORT+2.0, v(_fx0, -12.0, zbot-1.0)))   # center band Y[-12,12], clears corral
    # R18 outboard floor bands (user "till no more"): outboard of the corral ribs (Y+/-14), inboard of the belly
    # chamfer edge; X kept mid-span (corner blocks start at |X|~41).
    for _fx0,_fx1 in ((-39.0,-34.0),(-26.0,-4.0),(4.0,26.0),(34.0,39.0)):
        for _fy0,_fy1 in ((16.6,22.0),(-22.0,-16.6)):
            FR=FR.cut(box(_fx1-_fx0, _fy1-_fy0, FLOORT+2.0, v(_fx0,_fy0,zbot-1.0)))
    # ---- belly engraving (0.6 deep into the print's bed face; mirrored to read from below) ----
    try:
        ws=Part.makeWireString(u"ROBODOG v9",r"C:\Windows\Fonts\arialbd.ttf",9.0,0.0)
        txt=Part.makeCompound([Part.Face(ch).extrude(v(0,0,0.7)) for ch in ws if ch])
        mm=App.Matrix(); mm.A22=-1.0; txt=txt.transformGeometry(mm)     # mirror Y -> reads correctly from -Z
        tb=txt.BoundBox; txt.translate(v(-tb.Center.x,-tb.Center.y,zbot-0.1-tb.ZMin))
        FR=FR.cut(txt); log("  belly text engraved (%.0fx%.0fmm)"%(tb.XLength,tb.YLength))
    except Exception as e:
        log("  belly text SKIPPED: %s"%e)
    FR=FR.removeSplitter()
    fb=FR.BoundBox
    log("FRAME solids=%d valid=%s vol=%.0fmm3 (%.0fg PLA)  L=%.1f W=%.1f H=%.1f"%(
        len(FR.Solids),FR.isValid(),FR.Volume,FR.Volume/1000.0*1.24,fb.XLength,fb.YLength,fb.ZLength))
    log("  bay: X +/-%.1f  Y +/-%.1f  depth %.1f (open top)"%(bx-2.4,BY/2-RAILT,TOPZ-zbot-FLOORT))
    assert len(FR.Solids)==1, "frame split into %d solids"%len(FR.Solids)
    assert fb.XLength<=256 and fb.YLength<=256, "exceeds bed"
    # ---- export + render ----
    m=MeshPart.meshFromShape(Shape=FR,LinearDeflection=0.08,AngularDeflection=0.35,Relative=False)
    m.write(r"C:/ultrafish/robodog/stl/sm3sg90_v9_frame.stl")
    log("  STL sm3sg90_v9_frame facets=%d"%m.CountFacets)
    if App.GuiUp:
        nm="framemin"
        if nm in list(App.listDocuments()): App.closeDocument(nm)
        d=App.newDocument(nm)
        o=d.addObject("Part::Feature","frame"); o.Shape=FR
        try: o.ViewObject.ShapeColor=(0.55,0.57,0.60)
        except Exception: pass
        d.recompute(); Gui.activeDocument().activeView().viewAxonometric(); Gui.SendMsgToActiveView("ViewFit")
    else:
        log("  (headless: geometry+STL only, render when GUI is back)")
    print("OK framemin\n"+"\n".join(LOG))
except Exception:
    print("FAIL:\n"+traceback.format_exc())
