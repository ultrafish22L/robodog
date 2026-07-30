# part_frame.py -- split out of dog13.py (2026-07-19), VERBATIM.
# frame(): the chassis tub (subtractive-from-block), servo0 pockets, splay pillars.
# Loaded by dog13.py via dinc(); execs into the CALLER's globals so the long-standing
# `exec(open('dog13.py').read())` contract (bodyview/export/frameview/coxablock/frameblock)
# keeps seeing every name flat, exactly as before.

def frame():
    W=52.0+2*YSHIFT; Y0=-26.0-YSHIFT               # widened central body (60 -> 60+2*YSHIFT ~ 67.2 mm)
    TRAYZ=-30.0                                    # electronics tray floor (v21 was -40; raised to -30 for a slimmer SM3-ish body): internal cavity ~34 mm, frame 50 mm
    def dxin(s): s=s.copy(); s.translate(v(-DX,0,0)); return s   # bed-fit: pull each corner feature inboard by DX (matches -DX leg shift); central plates/rails stay put
    PL=185-2*DX; PX=-(92.5-DX)   # bed-fit: shorten plates/rails 2*DX so the inboard-shifted coxa/servo1 clear the deck ends (their inboard edge moved to x~88; deck now ends x~86.5)
    SIDE=34.25; TOPZ=18.0; FLR=TRAYZ           # v28: flat outer side (1mm past servo0 y33.25), tub rim, floor
    # v28 ELECTRONICS TUB: a proper open-top tub filling the space between the hips - 2mm floor + 2mm long side walls
    # (outer face = the flat frame side at y=+-SIDE) + the hip bulkheads/cross-rib as short walls; open top for the
    # cover with a 1cm cross-strip mid-span for rigidity. servo0 pockets carve into the corners (leaving a 1mm outer cover).
    fr=box(PL,2*SIDE,2,v(PX,-SIDE,FLR))                                                       # 2mm tub floor
    fr=fr.fuse(box(PL,2,TOPZ-FLR,v(PX,SIDE-2,FLR))).fuse(box(PL,2,TOPZ-FLR,v(PX,-SIDE,FLR)))  # 2mm long side walls (floor->rim)
    fr=fr.fuse(box(10,2*SIDE,2,v(-5,-SIDE,TOPZ-2)))                                            # 1cm top cross-strip (rigidity)
    for sx in (1,-1): fr=fr.fuse(tf(dxin(box(3,2*SIDE,TOPZ-FLR,v(82.4,-SIDE,FLR))),sx,1))     # front/back tub end walls moved IN to the servo0 flange plane (build x82.4-85.4 -> world x76.4-79.4) so the tabs sit against them
    for sx in (1,-1): fr=fr.fuse(tf(dxin(box(6,40,TOPZ-FLR,v(83.5,-20,FLR))),sx,1))            # x83.5 cross-rib, CENTRAL only (y+-20) so it doesn't run through the servo0 corners
    # v23 L/R shoulder girdle: top + bottom crossbars tie the left+right bearing pillars (at the nose & tail) into one
    # rigid axle -> kills the fore-aft fold + gives the O8 pin a 2nd shear support. y+-28 stays under the cover hip openings.
    for sx in (1,-1): fr=fr.fuse(tf(dxin(box(8,56,3.5,v(120.5,-28,16.5))),sx,1))            # top bar ties pillar tops (z20)
    for sx in (1,-1): fr=fr.fuse(tf(dxin(box(8,56,3.5,v(120.5,-28,-16))),sx,1))             # bottom bar ties bridge/pillar bottoms
    def ys(y): return y+YSHIFT                     # hip-mount features shift outboard with the hip
    # v24 FLAT FRAME SIDE: every outboard hip feature pulled to y<=ys(26)=33.2 (= central-plate edge = servo0 outer wall),
    # so the frame side is ONE plane. servo0's outer face is now flush with that side (the bulging outer well wall + outer
    # thin wall are removed); servo0 retention = M2 tab screws + inner wall + seat + closed top. Coxa/femur stay outboard (moving parts).
    # NB (2026-07-19): features dimensioned off servo0's BODY carry +S0DZ so they follow the pocket down;
    # oversized structural envelopes (bridge, pillar, gussets, fill block) still contain it, so they stay put.
    adds=[box(39,16,7,v(89.5,ys(10),-18)),box(8,15,31,v(120.5,ys(11.5),-11)),box(22.5,13,6.3,v(70,ys(13),-11.1+S0DZ)),  # v29 splay pillar widened 14->15 (+Y) for socket wall; 3rd = servo0 SEAT (follows servo0)
          box(19.5,1.4,20,v(70,ys(12.5),-11)),box(2,13.5,20,v(68,ys(12.5),-11)),
          box(20.5,1.8,25,v(70,ys(11.9),-11)),box(2.2,14.1,25,v(90.3,ys(11.9),-11)),
          # fore-aft gussets: outer web pulled inboard to the flat side (outer 33.2) + inner web; floor->bridge->pillar, below the coxa splay arc
          box(45,3,19,v(83.5,ys(23.0),-30)),box(45,3,19,v(83.5,ys(18.5),-30)),
          box(22.5,15,1.6,v(70,ys(12),18+S0DZ)),   # v28 servo0 TOP CAP (1.6mm above the servo top) -> servo0 covered on top; only the +X hip side stays open (follows servo0)
          box(24.5,16,31,v(68,ys(11),-11))]   # v28 FILL block round servo0 so the model cut leaves clean 5-sided walls; its inboard face (dxin x64) = the electronics-bay wall (motor end)
    # v28: the SG90 model cut IS the servo0 pocket now. All the old box-pocket relief cuts are DELETED (one of them,
    # box(24.5,6.2,21.5,v(69,ys(20.9),..)), was carving y28-34 = the whole outboard cover -> that opened the side).
    # v28 servo inserts FROM THE HIP SIDE (+X): the whole region beyond the seated tab must be clear of frame - the tab/body/
    # output/insertion path all live there. Cut everything outboard of the tab plane (world x79.4 -> build x85.4) in the servo0
    # footprint, up to just short of the O8 pillar. Only the tab-seating wall (the moved bulkhead, ending at x79.4) remains.
    cuts=[box(15.6,12,32.1,v(85.4,ys(13.8),-9.4+S0DZ)),                                  # CLEAR everything beyond the tab (hip side) (follows servo0)
          cyl(0.8,7,v(85.3,ys(20),-7.1+S0DZ),v(-1,0,0)),cyl(1.1,6,v(88.5,ys(20),-7.1+S0DZ),X),  # servo0 M2 tab-screw pilot + head clearance (follows servo0)
          cyl(4.15,10,v(119.5,ys(SPY),SPZ),X),                                           # O8 splay-pin bore in the pillar, ON the splay axis (servo0 wire slot is cut via wire_slot("s0") in the loop, at the actual nub)
          box(3.6,9.6,9.6,v(125.2,ys(SPY-4.8),SPZ-4.8))]                                  # v29 square SOCKET for the recessed pin head, CENTRED on the splay axis = anti-rotation key, keeps the head inside the frame face  # v26 servo0-lead channel: pass-through in the x83.5 cross-rib EXTENDED up (6->10) to overlap the servo0 pocket floor, so the static yaw-stator lead drops straight from its pocket into the central bay (no arc needed - servo0 crosses no moving joint)
    cr=[box(15,0.5,15,v(72,ys(13.65),-3+S0DZ)),   # v24 crush rib on servo0's inner pocket wall only (follows servo0; outer face is now the flush frame side)
        box(1.0,7,12,v(127.5,ys(SPY-3.5),SPZ-5))]   # v29 splay-pin RETAINER: vertical snap clip standing in the socket mouth (flush with the frame face), caps the recessed head; anchored at top in solid pillar above the socket, free lower end flexes outboard into open air to insert/release the pin. Fused in cr (AFTER the socket cut) so it survives.
    for sx in (1,-1):
        for sy in (1,-1):
            for a in adds: fr=fr.fuse(tf(dxin(a),sx,sy))
            fr=fr.cut(tf(dxin(servo_cut("s0")),sx,sy))    # v28 servo0 pocket = SG90 model cut into the filled corner (clean single cut)
            fr=fr.cut(tf(dxin(wire_slot("s0")),sx,sy))     # v28 servo0 wire slot at the nub (8mm connector + variation), through the wall
            for c in cuts: fr=fr.cut(tf(dxin(c),sx,sy))
            for r in cr: fr=fr.fuse(tf(dxin(r),sx,sy))
    # v28 the tub is open-top (no deck) so the old deck windows are gone; keep only the side power/charge/switch port
    fr=fr.cut(box(6,8,8,v(-52,26,-4)))
    fr=fr.removeSplitter()
    s=fr.Solids; return max(s,key=lambda q:q.Volume) if len(s)>1 else fr
