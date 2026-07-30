# part_shoulder.py -- split out of dog13.py (2026-07-19), VERBATIM.
# shoulder(): LEGACY coxa. Superseded for PRINTING by coxablock.py, but still
# the live SH used by the assembly/gates/renders and by frameblock.py -- keep until coxablock replaces it.
# Loaded by dog13.py via dinc(); execs into the CALLER's globals so the long-standing
# `exec(open('dog13.py').read())` contract (bodyview/export/frameview/coxablock/frameblock)
# keeps seeing every name flat, exactly as before.

def shoulder():
    sh=box(24.5,26.8,28.5,v(94,6,-8.5))
    sh=sh.cut(tri([(93.8,6,-0.5),(93.8,6,-8.5),(93.8,14,-8.5)],(25,0,0)))
    sh=sh.cut(tri([(93.8,32.8,-0.5),(93.8,32.8,-8.5),(93.8,24.8,-8.5)],(25,0,0)))
    top=[e for e in sh.Edges if all(abs(vv.Point.z-20.0)<1e-6 for vv in e.Vertexes)]
    try: sh=sh.makeFillet(4.0,top)
    except Exception:
        try: sh=sh.makeFillet(2.5,top)
        except Exception: pass
    # NB: shoulder() is the LEGACY built-up coxa, superseded for printing by coxablock.py; kept here for the
    # gate sweeps + assembly render, so its two axes are re-datumed with everything else (splay SPZ, pitch HPZ).
    sh=sh.cut(cyl(3.72,2.9,v(93.7,SPY,SPZ),X)).cut(cyl(3.78,2.2,v(96.3,SPY,SPZ),X))
    sh=sh.cut(box(2.2,ARM_W,ARM_R,v(96.3,SPY-ARM_W/2,SPZ-ARM_R)))
    sh=sh.cut(cyl(2.7,15.2,v(98.4,SPY,SPZ),X)).cut(cyl(4.4,2.2,v(111.4,SPY,SPZ),X)).cut(cyl(7.93,5.4,v(113.2,SPY,SPZ),X))
    sh=sh.cut(servo_cut("s1")).cut(wire_slot("s1")).cut(cyl(8.75,2.0,v(HPx,30.9,HPZ),Y))   # v28 servo1 pocket = SG90 model cut + wire slot at the nub
    # v28 push-fit ridges on both X-walls of the model-cut servo1 pocket (~0.35 proud into the 0.25 gap -> ~0.1 crush)
    # (the v27 box tab-slots are gone: the model cut already carves the flange + ears)
    sh=sh.fuse(box(0.35,18,15,v(99.45,10,-3))).fuse(box(0.35,18,15,v(112.1,10,-3)))
    # v26 WIRE ROUTING: splay(yaw)-axis eyelet. The coxa yaws +-45 about SP (a fore-aft/X line at build (20,12)); a bore along
    # -X exiting the inboard face into the frame corner well, offset ~7mm below the yaw axis to clear the O7.5 yaw-bearing bore
    # -> ~7mm arc radius (a small twist, not a wind-up). O5 (not O4) so the servo1+servo2 BUNDLE (~4-5mm) passes after a
    # horizontal FDM bore sags the crown ~0.6mm (verify-panel).
    sh=sh.cut(cyl(2.5,10,v(95,SPY,SPZ-7),v(-1,0,0)))   # 7mm below the yaw axis (follows SPZ)
    # v26 WIRE ROUTING "complete the path": (a) RECEIVER scoop just below the pitch-bearing seat (rim z~1.25) that drops the
    # femur/knee lead off its groove mouth into the servo1 pocket to bundle; (b) POCKET->EYELET connector through the ~4.4mm
    # wall between the servo1 pocket (x>=99.4) and the eyelet (x95) so the bundle actually reaches the bore; (c) flared lead-in
    # at the eyelet exit lip so the splay-twisting bundle bears on a radius, not a printed edge.
    sh=sh.cut(box(6,5,6,v(102,28,-5)))                             # receiver: femur-lead crossing -> servo1 pocket
    sh=sh.cut(box(6,4,4,v(94,SPY-2,SPZ-9)))                        # bundle: servo1 pocket -> eyelet mouth (CENTRED on the eyelet axis
                                                                   # (SPY,SPZ-7): the old literal z3 was centred on the pre-2026-07-19
                                                                   # datum 12-7=5, leaving the channel 1.6mm above its own bore)
    sh=sh.cut(Part.makeCone(2.5,3.6,1.5,v(93.7,SPY,SPZ-7),v(-1,0,0)))   # flared eyelet exit lip (coaxial with the eyelet)
    s=sh.Solids; return max(s,key=lambda q:q.Volume) if len(s)>1 else sh
