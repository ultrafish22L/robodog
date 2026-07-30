# part_femur.py -- split out of dog13.py (2026-07-19), VERBATIM.
# femur(): flat 8mm thigh plate, hip dome, servo2 pocket, knee carrier.
# Loaded by dog13.py via dinc(); execs into the CALLER's globals so the long-standing
# `exec(open('dog13.py').read())` contract (bodyview/export/frameview/coxablock/frameblock)
# keeps seeing every name flat, exactly as before.

def femur():
    # domed shoulder: top sections taper lateral width toward the top so the hip end rounds
    # smoothly from the top over to the outboard side (not cylinder-like); inboard face stays flat.
    ST=[(20.6,3.0,8.0,2.5),(19.4,5.5,14.0,5.0),(17.6,8.0,20.0,7.5),(15,9.8,25,9.0),(11.5,10.9,28,9.0),(10,11.0,28,9.0),
        (2,10.6,28,8.5),(-12,9.6,26,8),(-26,9.2,25.5,7.5),(-40,9.6,26,8),(-50,10.0,28,8),
        (-58,10.0,28,7.5),(-63,9.7,28,7.5),(-66.5,9.2,28,7),(-69.6,8.0,28,5.5),(-71.2,4.0,28,3.4)]
    ST=[(femz(z),hx,wy,rc) for (z,hx,wy,rc) in ST]   # lengthen shaft, keep hip dome + knee carrier
    fe=Part.makeLoft([rring(*s) for s in ST],True,False)
    fe=fe.cut(cyl(7.97,BRG1+0.05-F,v(HPx,F,HPZ),Y)).cut(cyl(3.72,HUB_CYL+0.15,v(HPx,F-0.05,HPZ),Y))   # hip-pitch seats ON the measured axis
    fe=fe.cut(cyl(3.78,PLATE_T+0.6,v(HPx,F+HUB_CYL,HPZ),Y)).cut(box(ARM_W,PLATE_T+0.6,ARM_R,v(HPx-ARM_W/2,F+HUB_CYL,HPZ-ARM_R)))
    fe=fe.cut(cyl(2.8,28,v(HPx,F+HORN_H+0.4,HPZ),Y)).cut(servo_cut("s2")).cut(wire_slot("s2"))   # v28 servo2 pocket = SG90 model cut + wire slot at the nub
    fe=fe.cut(cyl(0.85,4.5,v(HPx,F+5.4,-66.3+DKZ),Y)).cut(cyl(0.85,4.5,v(HPx,F+5.4,-38.8+DKZ),Y))   # M2 tab-screw pilots into the flange-rest walls
    fe=fe.fuse(cyl(2.0,3.2,v(HPx,F+28,KZ),Y).fuse(Part.makeCone(2.0,1.3,1.4,v(HPx,F+31.2,KZ),Y)))
    # v28 push-fit ridges on both X-walls of the model-cut servo2 pocket (~0.35 proud into the 0.25 gap -> ~0.1 crush)
    fe=fe.fuse(box(0.35,18,14,v(99.5,F,-62+DKZ))).fuse(box(0.35,18,14,v(112.15,F,-62+DKZ)))
    # v26 WIRE ROUTING: groove up the femur inboard (print-bed) face from the servo2 (knee/"servo3") cavity to a mouth just
    # below the hip-PITCH bearing rim (axis x106,z10; 688 lower rim z~2). Femur pitches a FULL CIRCLE about that axis, so the
    # lead exits ~10mm below it (arc r~10mm) and twists rather than winding a 60-90mm arc. Sized 4.2w x 3.0deep so a real SG90
    # 3-wire bundle seats past FDM elephant-foot (verify-panel: 3.5 nominal squishes to ~2.7); mouth recessed to z-3 so the
    # lead clears the 688 OD when the bearing is seated. Prints inboard-face-DOWN (slicer: brim + ~0.15mm e-foot comp).
    fe=fe.cut(box(4.2,3.5,55,v(102.65,F-0.5,-58)))
    s=fe.Solids; return max(s,key=lambda q:q.Volume) if len(s)>1 else fe
YI=F-5.5
KMID=F+14.0   # knee lateral middle == leg centerline; shank sections center here so the
BY=KMID       # ball foot drops straight under the knee & the flat inboard face tilts in
# rringT: flat-ish inboard face (small ri), rounded fore/aft/outboard (rc); centered on cy.
