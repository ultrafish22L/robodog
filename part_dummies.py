# part_dummies.py -- split out of dog13.py (2026-07-19), VERBATIM.
# DISPLAY/gate proxies only (idealised servos, 688/684 bearings, horns, splay pin).
# Real pockets are cut with the SCANNED mesh via servo_cut() -- never trust these bosses.
# Loaded by dog13.py via dinc(); execs into the CALLER's globals so the long-standing
# `exec(open('dog13.py').read())` contract (bodyview/export/frameview/coxablock/frameblock)
# keeps seeing every name flat, exactly as before.

# NB: these idealised servo models are DISPLAY/gate proxies only; the real pockets are cut with the scanned
# mesh. Their bosses are now placed on the MEASURED spline axes (SPZ/HPZ), not the old nominal 12 / 10.
servo0=box(22.5,12.1,22.7,v(70,13.95,-4.7+S0DZ)).fuse(box(2.4,12.1,32.1,v(85.4,13.95,-9.4+S0DZ))).fuse(cyl(2.9,BOSS_H,v(92.5,SPY,SPZ),X)).fuse(cyl(1.7,BOSS_H+SPL_H,v(92.5,SPY,SPZ),X))
servo1=box(12.1,22.5,22.7,v(HPx-6.1,8.5,-6.7)).fuse(cyl(2.9,BOSS_H,v(HPx,31,HPZ),Y)).fuse(cyl(1.7,BOSS_H+SPL_H,v(HPx,31,HPZ),Y))
servo2=box(12.1,22.5,22.7,v(HPx-6.05,F+KW,-63.9+DKZ)).fuse(cyl(2.9,BOSS_H,v(HPx,F+KW,KZ),v(0,-1,0))).fuse(cyl(1.7,BOSS_H+SPL_H,v(HPx,F+KW,KZ),v(0,-1,0)))
b688a=cyl(8,5,v(113.4,SPY,SPZ),X).cut(cyl(4.05,5,v(113.4,SPY,SPZ),X))
b688b=cyl(8,BRG1-BRG0,v(HPx,BRG0,HPZ),Y).cut(cyl(4.02,BRG1-BRG0,v(HPx,BRG0,HPZ),Y))
b684=cyl(4.5,4.0,v(HPx,F+29.15,KZ),Y).cut(cyl(2.01,4.0,v(HPx,F+29.15,KZ),Y))
hornH=cyl(3.59,HUB_CYL,v(HPx,F,HPZ),Y).fuse(cyl(3.55,PLATE_T,v(HPx,F+HUB_CYL,HPZ),Y))
hornK=cyl(3.59,HUB_CYL,v(HPx,F-2.7,KZ),v(0,-1,0)).fuse(cyl(3.55,PLATE_T,v(HPx,F-2.7,KZ),v(0,-1,0)))
pin=cyl(3.9,12.7,v(112.8,SPY,SPZ),X).fuse(box(2.0,9,9,v(125.5,SPY-4.5,SPZ-4.5)))   # v29 SQUARE head (9x9x2) centred on the splay axis, shaft shortened so the head recesses into the pillar socket (keyed=anti-rotation) -> frame stays <=246
