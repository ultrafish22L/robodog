# part_boot.py -- split out of dog13.py (2026-07-19), VERBATIM.
# boot(): the press-on TPU foot.
# Loaded by dog13.py via dinc(); execs into the CALLER's globals so the long-standing
# `exec(open('dog13.py').read())` contract (bodyview/export/frameview/coxablock/frameblock)
# keeps seeing every name flat, exactly as before.

def boot():
    C=v(99.3,BY,tibz(-135.8))
    o=Part.makeSphere(11.6,C).cut(Part.makeSphere(10.05,C))
    o=o.cut(box(14.6,20.0,12.5,v(C.x-7.3,BY-10.0,C.z)))
    o=o.cut(box(30,30,10,v(C.x-15.0,BY-15.0,C.z+4.5)))
    o=rot(o,28.0,C,Y)
    s=o.Solids; return max(s,key=lambda q:q.Volume) if len(s)>1 else o
