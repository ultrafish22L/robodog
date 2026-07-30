# part_tibia.py -- split out of dog13.py (2026-07-19), VERBATIM.
# tibia() + its private helpers (rringT/clev_wire/clev/clevB).
# Loaded by dog13.py via dinc(); execs into the CALLER's globals so the long-standing
# `exec(open('dog13.py').read())` contract (bodyview/export/frameview/coxablock/frameblock)
# keeps seeing every name flat, exactly as before.

def rringT(z,xc,hx,cy,hy,rc):
    ri=3.0; ri=min(ri,hx-0.5,hy-0.5); rc=min(rc,hx-0.5,hy-0.5); y0=cy-hy; y1=cy+hy; pts=[]
    def arc(cxa,cya,r,a0,a1,n):
        for i in range(n+1):
            a=a0+(a1-a0)*i/float(n); pts.append((cxa+r*math.cos(a),cya+r*math.sin(a)))
    arc(xc-hx+ri,y0+ri,ri,math.pi,1.5*math.pi,4)
    arc(xc+hx-ri,y0+ri,ri,1.5*math.pi,2.0*math.pi,4)
    arc(xc+hx-rc,y1-rc,rc,0.0,0.5*math.pi,8)
    arc(xc-hx+rc,y1-rc,rc,0.5*math.pi,math.pi,8)
    return Part.makePolygon([v(p[0],p[1],z) for p in pts]+[v(pts[0][0],pts[0][1],z)])
def clev_wire(y0,rr,bx0,bx1):
    pts=[(bx0,BZ),(bx1,BZ)]                  # bridge follows knee (BZ), arc centred on knee (KZ)
    for i in range(29):
        a=math.radians(-14.0+208.0*i/28.0)
        pts.append((HPx+rr*math.cos(a),KZ+rr*math.sin(a)))
    return Part.makePolygon([v(p[0],y0,p[1]) for p in pts]+[v(bx0,y0,BZ)])
def clev(y0):   # flat prong (inboard = horn seat side)
    return Part.Face(clev_wire(y0,8.5,97.0,115.0)).extrude(v(0,5.2,0))
def clevB(y0):  # outboard prong: inset outer face -> rounded lobe continuing the shank
    return Part.makeLoft([clev_wire(y0,8.5,97.0,115.0),clev_wire(y0+5.2,6.6,99.4,112.6)],True,False)
def tibia():
    armI=clev(YI)
    armO=clevB(F+28.4)
    ST=[(-78.0,106.0,8.8,KMID,19.55,8),(-84,107.2,9.7,KMID,15.5,9),(-92,107.8,10.0,KMID,12.5,9),
        (-102,107.4,9.3,KMID,10.25,8),(-108,107.2,8.7,KMID,9.65,7.8),(-116,106.4,8.3,KMID,9.25,7.4),
        (-123,104.9,7.8,KMID,9.1,7.0),(-129,102.8,7.4,KMID,9.0,6.5),(-134,100.2,7.2,KMID,9.0,6.0)]
    ST=[(tibz(z),xc,hx,cy,hy,rc) for (z,xc,hx,cy,hy,rc) in ST]   # stretch shank about the bridge
    body=Part.makeLoft([rringT(*s) for s in ST],True,False)
    p=Part.makeSphere(10.0,v(99.3,BY,tibz(-135.8)))
    t=armI.fuse(armO).fuse(body).fuse(p)
    t=t.cut(cyl(3.72,2.9,v(HPx,F-3.0,KZ),Y)).cut(cyl(3.78,2.0,v(HPx,F-4.7,KZ),Y))
    t=t.cut(box(ARM_W,2.0,ARM_R,v(HPx-ARM_W/2,F-4.7,KZ-ARM_R))).cut(cyl(2.1,1.2,v(HPx,F-5.6,KZ),Y))
    t=t.cut(cyl(4.46,4.5,v(HPx,F+29.1,KZ),Y)).cut(cyl(2.35,1.0,v(HPx,F+28.3,KZ),Y))
    s=t.Solids; return max(s,key=lambda q:q.Volume) if len(s)>1 else t
