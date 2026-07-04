
# assembly.py - fully assembled dog render (covers on + X-ray) + assembly audit.
# Execs bodyview.py (-> dog13 globals: parts[], FR, FE, TIF/TIR, BTF/BTR, SH, servo0/1/2,
# and the built covers topp/botp/face + fingers/windows + WALL). Renders ref/iter/assembly.png
# and writes a full interference audit to ref/iter/audit.txt.
import FreeCAD as App, FreeCADGui as Gui, Part, math, traceback
from PIL import Image, ImageDraw
exec(open(r"C:/ultrafish/robodog/bodyview.py").read())
OUT=r"C:/ultrafish/robodog/ref/iter"; SZ=760

try:
    # ---------- assembled render ----------
    nm="assembly"
    if nm in list(App.listDocuments()): App.closeDocument(nm)
    d=App.newDocument(nm)
    shown=list(parts)+[("lid",topp,BODYT,0),("tub",botp,BODYB,0),("facep",face,FACE,0)]
    objs={}
    for n,q,c,t in shown:
        o=d.addObject("Part::Feature",n); o.Shape=q
        try: o.ViewObject.ShapeColor=c; o.ViewObject.Transparency=t; o.ViewObject.Deviation=0.01
        except Exception: pass
        objs[n]=o
    d.recompute()
    gv=Gui.activeDocument().activeView()
    pl=[]
    for vn,lab in [("Axonometric","iso  covers ON"),("Front","side"),("Right","front (dog)")]:
        getattr(gv,"view"+vn)(); Gui.SendMsgToActiveView("ViewFit")
        fp=OUT+"/_as_%s.png"%vn; gv.saveImage(fp,SZ,SZ,"White"); pl.append((fp,lab))
    for n,o in objs.items():                       # X-ray: fade the covers to reveal internals
        if n in ("lid","tub","facep"):
            try: o.ViewObject.Transparency=80
            except Exception: pass
    d.recompute(); gv.viewAxonometric(); Gui.SendMsgToActiveView("ViewFit")
    fp=OUT+"/_as_xray.png"; gv.saveImage(fp,SZ,SZ,"White"); pl.append((fp,"iso  X-RAY internals"))
    cols=2; rows=2; W=cols*SZ; H=rows*SZ+24
    sheet=Image.new("RGB",(W,H),(245,245,245)); dr=ImageDraw.Draw(sheet)
    dr.text((6,6),"assembly - fully assembled dog (v22 frame + 4 legs/12 servos + snap-on covers)",fill=(0,0,0))
    for i,(p,lab) in enumerate(pl):
        im=Image.open(p).convert("RGB").resize((SZ,SZ)); dd=ImageDraw.Draw(im)
        dd.rectangle([0,0,210,20],fill=(30,30,30)); dd.text((4,4),lab,fill=(255,255,255))
        r,cc=divmod(i,cols); sheet.paste(im,(cc*SZ,24+r*SZ))
    sheet.save(OUT+"/assembly.png")
    App.closeDocument(nm)

    # ---------- audit ----------
    def ov(a,b):
        try: return a.common(b).Volume
        except Exception: return -1.0
    def bbx(s): b=s.BoundBox; return "X[%.0f,%.0f] Y[%.0f,%.0f] Z[%.0f,%.0f]"%(b.XMin,b.XMax,b.YMin,b.YMax,b.ZMin,b.ZMax)
    legparts=[(n,q) for n,q,c,t in parts if n.split('_')[0] in ('sh','fe','ti','bt','s0','s1','s2')]
    cover=topp.fuse(botp)
    L=["ASSEMBLY AUDIT  (units mm, mm^3)  -- v22 frame + covers","="*64]

    L.append("[BODY PANELS]")
    for nm2,pn in (("top lid",topp),("bottom tub",botp)):
        L.append("  %-11s valid=%s  solids=%d shells=%d  %s  vol=%.0f"%(
            nm2,pn.isValid(),len(pn.Solids),len(pn.Shells),bbx(pn),pn.Volume))
    L.append("  lid^tub overlap ......... %.1f   (want 0: split by the reveal gap)"%ov(topp,botp))
    L.append("  cover^FRAME ............. %.1f   (want ~0: cavity clears the frame)"%ov(cover,FR))
    byt={}
    for n,q in legparts:
        k=n.split('_')[0]; byt[k]=byt.get(k,0.0)+ov(cover,q)
    L.append("  cover^legs (4 corners summed, want 0 except hip pass-through):")
    for k in sorted(byt): L.append("      %-4s %.0f"%(k,byt[k]))
    L.append("  wall nominal = %.1f mm (build_body inset); frame-cut may thin it locally"%WALL)

    L.append("[CANTILEVER LATCHES]  6 fingers (tub) -> release windows (lid), toolless")
    L.append("  design: 1.5mm-thin finger, 0.7mm hook (0.7mm flex to release), lid window = finger access")
    L.append("  stations x=-55/0/55 x 2 sides;  finger y=34.8 vs frame 33.2 -> 1.6mm room to flex in")
    L.append("  tub is 1 solid -> all 6 fingers fused to the tub:        %s"%(len(botp.Solids)==1))
    L.append("  lid is 1 solid -> release windows didn't fragment it:    %s"%(len(topp.Solids)==1))
    L.append("  fingers^FRAME ........... %.1f   (want 0: finger clears the frame)"%ov(fingers,FR))
    fL=sum(ov(fingers,q) for n,q in legparts)
    L.append("  fingers^legs ............ %.1f   (want 0)"%fL)
    L.append("  fingers^lid seated ...... %.1f   (want ~0: hook rests in the window -> releasable)"%ov(fingers,topp))
    liftlid=topp.copy(); liftlid.translate(App.Vector(0,0,1.6))    # lift the lid 1.6mm
    L.append("  hook catch on 1.6mm lift  %.1f   (>0: window bottom edge hits the hook -> LATCHED)"%ov(fingers,liftlid))
    L.append("  windows^lid removed ..... vol=%.0f across 6 release windows"%windows.Volume)

    L.append("[SERVO POCKETS]  12 servos = 3 types x 4 legs (mirror-identical)")
    for nm2,sv,hn,host in (("servo0 hip-yaw ",servo0,"FRAME",FR),
                            ("servo1 hip-pitch",servo1,"COXA ",SH),
                            ("servo2 knee    ",servo2,"FEMUR",FE)):
        seat=ov(sv,host); sb=sv.BoundBox; hb=host.BoundBox
        inside=(sb.XMin>=hb.XMin-1 and sb.XMax<=hb.XMax+1 and sb.YMin>=hb.YMin-1 and sb.YMax<=hb.YMax+1)
        L.append("  %s in %s: servo^host=%6.1f (want 0=seats, no gouge)  host-bbox-captures=%s"%(nm2,hn,seat,inside))
    L.append("  servo vs covers: see cover^legs s0/s1/s2 above (0 = servos clear the body)")

    L.append("[FRAME LOCATING KEYS]  (panels latch to each other; frame is located + sandwiched)")
    L.append("  tub CRADLE nests the frame floor-plate (x+-92.5,y+-33.2) -> locates X/Y, rests z-30")
    L.append("  cradle^FRAME ............ %.1f   (want ~0: 0.5mm nest clearance, no gouge)"%ov(cradle,FR))
    L.append("  lid CLAMP PADS bear on the frame top-plate top (z14) -> vertical sandwich")
    L.append("  lidpads^FRAME ........... %.1f   (want ~0: pads meet the deck, no gouge)"%ov(lidpads,FR))
    L.append("  lidpads^legs ............ %.1f   (want 0)"%sum(ov(lidpads,q) for n,q in legparts))

    open(OUT+"/audit.txt","w").write("\n".join(L))
    print("OK assembly")
    print("\n".join(L))
except Exception:
    print("FAIL: "+traceback.format_exc())
for _d in list(App.listDocuments()): App.closeDocument(_d)
