
# frameview.py - render the CURRENT frame isolated, montage via PIL.
# Execs dog13.py so FR is always the live canonical frame (was previously reusing a
# stale session-global FR, which silently rendered an out-of-date frame).
import FreeCAD as App, FreeCADGui as Gui, Part, math, traceback
from PIL import Image, ImageDraw
exec(open(r"C:/ultrafish/robodog/dog13.py").read())        # -> live FR (also re-runs gates)
OUT=r"C:/ultrafish/robodog/ref/iter"
FRM=(.32,.34,.38)
SZ=620
def montage(pl,name,cols=3):
    imgs=[]
    for p,lab in pl:
        im=Image.open(p).convert("RGB").resize((SZ,SZ))
        d=ImageDraw.Draw(im); d.rectangle([0,0,170,20],fill=(30,30,30)); d.text((4,4),lab,fill=(255,255,255))
        imgs.append(im)
    rows=(len(imgs)+cols-1)//cols; W=cols*SZ; H=rows*SZ+24
    sheet=Image.new("RGB",(W,H),(245,245,245)); d=ImageDraw.Draw(sheet); d.text((6,6),name,fill=(0,0,0))
    for i,im in enumerate(imgs):
        r,c=divmod(i,cols); sheet.paste(im,(c*SZ,24+r*SZ))
    sheet.save(OUT+"/"+name+".png")
try:
    bb=FR.BoundBox
    open(OUT+"/frame.txt","w").write(
        "FRAME bbox X[%.1f,%.1f]=%.1f  Y[%.1f,%.1f]=%.1f  Z[%.1f,%.1f]=%.1f  vol=%.0f solids=%d L:W=%.2f\n"%(
        bb.XMin,bb.XMax,bb.XLength,bb.YMin,bb.YMax,bb.YLength,bb.ZMin,bb.ZMax,bb.ZLength,FR.Volume,len(FR.Solids),bb.XLength/bb.YLength))
    nm="framedetail"
    if nm in list(App.listDocuments()): App.closeDocument(nm)
    d=App.newDocument(nm)
    o=d.addObject("Part::Feature","frame"); o.Shape=FR
    try: o.ViewObject.ShapeColor=FRM; o.ViewObject.Deviation=0.01
    except Exception: pass
    d.recompute()
    gv=Gui.activeDocument().activeView()
    pl=[]
    for vn,lab in [("Top","top (plan)"),("Axonometric","iso"),("Front","side"),("Right","front (dog)"),("Bottom","bottom"),("Dimetric","iso-2")]:
        getattr(gv,"view"+vn)(); Gui.SendMsgToActiveView("ViewFit")
        fp=OUT+"/_fr_%s.png"%vn; gv.saveImage(fp,SZ,SZ,"White"); pl.append((fp,lab))
    montage(pl,"frame_views")
    App.closeDocument(nm)
    print("OK frameview")
except Exception:
    print("FAIL: "+traceback.format_exc())
