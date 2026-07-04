
# poseview.py - render the isolated leg (femur+tibia+boot) in several POSES x several ANGLES,
# montage each pose into one image via PIL. Reuses session globals FE,TIF,BTF,HP,K,SP.
import FreeCAD as App, FreeCADGui as Gui, Part, math, traceback, os
from PIL import Image, ImageDraw
v=App.Vector; X,Y,Z=v(1,0,0),v(0,1,0),v(0,0,1)
OUT=r"C:/ultrafish/robodog/ref/iter"
YEL=(.95,.74,.10); DRK=(.14,.14,.16); TPU=(.20,.20,.22)
def rot(s,deg,ctr,ax): s=s.copy(); s.rotate(ctr,ax,deg); return s
def Tf(q,s,p):    # femur: pitch about hip Y, then splay about X (no knee fold)
    q=rot(q,p,HP,Y); return rot(q,s,SP,X)
def Tt(q,s,p,f):  # tibia/boot: fold about knee Y, pitch about hip Y, splay about X
    q=rot(q,f,K,Y); q=rot(q,p,HP,Y); return rot(q,s,SP,X)
# poses: (name, splay, pitch, fold)
POSES=[("neutral",0,0,0),("stance",-6,35,-63),("high",-6,20,-45),("splay",28,45,-75),("foldflat",-6,82,-115)]
# views: (freecad view method suffix, label)
VIEWS=[("Right","front (dog)"),("Front","side"),("Rear","back"),("Axonometric","iso"),("Bottom","plan"),("Dimetric","iso-2")]
SZ=560
def montage(paths_labels,pname,cols=3):
    imgs=[]
    for p,lab in paths_labels:
        im=Image.open(p).convert("RGB").resize((SZ,SZ))
        d=ImageDraw.Draw(im); d.rectangle([0,0,150,20],fill=(30,30,30)); d.text((4,4),lab,fill=(255,255,255))
        imgs.append(im)
    rows=(len(imgs)+cols-1)//cols
    W=cols*SZ; H=rows*SZ+24
    sheet=Image.new("RGB",(W,H),(245,245,245))
    d=ImageDraw.Draw(sheet); d.text((6,6),"POSE: "+pname,fill=(0,0,0))
    for i,im in enumerate(imgs):
        r,c=divmod(i,cols); sheet.paste(im,(c*SZ,24+r*SZ))
    sheet.save(OUT+"/pose_%s.png"%pname)
try:
    for pname,s,p,f in POSES:
        nm="pv_"+pname
        if nm in list(App.listDocuments()): App.closeDocument(nm)
        d=App.newDocument(nm)
        for n,q,c in [("fe",Tf(FE,s,p),YEL),("ti",Tt(TIF,s,p,f),DRK),("bt",Tt(BTF,s,p,f),TPU)]:
            o=d.addObject("Part::Feature",n); o.Shape=q
            try: o.ViewObject.ShapeColor=c; o.ViewObject.Deviation=0.01
            except Exception: pass
        d.recompute()
        gv=Gui.activeDocument().activeView()
        pl=[]
        for vn,lab in VIEWS:
            getattr(gv,"view"+vn)(); Gui.SendMsgToActiveView("ViewFit")
            fp=OUT+"/_pv_%s_%s.png"%(pname,vn)
            gv.saveImage(fp,SZ,SZ,"White"); pl.append((fp,lab))
        montage(pl,pname)
        App.closeDocument(nm)
    print("OK poseview")
except Exception:
    print("FAIL: "+traceback.format_exc())
