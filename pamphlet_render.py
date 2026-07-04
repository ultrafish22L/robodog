
# pamphlet_render.py - clean isolated part + assembly renders for the wordless manga pamphlet.
# Execs bodyview (-> all dog13 parts + covers). Saves ref/iter/pam_*.jpg (white bg, crisp edges).
import FreeCAD as App, FreeCADGui as Gui, Part, traceback
from PIL import Image
exec(open(r"C:/ultrafish/robodog/bodyview.py").read())
OUT=r"C:/ultrafish/robodog/ref/iter"; SZ=640
def render(name, shapes, view="Axonometric"):
    nm="pam"
    if nm in list(App.listDocuments()): App.closeDocument(nm)
    d=App.newDocument(nm)
    for i,(q,c) in enumerate(shapes):
        o=d.addObject("Part::Feature","p%d"%i); o.Shape=q
        try: o.ViewObject.ShapeColor=c; o.ViewObject.Transparency=0; o.ViewObject.Deviation=0.004
        except Exception: pass
    d.recompute()
    gv=Gui.activeDocument().activeView()
    getattr(gv,"view"+view)(); Gui.SendMsgToActiveView("ViewFit")
    png=OUT+"/_pam.png"; gv.saveImage(png,SZ,SZ,"White")
    Image.open(png).convert("RGB").save(OUT+"/pam_%s.jpg"%name, quality=84)
    App.closeDocument(nm)
    print("  pam_%s.jpg"%name)
try:
    render("servo",[(servo1,GRY)])
    render("coxa",[(SH,DRK)])
    render("femur",[(FE,YEL)])
    render("tibia",[(TIF,DRK)])
    render("boot",[(BTF,TPU)])
    render("frame",[(FR,FRM)])
    render("tub",[(botp,BODYB)])
    render("lid",[(topp,BODYT)])
    render("leg",[(SH,DRK),(servo1,GRY),(FE,YEL),(servo2,GRY),(hornH,COR),(hornK,COR),(TIF,DRK),(BTF,TPU)])
    render("hero", [(q,c) for n,q,c,t in parts]+[(topp,BODYT),(botp,BODYB)])
    print("OK pamphlet_render")
except Exception:
    print("FAIL: "+traceback.format_exc())
for _d in list(App.listDocuments()): App.closeDocument(_d)
