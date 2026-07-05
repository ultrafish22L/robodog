
# testfit.py - FULL DRY-RUN TEST FIT from the exported STL meshes + the real SG90-Servo.stl in all 12 pockets.
# Loads each printed part's STL (build-coord, one corner), re-places it at its assembled pose by replaying
# dog13's transform chain (T = fold@K / pitch@HP / splay@SP, then tf = corner mirror), drops a real SG90
# servo mesh into every servo0/1/2 pocket, adds the 4 cover modules, and renders the assembled dog. Writes
# ref/iter/testfit.png (+ _tf_*.png) and ref/iter/testfit.txt. Run via exec() in the FreeCAD MCP.
import FreeCAD as App, FreeCADGui as Gui, Mesh, Part, math, traceback
from PIL import Image, ImageDraw
STL=r"C:/ultrafish/robodog/stl"; SRV=r"C:/ultrafish/robodog/SG90-Servo.stl"; OUT=r"C:/ultrafish/robodog/ref/iter"
G={'FAST':True}   # skip dog13 gate sweeps + doc render (geometry only) -> stay under the GUI timeout
exec(open(r"C:/ultrafish/robodog/dog13.py").read(), G)          # -> FR,SH,FE, servo0/1/2, HP,SP,K, X,Y,Z ...
HP=G["HP"]; SP=G["SP"]; K=G["K"]                                  # rotation anchors (build coords)
def tup(vv): return (vv.x,vv.y,vv.z)
POSE_F=(-6,35,-63); POSE_R=(-6,-35,63)                            # (splay s, pitch p, fold f) front / rear
def M_rot(ctr,axis,deg): return App.Placement(App.Vector(0,0,0),App.Rotation(App.Vector(*axis),deg),App.Vector(ctr.x,ctr.y,ctr.z)).toMatrix()
def M_mirY():
    m=App.Matrix(); m.A22=-1.0; return m                          # y -> -y (mirror across XZ plane)
def M_rotZ180(): return App.Placement(App.Vector(0,0,0),App.Rotation(App.Vector(0,0,1),180),App.Vector(0,0,0)).toMatrix()
def legxform(m,s,p,f,dofold,dopitch,sx,sy):
    if dofold:  m.transform(M_rot(K,(0,1,0),f))
    if dopitch: m.transform(M_rot(HP,(0,1,0),p))
    m.transform(M_rot(SP,(1,0,0),s))                              # T() done
    if sx>0:
        if sy<0: m.transform(M_mirY())
    else:
        if sy>0: m.transform(M_mirY())
        m.transform(M_rotZ180())                                  # tf() done
    return m
def loadmesh(path):
    mm=Mesh.Mesh(); mm.read(path); return mm
# servo alignment: rotate the SG90 mesh (local L=+X, W=+Z, Hout=+Y) to each pocket's frame, then move to the
# body-box centre (post-yout, build coords), then ride the leg transform.
SRVMAT={"s0":App.Matrix(0,1,0,0, 0,0,1,0, 1,0,0,0, 0,0,0,1),      # Hout->+X, L->+Z, W->+Y
        "s1":App.Matrix(0,0,-1,0, 0,1,0,0, 1,0,0,0, 0,0,0,1),    # Hout->+Y, L->+Z, W->-X
        "s2":App.Matrix(0,0,1,0, 0,-1,0,0, 1,0,0,0, 0,0,0,1)}    # Hout->-Y, L->+Z, W->+X
SRVCTR={"s0":(75.25,27.2,6.65),"s1":(99.95,26.95,4.65),"s2":(100.0,54.13,-69.55)}   # body-box centres (post-yout)
SRVROT={"s0":(0,0,0,0,0),"s1":(1,0,0,0,0),"s2":(1,1,0,0,0)}      # (dosplay,dopitch,-,-,-) which leg rots apply
def place_servo(kind,s,p,f,sx,sy):
    m=loadmesh(SRV); c=m.BoundBox.Center; m.translate(-c.x,-c.y+3.6,-c.z)   # body centre -> origin (boss skews bbox +3.6 Y)
    m.transform(SRVMAT[kind]); ctr=SRVCTR[kind]; m.translate(*ctr)      # orient + move to pocket
    ds,dp=SRVROT[kind][0],SRVROT[kind][1]
    if dp: m.transform(M_rot(HP,(0,1,0),p))
    if ds: m.transform(M_rot(SP,(1,0,0),s))
    if sx>0:
        if sy<0: m.transform(M_mirY())
    else:
        if sy>0: m.transform(M_mirY())
        m.transform(M_rotZ180())
    return m
try:
    items=[]   # (name, mesh, colour, kind)
    items.append(("frame",loadmesh(STL+"/sm3sg90_frame.stl"),(.34,.36,.40),"frame"))
    for sx in (1,-1):
        pose=POSE_F if sx>0 else POSE_R; tib="sm3sg90_tibia" if sx>0 else "sm3sg90_tibia_rear"
        for sy in (1,-1):
            tag="%d%d"%(sx,sy)
            items.append(("coxa_"+tag,  legxform(loadmesh(STL+"/sm3sg90_coxa.stl"),  pose[0],pose[1],pose[2],0,0,sx,sy),(.55,.57,.60),"p"))
            items.append(("femur_"+tag, legxform(loadmesh(STL+"/sm3sg90_femur.stl"), pose[0],pose[1],pose[2],0,1,sx,sy),(.90,.78,.20),"p"))
            items.append(("tibia_"+tag, legxform(loadmesh(STL+"/"+tib+".stl"),       pose[0],pose[1],pose[2],1,1,sx,sy),(.28,.30,.34),"p"))
            items.append(("boot_"+tag,  legxform(loadmesh(STL+"/sm3sg90_boot_TPU.stl"),pose[0],pose[1],pose[2],1,1,sx,sy),(.12,.12,.14),"p"))
            items.append(("s0_"+tag, place_servo("s0",pose[0],pose[1],pose[2],sx,sy),(.20,.45,.75),"srv"))
            items.append(("s1_"+tag, place_servo("s1",pose[0],pose[1],pose[2],sx,sy),(.20,.45,.75),"srv"))
            items.append(("s2_"+tag, place_servo("s2",pose[0],pose[1],pose[2],sx,sy),(.20,.45,.75),"srv"))
    covers=[("lid",loadmesh(STL+"/sm3sg90_body_top_lid.stl"),(.95,.74,.10)),
            ("tub",loadmesh(STL+"/sm3sg90_body_bottom_tub.stl"),(.86,.66,.06)),
            ("head",loadmesh(STL+"/sm3sg90_body_head.stl"),(.92,.60,.10)),
            ("rump",loadmesh(STL+"/sm3sg90_body_rump.stl"),(.92,.60,.10))]
    # HARDWARE (already placed in dog13's `parts`): coxa/splay PIN (printed) + 688ZZ/684ZZ bearing dummies +
    # servo horns (purchased/printed). Pulled straight from G["parts"] so the placement matches exactly.
    HW={"pin":(.62,.62,.66),"b688a":(.74,.76,.80),"b688b":(.74,.76,.80),"b684":(.74,.76,.80),"hH":(.80,.52,.28),"hK":(.80,.52,.28)}
    hwshapes=[(n,q,HW[n.split("_")[0]]) for n,q,c,t in G["parts"] if n.split("_")[0] in HW]
    # assembly bbox (mechanism + servos + hardware)
    allbb=None
    for n,m,c,k in items:
        b=m.BoundBox; allbb=b if allbb is None else allbb.united(b)
    for n,q,c in hwshapes: allbb=allbb.united(q.BoundBox)
    L=["FULL DRY-RUN TEST FIT (exported STLs + real SG90 servo x12 + pin/bearing/horn hardware)","="*70]
    L.append("mechanism+servos+hw bbox: X[%.0f,%.0f]=%.0f Y[%.0f,%.0f]=%.0f Z[%.0f,%.0f]=%.0f"%(
        allbb.XMin,allbb.XMax,allbb.XLength,allbb.YMin,allbb.YMax,allbb.YLength,allbb.ZMin,allbb.ZMax,allbb.ZLength))
    nh=len([1 for n,q,c in hwshapes])
    L.append("placed: 1 frame + 4x(coxa,femur,tibia,boot) + 12 servos + 4 covers + %d hardware (4 pin, 8x b688, 4 b684, 8 horn) = %d"%(nh,len(items)+len(covers)+nh))
    L.append("HARDWARE BOM per dog: 4x splay pin (print), 8x 688ZZ (8x16x5) + 4x 684ZZ (4x9x4) ball bearings (buy), 4x M2 shaft pins/horns")
    open(OUT+"/testfit.txt","w").write("\n".join(L))
    # render
    nm="testfit"
    if nm in list(App.listDocuments()): App.closeDocument(nm)
    d=App.newDocument(nm); objs=[]
    for n,m,c,k in items+[(cn,cm,cc,"cover") for cn,cm,cc in covers]:
        o=d.addObject("Mesh::Feature",n); o.Mesh=m
        try: o.ViewObject.ShapeColor=c
        except Exception: pass
        objs.append((o,k))
    for n,q,c in hwshapes:                              # pin + bearing dummies + horns (parametric shapes)
        o=d.addObject("Part::Feature",n); o.Shape=q
        try: o.ViewObject.ShapeColor=c
        except Exception: pass
        objs.append((o,"hw"))
    d.recompute(); gv=Gui.activeDocument().activeView()
    pl=[]
    # covers OFF (see servos in pockets)
    for o,k in objs:
        o.ViewObject.Visibility = (k!="cover")
    for vn,lab in (("Axonometric","iso  covers OFF (servos in)"),("Right","front  covers OFF"),("Front","side  covers OFF")):
        getattr(gv,"view"+vn)(); Gui.SendMsgToActiveView("ViewFit")
        fp=OUT+"/_tf_%s.png"%vn; gv.saveImage(fp,760,760,"White"); pl.append((fp,lab))
    # covers ON
    for o,k in objs: o.ViewObject.Visibility=True
    for o,k in objs:
        if k=="cover":
            try: o.ViewObject.Transparency=55
            except Exception: pass
    gv.viewAxonometric(); Gui.SendMsgToActiveView("ViewFit")
    fp=OUT+"/_tf_on.png"; gv.saveImage(fp,760,760,"White"); pl.append((fp,"iso  covers ON (x-ray)"))
    App.closeDocument(nm)
    SZ=760; cols=2; rows=2; sheet=Image.new("RGB",(cols*SZ,rows*SZ+24),(245,245,245)); dr=ImageDraw.Draw(sheet)
    dr.text((6,6),"testfit - full dry-run assembly from STLs + real SG90 servo x12",fill=(0,0,0))
    for i,(p,lab) in enumerate(pl):
        im=Image.open(p).convert("RGB").resize((SZ,SZ)); dd=ImageDraw.Draw(im)
        dd.rectangle([0,0,240,20],fill=(30,30,30)); dd.text((4,4),lab,fill=(255,255,255))
        r,cc=divmod(i,cols); sheet.paste(im,(cc*SZ,24+r*SZ))
    sheet.save(OUT+"/testfit.png")
    print("\n".join(L)); print("OK testfit")
except Exception:
    print("FAIL:\n"+traceback.format_exc())
    for _d in list(App.listDocuments()):
        if _d!="dog13":
            try: App.closeDocument(_d)
            except Exception: pass
