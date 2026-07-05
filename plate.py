
# plate.py - bin-pack every printed STL for ONE full dog onto Bambu X1C 256x256 plates and draw the layout.
# Footprint = the two largest bbox dims (lay-flattest face down); height = smallest dim. Greedy shelf pack
# per material (ABS rigid vs TPU boots), 6mm part gap, 250mm usable (brim/edge margin). Writes ref/iter/plates.png
# + ref/iter/plates.txt. Run via exec() in the FreeCAD MCP.
import FreeCAD as App, Mesh, traceback
from PIL import Image, ImageDraw
STL=r"C:/ultrafish/robodog/stl"; OUT=r"C:/ultrafish/robodog/ref/iter"
BED=256.0; USE=250.0; GAP=6.0
# (file, qty, material) for one complete dog
PARTS=[("sm3sg90_frame",1,"ABS"),("sm3sg90_coxa",4,"ABS"),("sm3sg90_femur",4,"ABS"),
       ("sm3sg90_tibia",2,"ABS"),("sm3sg90_tibia_rear",2,"ABS"),("sm3sg90_pin",4,"ABS"),
       ("sm3sg90_body_top_lid",1,"ABS"),("sm3sg90_body_bottom_tub",1,"ABS"),
       ("sm3sg90_body_head",1,"ABS"),("sm3sg90_body_rump",1,"ABS"),
       ("sm3sg90_boot_TPU",4,"TPU")]
def foot(name):
    m=Mesh.Mesh(STL+"/"+name+".stl"); b=m.BoundBox
    d=sorted([b.XLength,b.YLength,b.ZLength])   # smallest..largest
    return (d[2],d[1],d[0])                     # (w,h,z-height) lay-flattest
def pack(items):
    # items: list of (label,w,h). Greedy shelf (next-fit decreasing by h), rotate to fit. -> list of plates.
    items=sorted(items,key=lambda it:-max(it[1],it[2]))
    plates=[]; cur=None
    def newplate():
        p={"rects":[],"x":GAP,"y":GAP,"shelf":0.0}; plates.append(p); return p
    cur=newplate()
    for lab,w,h in items:
        placed=False
        for (ww,hh) in ((w,h),(h,w)):                       # try both orientations
            if ww>USE or hh>USE: continue
            for attempt in range(3):
                if cur["x"]+ww<=USE+GAP and cur["y"]+hh<=USE+GAP:
                    cur["rects"].append((lab,cur["x"],cur["y"],ww,hh))
                    cur["x"]+=ww+GAP; cur["shelf"]=max(cur["shelf"],hh); placed=True; break
                # next shelf
                ny=cur["y"]+cur["shelf"]+GAP
                if ny+hh<=USE+GAP:
                    cur["y"]=ny; cur["x"]=GAP; cur["shelf"]=0.0; continue
                # next plate
                cur=newplate()
            if placed: break
        if not placed:
            cur=newplate(); cur["rects"].append((lab,GAP,GAP,min(w,h),max(w,h))); cur["x"]=GAP+min(w,h)+GAP; cur["shelf"]=max(w,h)
    return plates
try:
    fp={n:foot(n) for n,q,mat in PARTS}
    L=["PLATE FIT - Bambu X1C 256x256 (usable %.0f, gap %.0f) - one full dog"%(USE,GAP),"="*60]
    for n,q,mat in PARTS:
        w,h,z=fp[n]; L.append("  %-24s x%d  %-3s  %5.1f x %5.1f  (h %4.1f)"%(n,q,mat,w,h,z))
    groups={}
    for n,q,mat in PARTS:
        w,h,z=fp[n]
        groups.setdefault(mat,[]).extend([("%s#%d"%(n.replace("sm3sg90_",""),i+1),w,h) for i in range(q)])
    allplates=[]
    for mat in ("ABS","TPU"):
        pls=pack(groups[mat]);
        for p in pls: p["mat"]=mat
        allplates+=pls
        L.append("  %s -> %d plate(s), %d parts"%(mat,len(pls),sum(len(p["rects"]) for p in pls)))
    L.append("TOTAL PLATES: %d  (ABS %d + TPU %d)"%(len(allplates),
             sum(1 for p in allplates if p["mat"]=="ABS"),sum(1 for p in allplates if p["mat"]=="TPU")))
    # draw
    PX=520; sc=PX/BED; cols=min(3,len(allplates)); rows=(len(allplates)+cols-1)//cols
    sheet=Image.new("RGB",(cols*PX+20,rows*PX+40),(250,250,250)); dr=ImageDraw.Draw(sheet)
    dr.text((8,6),"X1C plate fit - one full dog - %d plates"%len(allplates),fill=(0,0,0))
    for i,p in enumerate(allplates):
        ox=(i%cols)*PX+10; oy=(i//cols)*PX+30
        dr.rectangle([ox,oy,ox+BED*sc,oy+BED*sc],outline=(0,0,0),fill=(235,235,238))
        dr.text((ox+4,oy+2),"plate %d (%s) %d parts"%(i+1,p["mat"],len(p["rects"])),fill=(0,0,0))
        for lab,x,y,w,h in p["rects"]:
            col=(210,170,60) if p["mat"]=="ABS" else (120,170,220)
            dr.rectangle([ox+x*sc,oy+y*sc,ox+(x+w)*sc,oy+(y+h)*sc],outline=(40,40,40),fill=col)
            dr.text((ox+x*sc+2,oy+y*sc+2),lab[:14],fill=(20,20,20))
    sheet.save(OUT+"/plates.png")
    open(OUT+"/plates.txt","w").write("\n".join(L))
    print("\n".join(L)); print("OK plate")
except Exception:
    print("FAIL:\n"+traceback.format_exc())
