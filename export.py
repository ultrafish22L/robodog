
# export.py - committed STL exporter for the canonical printable parts.
# Execs dog13.py (leaves FR/SH/FE/TIF/TIR/BTF/BTR as globals, v23 geometry), meshes each printable
# solid and writes stl/sm3sg90_*.stl as single-solid manifolds for a Bambu X1C (256^3 bed).
# NOTE: exports the MECHANISM parts (frame + one corner: coxa/femur/tibia/boot + rear-tibia/boot mirrors).
# The two body covers (lid/tub) + head/rump come from bodyview.py and are exported there once its cradle
# is re-fit to the bed-fit frame (plates now end at x+-86.5, was +-92.5). Run via exec() in the FreeCAD MCP.
import FreeCAD as App, Mesh, MeshPart, Part, traceback
FAST=True   # skip dog13 gate sweeps + doc render (geometry only) -> stay under the GUI timeout
exec(open(r"C:/ultrafish/robodog/dog13.py").read())      # -> FR, SH, FE, TIF, TIR, BTF, BTR (v23, yout+bed-fit shifted)
OUT=r"C:/ultrafish/robodog/stl"; BED=256.0
DEFL=0.12                                                 # LinearDeflection: fine enough for print, ~modest file size
def repair_clean(m, path):
    # v29 PRE-PRINT REPAIR verified ON DISK: tessellating the model-cut pockets / organic shells yields
    # self-intersecting/non-manifold triangles even from a valid single-solid b-rep. fixSelfIntersections()
    # can report clean in-memory yet REGRESS after the float32 STL write (dense femur pocket), so we repair,
    # write, RE-READ, and repeat until the on-disk mesh is provably watertight+manifold+self-intersection-free.
    m.removeDuplicatedPoints(); m.removeDuplicatedFacets()
    for _ in range(6):
        n=0
        while m.hasSelfIntersections() and n<4: m.fixSelfIntersections(); n+=1
        if m.hasNonManifolds(): m.removeNonManifolds()
        m.harmonizeNormals(); m.write(path)
        r=Mesh.Mesh(); r.read(path)
        if not (r.hasSelfIntersections() or r.hasNonManifolds()): return r,"clean"
        m=r
    return m,"DEFECT!"
def export_stl(shape, name):
    m=MeshPart.meshFromShape(Shape=shape, LinearDeflection=DEFL, AngularDeflection=0.5, Relative=False)
    p=OUT+"/"+name+".stl"; m,defect=repair_clean(m,p)
    b=shape.BoundBox; fit="OK" if max(b.XLength,b.YLength,b.ZLength)<=BED else "OVER-BED!"
    return "%-26s solids=%d  %.0f x %.0f x %.0f mm  tris=%d  %s  %s"%(name,len(shape.Solids),b.XLength,b.YLength,b.ZLength,m.CountFacets,fit,defect)
try:
    # The leg bones are CHIRAL (coxa 0.87, femur 0.63, tibia 0.93 self-vs-mirror) so each needs a mirrored twin;
    # the 4 corners split 2 handedness H / 2 mirror H' (diagonally, because the rear legs also get a 180 Z spin).
    # Boot is symmetric (mirror==self) -> one STL, print x4.  tibia front!=rear (0.79 common) -> 4 distinct tibias.
    vv=App.Vector
    def mir(P): c=P.BoundBox.Center; return P.mirror(vv(c.x,c.y,c.z), vv(0,1,0))
    # printable DUMMY bearings = plain-ring bushings for dry test-assembly before the real ball bearings arrive.
    # OD -0.2 (drops into the seat), bore +0.2 (slips on the shaft/pin); print flat, tune the fit on a real print.
    def ring(od,bore,h): return Part.makeCylinder(od/2.0,h).cut(Part.makeCylinder(bore/2.0,h+2.0,vv(0,0,-1.0)))
    d688=ring(15.8,8.2,5.0); d684=ring(8.8,4.2,4.0)                          # 688ZZ 8x16x5, 684ZZ 4x9x4
    parts=[(FR,"sm3sg90_frame"),
           (SH,"sm3sg90_coxa"),(mir(SH),"sm3sg90_coxa_mir"),                 # x2 each
           (FE,"sm3sg90_femur"),(mir(FE),"sm3sg90_femur_mir"),               # x2 each
           (TIF,"sm3sg90_tibia"),(mir(TIF),"sm3sg90_tibia_mir"),             # front L/R
           (TIR,"sm3sg90_tibia_rear"),(mir(TIR),"sm3sg90_tibia_rear_mir"),   # rear  L/R
           (BTF,"sm3sg90_boot_TPU"),                                         # symmetric -> x4
           (pin,"sm3sg90_pin"),                                             # x4
           (d688,"sm3sg90_dummy_688"),(d684,"sm3sg90_dummy_684")]           # printable bearing dummies: 688 x8, 684 x4
    log=["STL EXPORT v28 (bed-fit + reinforce + servo mounting + cavity + wire routing + model-cut pockets + PIN + L/R MIRRORS) -> stl/","="*70]
    for s,n in parts: log.append(export_stl(s,n))
    log.append("QTY per dog: frame x1 | coxa+coxa_mir x2 ea | femur+femur_mir x2 ea | tibia/tibia_mir/tibia_rear/tibia_rear_mir x1 ea | boot x4 | pin x4")
    log.append("orientation: femur/tibia print FLAT-INBOARD-FACE down; coxa on its bearing face; boot=flexible TPU; rest rigid ABS")
    open(OUT+"/export_log.txt","w").write("\n".join(log))
    print("\n".join(log)); print("OK export")
except Exception:
    print("FAIL:\n"+traceback.format_exc())
for _d in list(App.listDocuments()): App.closeDocument(_d)
