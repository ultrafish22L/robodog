import sys, trimesh, numpy as np
src, dst = sys.argv[1], sys.argv[2]
m = trimesh.load(src, force='mesh')
# Hunyuan GLBs come Y-up, ~unit scale, arbitrary center. Report so we can place in FreeCAD.
b = m.bounds
size = (b[1] - b[0])
print("verts=%d faces=%d" % (len(m.vertices), len(m.faces)))
print("bounds_min=%s" % np.round(b[0], 4).tolist())
print("bounds_max=%s" % np.round(b[1], 4).tolist())
print("size(x,y,z)=%s" % np.round(size, 4).tolist())
m.export(dst)
print("wrote", dst)
