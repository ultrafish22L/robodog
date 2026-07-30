# horns.py -- ONE source for the 3 servo-horn RECEIVER modes (mesh / round / arm), shared by
# coxablock.py (splay axis X) and legflat_v9.py (hip/knee axis Y).  User 2026-07-28:
#   "all servo receivers need 3 ways to print: print in mesh, embedded round, embedded arm."
# Modes:
#   mesh   -- a printed 23T FEMALE spline cut into the solid; no hardware, prints in place.
#   round  -- an OEM ROUND horn (flat disc) EMBEDDED at a print pause: disc chamber + roof captures it.
#   arm    -- an OEM SINGLE-ARM horn embedded the same way: hub pocket + arm through-slot + roof.
# Capture mechanic (every embed): print the part with the SERVO/spline face DOWN on the bed so the void
#   opens UP; PAUSE at the roof-underside layer; drop the horn in; RESUME so the roof prints over it and
#   traps it axially (an M2 centre screw / the servo screw carries pull-off).
# Real OEM arm-horn dims are USER-MEASURED (hub O7.3 x 4.25 tall, arm 12.5 long); the arm blade W/T are
#   the ONE assumed pair (typical SG90) -- verify on the bench before committing the arm STLs.
# EXEC this file (project convention: exec-not-import avoids stale caches across a warm FreeCAD session);
#   the functions/constants land in the caller's namespace and are called bare (horn_void(...), MODES, ...).
import Part, math
from FreeCAD import Vector

MODES = ('mesh', 'round', 'arm')
# back-compat: old callers said 'embed' (canonical, part-specific) or 'spline'.
_ALIAS = {'spline': 'mesh', 'embed': None, 'mesh': 'mesh', 'round': 'round', 'arm': 'arm'}
def resolve(mode, canon):
    """Map a (possibly legacy) mode string to one of MODES. 'embed' -> the part's canonical embed."""
    m = _ALIAS.get(mode, mode)
    m = canon if m in (None, 'embed') else m
    if m not in MODES:
        raise SystemExit("horns: mode %r -> %r not in %s" % (mode, m, MODES))
    return m

# ---- horn dims (mm) ----
SPL_TEETH = 23; SPL_MAJOR = 4.9; SPL_ROOT = 4.4; SPL_TCLR = 0.10   # MESH: 23T female (matches the real servo)
RND_DISK  = 20.0; RND_DT = 1.5; RND_SPL = 7.0; RND_ST = 3.0        # ROUND: O20 disc x1.5 + O7 x3 spline hole
ARM_HUB_D = 7.3;  ARM_HUB_H = 4.25; ARM_LEN = 12.5                 # ARM: hub O7.3 x4.25 + arm 12.5 (USER MEASURED)
ARM_W = 5.0; ARM_T = 1.7                                           # ARM blade: 1.7 thick, FLUSH WITH THE HUB TOP (user 2026-07-29); W still assumed
HFIT = 0.15                                                        # horn drop-in clearance (FDM bores print undersize)

def arm_reach():  return ARM_LEN + HFIT

# ---- axis-generic primitive helpers (D/A are world unit axes: +/-X,+/-Y,+/-Z) ----
def _idx(u):
    for i in range(3):
        if abs(u[i]) > 0.5: return i, (1 if u[i] > 0 else -1)
def _cyl(r, P, D, z0, z1):
    b = Vector(P[0]+D[0]*z0, P[1]+D[1]*z0, P[2]+D[2]*z0)
    return Part.makeCylinder(r, z1-z0, b, Vector(*D))
def _abox(P, D, A, dlo, dhi, alo, ahi, wlo, whi):    # box from local depth(D)/arm(A)/width(W=DxA) spans
    W = (D[1]*A[2]-D[2]*A[1], D[2]*A[0]-D[0]*A[2], D[0]*A[1]-D[1]*A[0])
    rng = [[0, 0], [0, 0], [0, 0]]
    di, ds = _idx(D); rng[di] = sorted((ds*dlo, ds*dhi))
    ai, _  = _idx(A); rng[ai] = sorted((alo, ahi))
    wi, _  = _idx(W); rng[wi] = sorted((wlo, whi))
    lo = Vector(P[0]+rng[0][0], P[1]+rng[1][0], P[2]+rng[2][0])
    return Part.makeBox(rng[0][1]-rng[0][0], rng[1][1]-rng[1][0], rng[2][1]-rng[2][0], lo)
def _big(s):
    q = s.Solids; return max(q, key=lambda k: k.Volume) if q else s

# ---- real OEM arm horn as a solid (servos/hornarm.stl), cached. Local frame: spline axis +Z (Z0=servo/mating
#      face, Z~4.2=hub top), arm +X, width Y, hub centred at the origin. The parametric hub+slot approximation
#      never seated the real blade -- embed the actual mesh so the horn drops in. ----
ARM_STL = r"C:/ultrafish/robodog/servos/hornarm.stl"
_ARM_SOLID = None
def _arm_horn():
    global _ARM_SOLID
    if _ARM_SOLID is None:
        import Mesh as _M
        hm = _M.Mesh(ARM_STL)
        sh = Part.Shape(); sh.makeShapeFromMesh(hm.Topology, 0.1)
        s = Part.makeSolid(sh).removeSplitter()
        try: s = s.makeOffsetShape(HFIT, 0.01)                      # drop-in clearance (FDM prints bores undersize)
        except Exception: pass
        _ARM_SOLID = s
    return _ARM_SOLID

def horn_void(part, P, D, A, mode, floor=0.5, mesh_len=4.5, m2=None, rnd_disk=RND_DISK, slot='through'):
    """Apply the chosen servo-horn receiver at mating-face point P, into-part dir D, arm dir A.
       mode in MODES. floor = arm-hub spline-clearance shelf depth. rnd_disk = round disc OD (per-receiver).
       slot: arm-slot span -- 'through' (both +/-A, captures either orientation; use where fully enclosed) or
             'down' (only the -A side, so the arm points -A = down the bone; no open notch, keeps more flange)."""
    P = tuple(P); D = tuple(D); A = tuple(A)
    if mode == 'mesh':                                             # printed 23T female spline
        rmaj = SPL_MAJOR/2.0 + SPL_TCLR; rroot = SPL_ROOT/2.0 + SPL_TCLR
        part = part.cut(_cyl(rmaj, P, D, -0.1, mesh_len))          # bore at the shaft tip dia
        tw = math.pi*(rmaj+rroot)/SPL_TEETH/2.0                    # tooth width = half circular pitch at mid radius
        for i in range(SPL_TEETH):
            t = _abox(P, D, A, 0.0, mesh_len, rroot, rmaj+0.05, -tw/2.0, tw/2.0)   # tooth seated on +A
            t.rotate(Vector(*P), Vector(*D), 360.0/SPL_TEETH*i)
            part = part.fuse(t)
        return _big(part.removeSplitter())
    if mode == 'round':                                           # OEM round disc, embedded at a pause
        part = part.cut(_cyl(RND_SPL/2.0+HFIT, P, D, -0.1, RND_ST+0.1))            # O7 spline-clearance hole
        part = part.cut(_cyl(rnd_disk/2.0+HFIT, P, D, RND_ST, RND_ST+RND_DT))      # disc chamber (roof = solid beyond)
        if m2: part = part.cut(_cyl(m2/2.0, P, D, -1.0, 100.0))
        return _big(part)
    if mode == 'arm':                                             # EMBED the real OEM arm-horn mesh (the void = the horn)
        import FreeCAD as _FC
        horn = _arm_horn().copy()
        at = (-A[0], -A[1], -A[2])                                                 # arm points -A (down the bone)
        cX = at; cZ = D                                                            # horn +X->arm(-A), +Z(spline)->D(into part)
        cY = (cZ[1]*cX[2]-cZ[2]*cX[1], cZ[2]*cX[0]-cZ[0]*cX[2], cZ[0]*cX[1]-cZ[1]*cX[0])   # +Y(width) -> D x (-A)
        Pf = (P[0]+D[0]*floor, P[1]+D[1]*floor, P[2]+D[2]*floor)                   # hub bottom sits `floor` past the mating face (servo-shaft gap)
        M = _FC.Matrix(cX[0],cY[0],cZ[0],Pf[0], cX[1],cY[1],cZ[1],Pf[1], cX[2],cY[2],cZ[2],Pf[2], 0,0,0,1)
        horn = horn.transformGeometry(M)
        part = part.cut(_cyl(RND_SPL/2.0+HFIT, P, D, -0.1, floor))                 # servo-shaft clearance to the hub bottom
        part = part.cut(horn)                                                      # the real horn shape = the void (roof = solid beyond the hub top)
        if m2: part = part.cut(_cyl(m2/2.0, P, D, -1.0, 100.0))
        return _big(part.removeSplitter())
    raise SystemExit("horn_void: bad mode %r" % mode)
