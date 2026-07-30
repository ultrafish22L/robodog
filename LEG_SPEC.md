# robodog LEG — canonical design spec (single source of truth)

> Read this FIRST before any leg work. Edit lines **in place** when a decision changes — this is a
> spec, not a changelog. The build script's header must mirror this file; if they disagree, this wins.
> Spec version: **v9 · updated 2026-07-27** (direct-drive knee, 0.82× scale, body-through-slot + tab-flat
> mount, mode-consistent knee interface, horn variants).
> Live script: **`legflat_v9.py`** (STLs `sm3sg90_v9_*`). v8 (`legflat_v8.py`, reducing-lever knee) is
> preserved on disk but superseded.

## System (3-DOF leg, SG90-class servos ~22.7×12.1×22.5 mm; some are EXI S1123, bulkier top)
- **servo1 = frame → splay** (hip abduction).
- **servo2 = coxa → hip pitch** (femur swings fore/aft), direct-drive.
- **servo3 = on the femur → knee, DIRECT-DRIVE 1:1** (v9 dropped v8's crank/pushrod/kneecap-lever/688/Ø8-pin
  — halving the moving-part count is the lightness win). It lies flat along the **lower femur** (long axis
  down the thigh ∥ Z), body **outboard** of the femur plate, **output spline pokes laterally INBOARD (−Y) at the
  knee** (servo FLIPPED 2026-07-22; only the body stays outboard); the tibia mounts on its horn from the inside.

## Bones — 0.82× scale, Spot/Unitree proportions kept
- Femur + tibia = flat **7 mm plates** (thin dim = lateral Y), curved sagittal profiles + racetrack
  lightening slots. `BONE=7.0`.
- **Femur 70 mm : tibia 80 mm ≈ 0.87:1** (Spot ratio held through the 245→200 frame scale-down).
- Tibia shin symmetric about x=106, tapering ≈18 → 10 mm; toe arc r4.5. All bores centered on x=106.

## KNEE — direct drive, single-shear on a MANDATORY MG90S
- **The servo3 output boss lands ON the knee axis (XC, KZ)** — `place_servo` seats the boss on the axis via
  `BOSS_DZ`=−5.62 (the boss's Z-offset from the servo CENTRE, measured off the mesh), after `rotate(Y,90)`+
  `rotate(Z,180)`. Before the 2026-07-22 fix the output sat ~18 mm off the knee = the tibia could not be driven.
  The tibia head is INBOARD (Y 28.7–35.7) and the servo body is outboard, so the spline reaches inboard to the tibia.
- Knee load is carried by the **servo output shaft alone (single-shear)** — lightest possible, no separate knee
  bearing/pin. A printed inboard double-shear stub was built and REMOVED (audit 2026-07-27: PLA-on-PLA journal +
  thin bridges = a fresh failure point). Therefore **MG90S (metal gears + output bushing) is MANDATORY at the
  knee, not optional** — the metal output IS the bearing (user decision 2026-07-27).
- **Torque budget is the crux** (this reverses the v8 lever the v7 audit demanded — accepted at the new
  scale): ~0.15 N·m trot-hold vs SG90 stall 0.176 (≈85%, runs hot) / **MG90S 0.216 (≈69%, ok)**. Static
  stand ≈35%. → **recommend MG90S metal-output at the knee**; keep gaits tucked. This is the proven
  cheap-SG90-quadruped formula — it works *because* the dog is small and light.
- Travel **0–70°**, foot swings rear (−X)/up (+Z); collision sweep asserts 0 mm³ over the range.
- Servo output clearance: Ø6.8 bore through the femur plate at K (the shaft passes the plate; the plate
  does NOT bear on it).

## Servo3 mount — body-through-SLOT + offset TAB FLATS (tibia on the inside)
- **servo3 output points INBOARD (−Y)** (user 2026-07-22 flip): `place_servo` does `rotate(Y,90)` then
  `rotate(Z,180)`; the spline tip (bbox YMin) seats 2 mm inboard of the femur inner face (`FIY0`−2), boss on
  the knee axis via `BOSS_DZ`=−5.62. Only the **servo body sticks out outboard** (to Y≈64) → clean outboard
  profile; the **tibia tucks on the INSIDE** against the femur inner face.
- The femur mount (user 2026-07-28 rev: "cap the top, only opening = spline side it's inserted from"):
  a **CLOSED body pocket with a CAPPED TOP** — a housing wraps the servo BODY on ±X, outboard +Y, bottom −Z **and
  the top +Z (2 mm cap)**. The **ONLY opening is the inboard −Y (spline/tibia) face**: the servo slides in
  **LATERALLY** from there, its output spline pokes −Y to the tibia, and the **tibia + M2 centre screw cap the −Y
  opening = retention**. Above the servo the thigh reverts to **SOLID plate** up to the hip. Walls **2.0 mm** all
  round (W=2.6, CLR=0.6; audit 2026-07-28 fixed the old 0.4–1.4 mm walls + 2.15 mm gutted rails → closed box + 3.4 mm
  rails). `servo_mount()`; `FEMUR∩servo3(0.5-clr)=5.1` (real body 3.2, sliver). Femur ≈ **18.3k mm³** (closed box).

## Interfaces
- **Servo horn receiver = 3 modes (mesh / round / arm) via `horns.py`** (user 2026-07-28: "all servo receivers need
  3 ways to print"; **exec `horns.py`, don't import**). `horn_void(part, P, D, A, mode, floor, mesh_len, m2, rnd_disk,
  slot)`: P=mating face, D=into-part axis, A=arm axis. **mesh** = printed 23T female spline (no hardware). **round** =
  OEM round disc embedded at a print pause (bones use **O14** so it fits the O20 boss with a roof; O7 hole + 1.5
  chamber + roof). **arm** (leg CANONICAL = `sm3sg90_v9_*` names) = OEM single-arm horn embedded: hub **O7.3×4.25** +
  arm **12.5** (user-measured), blade **5.0×1.6 ASSUMED** (slot 5.3×1.75 — bench-verify the real blade, it is the
  torque path). Bones pass **`slot='down'`** (single-sided −Z, arm points down the bone → no flange-top notch).
  HIP: `horn_void(FE0,(XC,FIY0-3,HIPZ),+Y,+Z,mode,floor=1.0,mesh_len=5.0,m2=2.7,rnd_disk=14,slot='down')`.
  KNEE: `horn_void(TI0,(XC,TIY1,KZ),-Y,+Z,mode,floor=0.4,mesh_len=2.0,m2=None,rnd_disk=14,slot='down')`.
  Export builds the blank ONCE, applies all 3 → 12 leg STLs (canonical no-suffix = arm; `_mesh`, `_round`). All
  watertight/manifold/1-solid; adversarial-verify clean (wf_6e2a558c). **PAUSE-LAYER (embed): femur 1.0mm (inboard
  face down), tibia 0.4mm (outboard face down); coxa 0.5mm (−X down). Frame + covers = no pause.**
- **Knee engagement (arm/embed): only ~1.3 mm spline grip** on the loaded knee (thin tibia + ~2mm SG90 spline); the
  **M2 centre screw carries pull-off** as designed — watch it. Fallbacks: tibia `round`, or a knee bolt-through.
- (KNOWN mesh caveats, unfixed — arm is what ships: tibia mesh has ~1.16mm³ servo-boss interference (needs the O7.3
  counterbore round/arm have); 23T printed spline may not press onto a real 20–21T SG90 output.)
  The **M2 centre screw** (O2.7 clearance bore) through the head into the output boss clamps the tibia axially onto
  it — it, not the shallow spline, carries pull-off. M2 self-taps into the SG90/MG90S boss; **M2.5 would split the
  O4.9 boss** (code comment + audit 2026-07-27). It is a **running-clearance coupling** (bore 0.1 mm over the shaft,
  teeth nest) so `common()=0` is CORRECT, not a gap. Tibia face 0.3 mm off the case = max grip.
- Both hip AND knee take the **`--horn=embed | spline`** switch (whole leg is horn-consistent).
- **embed** (default) = OEM SG90 cross-horn capture + M2 centre screw; canonical STL names
  (`sm3sg90_v9_{femur,tibia}[_mir]`). CONTROL STACK REGENERATED from v9/dog14 (2026-07-28): `kinematics.py`
  now SP(0,27.2,10.398)/HP(82.65,0,10.398)/K(82.65,0,-59.602) femur 70, positive-knee flex (0-70), foot0 from
  the v9 ball foot (-150.6); `gen_urdf.py` points at `sm3sg90_v9_{frame,femur,tibia}[_mir]`+`sm3sg90_coxa[_mir]`
  +cover, single tibia (no front/rear split, no boot), and applies the T_COXA/T_LEG raw->body shift +
  convention-agnostic mirror-plane; `robodog.urdf` loads clean in PyBullet, all 4 corners' meshes align to the
  FK feet to 0.000 mm. Regen after any CAD change: `scratchpad/dump_v9_axes.py` -> update the 5 constants -> rerun gen_urdf.
- **spline** = printed **23T female spline** receiver (SPL_MAJOR 4.9 / SPL_ROOT 4.4 / SPL_TCLR 0.10, teeth
  ≈0.32 mm — below a 0.4 nozzle, print-unproven; user's explicit priority). Files get a `_spline` suffix and
  coexist in `/stl`. Params MUST stay equal to `coxablock.py`'s.

## Hip mount (femur ↔ coxa) — VERIFIED same x-z frame
- Coxa hip-pitch servo output axis = **X=106, Z=10.3984, +Y** (`dogcommon.spline_axis("s1")`) = femur hip.
  `legflat_v9.py` builds at nominal `HIPZ=10.0` and applies a **rigid `HPDZ=0.3984` lift at export** (same
  contract as v8): translating the whole chain keeps every bone shape bit-identical, whereas re-datuming
  `HIPZ` would deform the profile. Keep `HPDZ` equal to `dog13.HPDZ`.
- The coxa presents its bare SG90 spline through the cover opening; the femur rubs **directly on the cover's
  flat outer face `FACE_OUT` ≈ Y 35.1** (the RING insert is gone — COXA_SPEC R8). Femur hip = **Ø20 rub
  flange + horn interface** (embed or spline per the switch) on the inboard face.
- Assembly = translate the leg +Y so the flange seats on the cover face. No floating spline.
- **EXI caveat**: an OEM horn must clear the ~12×16 EXI top block poking through the cover opening — verify
  on a test print (SG90 fine).

## Lateral Y-layer stack (inboard → outboard), leg-local — FLIPPED (tibia inside)
**tibia Y[28.7,35.7] | (0.3 clr) | femur plate Y[36,43] + tab flats to Y45 | servo3 body Y[≈38,64], output
spline Y[34,36] on the knee axis (poking inboard through the plate slot)**. The tibia grips the spline just
inside the femur inner face (Y35.7 vs 36 = 0.3 clr, max engagement).
Leg width tibia-inner(28.7)→servo-outer(64) ≈ **35 mm**; the outboard face is servo-only (clean).
⚠ Trade-off: the inboard tibia costs frame clearance in EXTREME poses — see the decision log.

## Kinematic frame (v9 — current, verified against `legflat_v9.py`)
Sagittal x-z, +x = forward, +z = up, mm. Hip **H=(106,10)**, Knee **K=(106,−60)** (femur 70), toe (106,−140)
(tibia 80). servo3 axis **= the knee axis** (direct drive — no separate S, no linkage params).
Build stations are at nominal `HIPZ=10.0`; the export lift puts the shipped STLs on the measured axis.

## Manufacturing
- Every printable part = **single watertight/manifold STL** → `C:/ultrafish/robodog/stl/`, named
  `sm3sg90_v9_{femur,tibia}[_spline][_mir]`; **L/R mirrored** (chiral legs); fit Bambu 256 bed.
  Femur+tibia mirror about a **shared** bbox centre (`MIRC`) so the pair stays in one frame.
- PLA test. **FOOT**: plain **pointed blade toe** (the O7 ball foot was REMOVED per user 2026-07-28); foot0 z −144.1.
  A press-on TPU cap can still be added later for traction if the bare PLA toe slips.
- Frame: `framemin.py` → `sm3sg90_v9_frame.stl` (200 body + O16 splay-pin collars = 206 mm X, ~122 g). Corner blocks
  carry the coxa (R16 COXA/CLIP)/servo0/splay-dowel; bulkheads raised (BHH 28) for torsion. The leg mates the coxa only.
  **BUILD (splay dowel):** the O7.9 bore now carries a COARSE INTERNAL THREAD (P2.0 × 0.7 deep, ~13 mm) so a
  threaded/self-tapping O8 pin **screws in and self-retains** (user 2026-07-28); the O3 set-screw is a secondary
  lock. Ream/chase the printed thread before the first insert. **BELLY = ONE continuous chamfer (user 2026-07-28
  "match the head/rump chamfer on the belly, one larger chamfer"):** a single 45° bottom-outer chamfer runs the FULL
  length head→rump — wall at (BY/2, ZTOP=−11) down-and-in to (BY/2−DEEP=−12, zbot). It doubles as the **toe-in
  clearance**: a toed-in leg still clears at the hips — **~−30° toed-in gait = 0.0 mm³, LOCKED; −45° needs a slimmer
  femur housing.**
- **CLOSED ELECTRONICS TRAY (user 2026-07-28 "add webbing that closes the tray + redo the weight cutouts; hold the
  electronics safely; snap-on body eventually"; design-panel + adversarial-verify):** the tray (X±43.6 × Y±32.7 ×
  ~41 deep, OPEN TOP) is now a closed basket. (1) **Side walls** = a continuous **SKIN=1.2 mm inner skin** (the
  through-Warren truss is gone) with the outer 1.8 mm hollowed as a blind pocket braced by **4 external ribs/side**
  over z[tz0=−13, tz1=14.7]; solid **6 mm rim cap** above 14.7 for the snap body; keel solid below. (2) **Floor** =
  windows DELETED (solid **1.6 mm** shear panel; FLOORT 2.0→1.6 pinned the floor-top so only the belly rose 0.4 mm =
  more toe-in); zip-tie slots kept. (3) **KEEL WEB** = a **KWEB=1.4 mm** skin just inboard of the chamfer plane, X to
  ±(cbx0+0.5) so it ties floor→rail the full length + into the corner blocks (the chamfer had left a full-length gap);
  fused BEFORE the chamfer cut → trimmed flush, zero exterior/toe-in change. (4) **Battery corral** = 2 longitudinal
  floor ribs at Y±14 + a fore end-stop at X+30 (generic retention with the kept zip-ties). **Verified:** walls
  inner-skin 1.00, floor flat-zone 0.96 (zip-slots only, 0/768 holes), keel sealed incl. corners, upper-tray
  intrusion 0, toe-in/working/servo0 = 0, 1-solid watertight manifold, **122 g (3 g LIGHTER than the open version)**.
  Board-specific standoff bosses + a wall-to-wall diaphragm were **deferred** (board not chosen; diaphragm corner
  would float in the keel void). Top rim z=20.7 + outer face Y=±35.7 (top 6 mm) + corner-block tops left CLEAN =
  snap-body register (body designer: seat on the z=20.7 rim + Y=±35.7 face, anchor snaps at the 4 corner-block tops).
- **servo3 lead (no printed features on purpose):** the bone is a 7 mm flat plate — **zip-tie the lead
  around the plate** (2 ties up the thigh), cross the hip **near-axis over the flange top** with a small
  service loop, drop into the coxa's top wire slot. Same near-axis rule as v25/v26 wire routing.

## FreeCAD traps (do not relearn)
- `biggest()`/`max(Solids,…)` silently drops disjoint islands → guard with solid-count + volume asserts.
- Self-intersecting outline polygons → invalid faces → silent geometry drop → validate every fuse.
- Coplanar-touching solids don't fuse.
- `execute_code` returns a ~25k-token render → batch calls, read text only. When the MCP refuses connections
  the GUI is closed → build headless via `freecadcmd.exe` (Gui tails guarded with `App.GuiUp`).
- `slot(cx,z1,z2,r,…)` (racetrack helper) extends **±r past z1/z2** (disc caps) — pass z1−r/z2+r for a true
  extent (this nearly swallowed the mount pilots once).

## Verification (every build)
1. Knee fold sweep 0–70° (10° steps): tibia∩femur = 0 AND tibia∩servo-envelope(+0.5 clr) = 0.
2. Both parts valid single-solid; all STLs watertight/manifold/no-self-intersections (checked via `Mesh`).
3. L/R mirrors export to `/stl`; embed set keeps canonical names for `gen_urdf`.
4. Render before showing a pose — check fold direction (foot rear+up).

## Decision log (WHY only — never live values)
- 2026-07-27: **mount = body-through-slot + tab flats; knee connection made real** (user: "tibia still doesn't
  actually connect to the servo spline. femur servo body should fit thru a slot with offset flats for the tabs
  to sit against, no side wall like now"). Two root causes found by probing the SG90 mesh: (1) the old servo
  slot used **side rails**, not the panel-mount the user wanted → replaced with a body-through-slot + offset
  tab flats (a short knee tang carries the lower flat, which sits below the pivot). (2) "doesn't connect" was
  real for **embed** mode — the tibia captured the OEM cross-horn but that horn isn't in the STL, so tibia↔
  (missing horn)↔spline rendered as a gap; the knee now runs the **same `horn_iface` switch as the hip**, so
  embed = OEM-horn capture and spline = printed female gripping the O4.9 shaft DIRECTLY. The male spline is
  only ~2 mm (measured Y[34,36], flush to the case) → engagement maxes at ≈1.7 mm; the **M2 centre screw**,
  not the spline, carries pull-off (normal SG90-horn practice). It is a running-clearance coupling so
  `common()=0` is correct. Working-range & coxa gates still 0.00; full-envelope extreme trade-off unchanged.
- 2026-07-22: **knee FLIPPED — tibia to the inside** (user: "flipped so tibia is on the inside … servo slot
  shift outward so the spline is aligned for the tibia to be very close to the inside of the femur"). servo3
  output now points inboard, servo body sticks out outboard only (clean profile), tibia tucks against the
  femur inner face gripping the inboard-poking spline; femur mount reduced from a full cradle to a slide-in
  slot ("doesn't need full walls" — later hardened to a 2 mm closed box per the 2026-07-28 audit). ⚠ **Trade-off
  surfaced by the gate**: the leg (servo3 housing/tibia) hits the frame corner only when **positive pitch meets
  toed-in (negative) splay** (worst 708 mm³ at splay−25/pitch+45/knee70, + the pitch+90 extremes). With pitch≤0
  the leg swings AWAY from the frame, so the **whole default-gait envelope** (splay±25, pitch−58…−20, knee 41–70)
  is **clear (0.00)** and the dog stands+walks in PyBullet. Enforced by `kinematics.in_working_envelope` (coupled
  pitch×splay rule) + `poses.validate()` self-collision backstop. fold 0–70° 0, FEMUR∩servo3 5.1 (sliver).
- 2026-07-22: **"design doesn't actually work" pass** (user). Three fixes made it real: (1) frame servo0
  pockets get 2 mm walls all round bar the +Z insertion end (framemin); (2) the femur gets a servo cradle,
  not a flat-plate screw mount; (3) the tibia gets a spline receiver gripping the output directly. Probing
  for #3 exposed the root cause: **the servo3 output was 18.5 mm off the knee axis** (at −41.5, knee at −60),
  so nothing the tibia held could grip it — the leg was undriveable as drawn. Fixed by dropping the servo so
  its output boss lands on the knee (BOSS_OFF); user chose this over moving the knee, to keep femur 70/tibia
  80, accepting a ~10 mm servo overhang below the knee. All rebuilt: FEMUR∩servo3 0, fold 0-70° 0, assembly
  gates 0.
- 2026-07-21: **HPDZ lift restored in v9 export** — first v9 export shipped at nominal z and silently broke
  the v8-era STL contract (gen_urdf mates on the measured axis). The lift is a whole-chain translate, so the
  fold check (done unlifted, self-consistent) needed no re-run.
- 2026-07-21: **servo3 mount = M2 through the tabs into the plate** (tab holes happen to run along the plate
  normal — no bracket needed, the femur is the bracket). c-c 27.8 chosen to split the 27.5/28.0 clone spread
  rather than trust any one datasheet. Only after the mount was real did the body-footprint window go in.
- 2026-07-20: **v9 = direct-drive knee + 0.82× scale** (user: "ditch the tibia lever", frame → ~200 mm,
  femur 70/tibia 80 chosen via question). Reverses the v8 reduction the v7 audit demanded — accepted
  knowingly at the smaller scale; mitigation = MG90S at the knee + tucked gaits. Lightness beats ratio.
- 2026-07-20: **horn variants unified robot-wide** (user: leg parts "get the same embedded or printed spline
  recievers" as the coxa). 23T printed spline kept available despite the sub-nozzle tooth caveat — user's
  call, printability objection recorded.
- 2026-07-20: bones thinned **8 → 7 mm** (user). Linkage-facing faces held fixed so the mm came off open air;
  collision sweep stayed 0; mirror centre unmoved. ("No fork" was already true — v8 was single-shear.)
- 2026-07-11: (v8, superseded but kept for why-history) parallelogram → **reducing lever ≈2.5:1** to buy
  knee-torque margin; capped by SG90 travel. v9 traded that margin away for part count at smaller scale.
- 2026-07-11: servo3 kept **on the leg** (not body-mounted) — decoupling is automatic; fits the coxa build.
- 2026-07-19: hip datum moved onto the **measured** servo1 spline (z 10 → 10.3984), applied as a rigid lift
  at export, never by re-datuming `HIPZ` (that deforms the bone — profile stations are absolute z).
- 2026-07-19: coxa RING insert deleted (COXA_SPEC R8) → femur rubs the cover face directly, 1.5 mm inboard
  of the old rim. Any dy derived from the ring is wrong by 1.5 mm.
- 2026-07-11 (v7): user audit → single-blade + centered bores + Spot proportions; printed-spline-only horns
  rejected for driving torque (hence the embed default today).
