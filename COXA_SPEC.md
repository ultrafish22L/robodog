# COXA SPEC — canonical source of truth

**Read this first before touching the coxa. Edit it IN PLACE** — it is a spec (current state),
not a changelog. Only the short decision log at the bottom records *why*.

Built by `coxablock.py` (subtractive-from-block). Supersedes `dog13.py`'s `shoulder()` **for
printing**; `shoulder()` is still the live `SH` used by the assembly/gates/renders and by
`frameblock.py`, so it stays until coxablock replaces it there too.

---

## Parts — 2 printable, ×2 chirality = 4 STLs in `/stl`

| STL | vol mm³ | role |
|---|---|---|
| `sm3sg90_coxa.stl` + `_mir` | ~18.5k | the **solid block**: servo1 pocket, tab channel, wire slot, splay bearing cup, splay horn — **R17 solid end housings (2026-07-29, un-did the R16 barrel strip; solid walls on the servo ±X faces)** |
| `sm3sg90_coxa_cover.stl` + `_mir` | 4238 | black snap cover: closes the +Y mouth, cages the servo, **is the femur bearing face** (was 5433; shrinks with the seam profile) |

All four (plus the `_spline` pair, coxa 9152) verified on disk: watertight, manifold, no self-intersections —
the export now runs export.py's `repair_clean` write-re-read loop, and the coxa build FAILS (no cover STL)
if the cover diverges from the seam profile.

### Horn variants — 3 modes (`--horn=mesh|round|arm`) via `horns.py` (user 2026-07-28)

The −X splay horn end is built in one of THREE modes via the shared **`horns.py`** (`exec`'d, not imported;
`horn_void(COXA,(hrn_face,SPY,SPZ),(1,0,0),(0,0,1),MODE,floor=0.5,mesh_len=HRN_DT+HRN_ST,m2=None)`). Back-compat:
`embed`→`round`, `spline`→`mesh`. **The COXA alone differs; the cover is horn-independent, exported once (canonical
round run only).** The block genuinely rebuilds per mode (R_END + the chamfer envelope depend on the horn footprint).

| mode | what the −X horn end gets | STL names |
|---|---|---|
| `round` (canonical) | O20 disc chamber (1.5) + O7 spline hole + roof (**3.5mm WSRV wall**) — OEM round horn dropped in at a print pause | `sm3sg90_coxa[_mir].stl` — the **canonical** names |
| `mesh` | printed 23T female spline (prints in place, no hardware) | `sm3sg90_coxa_mesh[_mir].stl` |
| `arm` | O7.6 hub pocket (4.25) + arm through-slot (12.5, ±Z so it clears the cover seam) + roof (1.75mm to servo1); M2=None (would breach servo1) | `sm3sg90_coxa_arm[_mir].stl` |

- **PAUSE-LAYER (embed): −X face DOWN on the bed, pause at 0.5mm** (round: at the disc-chamber floor / arm: at the
  arm-slot shelf), drop the OEM horn hub-down, resume — the WSRV wall roof captures it.
- `round` (canonical) must keep the exact filenames: **`control/gen_urdf.py` requires `stl/sm3sg90_coxa_mir.stl`**.
  `mesh`/`arm` write suffixed alongside. Verified watertight/manifold/1-solid + adversarial-clean (wf_6e2a558c).
- The receiver is **parametric, not a mesh**. A donor STL (`MicroServo_9g_Gear_Base_V4`) was tried and
  abandoned: `makeShapeFromMesh` gives one face per facet, and at 1448 facets the fuse ran >196 s and
  timed out the GUI. Decimating to 970 facets **inverted the normals** (negative volume → fused to
  nothing); harmonising fixed the sign but left 6% of the volume with the teeth gone. Parametric teeth
  build in ~40 s and are exact. Do not reach for the mesh again.
- **`SPL_TEETH = 23` must equal the real servo's spline** (user, 2026-07-19). ⚠ At 23T the teeth are
  ~0.32 mm wide × 0.25 mm deep, **under a 0.4 mm nozzle** — dimensional match was chosen over
  printability, and this variant is UNTESTED on the printer. `embed` remains the default for a reason;
  LEG_SPEC's v7 audit found a printed spline cannot drive a real servo.

There was a third part — `sm3sg90_coxa_insert.stl`, the yellow slip-fit femur ring — **deleted 2026-07-19**
(decision log R8). coxablock now also **removes any stale insert STL from `/stl`** on each run, so a
retired part cannot linger in the slicer set.

Both: **valid, single solid**. Mirrors are reflected about **each part's own bbox centre** —
that is `gen_urdf.mesh_world_placement`'s contract (it then applies `T(0,-2c,0)`). Mirror export is
validity-gated: a mirror is only written if `isValid() and solids==1 and vol==orig`.

## Axes — MEASURED, never idealised

Both hip axes come from the **scanned** servo meshes via `dogcommon.spline_axis()`, never from the
idealised boss cylinders (on the EXI-style case the spline is **not** centred in the bulky top block,
so those bosses sit 0.398 low).

- splay (X) axis @ `(y 20.0, z 10.3984)` = `SPY, SPZ`
- hip-pitch (Y) axis @ `z 10.3984` = `HPZ`
- **skew 0.0000** — the two axes intersect, i.e. a proper 2-DOF hip. (They were 2 mm skew before 2026-07-19.)
- `S0DZ = -2.000` drops the servo0 pocket so servo0's spline lands on the splay axis.

## Block

- extents `X[93.0,119.9] Y[4.7,31.0] Z[-16.8,26.1]`, **2 mm wall**, front +Y **open** at y=31.0
- back wall **2.0 mm** (`BACK=WALL`) — the face opposite the clip is pulled in as far as the minimum
  wall allows. This is the binding constraint on the big chamfers; see Edges.
- servo1 pocket cut with the **scanned mesh** (`servo_cut("s1")`), front-insert, +0.25 mm/side
- **`POCKD = 1.5`** extra pocket depth: a pure relief cut (`PDEEP`) behind the scanned servo body, so the
  pocket floor sits at y 6.72 instead of 8.22. It does **not** move the servo datum — `SPY`/`SPZ` stay put;
  it only lets the servo sit 1.5 mm deeper. `by0 = sb.YMin - POCKD - BACK`, so the back face follows the
  floor out to y 4.72 automatically and the 2.0 wall is preserved.
- ⚠ `PDEEP` is sized from **`sbody`** — the servo BODY cross-section sampled at the servo's back face
  (`S1.common(<1 mm slab at sb.YMin>)`, 13.00 × **23.25**) — **not** from `sb`. `sb` is the bbox of the whole
  scanned mesh and its Z span (**32.96**) is the MOUNTING-TAB span; the tabs sit at the FRONT (`y>=24.58`),
  while at the pocket floor the servo is only its base. Sizing the relief from `sb` gouged it 9.71 mm taller
  than the base over the full 1.5 mm depth — a gap at the bottom of the pocket shaped like the tabs. See
  decision log R9. The `feats` chamfer-clearance envelope carries the same split: tab-height servo box +
  a separate base-height box for the deepened region.
- tabs at `Y>=24.6` → front channel `Z[-11.9,21.2]` clears them
- wire slot 4 mm wide `X[104.0,108.0]`, `Y[9.2 → 31.0 face]`, down to `Z10.3` (past the wire nub)

## Splay end (+X) and horn end (−X)

- **SOLID SQUARE END HOUSINGS (R17, 2026-07-29)** — the R16 barrel strip-to-minimum was REMOVED. It cut
  everything outside a `R_END≈12.6` cylinder from the two end bands, trimming both WSRV=3.5 servo-face walls
  to a Ø25.2 disc and opening the servo ±X faces as windows + freestanding webs (user: *"no actual wall
  against the side of the servo, lots of free standing flimsy walls"*). Now COXA is the **full solid block**
  cut only by its functional pockets, so a **solid 3.5 mm wall bears directly on the servo's bearing(+X) and
  spline/horn(−X) long faces**; the bearing cup + horn disk/spline are blind bores INTO the solid ends. Chamfer
  dropped 6→3 (`CHAMF_MIN` raised to 3 so the tight back-wall edge stays sharp, not a corrupting sliver).
  COXA vol ~9047(R15)→10307(R16 stripped)→**~18.5k (solid)**. The mid band still carries servo pocket, snap-hook
  corridor, wire slot, seam. **Export note:** the full-length solid ends tessellate with corrupt-chamfer boundary
  dropouts that FreeCAD's `hasNonManifolds()` misses; `_repair_clean` now `fillupHoles()` **last** → watertight/
  manifold/1-solid, valid volume (trimesh-verified; FreeCAD's residual self-int flag on the coplanar fills is
  benign → label "sealed"). The lofted cover re-matches the fuller seam automatically.
- **`WSRV=3.5`** (was 2.0, user 2026-07-29 "servo long-face walls too thin"): the wall on **each** servo long
  face (−X horn side + +X bearing side). Block X width **26.9 → 30.0** (+3mm splay span); hrn_face 92.95→**91.45**,
  bearing/block +X out +1.5. **The frame follows automatically** (pocket grown from COXA; splay pin from `brg_x0`)
  EXCEPT servo0, which `framemin` now tracks via `DHORN = 92.95 - hrn_face` (shift servo0 + pocket −X so the splay
  spline stays engaged, foot/URDF unchanged). All X coords below are the OLD WSRV=2.0 values → add the shift.
- **bearing** 688 Ø15.94 × 5.4, cup open on +X, wall to servo **3.5**
- **crush ribs** on the cup lead-in stand **0.10 proud** — see trap #1
- **horn** (−X): disk Ø20.0 × 1.5, fully enclosed (wall to servo **3.5**);
  spline Ø7.0 × 3.0 protruding. Print-in-place via pause-drop (worked in PLA).

## Cover (CLIP)

- flat cover `Y[31.0,35.1]`; **built in BOTH horn modes** (its hook pockets shape the COXA — skipping it
  desyncs the variants) **but exported only by `embed`**: the GEAR fuse perturbs the edge passes enough
  that the two "identical" covers differ, and only the embed one tessellates watertight (R16).
- the seam profile is the **fused whole seam REGION** (0.2 slab + mouth/notch patch solids → single +Y
  face → OuterWire), not the largest front-face fragment — since the end trims the front face is
  fragmented, and the largest piece is a bearing-end patch, not the silhouette. The seam check is FATAL:
  a diverging cover never reaches `/stl`.
- the **bottom snap hook fuses unfilleted** — its rounded variant tessellates self-intersecting slivers at
  the gusset root corners (the same fuse that flips the BREP invalid), which mesh repair could only clear
  by punching holes. Top hook keeps the rounding.
- **no inner-face pocket**: with the `BEAR_OD` disk, tab roots and hook bands kept solid, a 2 mm-wall
  pocket only reaches ~0.2 g of material, and its walls graze the end arcs tangentially (self-x again).
  The cover's strip is the profile shrink it inherits from the coxa end trim (5433 → 4238).
- **FLUSH** chunky snap: 3.5 mm beam ×14, recessed flush (block grew `CLIPGROW=3.0`/side,
  wall-to-servo 1.50) + flush lateral gusset +3.0/side; leg 6.1 mm deep × 3.5 thick, nub r1.0;
  2-tab retention, tabs → `Y27.1`. Strain 0.98 %, no thin parts.
- output opening = rounded-rect **MEASURED off the servo's top block**, not hardcoded: sample the servo
  across the cover's Y band (skipping the first 0.3 mm, where the section is still the servo BODY
  13.00 × 23.25) → top block **11.96 × 14.56**, grown by `OPEN_CLR = 1.0`/side → **13.96 × 16.56 r3**.
  Centred on the **BLOCK** (105.95, 8.99), *not* on the spline — the spline sits 1.41 mm above the block
  centre, so the old spline-centred 15 × 18 left only **0.31 mm** clearance at the bottom and 3.13 at the
  top. Now 1.00 mm all four sides; `CLIP ∩ servo1 = 0`.
- **silhouette matches the body exactly**, by construction rather than by re-deriving edges: slice COXA in
  a thin slab at the seam, fuse the two hook *cutters* (the same `g=0.35` that made the rebates) to fill
  the rebate notches back in, extrude that profile forward and cut away everything in the cover's Y band
  outside it. The cover was previously built from the **largest fragment** of the coxa +Y face — the snap
  rebates split that face — then squared back up with a butt box and two square trims, so it came out as
  the raw block rectangle (CLIP Z[−16.83, 26.13] vs the coxa front face Z[−15.33, 26.04]).
  The cut is confined to `y >= FRONT` so the snap arms behind the seam are untouched; an earlier attempt
  clipped the cover over its whole Y range and fragmented it into 5 solids.
  Verified: **0.000 mm³** of cover material outside the coxa profile, and the top-edge X-extents agree with
  COXA at every z (96.26/115.45 at z 25.93 → 92.95/118.85 at z 22.13), i.e. the chamfer taper carries
  across the seam. It also narrows the finger/gusset top where it overhung the chamfer — what the user
  asked for, and desirable anyway since a proud corner there has nothing to grip.
- ⚠ Naming trap: `_slab` is the **hook-builder helper**. Shadowing it with a solid makes `_hook()` raise
  `'Part.Solid' object is not callable` from a frame that points at `_hook`, not at the shadowing line.

## Femur bearing face (was the ring insert)

- **no separate insert**: the femur runs directly on the cover's own flat outer face at `FACE_OUT`.
- `BEAR_OD = 24.0` — the disk around the output opening kept **solid** (excluded from the hollowing
  cut) so the femur bears on solid material, not on the `CWALL` 1.6 shell. It is the only surviving
  trace of the old ring and it still drives the edge-rounding guard.
- Measured: **4.12 mm** of solid backing clear of the servo footprint, **3.52 mm** over it (the `SKIM = 0.6`
  servo-top relief). The old build had only ~1.52 mm of backing plus a loose 1.5 mm proud ring.
- The servo-top relief is cut with the **scanned servo itself** (`S1`, already grown `SCLR`/side) restricted
  to the skim band — exactly "as wide as the servo" and nothing more (**104 mm³**). It used to be a box
  `sb.XLength+2 × sb.ZLength` = 15.00 × **32.96** = 296.6 mm³: 2 mm too wide *and* the full MOUNTING-TAB
  span, though the tabs are nowhere near this plane. Same `sb`-vs-body error as `PDEEP` (R9).
- ⚠ The femur's rub plane moved **1.5 mm inboard** — it used to sit on the ring's proud rim at
  `FACE_OUT + 1.5`, it now sits on the cover face at `FACE_OUT`. Nothing in the repo consumed that
  standoff, but it is a real change to the leg's Y datum: check it at assembly.
- Trade-off accepted: the ring was a swappable wear part and a thrust-washer seat. PLA now rubs
  directly on PLA. If drag or wear is a problem, the fix is a counterbore in the cover for a hobby
  thrust washer — not a return to the loose ring.

## Edges

`CHAMF = 6.0` on every outer **block** edge that has the clearance headroom, then a **graded
fallback** giving each remaining non-clip edge the largest chamfer it can take (quantised to 0.5),
then `ROUND = 1.0` fillet on the remaining exterior edges. `CLR = 2.0` min wall to any internal feature.

- The **+Y clip-mating face is entirely sharp** — `on_front` guards *both* the chamfer and the fillet
  pass. A chamfer there would open a gap under the cover and make one clip edge differ from its
  neighbours. Its only non-flat neighbours are pre-existing horn/bearing pocket walls (79.3deg off Y,
  |ny|=0.186 — steep walls, NOT bevels) and the terminations of fillets on adjacent non-clip edges.
- **Zero** non-clip block edges are left sharp — verified by re-deriving the sharp set from the finished
  solid after the pass, not by trusting the pass's own report.
- The graded pass runs as a **sweep (up to 4 rounds)**, re-deriving its target set from the *current* solid
  each round and stopping when a round chamfers nothing. One pass is not enough: chamfering an edge
  **shortens the edges it meets**, so their centres of mass move and `edge_op`'s per-edge re-find (which
  matches on the centre captured before the batch, tolerance 1e-4) silently skips them. See trap #6 and
  decision log R11. The printed tally reports what was actually cut across all rounds, not round 1.
- `BACK` and `CHAMF` compete for the same material: at `BACK=2.0` the back edges sit close to the
  CLR-grown envelope, so they grade down (5.0 x4, 4.0 x1, 3.0 x2, 2.0 x1) rather than take the full 6.0.
  Restoring `BACK=8.0` would let everything take 6.0 but costs 6 mm of depth. Depth won, by decision R6.
- The horn pocket is grown by **`HORN_CLR = 0.6`**, not the global `CLR = 2.0` (decision R13). At the
  full `CLR` the Ø20 disk crowded the −X end and starved ONE side bevel to 3.5 while its neighbours ran
  at 6.0. The pocket is a captive drop-in for an OEM horn, not a cavity the chamfer can breach, so it
  does not need the full wall. The disk is recessed 3 mm behind the face, so the 45° chamfer stops at
  z 23.13 vs disk top 20.4 — the pocket roof survives.

## Clearances — expected values, do not re-hunt

- `COXA ∩ servo1 = 28.18 mm³` in **both** horn modes — **scan noise, not a breach.** `S1` is a scanned
  mesh with ~0.2–0.3 mm surface noise, so `common()` against a clean BRep always leaves slivers wherever
  a face runs at nominal clearance. Measured: **4 disjoint solids, mean thickness 0.267 mm**, sitting
  exactly on the Ø13 output-slot footprint. A real interference is ONE contiguous solid with depth.
- The build therefore asserts on **depth, not volume**: `SERVO_CLR_TOL = 0.35` mm per solid. Exact zero
  is unachievable against a scan — an earlier 0.000 reading was luck, and chasing it back cost two wrong
  "fixes" (see R13). If this ever trips, measure the intersection **bbox and solid count** before
  changing any geometry: a number without its shape is not evidence.
- `COXA ∩ servo1 pocket wall = 0` where it matters; horn disk 5.73 mm to block top, bearing cup 7.76.

## OPEN

1. **Re-print the set in PLA** (3rd attempt) — now only 2 parts. Confirm the flush chunky clips survive
   support removal, and check how the femur runs on the bare cover face (drag, wear, slop) now that the
   ring is gone. Dial fits from there.
2. ~~`RING ∩ CLIP ≈ 37 mm³`~~ — moot: retired with the ring (R8).
3. Eventually retire `dog13.shoulder()` in favour of this block.
4. The two `/stl/*.3mf` slicer projects (`sm3sg90_coxa.3mf`, `sm3sg90_coxablock.3mf`) predate R7/R8 and
   still contain the insert. They are the user's slicer files, not build output, so coxablock does not
   touch them — **re-import the STLs before the next print.**
5. **Dead-code sweep is INCOMPLETE.** R10 asked for "clean it all up, remove old dead code". Done by hand:
   stale file header, the superseded SIMPLE-HOOK-CLIP and MATCHING-CHAMFER comment blocks, the ring-deletion
   parenthetical, the unused `shape` param on `_oplane`, the duplicated `earY` probe, and the boss-walk
   (now `_boss_top()`, so its six scratch shapes stop leaking into the shared session globals). An
   exhaustive multi-lens audit was launched but was killed before producing findings — **it has NOT run**,
   so more dead code may remain. Resume it, or re-audit, before treating this as closed.

---

## Traps — do not relearn

1. **Never key "is this an outer face" off a live `BoundBox`.** `_oplane()` used to compare against
   `shape.BoundBox`; the +X cup crush-rib lead-in stands 0.10 proud, so `XMax` read 119.95 while the
   +X face sits at 119.85 — past the 0.06 tolerance. The **whole +X face** therefore failed the test,
   every edge on it was invisible to `blk_edges`, and the back came out chamfered on 3 sides and sharp
   on the 4th. Fix: test against the **block extents** (`bx0..bz1`), which are design intent and don't drift.
2. `makeFillet` / `makeChamfer` / `fuse` can **"succeed" yet return an INVALID spike that silently
   drops geometry**. Validate every edge op (`isValid` / `Solids` / `Volume`) and revert to the
   pre-op solid. All edge ops go through the guarded `edge_op()` helper.
3. FreeCAD's **self-intersection remover HANGS** — avoid it. `.fix()` (ShapeFix) repairs the hollow
   cover BREP fast and is the right tool.
4. The **EXI S1123 has a bulkier top block than a stock SG90** (confirmed by photo + scan). Anything
   sized to the SG90 taper will foul it.
5. `servo_cut("s0")` is built in the FRAME's **post-yshift** y; `servo_cut("s1"/"s2")` are **pre-yout**.
   coxablock works pre-yout → subtract `YSHIFT` for s0 only.
6. **`edge_op`'s per-edge fallback re-finds edges by a CenterOfMass captured BEFORE the batch.** Chamfering
   an edge shortens its neighbours, moving their centres, so those neighbours miss the 1e-4 match and are
   **silently skipped — the pass still reports success**. Any caller that chamfers a set of mutually
   adjacent edges must sweep (re-derive the target set from the current solid and repeat) rather than
   trusting one pass. Corollary: **verify edge passes by re-measuring the finished solid**, never by the
   pass's own log.

## Decision log (why only — never live values)

- **R17 (2026-07-29)** — user: *"there's still no actual wall against the side of the servo on the sides with
  bearing and spline receiver, just build a wall right into them… much closer to what we had before all the
  recent 'optimizations'… lots of free standing flimsy walls."* **Reverted R16's barrel strip** (the root cause:
  R_END trimmed the servo-face walls to a Ø25.2 disc, opening windows). COXA is now the full solid block; solid
  3.5 mm walls bear on the servo bearing(+X) + spline(−X) faces. Prior in the same session: WSRV 2→3.5 widened
  both walls (frame servo0 tracks via `DHORN`); CHAMF 6→3. Two implementation notes: (1) `CHAMF_MIN`→3 leaves the
  tight back edge sharp (the graded sliver-chamfer was NOT the mesh corruption, but sharp = more solid = on-intent);
  (2) the real corruption is corrupt-chamfer boundary dropouts on the full-length ends that FreeCAD's manifold
  check misses — `_repair_clean` now `fillupHoles()` last (trimesh `is_volume` confirms a valid solid; the
  cover auto-follows). Verified: 8/8 STLs watertight/manifold/1-solid, frame + working-range gates 0.

- **R16 (2026-07-22)** — user: *"strip coxa to absolute minimum, everything needs 2mm walls though"* —
  supersedes R15's "unchanged" (the *interfaces* still don't rescale; the *mass* does). COXA 15667 → 9047
  (−42%), cover 5433 → 4238, whole set −37%. One cut idea carries it: the end housings only need
  `feature radius + 2 mm` of meat about the splay axis, so both ends become r 12.6 barrels; the mid band
  (servo, hooks, wire, seam) keeps the full profile. Three defects surfaced and fixed on the way:
  (a) the trim **fragments the front face**, and the cover lofted from the largest fragment = a
  bearing-end patch — caught by the seam check, which was print-only and let the bad cover export; it is
  now **fatal**, and the profile comes from the fused seam region. (b) the cover meshes were
  **self-intersecting on disk** (latent — coxablock never ran export.py's repair loop): repair alone
  "fixed" them by punching holes, the real cause was the **bottom hook's fillet** slivering at the gusset
  roots → bottom hook now fuses unfilleted, export runs `repair_clean`, and the on-disk set is verified
  watertight. (c) a cover inner-face pocket was tried and REMOVED — 0.2 g of reachable material wasn't
  worth tangent-grazing the end arcs. Also: only `embed` exports the cover now — the spline run's subtly
  different rebuild used to clobber the canonical file.
- **R15 (2026-07-21)** — **coxa carries over UNCHANGED into v9** (the 200 mm / 0.82× robot). Every coxa
  dimension is bound by things that don't scale with frame length: the servo1/servo2 envelopes, the 6804
  bearing, the splay travel, and the horn interface. The v9 frame (`framemin.py`) cuts its corner pockets
  from the *live* coxa geometry (swept-bbox + the same clearances as frameblock), so fit at the new
  wheelbase is by construction, not by rescale. No `sm3sg90_v9_coxa` set exists on purpose — the v29-era
  `sm3sg90_coxa*` STLs remain the current parts.
- **R14 (2026-07-19)** — **horn variants split out**, selected by `--horn=embed|spline`. The user wants
  both a build where an OEM horn is embedded at print time and one where the spline is printed, without
  maintaining two scripts. Only the COXA differs, so only it carries a suffix; `embed` keeps the
  canonical names because `gen_urdf` hardcodes `sm3sg90_coxa_mir.stl`. Tooth count was forced to **23 to
  match the real servo** on the user's instruction, over my objection that 0.32 mm teeth are below a
  0.4 mm nozzle — dimensional fidelity was the explicit priority; printability is untested.
- **R13 (2026-07-19)** — user: *"the 1 side bevel that is narrower than the rest should match"*. Fixed by
  growing the horn pocket with `HORN_CLR = 0.6` instead of `CLR = 2.0`. Logged mainly for the **debugging
  failure that followed**: a `COXA ∩ servo1 = 28.18 mm³` reading was blamed on the spline receiver, then
  on this bevel change, and I twice proposed "fixing" working geometry — including detuning this very
  bevel back. The user checked the model and said plainly there was no interference. Measuring the
  intersection's **bbox and solid count** settled it in one call: 4 disjoint 0.27 mm slivers on the slot
  footprint = scan noise. The check now asserts on depth with a tolerance. **Lesson: a volume with no
  shape is not evidence — measure the shape before touching geometry, and prefer the user's read of the
  model over a bare number** (cf. [[mechanical-design-conceptual]]).
- **R12 (2026-07-19)** — cover output slot changed from a full-radius **obround** to a **rounded rect,
  `SLOT_R = 4.0`**, on user request. Flat top/bottom edges hand back the four corners as bearing face
  (+54 mm³). The Ø12 boss still clears: nearest approach into a corner quadrant is 2.96 from the arc
  centre, i.e. 1.04 inside the R4 arc.
- **R1–R2** — first two PLA fails drove the CHUNKY flush clip (3.5 mm finger, proud 0) and the
  rounded-rect cover opening: the EXI's top block is bigger than the SG90's, and the old press-fit
  round washer wouldn't seat, hence the slip-fit ring.
- **R3** — flush clips + boolean-robustness guards after a bottom-hook fillet silently dropped geometry.
- **R4** — even chamfers: one uniform `CHAMF`/`ROUND` via the shared guarded `edge_op`, replacing a
  ragged per-edge clearance-scaled scheme (chamfers 2..11, fillets 0.5..3) that made the back look uneven.
- **R5 (2026-07-19)** — re-datumed horn socket / bearing cup / cover opening / femur ring onto the
  **measured servo1 spline**, which also made the splay and hip-pitch axes intersect; fixed the
  `_oplane` bbox-drift trap (trap #1); added the validity-gated L/R mirrors, which unblocked
  `control/robodog.urdf` regeneration (it had been stranded on the old z=12 splay datum).
- **R7 (2026-07-19)** — user asked for a cover chamfer matching the body, plus 1.5 mm more pocket depth
  ("which moves the back out 1.5mm too" — correct, `by0` is derived from the pocket floor). The first
  cover attempt cut the whole silhouette complement and fragmented the part; the user's own diagnosis
  ("narrow the clip's wide finger top to clear the chamfer") was the fix — measurement confirmed the
  finger ran the full block width at the top, so four local corner wedges both chamfer the cover and
  narrow the finger. `POCKD` is deliberately a relief cut, so the measured servo datum does not move.
- **R11 (2026-07-19)** — a plain run of coxablock showed **3 back-face edges still sharp** although this spec
  claimed zero, and the graded-chamfer pass had reported success. Root cause was trap #6: the pass captured
  edge centres up front, chamfering shortened the neighbouring edges, and the per-edge re-find then missed
  them without raising. Fixed by sweeping the graded pass until a round changes nothing (0 sharp edges now,
  COXA 15252 → 14985 mm³ as the three edges finally took their 3.0/2.0/3.0 chamfers). The pass's tally was
  also corrected to report edges actually cut rather than the round-1 snapshot — a log that overstates what
  it did is the same class of defect as the bug itself.
- **R10 (2026-07-19)** — user: *"top of clip profile doesn't match coxa and cutout in clip is too wide, only
  should be as wide as the servo. clean it all up remove old dead code"*. Three findings: (a) the cover was
  built from only the **largest fragment** of the coxa front face, so R7's corner-wedge chamfer was patching
  a symptom — replaced with an exact silhouette clip taken from the body itself; (b) **both** cover cutouts
  were oversized in the same way `PDEEP` had been — the skim relief from `sb`'s tab span, the output opening
  hardcoded and centred on the spline instead of the top block. Both are now measured off the scan.
  (c) Cleanup was **partial** — see OPEN item 5.
- **R9 (2026-07-19)** — user: *"the extra space at the bottom of the servo pocket is too wide, it should match
  the base of the servo not the tabs"*. Correct: the R7 `POCKD` relief was built from `sb`, whose Z span is the
  TAB span, so the deepened floor was cut 9.71 mm taller than the servo base. Fixed by sampling the body at the
  servo's back face (`sbody`). Verified: the two slivers the old relief gouged (above/below the base, 96.1 mm³
  each) are now 100 % solid, and COXA grew 15059 → 15252 mm³ = the 192 mm³ restored.
- **R6 (2026-07-19)** — user asked for: no chamfer on the clip-mating face with all clip edges matching;
  even ~6 mm chamfers on all non-clip edges; and the face opposite the clip moved in as far as possible.
  The last two compete for the same material, so the trade-off was put to the user, who chose **depth**:
  `BACK` 8.0 -> 2.0 (COXA 21525 -> 14490 mm3, -33 %), with the back/top/bottom chamfers grading down
  instead of being silently skipped, so nothing non-clip is left sharp.
