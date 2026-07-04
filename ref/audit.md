# ROBODOG design audit — 2026-07-01

All volumes mm³, measured in FreeCAD on the placed assembly (26 parts) + leg-local sweeps.
Raw data: `3dprint/sm3-sg90/ref/iter/audit_static.txt`, `audit_sweep.txt`, `audit_fix1.txt`, `audit_fix2.txt`.

## Verdict summary (numeric audit)

| # | Severity | Finding | Status |
|---|----------|---------|--------|
| 1 | CRITICAL | Front-hip `coxa_rear` interpenetrates the femur at every pitch (6841@0° → 1983@66°); rear coxa 1983@66 stance too. | **FIXED** — `_sweep_clear()` carves the femur's swept envelope (scaled 1.05 about HP, journal masked) out of both coxa types. Post-fix: rear 0–48 over walking range, front 12–271 over 41–91°. Foldflat 0° stays impossible for front hips (femur coaxial with barrel) — documented. |
| 2 | CRITICAL | Front leg forward-pitch blocked by own bracket+servo beyond ~stance+20–28°. | Inherent to the inboard tuck; documented as a firmware limit (fwd pitch ≤ stance+20°). |
| 3 | CRITICAL | Cover "snaps" do not lock (upper friction-only, monotone decay 446→379; lower wedges 238→520). Nub y26..27.6 never meets flange y28..33.8 (0.4mm air gap), no Z undercut. | OPEN → covers redesign. |
| 4 | CRITICAL | Frame ends poke into the covers' rounded front wall (cover_up^frame 446 @x[88,92.4]; cover_lo^frame 238). | OPEN → covers redesign. |
| 5 | CRITICAL | Head unmounted + collides (frame 1542 / cover_up 971 / cover_lo 944; root x86.5 vs cover end x92.5). | OPEN → head socket + forward shift. |
| 6 | MAJOR | Bracket mount foot pokes upper-cover ceiling ×4 (235; deck→ceiling gap 1.8mm). | OPEN → ceiling relief or deck pocket. |
| 7 | MAJOR | On-deck electronics impossible (1.8mm headroom). | **FIXED** — elec dummy moved into the cavity (bz0+10, on standoffs). |
| 8 | MAJOR | Adduction roll ≥25–30° drives femur into frame (336@-25°, 1942@-40°). Walking ±20° clean. | Documented as firmware roll limit ±20°. |
| 9 | minor | Femur grazes lower-cover hip lip (290×4); knee 110° foot-up fold = 866. | Lip → strip redesign; fold direction see #27 below. |
| 10 | minor | 8 unused cover-peg holes; stale rails comment. | Comment fixed; peg holes → covers redesign. |

## Second-pass findings (40-finding multi-lens workflow sweep, adversarially verified)

Fixed this session:

- **Bracket end-wall + shell overlap the coxa roll barrel 503/215** (the numeric audit's "constant under roll → bearing" call was WRONG — constant because both parts are rotationally symmetric there; two printed parts still can't share space). Front-variant end wall also sat across the horn (shaft relief cut at `bb.XMax` = tail for `servo1_rear`). **FIXED** — horn end auto-detected (thin end-slab volume), r8.6 × 12 bore cleared through it. Verified brkt^coxa = 0 on all 4 corners, roll sweep ±40° all 0, brackets single solids.
- **Outer bracket screw hole 0.5mm off the frame pilot** (holes were `bb.XMin+8 / bb.XMax-8` where bb includes the 8mm horn → 14.5mm spacing vs 15mm pilots). **FIXED** — holes at `bb.XMin+8 / +23`. Screw probes: x25 clean; x40 probe still meets 0.8mm³ of reveal-post material *below* the pilot = self-tap bite, fine.
- **Knee gate swept the wrong direction** (foot-up folds {45,-15,-65}, while crouch/foldflat fold feet-DOWN past standing). **FIXED** — added `kneeD` gate at +55/+100/+115: 689/1137/1314. Crouch fold ~100 needs a femur-front relief before deep crouch is physically clean (known, crouch.py header).
- **Export chirality undocumented** (as-exported = RIGHT-side legs; LEFT legs are true mirrors; canted tibia+foot are chiral). **FIXED** — documented at the export block: print one set as-is + one set slicer-mirrored.
- **crouch.py placed the body 15.5mm too high** (`bz0 = HIP_DZ-BH/2+6` vs dog.py's `HIP_DZ-BH+7`), invalidating its belly-height readout. **FIXED**.
- Missing `sm3sg90_coxa_front` STL (front legs use `coxa_rear()` but it was never exported; slicer-mirroring cannot produce it from the rear coxa). **FIXED** earlier this session.

Mechanism-level fixes (user-approved "fix all those", 2026-07-01 second pass):

- **No axial retention on the hip-roll joint** — **FIXED**: M2 horn-screw channel (r1.1 through + r3.0 counterbore/driver channel) cut through both coxa barrel backs, 2mm head shoulder behind the spline bore. Verified: screw path clear both types; rear shoulder ring full (31.7mm³); FRONT shoulder only ~30% survives the femur sweep-clearance — use a washer under the front horn-screw head. Assembly order: screw the coxa onto the horn BEFORE the femur enters the cradle (the knuckle blocks the driver channel after).
- **SG90 dummy wrong (centered shaft, no tabs)** — **FIXED for the hip-roll pair**: `_sg90_roll()` true geometry (22.8×12.2×22.5 body, shaft 5.5mm off length-center, tabs w/ screw holes, lead stub). Bracket rebuilt: open-top drop-in pocket, bottom-tab slot through the floor + M2 boss (tab screw = servo can't lift out, fixes "no servo retention"), r8.6 roll-barrel bore, mount foot keyed to the output face (screws land exactly on the frame pilots — verified with r0.95 probes, all corners). brkt^coxa=brkt^servo=coxa^servo=0 everywhere. NOTE: servo2/knee servo already had the offset shaft; they got tabs/lead/spline-r2.4 via `_sg90_pitch()`.
- **8 of 12 servos absent** — **FIXED (in verification)**: `servo2()`/`servoK()` (true SG90, knee one canted per KNEE_CANT) placed in all 4 legs → 34 parts. Placement measurement revealed the deeper truth: the rotating knuckle caps sweep THROUGH the servo bodies (rootcap vs knee servo 760–2171mm³ at every fold; femur knuckle vs servo2 1670–2009 at every pitch) and the coxa bridge ARM was fused straight through servo2's body (1722/387). Fixes: swept-envelope carves (16-step fold sweep out of the tibia rootcap; 12-step pitch sweep + static pocket/tab-slot envelope out of the femur; servo2 pocket cut out of both coxae). The joint bearing is the +Y spline hub + −Y idle peg (as in the real SM3); the caps survive as partial nesting arcs.
- **Knee clevis idle arm vs femur boss (252 solid clash)** — **FIXED (in verification)**: pegarm+peg swing sector (13-step) carved out of the femur idle boss; bore keeps the r3 peg. Idle pegs/arms also shortened 1mm (y −19..−14) on both femur and tibia: the real SG90 body reaches y −13.5 (was 0.5mm interference).
- **Clevis hub bores** — **FIXED**: modeled spline r2.7→r2.4; hub bores now r2.5×4.7 spline pocket + r1.2 through = M2 clamp-screw shoulder (head no longer falls through), both knee (tibia) and hip-pitch (femur) hubs.
- **Electronics serviceability** — **FIXED (in verification)**: r2.0 driver holes through the deck above all 4 standoffs; LiPo belly hatch (60×24) in the floor under the pocket + 4 strap slots — battery swaps through the open belly.
- **TPU boot chirality** — **FIXED**: second mirrored export `sm3sg90_boot_L` (as-exported tibia/boot = RIGHT feet).
- **Foldflat limits** — documented in foldflat.py header: PIT=0 unreachable (front coxa + intact central cover band; real floor ≈26–31°); canted fold splays feet ~62mm/side.

## What's GOOD (verified)

- **Mirroring is exact**: every leg-pair volume identical across all 4 legs. One printed leg + slicer-mirror is sound (see chirality note in dog.py exports).
- **All 26 parts single solids**; all 10 STLs export 1-solid.
- Knee pivot 252 constant (but see clevis/boss note above); servo seats servo^coxa=servo^brkt=0; bracket↔coxa now a true clearance bearing (0 through ±40° roll).
- Rear-type coxa post-fix: 0–48 across the walking window; front-type 12–271 over 41–91°.
- Bracket foot ↔ deck M2 screws align exactly (15mm spacing = pilots).
- Frame + leg fit the A1 bed; boot pairs with the SM3 foot by construction.

## Covers / side-strip / head redesign (audit #3/#4/#5/#6/#9/#10 — implemented 2026-07-01)

- **Upper clamshell** (snap-on): shell z≥4 (side strip widened), length +4 so the front wall CLEARS the frame end (was embedded, #4). Full-height hip windows unchanged. 4 ceiling windows over the bracket mount feet (#6 poke fixed + M2 driver access from above). Front-wall opening for the head plug.
- **Snap latch** (#3 fixed): 3 catch ARMS per side on the frame (deck edge → y34.4, z10..13) pierce matching WINDOWS in the cover side wall; the wall panel flexes ~0.8mm on push-down, then the arm is pinned in the window = positive vertical lock. Old non-engaging nubs/flanges deleted.
- **Lower cover DELETED** → **screwed belly pan**: the femur crosses the wall band y[33.8,36] from z0 to ~−19 at every working pitch (290mm³ lip graze, #9; roll grinds deeper — hip-roll had zero clean travel), so no lower side wall can exist in the hip windows and the "one-solid lower ring" is geometrically dead. Flat pan (164×48×1.6) under the floor, 4× M2 into repurposed floor pilots (#10: deck-side dead holes deleted), hatch window matching the frame's LiPo hatch.
- **Head mounted** (#5): root moved to x96.7 (0.2 clear of the cover front), under-deck plug + 2× M2 down through new deck pilots. Face panel follows.
- **Side strip**: one bold open band from the shell's z4 edge down to the pan — dark frame rails/posts read through it ("like SM3 but even better" = wider, continuous, frames the legs).

## Final verification (34-part placed assembly + covers, 2026-07-01)

Contact matrix residuals — everything else is 0: tibia^servoK 135×4 (rootcap-arc sliver at standing; was 760+),
femur^tibia 38×4 (knee pivot graze; was 252), femur^servo2 10×4 (sweep tolerance), cover_up^frame 9 (catch arms
seated in their windows), head^face 54 (cosmetic, intended). Latch lift test 1→16 mm³ ramp = arms pinned in the
windows (positive lock); slide-on flex load 15–29 mm³. All 11 STLs single solids.

Gates: knee 0/60/110 = 38/48/199 · kneeD +55/+100/+115 = 79/661/881 · hip −45/0/+45 = 21/4/0 ·
hipR −45/0/+45 = 1832/4/27 (−45 is outside the front-hip demand; walking 41–91 clean) ·
svc2 = 8–10 at all pitches, pockets 0 · svcK −110/0/+55/+115 = 156/135/222/945, cradle 0.

## Remaining / accepted limits

- Front horn-screw shoulder ~30% arc (femur sweep-clearance wins) → washer under the head.
- Knee-servo vs shin at deep feet-down fold (945mm³ @+115, 222 @+55): firmware fold limit ~+100, or a future shin relief.
- kneeD +100 = 661 (crouch): thigh/shin tube contact, pre-existing (crouch.py header) — needs femur-front relief2 deepening for a physically clean deep crouch.
- Front-leg fwd pitch ≤ stance+20° (own bracket/servo, #2); adduction roll ≥25° hits frame (#8); front hip pitch < ~38° unreachable (barrel coaxial) — firmware limits.
- tibia^servoK 135 at standing: rootcap remnant arc — cosmetic-arc trim candidate if it binds on the printed part.
