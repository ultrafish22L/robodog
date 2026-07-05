"""Phase 4 — gait parameter search. Random sampling then hill-climb over the Crawl gait params,
each candidate scored by a headless PyBullet rollout (gait.run). Finds a faster/steadier crawl and
reports the best-practice ranges of the top gaits. (Dependency-free; CMA-ES is a drop-in upgrade.)
"""
import os
import json
import numpy as np
import gait

SPACE = {  # param: (lo, hi)
    "height": (105.0, 140.0), "width": (60.0, 86.0), "stride": (28.0, 62.0), "lift": (18.0, 40.0),
    "duty": (0.72, 0.86), "sway": (12.0, 42.0), "period": (1.5, 3.0),
}

def fitness(m):
    """Reward forward travel; punish falling, veering, tilting, and sagging."""
    if m["fell"]:
        return -500.0
    f = m["travel_mm"]
    f -= abs(m["yaw_deg"]) * 4.0                             # go straight
    f -= (abs(m["roll_deg"]) + abs(m["pitch_deg"])) * 2.0    # stay level
    f -= max(0.0, 110.0 - m["final_z_mm"]) * 1.5             # don't sag/crouch
    return f

def evaluate(params):
    m = gait.run(params=params, cycles=3)
    return fitness(m), m

def _clamp(params):
    return {k: float(np.clip(v, *SPACE[k])) for k, v in params.items()}

def search(budget=44, seed=1):
    rng = np.random.default_rng(seed)
    hist = []
    best = None
    n_rand = budget // 2
    for i in range(n_rand):
        pr = {k: float(rng.uniform(*v)) for k, v in SPACE.items()}
        fit, m = evaluate(pr); hist.append((fit, pr, m))
        if best is None or fit > best[0]:
            best = (fit, pr, m)
        print(f"  rand  {i:2d}: fit {fit:7.1f}  travel {m['travel_mm']:6.1f}mm  {'FELL' if m['fell'] else ''}")
    for i in range(budget - n_rand):
        cand = _clamp({k: v + rng.normal(0, (SPACE[k][1] - SPACE[k][0]) * 0.12) for k, v in best[1].items()})
        fit, m = evaluate(cand); hist.append((fit, cand, m))
        tag = ""
        if fit > best[0]:
            best = (fit, cand, m); tag = "  *improved*"
        print(f"  climb {i:2d}: fit {fit:7.1f}  travel {m['travel_mm']:6.1f}mm{tag}")
    return best, hist

if __name__ == "__main__":
    HERE = os.path.dirname(os.path.abspath(__file__))
    base_fit, base_m = evaluate({})  # default hand-tuned gait, as a baseline
    print(f"baseline (hand-tuned) fitness {base_fit:.1f}, travel {base_m['travel_mm']:.1f}mm/3cyc\n")
    print("=== search: random exploration -> hill-climb ===")
    best, hist = search()
    fit, params, m = best
    print(f"\nBEST fitness {fit:.1f} (baseline {base_fit:.1f})  travel {m['travel_mm']:.1f}mm/3cyc"
          f"  roll {m['roll_deg']:.1f} pitch {m['pitch_deg']:.1f} yaw {m['yaw_deg']:.1f}")
    print("params:", {k: round(v, 2) for k, v in params.items()})
    good = sorted([h for h in hist if not h[2]["fell"]], key=lambda h: -h[0])[:10]
    print(f"\nbest-practice ranges (top-{len(good)} non-fallen gaits):")
    for k in SPACE:
        vals = [g[1][k] for g in good]
        print(f"  {k:8s} {np.mean(vals):6.1f} +/- {np.std(vals):4.1f}   [{min(vals):.1f}, {max(vals):.1f}]")
    json.dump({k: round(v, 3) for k, v in params.items()}, open(os.path.join(HERE, "best_gait.json"), "w"), indent=2)
    gait.run(params=params, cycles=6, filmstrip="_best_walk.png")
    print("\nbest gait -> best_gait.json ; filmstrip -> _best_walk.png")
