"""
test_relay_logic.py

Standalone unit test for find_relay_path() in v2x_city.py, run WITHOUT
SUMO/traci (stubbed out, since this file never actually calls traci).
Run it any time v2x_city.py's relay logic changes:

    python test_relay_logic.py
"""

import sys
import types

# v2x_city.py does `import traci` and `import overlay` at module level;
# neither is touched by find_relay_path/euclidean, so stub them out rather
# than requiring a real SUMO install just to unit-test this one function.
sys.modules.setdefault("traci", types.ModuleType("traci"))
_overlay_stub = types.ModuleType("overlay")
_overlay_stub.label = _overlay_stub.junction_tag = _overlay_stub.board = (
    lambda *a, **k: (None, 0, 0)
)
sys.modules.setdefault("overlay", _overlay_stub)

import importlib.util
import os

HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location(
    "v2x_city", os.path.join(HERE, "v2x_city.py")
)
v2x_city = importlib.util.module_from_spec(spec)
spec.loader.exec_module(v2x_city)

find_relay_path = v2x_city.find_relay_path
V2I_RADIUS = v2x_city.V2I_RADIUS
I2I_RADIUS = v2x_city.I2I_RADIUS


def check(name, got, expected):
    ok = got == expected
    print(f"[{'PASS' if ok else 'FAIL'}] {name}: got={got!r} expected={expected!r}")
    return ok


results = []

# find_relay_path is only ever called by v2x_city.py's own main loop AFTER
# it has already confirmed the ambulance is NOT within direct/armed range
# (see the "if dist <= radius: ... elif ready and args.relay:" branch) -
# so every case below is consistent with that real calling contract.

# ------------------------------------------------------------------
# A 2-hop relay chain closes a gap beyond V2I_RADIUS (100 m).
# ------------------------------------------------------------------
positions = {"amb1": (0.0, 0.0), "r1": (85.0, 0.0), "r2": (170.0, 0.0)}
alive = set(positions)
r = find_relay_path("amb1", positions["amb1"], (260.0, 0.0), V2I_RADIUS, positions, alive)
results.append(check("2-hop relay closes the V2I_RADIUS gap", r, (True, 2, ["r1", "r2"])))

# ------------------------------------------------------------------
# Same geometry, but the signal is already I2I-armed (wider 220 m
# radius) - only 1 hop should now be needed, since r1 alone is already
# within I2I_RADIUS of the junction.
# ------------------------------------------------------------------
r = find_relay_path("amb1", positions["amb1"], (260.0, 0.0), I2I_RADIUS, positions, alive)
results.append(check("I2I-armed radius needs fewer hops", r, (True, 1, ["r1"])))

# ------------------------------------------------------------------
# No background traffic nearby at all -> unreachable.
# ------------------------------------------------------------------
r = find_relay_path("amb1", (0.0, 0.0), (1000.0, 0.0), V2I_RADIUS,
                     {"amb1": (0.0, 0.0)}, {"amb1"})
results.append(check("no traffic nearby -> unreachable", r, (False, None, None)))

# ------------------------------------------------------------------
# A candidate relay vehicle just outside V2V_RELAY_RANGE (90 m) must
# NOT be used.
# ------------------------------------------------------------------
positions2 = {"amb1": (0.0, 0.0), "far": (95.0, 0.0)}
r = find_relay_path("amb1", positions2["amb1"], (150.0, 0.0), V2I_RADIUS,
                     positions2, set(positions2))
results.append(check("relay vehicle just out of V2V range -> unreachable", r, (False, None, None)))

# ------------------------------------------------------------------
# Shortest-hop guarantee: a direct 1-hop relay exists alongside a
# longer path; BFS must return the shorter one.
# ------------------------------------------------------------------
positions3 = {
    "amb1": (0.0, 0.0),
    "near": (90.0, 0.0),    # amb->near = 90 (hop1); near->junction(185) = 95 <= 100
    "chain_a": (85.0, 0.0),  # also a valid hop1 start, but its own distance to
                              # junction is > radius, so it can't finish in 1 hop
}
r = find_relay_path("amb1", positions3["amb1"], (185.0, 0.0), V2I_RADIUS,
                     positions3, set(positions3))
results.append(check("shortest viable chain wins (1 hop via 'near')", r[1], 1))

print()
total, passed = len(results), sum(results)
print(f"{passed}/{total} checks passed")
if passed != total:
    raise SystemExit(1)
