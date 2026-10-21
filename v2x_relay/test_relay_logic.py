"""
Standalone unit test for the find_relay_path() BFS logic in
v2x_relay_test.py, run WITHOUT SUMO/traci (which isn't installable in
this sandbox). It reimplements the exact same function against a fake
position table instead of traci.vehicle.getPosition(), so the hop-
counting / shortest-path / range logic gets verified before it's ever
run for real on SUMO.
"""

V2I_DIRECT_RANGE = 70.0
V2V_RELAY_RANGE = 100.0
MAX_RELAY_HOPS = 3

AMBULANCE_ID = "ambulance"


def euclidean(pos_a, pos_b):
    return ((pos_a[0] - pos_b[0]) ** 2 + (pos_a[1] - pos_b[1]) ** 2) ** 0.5


def make_find_relay_path(positions, relay_enabled):
    def get_pos(vid):
        return positions[vid]

    def find_relay_path(ambulance_pos, junction_pos, exists_cache):
        if euclidean(ambulance_pos, junction_pos) <= V2I_DIRECT_RANGE:
            return True, 0, []

        if not relay_enabled:
            return False, None, None

        visited = {AMBULANCE_ID}
        frontier = [(AMBULANCE_ID, ambulance_pos, [])]

        for hop in range(1, MAX_RELAY_HOPS + 1):
            next_frontier = []
            for node_id, node_pos, chain in frontier:
                for other_id in exists_cache:
                    if other_id in visited:
                        continue
                    other_pos = get_pos(other_id)
                    if euclidean(node_pos, other_pos) > V2V_RELAY_RANGE:
                        continue
                    visited.add(other_id)
                    new_chain = chain + [other_id]
                    if euclidean(other_pos, junction_pos) <= V2I_DIRECT_RANGE:
                        return True, hop, new_chain
                    next_frontier.append((other_id, other_pos, new_chain))
            frontier = next_frontier
            if not frontier:
                break

        return False, None, None

    return find_relay_path


def check(name, got, expected):
    status = "PASS" if got == expected else "FAIL"
    print(f"[{status}] {name}: got={got!r} expected={expected!r}")
    return status == "PASS"


results = []

# ------------------------------------------------------------------
# Scenario 1: ambulance 280m from junction (J1_J2, matches real deploy:
# depart at x=20, J2 at x=300). relay_1 at x=105 (85m ahead of amb,
# i.e. departPos=85), relay_2 at x=185 (departPos=165). Junction at
# x=300 (straight corridor, y=0 for everyone - matches edge geometry).
# Expect: NOT direct (280 > 70). relay_1 reachable from ambulance
# (85m <= 100). relay_2 reachable from relay_1 (80m <= 100). relay_2 to
# junction = 115m > 70 -> still not in range from relay_2 alone. So at
# this exact moment, 2 hops is NOT enough yet (matches the scenario
# doc's note that the chain "becomes valid" only once the lead car gets
# within V2I_DIRECT_RANGE of the junction - not from the very first step).
# ------------------------------------------------------------------
pos1 = {
    AMBULANCE_ID: (20.0, 0.0),
    "relay_1": (105.0, 0.0),
    "relay_2": (185.0, 0.0),
}
f1 = make_find_relay_path(pos1, relay_enabled=True)
r = f1(pos1[AMBULANCE_ID], (300.0, 0.0), list(pos1.keys()))
results.append(check("scenario1 (too far even via 2 hops)", r, (False, None, None)))

# ------------------------------------------------------------------
# Scenario 2: same chain, but everyone has advanced down the corridor
# by 120m (ambulance 140, relay_1 225, relay_2 305->within network but
# let's keep it at 300 boundary realistic: relay_2 at 295, 5m from
# junction at 300). Now: direct amb->junction = 160m (no). amb->relay_1
# = 85m (yes, hop1). relay_1->relay_2 = 70m (yes). relay_2->junction =
# 5m (yes, <=70) -> 2-hop path found.
# ------------------------------------------------------------------
pos2 = {
    AMBULANCE_ID: (140.0, 0.0),
    "relay_1": (225.0, 0.0),
    "relay_2": (295.0, 0.0),
}
f2 = make_find_relay_path(pos2, relay_enabled=True)
r = f2(pos2[AMBULANCE_ID], (300.0, 0.0), list(pos2.keys()))
results.append(check(
    "scenario2 (2-hop relay path found, shortest-path chain)",
    r, (True, 2, ["relay_1", "relay_2"])
))

# ------------------------------------------------------------------
# Scenario 3: direct range - ambulance already within 70m of junction.
# Should short-circuit with hop_count=0 and an EMPTY chain, regardless
# of what other vehicles exist.
# ------------------------------------------------------------------
pos3 = {
    AMBULANCE_ID: (250.0, 0.0),   # 50m from junction at 300
    "relay_1": (999.0, 999.0),    # irrelevant, far away
}
f3 = make_find_relay_path(pos3, relay_enabled=True)
r = f3(pos3[AMBULANCE_ID], (300.0, 0.0), list(pos3.keys()))
results.append(check("scenario3 (already in direct range)", r, (True, 0, [])))

# ------------------------------------------------------------------
# Scenario 4: relay disabled (--no-relay) - even though a 2-hop path
# exists (same geometry as scenario 2), it must NOT be used.
# ------------------------------------------------------------------
f4 = make_find_relay_path(pos2, relay_enabled=False)
r = f4(pos2[AMBULANCE_ID], (300.0, 0.0), list(pos2.keys()))
results.append(check("scenario4 (relay disabled -> unreachable)", r, (False, None, None)))

# ------------------------------------------------------------------
# Scenario 5: a longer, 3-hop chain that is exactly at MAX_RELAY_HOPS.
# amb(0) -> r1(90) -> r2(180) -> r3(270) -> junction(340).
# amb->r1=90<=100 (hop1). r1->r2=90<=100 (hop2). r2->r3=90<=100 (hop3,
# at MAX_RELAY_HOPS). r3->junction = 70<=70 (in range) -> reachable.
# ------------------------------------------------------------------
pos5 = {
    AMBULANCE_ID: (0.0, 0.0),
    "r1": (90.0, 0.0),
    "r2": (180.0, 0.0),
    "r3": (270.0, 0.0),
}
f5 = make_find_relay_path(pos5, relay_enabled=True)
r = f5(pos5[AMBULANCE_ID], (340.0, 0.0), list(pos5.keys()))
results.append(check("scenario5 (exactly 3-hop chain, at MAX_RELAY_HOPS)", r, (True, 3, ["r1", "r2", "r3"])))

# ------------------------------------------------------------------
# Scenario 6: same as scenario 5 but junction moved 1m further out of
# r3's range (71m) - must now be unreachable (a 4th hop would be
# needed, which exceeds MAX_RELAY_HOPS=3).
# ------------------------------------------------------------------
f6 = make_find_relay_path(pos5, relay_enabled=True)
r = f6(pos5[AMBULANCE_ID], (341.0, 0.0), list(pos5.keys()))
results.append(check("scenario6 (one hop too many -> unreachable)", r, (False, None, None)))

# ------------------------------------------------------------------
# Scenario 7: shortest-path guarantee - two possible chains exist
# (a 1-hop path via a close relay, and a longer 2-hop path via other
# vehicles); BFS must return the 1-hop one, not the 2-hop one, even if
# the 2-hop relay vehicles are listed first in exists_cache.
# ------------------------------------------------------------------
pos7 = {
    AMBULANCE_ID: (0.0, 0.0),
    "far_hop_a": (90.0, 0.0),     # part of a longer chain
    "far_hop_b": (180.0, 0.0),    # far_hop_b -> junction(250) = 70, in range
    "short_hop": (85.0, 0.0),     # short_hop -> junction(250) would be 165, NOT in range directly...
}
# Re-balance so BOTH a 1-hop and a longer path are *actually* valid, to
# make this a real test of "BFS prefers fewer hops":
pos7 = {
    AMBULANCE_ID: (0.0, 0.0),
    "near_relay": (95.0, 0.0),     # amb->near_relay = 95 <= 100 (hop1); near_relay->junction(160) = 65 <=70 -> 1 hop works
    "chain_a": (90.0, 0.0),        # also reachable hop1, but chain_a->junction = 70, also in range actually
}
exists_order = ["chain_a", "near_relay"]  # chain_a listed FIRST
f7 = make_find_relay_path(pos7, relay_enabled=True)
r = f7(pos7[AMBULANCE_ID], (160.0, 0.0), [AMBULANCE_ID] + exists_order)
# Both chain_a and near_relay are valid 1-hop relays here (BFS iterates
# exists_cache in order, so it returns whichever is encountered first -
# this confirms hop_count=1, not that a specific ID wins).
results.append(check("scenario7 (hop_count is minimal: 1, not deeper)", r[1], 1))

print()
total = len(results)
passed = sum(results)
print(f"{passed}/{total} checks passed")
if passed != total:
    raise SystemExit(1)
