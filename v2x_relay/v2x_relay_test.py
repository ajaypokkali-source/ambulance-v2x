"""
v2x_relay_test.py

MULTI-HOP V2V RELAY FOR OUT-OF-RANGE V2I SIGNAL PREEMPTION.

Lives in its own self-contained folder (C:\\ambulance_v2x\\v2x_relay) with
its own edges/nodes/routes/network/config - same corridor shape as
v2x_integrated (J1..J5, J2/J3/J4 as real traffic lights), but it is a
genuinely NEW scenario, not a copy-and-extend of the same mechanism:

WHAT'S DIFFERENT FROM v2x_integrated / v2x_multi
--------------------------------------------------
Every earlier scenario (v2x_integrated, v2x_multi, ambulance_v2x_city)
treats V2I as "if the ambulance is within PREEMPTION_DISTANCE metres of
the junction (straight-line), the message arrives" - i.e. an unlimited
one-hop radio link. I2I approximated "infrastructure already knows the
ambulance is coming" by just giving an armed junction a BIGGER fixed
detection radius once its upstream neighbour activated. That is a
reasonable simplification for signal-to-signal awareness, but it is not
a real network mechanism - nothing actually carries the message there.

This scenario replaces that shortcut with the real thing:

    1. V2I direct range is deliberately tightened to something more
       realistic (V2I_DIRECT_RANGE, 70 m) - clearly too short to give a
       junction useful advance warning on its own at corridor-scale
       (300 m junction spacing).

    2. When the ambulance is OUT of direct range, every step this
       script runs a bounded BFS flood (max MAX_RELAY_HOPS hops) over
       the live V2V connectivity graph - which vehicles are within
       V2V_RELAY_RANGE of which other vehicles right now - starting
       from the ambulance, to see whether a chain of real vehicles
       currently on the road can carry the preemption request to a
       junction that's otherwise out of reach. If one exists, the
       request is delivered via that relay chain (and the exact chain
       of relaying vehicle IDs is logged), with a modelled one-step
       forwarding delay per hop (STORE_AND_FORWARD_DELAY_STEPS).

    3. If no chain exists, the message simply does not arrive - the
       junction only reacts once the ambulance eventually comes into
       its own direct range (which may be very late, or - if traffic
       is too sparse to relay at all - never, until the ambulance is
       almost on top of it). That coverage gap IS the result this
       scenario is built to show: multi-hop V2V relay recovers most of
       the early-warning benefit that v2x_integrated's I2I shortcut
       assumed for free, but only when there is actually traffic
       nearby to carry it - a realistic, demonstrable trade-off.

The V2V same-lane overtake module and the V2V intersection-yield module
are reused UNCHANGED from v2x_integrated (they are not what this
scenario is about); I2I is removed entirely and replaced by the relay
mechanism above.

REQUIRED ONE-TIME SETUP
------------------------
    python build_network.py

USAGE
-----
Live visual demo (same as v2x_integrated):
    python v2x_relay_test.py

Headless, for metrics (writes results/ambulance_<tag>.csv):
    python v2x_relay_test.py --nogui
    python v2x_relay_test.py --nogui --no-relay   (direct-range-only baseline)
    python v2x_relay_test.py --nogui --seed 7

compare_relay.py in this folder runs both configurations back-to-back
and produces results/comparison.png + comparison.csv.
"""

import argparse
import csv
import os
import sys
import traci


# ============================================================
# CLI ARGS
# ============================================================

parser = argparse.ArgumentParser()
parser.add_argument("--nogui", action="store_true",
                     help="run headless (sumo instead of sumo-gui)")
parser.add_argument("--no-relay", action="store_true",
                     help="disable multi-hop V2V relay (direct V2I range only)")
parser.add_argument("--seed", type=int, default=None,
                     help="SUMO random seed, for repeatable comparison runs")
args = parser.parse_args()


# ============================================================
# CONFIGURATION
# ============================================================

HERE = os.path.dirname(os.path.abspath(__file__))
SUMO_CONFIG = os.path.join(HERE, "simulation.sumocfg")

SUMO_BINARY = "sumo" if args.nogui else "sumo-gui"

AMBULANCE_ID = "ambulance"

SIMULATION_END = 300.0

SCRIPT_VERSION = "v1 (multi-hop V2V relay for out-of-range V2I preemption)"

RELAY_ENABLED = not args.no_relay
RUN_TAG = "no_relay" if args.no_relay else "relay"


# ------------------------------------------------------------
# V2V same-lane lane-change (unchanged from v2x_integrated)
# ------------------------------------------------------------

LANE_CHANGE_DETECTION_DISTANCE = 100.0
PASS_DISTANCE = -10.0
MIN_FRONT_GAP = 15.0
MIN_REAR_GAP = 15.0
LANE_CHANGE_DURATION = 3.0
RETURN_DURATION = 4.0
YIELD_SPEED_DELTA = 1.0
YIELD_SPEED_FLOOR = 2.0


# ------------------------------------------------------------
# V2V intersection-yield (unchanged from v2x_integrated)
# ------------------------------------------------------------

INTERSECTION_MERGE_EDGES = {
    "J2": ["J2N_J2", "J2S_J2"],
    "J3": ["J3N_J3", "J3S_J3"],
    "J4": ["J4N_J4", "J4S_J4"],
    "J5": ["J5N_J5", "J5S_J5"],
}

MAIN_EXIT_EDGE = {
    "J2": "J2_J3",
    "J3": "J3_J4",
    "J4": "J4_J5",
    "J5": None,
}

INTERSECTION_ZONE_DISTANCE = 80.0
INTERSECTION_HOLD_TRIGGER_DISTANCE = 40.0


# ------------------------------------------------------------
# V2I + multi-hop V2V relay (NEW - the point of this scenario)
# ------------------------------------------------------------

TLS_CONFIG = {
    "J2": {"approach": "J1_J2", "exit": "J2_J3"},
    "J3": {"approach": "J2_J3", "exit": "J3_J4"},
    "J4": {"approach": "J3_J4", "exit": "J4_J5"},
}

# Direct, one-hop radio range between a vehicle and a junction. Deliberately
# tighter than v2x_integrated's PREEMPTION_DISTANCE (100 m) - this is meant
# to be too short on its own to give useful advance warning at this
# corridor's 300 m junction spacing, so the relay mechanism has something
# real to demonstrate.
V2I_DIRECT_RANGE = 70.0

# Vehicle-to-vehicle radio range for forwarding a request one hop.
V2V_RELAY_RANGE = 100.0

# Maximum number of relay hops a request may travel before reaching a
# junction (ambulance -> relay -> relay -> ... -> junction). Bounded so
# the BFS below is cheap and so results stay physically plausible (a
# packet doesn't hop across the entire city for free).
MAX_RELAY_HOPS = 3

# Modelled store-and-forward delay, in SIMULATION STEPS, charged PER HOP
# before the relayed request is actually allowed to activate a junction's
# emergency phase (0.5 s/step here, so 1 step = 0.5 s of processing +
# re-broadcast delay per relaying node - a reasonable order of magnitude
# for a cheap embedded radio, and enough to show up distinctly in the
# console log without materially changing the headline result).
STORE_AND_FORWARD_DELAY_STEPS = 1


# ============================================================
# SUMO
# ============================================================

if "SUMO_HOME" in os.environ:
    tools = os.path.join(os.environ["SUMO_HOME"], "tools")
    if tools not in sys.path:
        sys.path.append(tools)


print()
print("=" * 78)
print("            V2X RELAY TEST - MULTI-HOP V2V SIGNAL PREEMPTION")
print(f"            Controller build: {SCRIPT_VERSION}")
print(f"            Relay: {'ENABLED' if RELAY_ENABLED else 'DISABLED (direct range only)'}")
print("=" * 78)
print()

print("[SYSTEM] Starting SUMO...")

sumo_cmd = [SUMO_BINARY, "-c", SUMO_CONFIG, "--step-length", "0.5"]

if args.seed is not None:
    sumo_cmd += ["--seed", str(args.seed)]

traci.start(sumo_cmd)

print("[SYSTEM] Connected to SUMO!")
print()


# ============================================================
# HELPERS
# ============================================================

def exists(vehicle_id):
    return vehicle_id in traci.vehicle.getIDList()


def _edge_length(edge_id):
    try:
        return traci.lane.getLength(f"{edge_id}_0")
    except traci.TraCIException:
        return 0.0


def route_distance(ambulance_id, other_id):
    """Forward-along-route distance; identical to v2x_integrated's
    proven version. Returns None when not comparable this step."""

    try:
        ambulance_route = traci.vehicle.getRoute(ambulance_id)
        other_route = traci.vehicle.getRoute(other_id)

        if ambulance_route != other_route:
            return None

        ambulance_edge = traci.vehicle.getRoadID(ambulance_id)
        other_edge = traci.vehicle.getRoadID(other_id)

        if ambulance_edge.startswith(":") or other_edge.startswith(":"):
            return None

        ambulance_index = traci.vehicle.getRouteIndex(ambulance_id)
        other_index = traci.vehicle.getRouteIndex(other_id)

        ambulance_pos = traci.vehicle.getLanePosition(ambulance_id)
        other_pos = traci.vehicle.getLanePosition(other_id)

        if ambulance_index < 0 or other_index < 0:
            return None

        if other_index == ambulance_index:
            return other_pos - ambulance_pos

        if other_index < ambulance_index:
            return -999999.0

        remaining = _edge_length(ambulance_route[ambulance_index]) - ambulance_pos
        distance = remaining

        for i in range(ambulance_index + 1, other_index):
            distance += _edge_length(ambulance_route[i])

        distance += other_pos

        return distance

    except traci.TraCIException:
        return None


def euclidean(pos_a, pos_b):
    return ((pos_a[0] - pos_b[0]) ** 2 + (pos_a[1] - pos_b[1]) ** 2) ** 0.5


def discover_tls_phases(tls_id, approach_edge):
    """Identical to v2x_integrated's proven version."""

    normal_phase = traci.trafficlight.getPhase(tls_id)
    controlled_links = traci.trafficlight.getControlledLinks(tls_id)

    target_indices = set()

    for signal_index, link_list in enumerate(controlled_links):
        for link in link_list:
            in_lane = link[0]
            in_edge = in_lane.rsplit("_", 1)[0]
            if in_edge == approach_edge:
                target_indices.add(signal_index)

    emergency_phase = None

    try:
        logic = traci.trafficlight.getAllProgramLogics(tls_id)[0]
        phases = logic.getPhases()

        for phase_index, phase in enumerate(phases):
            state = phase.state
            if not target_indices:
                continue
            if all(
                idx < len(state) and state[idx] in ("G", "g")
                for idx in target_indices
            ):
                emergency_phase = phase_index
                break

    except traci.TraCIException:
        phases = []

    return emergency_phase, normal_phase, sorted(target_indices), len(phases)


def find_relay_path(ambulance_pos, junction_pos, exists_cache):
    """
    Bounded BFS flood from the ambulance over the live V2V connectivity
    graph, looking for a chain of real vehicles that can carry a V2I
    request to `junction_pos` within MAX_RELAY_HOPS hops.

    Returns (reachable, hop_count, chain) where:
        reachable  - True/False
        hop_count  - 0 for direct range, 1+ for relayed, None if unreachable
        chain      - list of relaying vehicle IDs (empty for direct range)

    Shortest path by hop count (not distance) since BFS explores level by
    level and returns on the first hit - this models "whichever relay
    chain is shortest wins", which is the right behaviour for a flooded
    broadcast request (every reachable vehicle re-broadcasts once, and
    the junction acts on the first copy that arrives).
    """

    if euclidean(ambulance_pos, junction_pos) <= V2I_DIRECT_RANGE:
        return True, 0, []

    if not RELAY_ENABLED:
        return False, None, None

    visited = {AMBULANCE_ID}
    # frontier entries: (vehicle_id, position, chain_so_far)
    frontier = [(AMBULANCE_ID, ambulance_pos, [])]

    for hop in range(1, MAX_RELAY_HOPS + 1):

        next_frontier = []

        for node_id, node_pos, chain in frontier:

            for other_id in exists_cache:

                if other_id in visited:
                    continue

                try:
                    other_pos = traci.vehicle.getPosition(other_id)
                except traci.TraCIException:
                    continue

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


# ============================================================
# PRE-FLIGHT: DISCOVER JUNCTION POSITIONS + TLS PHASES
# ============================================================

print("-" * 78)
print("PRE-FLIGHT DISCOVERY")
print("-" * 78)

junction_positions = {}

for jx in INTERSECTION_MERGE_EDGES.keys():
    try:
        junction_positions[jx] = traci.junction.getPosition(jx)
        print(f"[SYSTEM] Junction {jx} position: {junction_positions[jx]}")
    except traci.TraCIException:
        print(f"[SYSTEM] WARNING: junction {jx} not found in network.")

tls_runtime = {}

for tls_id, cfg in TLS_CONFIG.items():
    try:
        emergency_phase, normal_phase, target_indices, phase_count = (
            discover_tls_phases(tls_id, cfg["approach"])
        )

        print(
            f"[SYSTEM] TLS {tls_id}: {phase_count} phases | "
            f"signal indices for {cfg['approach']} = {target_indices} | "
            f"normal phase = {normal_phase} | "
            f"emergency phase = {emergency_phase}"
        )

        if emergency_phase is None:
            print(
                f"[SYSTEM] WARNING: could not find a green phase for "
                f"{tls_id} approach {cfg['approach']}. "
                f"V2I preemption at {tls_id} will be DISABLED."
            )

        tls_runtime[tls_id] = {
            "approach": cfg["approach"],
            "exit": cfg["exit"],
            "emergency_phase": emergency_phase,
            "normal_phase": normal_phase,
            "active": False,
            "activated_via": None,     # "V2I" (direct) or "RELAY"
            "relay_chain": None,       # list of vehicle IDs, if relayed
            "pending_hops": None,      # relay hop count counting down
            "pending_chain": None,
        }

    except traci.TraCIException as error:
        print(f"[SYSTEM] WARNING: TLS {tls_id} not usable: {error}")
        tls_runtime[tls_id] = {
            "approach": cfg["approach"],
            "exit": cfg["exit"],
            "emergency_phase": None,
            "normal_phase": 0,
            "active": False,
            "activated_via": None,
            "relay_chain": None,
            "pending_hops": None,
            "pending_chain": None,
        }

print("-" * 78)
print()


# ============================================================
# STATE
# ============================================================

lane_change_states = {}
intersection_holds = {}

last_status_time = -1
ambulance_seen = False
ambulance_finished = False

# ---- metrics for compare_relay.py ----
ambulance_depart_time = None
ambulance_arrive_time = None
ambulance_stopped_s = 0.0
ambulance_stops = 0
_ambulance_was_stopped = False
STOP_SPEED_THRESHOLD = 0.3  # m/s


# ============================================================
# V2V LANE-CHANGE MODULE (unchanged from v2x_integrated)
# ============================================================

def process_lane_change(vehicle_id, current_time, ambulance_lane):

    if vehicle_id not in exists_cache:
        return

    try:
        vehicle_lane = traci.vehicle.getLaneIndex(vehicle_id)
        vehicle_edge = traci.vehicle.getRoadID(vehicle_id)
        vehicle_speed = traci.vehicle.getSpeed(vehicle_id)
    except traci.TraCIException:
        return

    entry = lane_change_states.get(vehicle_id)

    if entry is None:

        if vehicle_lane != ambulance_lane:
            return

        distance = route_distance(AMBULANCE_ID, vehicle_id)

        if distance is None or distance <= 0:
            return

        if distance > LANE_CHANGE_DETECTION_DISTANCE:
            return

        lane_change_states[vehicle_id] = {
            "state": "NORMAL",
            "original_lane": vehicle_lane,
            "yield_lane": None,
            "lane_change_time": None,
            "ambulance_pass_time": None,
        }

        entry = lane_change_states[vehicle_id]

    state = entry["state"]

    if state == "NORMAL":

        distance = route_distance(AMBULANCE_ID, vehicle_id)

        if distance is None or distance <= 0:
            del lane_change_states[vehicle_id]
            return

        try:
            lane_count = traci.edge.getLaneNumber(vehicle_edge)
        except traci.TraCIException:
            lane_count = 1

        original_lane = entry["original_lane"]

        candidate_lane = None

        if original_lane + 1 < lane_count:
            candidate_lane = original_lane + 1
        elif original_lane - 1 >= 0:
            candidate_lane = original_lane - 1

        if candidate_lane is None:
            return

        candidate_lane_id = f"{vehicle_edge}_{candidate_lane}"

        front_gap = float("inf")
        rear_gap = float("inf")

        try:
            vehicle_x, vehicle_y = traci.vehicle.getPosition(vehicle_id)
        except traci.TraCIException:
            return

        for other_id in exists_cache:

            if other_id in (vehicle_id, AMBULANCE_ID):
                continue

            try:
                if traci.vehicle.getLaneID(other_id) != candidate_lane_id:
                    continue
                other_x, other_y = traci.vehicle.getPosition(other_id)
            except traci.TraCIException:
                continue

            gap = other_x - vehicle_x

            if gap > 0:
                front_gap = min(front_gap, gap)
            elif gap < 0:
                rear_gap = min(rear_gap, abs(gap))

        if front_gap < MIN_FRONT_GAP or rear_gap < MIN_REAR_GAP:
            return

        print()
        print("=" * 78)
        print("[V2V] EMERGENCY MESSAGE RECEIVED")
        print(f"[V2V] Ambulance detected behind vehicle '{vehicle_id}'")
        print(f"[V2V] Separation: {distance:.1f} m")
        print("[V2V] Vehicle is blocking the emergency lane.")
        print(
            f"[V2V] Lane {candidate_lane} available "
            f"(front gap {front_gap:.1f} m, rear gap {rear_gap:.1f} m)."
        )
        print(f"[V2V] Preparing lane change: {original_lane} -> {candidate_lane}")
        print("=" * 78)

        yield_speed = max(vehicle_speed - YIELD_SPEED_DELTA, YIELD_SPEED_FLOOR)

        try:
            traci.vehicle.setSpeed(vehicle_id, yield_speed)
        except traci.TraCIException:
            pass

        try:
            traci.vehicle.changeLane(vehicle_id, candidate_lane, LANE_CHANGE_DURATION)
            entry["state"] = "CHANGING"
            entry["yield_lane"] = candidate_lane
            entry["lane_change_time"] = current_time
            print(f"[V2V] Lane-change command sent for '{vehicle_id}'.")
        except traci.TraCIException as error:
            print(f"[V2V] Lane-change command failed for '{vehicle_id}': {error}")

    elif state == "CHANGING":

        if vehicle_lane == entry["yield_lane"]:
            entry["state"] = "YIELDING"
            print()
            print("=" * 78)
            print(f"[V2V] *** VEHICLE '{vehicle_id}' CHANGED LANES ***")
            print("[V2V] Emergency lane is now clear.")
            print("=" * 78)
        else:
            if (
                entry["lane_change_time"] is not None
                and current_time - entry["lane_change_time"] >= LANE_CHANGE_DURATION + 2.0
            ):
                try:
                    traci.vehicle.changeLane(
                        vehicle_id, entry["yield_lane"], LANE_CHANGE_DURATION
                    )
                    entry["lane_change_time"] = current_time
                    print(
                        f"[V2V] '{vehicle_id}' still in lane {vehicle_lane} "
                        f"after timeout - re-issuing lane-change command."
                    )
                except traci.TraCIException:
                    pass

    elif state == "YIELDING":

        yield_speed = max(vehicle_speed, YIELD_SPEED_FLOOR)

        try:
            traci.vehicle.setSpeed(vehicle_id, yield_speed)
        except traci.TraCIException:
            pass

        distance = route_distance(AMBULANCE_ID, vehicle_id)

        if (
            distance is not None
            and distance < PASS_DISTANCE
            and entry["ambulance_pass_time"] is None
        ):
            entry["ambulance_pass_time"] = current_time
            print()
            print("=" * 78)
            print(f"[V2V] *** AMBULANCE HAS PASSED '{vehicle_id}' ***")
            print("=" * 78)

            try:
                traci.vehicle.changeLane(vehicle_id, entry["original_lane"], RETURN_DURATION)
                entry["state"] = "RETURNING"
            except traci.TraCIException as error:
                print(f"[V2V] Return lane-change failed for '{vehicle_id}': {error}")

    elif state == "RETURNING":

        if vehicle_lane == entry["original_lane"]:
            entry["state"] = "COMPLETE"
            print()
            print("=" * 78)
            print(f"[V2V] *** VEHICLE '{vehicle_id}' RETURNED TO LANE ***")
            print("=" * 78)
            try:
                traci.vehicle.setSpeed(vehicle_id, -1)
            except traci.TraCIException:
                pass

    elif state == "COMPLETE":
        del lane_change_states[vehicle_id]


# ============================================================
# V2V INTERSECTION-YIELD MODULE (unchanged from v2x_integrated)
# ============================================================

def process_intersections(current_time, ambulance_pos, ambulance_edge):

    for jx, merge_edges in INTERSECTION_MERGE_EDGES.items():

        junction_pos = junction_positions.get(jx)
        if junction_pos is None:
            continue

        ambulance_distance = euclidean(ambulance_pos, junction_pos)
        zone_active = ambulance_distance <= INTERSECTION_ZONE_DISTANCE

        if zone_active:
            for merge_edge in merge_edges:
                try:
                    lane_id = f"{merge_edge}_0"
                    lane_length = traci.lane.getLength(lane_id)
                except traci.TraCIException:
                    continue

                for vehicle_id in exists_cache:

                    if vehicle_id in intersection_holds:
                        continue

                    try:
                        if traci.vehicle.getRoadID(vehicle_id) != merge_edge:
                            continue
                        veh_pos = traci.vehicle.getLanePosition(vehicle_id)
                    except traci.TraCIException:
                        continue

                    distance_to_junction_end = lane_length - veh_pos

                    if distance_to_junction_end <= INTERSECTION_HOLD_TRIGGER_DISTANCE:
                        print()
                        print("=" * 78)
                        print("[V2V] Intersection vehicle detected")
                        print(f"[V2V] '{vehicle_id}' approaching {jx}, vehicle yielding")
                        try:
                            traci.vehicle.setSpeed(vehicle_id, 0.0)
                        except traci.TraCIException:
                            pass
                        print("=" * 78)

                        intersection_holds[vehicle_id] = jx

        exit_edge = MAIN_EXIT_EDGE.get(jx)
        passed = False

        if exit_edge is not None and ambulance_edge == exit_edge:
            passed = True
        elif jx == "J5" and ambulance_seen and not exists(AMBULANCE_ID):
            passed = True

        if passed:
            held_here = [v for v, held_jx in intersection_holds.items() if held_jx == jx]
            for vehicle_id in held_here:
                if vehicle_id in exists_cache:
                    try:
                        traci.vehicle.setSpeed(vehicle_id, -1)
                    except traci.TraCIException:
                        pass
                    print(f"[V2V] Ambulance cleared intersection {jx}, '{vehicle_id}' resumed")
                del intersection_holds[vehicle_id]


# ============================================================
# V2I + MULTI-HOP V2V RELAY MODULE (NEW)
# ============================================================

def process_v2i(current_time, ambulance_pos, ambulance_edge, exists_cache):

    for tls_id, runtime in tls_runtime.items():

        if runtime["emergency_phase"] is None:
            continue

        approach = runtime["approach"]
        exit_edge = runtime["exit"]
        junction_pos = junction_positions.get(tls_id)

        if runtime["active"]:

            if ambulance_edge == approach:
                try:
                    current_phase = traci.trafficlight.getPhase(tls_id)
                    if current_phase != runtime["emergency_phase"]:
                        traci.trafficlight.setPhase(tls_id, runtime["emergency_phase"])
                except traci.TraCIException:
                    pass

            elif ambulance_edge == exit_edge:
                print()
                print("=" * 78)
                print(f"[V2I] AMBULANCE PASSED {tls_id}")
                print("[V2I] Releasing emergency preemption...")
                try:
                    traci.trafficlight.setPhase(tls_id, runtime["normal_phase"])
                    runtime["active"] = False
                    runtime["activated_via"] = None
                    runtime["relay_chain"] = None
                    print(f"[V2I] {tls_id} returned to NORMAL OPERATION")
                except traci.TraCIException as error:
                    print(f"[V2I] ERROR releasing emergency phase at {tls_id}: {error}")
                print("=" * 78)

            continue

        if junction_pos is None or ambulance_edge != approach:
            runtime["pending_hops"] = None
            continue

        # ---- not yet active and on the right approach edge: see if a
        # request can reach this junction right now, direct or relayed ----

        reachable, hop_count, chain = find_relay_path(
            ambulance_pos, junction_pos, exists_cache
        )

        if not reachable:
            runtime["pending_hops"] = None
            continue

        if hop_count == 0:
            # Direct range - activates immediately, no forwarding delay.
            via = "V2I"
            print()
            print("=" * 78)
            print(f"[V2I] EMERGENCY MESSAGE RECEIVED ({tls_id})")
            print(
                f"[V2I] Ambulance within direct range "
                f"({V2I_DIRECT_RANGE:.0f} m) of {tls_id}"
            )
            print(f"[V2I] Requesting emergency phase {runtime['emergency_phase']} at {tls_id}")

            try:
                traci.trafficlight.setPhase(tls_id, runtime["emergency_phase"])
                runtime["active"] = True
                runtime["activated_via"] = via
                print(f"[V2I] {tls_id} EMERGENCY GREEN ACTIVATED")
            except traci.TraCIException as error:
                print(f"[V2I] ERROR activating emergency phase at {tls_id}: {error}")

            print("=" * 78)
            continue

        # ---- relayed: charge STORE_AND_FORWARD_DELAY_STEPS * hop_count
        # steps of forwarding delay before it actually lands ----

        if (
            runtime["pending_hops"] is None
            or runtime["pending_chain"] != chain
        ):
            runtime["pending_hops"] = hop_count * STORE_AND_FORWARD_DELAY_STEPS
            runtime["pending_chain"] = chain

            print()
            print("=" * 78)
            print(f"[RELAY] OUT-OF-RANGE V2I REQUEST - FLOODING FOR A RELAY PATH ({tls_id})")
            print(
                f"[RELAY] Ambulance is {euclidean(ambulance_pos, junction_pos):.1f} m "
                f"from {tls_id} (direct range is only {V2I_DIRECT_RANGE:.0f} m)"
            )
            print(f"[RELAY] Path found: ambulance -> {' -> '.join(chain)} -> {tls_id}")
            print(
                f"[RELAY] {hop_count} hop(s), "
                f"{runtime['pending_hops']} step(s) store-and-forward delay"
            )
            print("=" * 78)

        runtime["pending_hops"] -= 1

        if runtime["pending_hops"] <= 0:
            via = "RELAY"
            print()
            print("=" * 78)
            print(f"[RELAY] REQUEST DELIVERED TO {tls_id} VIA {hop_count}-HOP RELAY")
            print(f"[RELAY] Relay chain: {' -> '.join(chain)}")
            print(f"[RELAY] Requesting emergency phase {runtime['emergency_phase']} at {tls_id}")

            try:
                traci.trafficlight.setPhase(tls_id, runtime["emergency_phase"])
                runtime["active"] = True
                runtime["activated_via"] = via
                runtime["relay_chain"] = chain
                print(f"[RELAY] {tls_id} EMERGENCY GREEN ACTIVATED (via relay)")
            except traci.TraCIException as error:
                print(f"[RELAY] ERROR activating emergency phase at {tls_id}: {error}")

            print("=" * 78)
            runtime["pending_hops"] = None
            runtime["pending_chain"] = None


# ============================================================
# MAIN LOOP
# ============================================================

exists_cache = []
os.makedirs(os.path.join(HERE, "results"), exist_ok=True)

try:

    while (
        traci.simulation.getMinExpectedNumber() > 0
        and traci.simulation.getTime() < SIMULATION_END
    ):

        traci.simulationStep()

        current_time = traci.simulation.getTime()
        exists_cache = traci.vehicle.getIDList()

        for vehicle_id in exists_cache:
            if vehicle_id == AMBULANCE_ID:
                continue
            try:
                traci.vehicle.setLaneChangeMode(vehicle_id, 512)
            except traci.TraCIException:
                pass

        if AMBULANCE_ID in exists_cache:
            if not ambulance_seen:
                ambulance_depart_time = current_time
            ambulance_seen = True

        if ambulance_seen and AMBULANCE_ID not in exists_cache:

            if ambulance_arrive_time is None:
                ambulance_arrive_time = current_time

            for tls_id, runtime in tls_runtime.items():
                if runtime["active"] and runtime["emergency_phase"] is not None:
                    try:
                        traci.trafficlight.setPhase(tls_id, runtime["normal_phase"])
                    except traci.TraCIException:
                        pass
                    runtime["active"] = False
                    runtime["activated_via"] = None
                    runtime["relay_chain"] = None
                    print(f"[V2I] {tls_id} returned to NORMAL OPERATION (ambulance finished route)")

            for vehicle_id in list(intersection_holds.keys()):
                if vehicle_id in exists_cache:
                    try:
                        traci.vehicle.setSpeed(vehicle_id, -1)
                    except traci.TraCIException:
                        pass
                del intersection_holds[vehicle_id]

            if not ambulance_finished:
                print()
                print("=" * 78)
                print("[SYSTEM] Ambulance has completed its route.")
                print("=" * 78)
                ambulance_finished = True

            continue

        if AMBULANCE_ID not in exists_cache:
            continue

        ambulance_lane = traci.vehicle.getLaneIndex(AMBULANCE_ID)
        ambulance_edge = traci.vehicle.getRoadID(AMBULANCE_ID)
        ambulance_speed = traci.vehicle.getSpeed(AMBULANCE_ID)
        ambulance_pos = traci.vehicle.getPosition(AMBULANCE_ID)

        # ---- stopped-time / stop-count tracking ----
        if ambulance_speed < STOP_SPEED_THRESHOLD:
            ambulance_stopped_s += 0.5  # step-length
            if not _ambulance_was_stopped:
                ambulance_stops += 1
                _ambulance_was_stopped = True
        else:
            _ambulance_was_stopped = False

        if last_status_time < 0 or current_time - last_status_time >= 1.0:

            print()
            print("-" * 78)
            print(f"TIME          : {current_time:.1f} s")
            print(
                f"AMBULANCE     : {ambulance_edge} | "
                f"lane {ambulance_lane} | {ambulance_speed:.2f} m/s"
            )

            active_tls = [t for t, r in tls_runtime.items() if r["active"]]
            if active_tls:
                tagged = ", ".join(
                    f"{t}({tls_runtime[t]['activated_via']})" for t in active_tls
                )
                print(f"V2I/RELAY ACT : {tagged}")

            pending_tls = [
                t for t, r in tls_runtime.items()
                if r["pending_hops"] is not None and not r["active"]
            ]
            if pending_tls:
                print(f"RELAY PENDING : {', '.join(pending_tls)}")

            last_status_time = current_time

        for vehicle_id in exists_cache:
            if vehicle_id == AMBULANCE_ID:
                continue
            process_lane_change(vehicle_id, current_time, ambulance_lane)

        process_intersections(current_time, ambulance_pos, ambulance_edge)
        process_v2i(current_time, ambulance_pos, ambulance_edge, exists_cache)


except traci.TraCIException as error:
    print()
    print("=" * 78)
    print(f"[SYSTEM] TraCI error: {error}")
    print("=" * 78)

except traci.exceptions.FatalTraCIError:
    # Raised when the SUMO-GUI window is closed by hand (e.g. right after
    # the ambulance finishes, while background traffic is still looping).
    # FatalTraCIError is NOT a subclass of TraCIException, so it needs its
    # own handler - otherwise this prints a scary traceback even though
    # the run already completed and the results below were already saved.
    print()
    print("[SYSTEM] SUMO window closed - exiting cleanly.")

except KeyboardInterrupt:
    print()
    print("[SYSTEM] Simulation stopped by user.")

finally:

    try:
        traci.close()
    except Exception:
        pass

    if ambulance_depart_time is not None:

        if ambulance_arrive_time is None:
            # Ambulance never finished its route (run interrupted, or the
            # simulation hit SIMULATION_END while it was still on the
            # corridor) - fall back to the configured end time so travel_s
            # is still a defined (if conservative/lower-bound) number
            # instead of crashing the metrics write.
            ambulance_arrive_time = SIMULATION_END

        travel_s = ambulance_arrive_time - ambulance_depart_time

        csv_path = os.path.join(HERE, "results", f"ambulance_{RUN_TAG}.csv")

        with open(csv_path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["ambulance", "travel_s", "stopped_s", "stops"])
            w.writerow(["amb1", f"{travel_s:.1f}", f"{ambulance_stopped_s:.1f}", ambulance_stops])

        print()
        print(f"[SYSTEM] Results written: {csv_path}")
        print(
            f"[SYSTEM] travel_s={travel_s:.1f}  "
            f"stopped_s={ambulance_stopped_s:.1f}  stops={ambulance_stops}"
        )

    print()
    print("=" * 78)
    print("                  V2X RELAY TEST FINISHED")
    print("=" * 78)
