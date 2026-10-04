"""
v2x_multi_test.py

BIGGER-NETWORK, MULTI-AMBULANCE V2X TEST.

This is a SEPARATE scenario from v2x_integrated - it lives in its own
folder (C:\\ambulance_v2x\\v2x_multi) with its own nodes/edges/routes/
network/config, and running it never reads or writes anything under
v2x_integrated, v2v_stress_test, tls_simulation or simulation. The
proven v2x_integrated\\v2x_integrated_test.py (v9) is NOT touched by
this file or by anything in this folder.

WHAT'S DIFFERENT FROM v2x_integrated (v9)
-------------------------------------------
Same four building blocks (V2V same-lane overtake, V2V intersection-
yield, V2I signal preemption, I2I infrastructure-to-infrastructure
pre-arming), generalized along two axes:

    1. NETWORK SIZE: 5 main junctions / 3 signals (J2,J3,J4)
       -> 12 main junctions / 10 signals (J2..J11). TLS_CONFIG,
       INTERSECTION_MERGE_EDGES, MAIN_EXIT_EDGE and TLS_ORDER are all
       now BUILT PROGRAMMATICALLY from a single MAIN_ORDER list
       instead of being hand-typed per junction, so this scales to any
       corridor length just by changing MAIN_ORDER.

    2. AMBULANCE COUNT: 1 -> 3 (AMBULANCE_IDS). Every module that used
       to compare everything against one hardcoded AMBULANCE_ID now
       works off a per-step "ambulance snapshot" (lane/edge/speed/pos
       of every ambulance CURRENTLY in the simulation) and tracks,
       per vehicle / per junction, WHICH ambulance it is reacting to:

         - lane_change_states[vehicle_id] gained an "ambulance_id"
           field: a blocker now yields to whichever ambulance is
           actually closing on it (the nearest one in its lane, in
           detection range) - not a fixed id. This also means a
           vehicle that already returned to its lane after one
           ambulance passed can be picked up again by a LATER
           ambulance further down the corridor, which is correct,
           real V2V behaviour (and does happen in this scenario,
           since ambulance_2 eventually catches up to blocker_1 too).

         - intersection_holds[vehicle_id] gained the same
           "ambulance_id" field, so a merging vehicle held for one
           ambulance is released as soon as THAT ambulance has
           cleared the junction - independent of any other ambulance
           that might also be near a DIFFERENT junction at the same
           time.

         - tls_runtime[tls_id]["active_ambulances"] replaces the old
           single boolean "active" as the real state: a SET of
           ambulance ids currently being served by that junction's
           emergency phase. The phase is requested the moment the set
           goes from empty to non-empty, and only released to normal
           once the set is empty again - so if two ambulances need the
           same junction back-to-back (or, in principle, at the same
           time), the light simply stays green for emergency traffic
           until BOTH have cleared, instead of snapping back to normal
           in between and immediately preempting again.

I2I itself (the arm-then-activate two-stage design from v9) is
UNCHANGED: it only ever looks at a junction's "is anyone currently
being served" boolean (now backed by that set instead of a single
flag), so it chains correctly across all 10 signals regardless of how
many ambulances are involved.

REQUIRED ONE-TIME SETUP
------------------------
J2..J11 must exist as real traffic-light junctions. Run this once
first, from this same folder:

    python build_network.py

That produces network.net.xml from nodes.nod.xml + edges.edg.xml via
netconvert.
"""

import os
import sys
import traci


# ============================================================
# CONFIGURATION
# ============================================================

SUMO_CONFIG = r"C:\ambulance_v2x\v2x_multi\simulation.sumocfg"

SUMO_BINARY = "sumo-gui"

AMBULANCE_IDS = ["ambulance_1", "ambulance_2", "ambulance_3"]

# DIAGNOSTIC ONLY (temporary): blocker_1/blocker_2 were reported missing
# entirely from a real run (never detected, never visible in the GUI),
# while blocker_3 worked fine. This list + the "first ever seen" /
# "never seen" reporting below (search BLOCKER_IDS) exists purely to
# find out, with certainty, whether SUMO is actually inserting them at
# all - it does not change any V2X behaviour.
BLOCKER_IDS = ["blocker_1", "blocker_2", "blocker_3"]

SIMULATION_END = 600.0

SCRIPT_VERSION = (
    "v1 (v2x_multi: 10-signal corridor, 3 ambulances - "
    "generalized from v2x_integrated v9, which this file never touches)"
)


# ------------------------------------------------------------
# CORRIDOR LAYOUT (programmatic - this is the one place that
# changes if the network ever gets bigger or smaller)
# ------------------------------------------------------------

# J1 .. J12 : J1 and J12 stay plain "priority" entry/exit junctions,
# exactly like J1/J5 in v2x_integrated. J2..J11 (10 of them) are the
# signalized junctions.
MAIN_ORDER = [f"J{i}" for i in range(1, 13)]

TLS_JUNCTIONS = MAIN_ORDER[1:-1]  # J2 .. J11 (10 signals)

TLS_CONFIG = {}
for _i in range(1, len(MAIN_ORDER) - 1):
    _jid = MAIN_ORDER[_i]
    TLS_CONFIG[_jid] = {
        "approach": f"{MAIN_ORDER[_i - 1]}_{_jid}",
        "exit": f"{_jid}_{MAIN_ORDER[_i + 1]}",
    }

# Junctions whose side-road merges get the V2V intersection-yield
# treatment: every TLS junction PLUS the terminal exit junction (J12),
# mirroring v2x_integrated exactly (there it was J2,J3,J4 + terminal
# J5). J1 is deliberately excluded for the same reason as before: the
# ambulances all depart already just past J1, onto J1_J2, so they
# never actually "approach" J1 as a junction to cross.
INTERSECTION_JUNCTIONS = TLS_JUNCTIONS + [MAIN_ORDER[-1]]

INTERSECTION_MERGE_EDGES = {
    jx: [f"{jx}N_{jx}", f"{jx}S_{jx}"] for jx in INTERSECTION_JUNCTIONS
}

# Edge a vehicle ends up on once it has merged past that junction onto
# the main corridor. The terminal junction (J12) has no such edge -
# None is also the generalized signal used below to mean "there is no
# downstream edge to watch; release this hold once its ambulance is
# simply gone from the simulation".
MAIN_EXIT_EDGE = {jx: TLS_CONFIG[jx]["exit"] for jx in TLS_JUNCTIONS}
MAIN_EXIT_EDGE[MAIN_ORDER[-1]] = None

INTERSECTION_ZONE_DISTANCE = 80.0
INTERSECTION_HOLD_TRIGGER_DISTANCE = 40.0


# ------------------------------------------------------------
# V2V same-lane lane-change (generalized baseline, same tuning
# as v2x_integrated - this part of the physics didn't change)
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
# V2I signal preemption
# ------------------------------------------------------------

PREEMPTION_DISTANCE = 100.0


# ------------------------------------------------------------
# I2I (Infrastructure-to-Infrastructure) chain
# ------------------------------------------------------------

# Sequential corridor order for the I2I chain - identical mechanism to
# v2x_integrated's TLS_ORDER, just 10 entries instead of 3.
TLS_ORDER = TLS_JUNCTIONS

I2I_PREARM_DISTANCE = 220.0  # same tuning as v2x_integrated (same
                              # 300 m junction-to-junction spacing)


# ============================================================
# SUMO
# ============================================================

if "SUMO_HOME" in os.environ:

    tools = os.path.join(os.environ["SUMO_HOME"], "tools")

    if tools not in sys.path:

        sys.path.append(tools)


print()
print("=" * 78)
print("           V2X MULTI-AMBULANCE / 10-SIGNAL CORRIDOR TEST")
print(f"           Controller build: {SCRIPT_VERSION}")
print("=" * 78)
print()

print("[SYSTEM] Starting SUMO...")

sumo_cmd = [
    SUMO_BINARY,
    "-c",
    SUMO_CONFIG,
    "--step-length",
    "0.5",
]

traci.start(sumo_cmd)

print("[SYSTEM] Connected to SUMO!")
print()


# ============================================================
# HELPERS
# ============================================================

def _edge_length(edge_id):
    """
    Length of a real (non-internal) edge, in meters. traci.edge.getLength()
    does not exist in TraCI at all - this is the fix already proven in
    v2x_integrated v8/v9; see the long comment there for the full story.
    """

    try:
        return traci.lane.getLength(f"{edge_id}_0")
    except traci.TraCIException:
        return 0.0


def route_distance(ambulance_id, other_id):
    """
    Distance from `ambulance_id` to `other_id` measured FORWARD along
    the ambulance's own route. Identical, unmodified logic to the
    proven v2x_integrated v8/v9 route_distance() - it already takes
    the ambulance id as a parameter, so generalizing to multiple
    ambulances needed no changes here at all, only in how callers
    choose WHICH ambulance id to pass in.

    Positive  -> other_id is ahead of the ambulance.
    Negative  -> other_id is behind the ambulance.
    None      -> not comparable (different routes / a vehicle is gone /
                 one of the two is transiting an internal junction lane
                 right now, for a single simulation step).
    """

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

    return (
        (pos_a[0] - pos_b[0]) ** 2 + (pos_a[1] - pos_b[1]) ** 2
    ) ** 0.5


def discover_tls_phases(tls_id, approach_edge):
    """
    Unmodified from v2x_integrated: works out, from the ACTUAL compiled
    network, which phase index gives the approach edge a green light
    ("emergency phase"), done dynamically because network.net.xml is
    generated by netconvert, which numbers phases however it likes.
    """

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
            "activated_via": None,       # "V2I" or "I2I" - set on first activation
            "i2i_armed": False,          # extended-range watch, set by I2I
            "active_ambulances": set(),  # WHO this junction is currently serving
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
            "i2i_armed": False,
            "active_ambulances": set(),
        }

print("-" * 78)
print()


# ============================================================
# STATE
# ============================================================

lane_change_states = {}
# vehicle_id -> {
#     "state": "NORMAL" | "CHANGING" | "YIELDING" | "RETURNING" | "COMPLETE",
#     "ambulance_id": str,       # WHICH ambulance this vehicle is yielding to
#     "original_lane": int,
#     "yield_lane": int,
#     "lane_change_time": float | None,
#     "ambulance_pass_time": float | None,
# }

intersection_holds = {}
# vehicle_id -> {"jx": junction_id, "ambulance_id": str}

last_status_time = -1

ambulance_seen = {aid: False for aid in AMBULANCE_IDS}
ambulance_finished = {aid: False for aid in AMBULANCE_IDS}

# DIAGNOSTIC ONLY (temporary, see BLOCKER_IDS above).
blocker_ever_seen = {bid: False for bid in BLOCKER_IDS}


# ============================================================
# V2V LANE-CHANGE MODULE
# ============================================================

def process_lane_change(vehicle_id, current_time, ambulance_snapshot):

    if vehicle_id not in exists_cache:
        return

    try:
        vehicle_lane = traci.vehicle.getLaneIndex(vehicle_id)
        vehicle_edge = traci.vehicle.getRoadID(vehicle_id)
        vehicle_speed = traci.vehicle.getSpeed(vehicle_id)
    except traci.TraCIException:
        return

    entry = lane_change_states.get(vehicle_id)

    # --------------------------------------------------------
    # NOT YET TRACKED: scan every ambulance CURRENTLY in the
    # simulation and pick the nearest one that is behind this
    # vehicle, in the same lane, within detection range. This is
    # the only genuinely new piece of logic versus v2x_integrated -
    # everything else in this function is the proven baseline,
    # just reading entry["ambulance_id"] instead of a single
    # hardcoded AMBULANCE_ID.
    # --------------------------------------------------------

    if entry is None:

        best_aid = None
        best_distance = None

        for aid, snap in ambulance_snapshot.items():

            if snap["lane"] != vehicle_lane:
                continue

            distance = route_distance(aid, vehicle_id)

            if distance is None or distance <= 0:
                continue

            if distance > LANE_CHANGE_DETECTION_DISTANCE:
                continue

            if best_distance is None or distance < best_distance:
                best_distance = distance
                best_aid = aid

        if best_aid is None:
            return

        lane_change_states[vehicle_id] = {
            "state": "NORMAL",
            "ambulance_id": best_aid,
            "original_lane": vehicle_lane,
            "yield_lane": None,
            "lane_change_time": None,
            "ambulance_pass_time": None,
        }

        entry = lane_change_states[vehicle_id]

    state = entry["state"]
    ambulance_id = entry["ambulance_id"]

    # ----------------------------------------------------------
    # STATE: NORMAL -> try to find a safe adjacent lane and go
    # ----------------------------------------------------------

    if state == "NORMAL":

        distance = route_distance(ambulance_id, vehicle_id)

        if ambulance_id not in exists_cache or distance is None or distance <= 0:
            # Ambulance already passed it, or finished its route
            # entirely, before this vehicle could react.
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

            if other_id == vehicle_id or other_id in AMBULANCE_IDS:
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
        print(f"[V2V] '{ambulance_id}' detected behind vehicle '{vehicle_id}'")
        print(f"[V2V] Separation: {distance:.1f} m")
        print("[V2V] Vehicle is blocking the emergency lane.")
        print("[V2V] Checking adjacent lane...")
        print(
            f"[V2V] Lane {candidate_lane} available "
            f"(front gap {front_gap:.1f} m, rear gap {rear_gap:.1f} m)."
        )
        print("[V2V] Vehicle instructed to yield.")
        print(
            f"[V2V] Preparing lane change: "
            f"{original_lane} -> {candidate_lane}"
        )
        print("=" * 78)

        yield_speed = max(vehicle_speed - YIELD_SPEED_DELTA, YIELD_SPEED_FLOOR)

        try:
            traci.vehicle.setSpeed(vehicle_id, yield_speed)
        except traci.TraCIException:
            pass

        try:
            traci.vehicle.changeLane(
                vehicle_id, candidate_lane, LANE_CHANGE_DURATION
            )

            entry["state"] = "CHANGING"
            entry["yield_lane"] = candidate_lane
            entry["lane_change_time"] = current_time

            print(f"[V2V] Lane-change command sent for '{vehicle_id}'.")

        except traci.TraCIException as error:
            print(f"[V2V] Lane-change command failed for '{vehicle_id}':")
            print(error)

    # ----------------------------------------------------------
    # STATE: CHANGING -> waiting for the lane index to update
    # ----------------------------------------------------------

    elif state == "CHANGING":

        if vehicle_lane == entry["yield_lane"]:

            entry["state"] = "YIELDING"

            print()
            print("=" * 78)
            print(f"[V2V] *** VEHICLE '{vehicle_id}' CHANGED LANES ***")
            print(
                f"[V2V] {vehicle_id}: lane "
                f"{entry['original_lane']} -> lane {entry['yield_lane']}"
            )
            print("[V2V] Emergency lane is now clear.")

            if entry["lane_change_time"] is not None:
                print(
                    f"[V2V] Lane-change time: "
                    f"{current_time - entry['lane_change_time']:.1f} s"
                )

            print("=" * 78)

        else:

            if (
                entry["lane_change_time"] is not None
                and current_time - entry["lane_change_time"]
                >= LANE_CHANGE_DURATION + 2.0
            ):

                try:
                    traci.vehicle.changeLane(
                        vehicle_id, entry["yield_lane"], LANE_CHANGE_DURATION
                    )
                    entry["lane_change_time"] = current_time

                    print(
                        f"[V2V] '{vehicle_id}' still in lane "
                        f"{vehicle_lane} after timeout - "
                        f"re-issuing lane-change command."
                    )

                except traci.TraCIException:
                    pass

    # ----------------------------------------------------------
    # STATE: YIELDING -> hold position until THIS vehicle's
    # ambulance has cleared it (or has simply finished its route
    # while this vehicle was still waiting - the safety net that
    # did not exist in v2x_integrated because there was always
    # exactly one ambulance to wait for).
    # ----------------------------------------------------------

    elif state == "YIELDING":

        yield_speed = max(vehicle_speed, YIELD_SPEED_FLOOR)

        try:
            traci.vehicle.setSpeed(vehicle_id, yield_speed)
        except traci.TraCIException:
            pass

        distance = route_distance(ambulance_id, vehicle_id)

        ambulance_gone = ambulance_id not in exists_cache

        if (
            entry["ambulance_pass_time"] is None
            and (ambulance_gone or (distance is not None and distance < PASS_DISTANCE))
        ):

            entry["ambulance_pass_time"] = current_time

            print()
            print("=" * 78)
            print(f"[V2V] *** AMBULANCE HAS PASSED '{vehicle_id}' ***")
            print(f"[V2V] ({ambulance_id} cleared this vehicle.)")
            print("[V2V] Preparing vehicle to return to original lane.")
            print("=" * 78)

            try:
                traci.vehicle.changeLane(
                    vehicle_id, entry["original_lane"], RETURN_DURATION
                )
                entry["state"] = "RETURNING"

            except traci.TraCIException as error:
                print(f"[V2V] Return lane-change failed for '{vehicle_id}':")
                print(error)

    # ----------------------------------------------------------
    # STATE: RETURNING
    # ----------------------------------------------------------

    elif state == "RETURNING":

        if vehicle_lane == entry["original_lane"]:

            entry["state"] = "COMPLETE"

            print()
            print("=" * 78)
            print(f"[V2V] *** VEHICLE '{vehicle_id}' RETURNED TO LANE ***")
            print(
                f"[V2V] {vehicle_id}: lane "
                f"{entry['yield_lane']} -> lane {entry['original_lane']}"
            )
            print("[V2V] V2V emergency maneuver complete.")
            print("=" * 78)

            try:
                traci.vehicle.setSpeed(vehicle_id, -1)
            except traci.TraCIException:
                pass

    # ----------------------------------------------------------
    # STATE: COMPLETE -> stop tracking. This vehicle can be
    # picked back up fresh by a LATER ambulance further down the
    # corridor (deliberately: see module docstring).
    # ----------------------------------------------------------

    elif state == "COMPLETE":

        del lane_change_states[vehicle_id]


# ============================================================
# V2V INTERSECTION-YIELD MODULE
# ============================================================

def process_intersections(current_time, ambulance_snapshot):

    for jx, merge_edges in INTERSECTION_MERGE_EDGES.items():

        junction_pos = junction_positions.get(jx)

        if junction_pos is None:
            continue

        # Nearest ambulance currently within this junction's V2V zone,
        # if any - that is the ambulance any NEW hold at this junction
        # will be recorded against.
        best_aid = None
        best_distance = None

        for aid, snap in ambulance_snapshot.items():

            distance = euclidean(snap["pos"], junction_pos)

            if distance <= INTERSECTION_ZONE_DISTANCE:
                if best_distance is None or distance < best_distance:
                    best_distance = distance
                    best_aid = aid

        zone_active = best_aid is not None

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
                        print(
                            f"[V2V] '{vehicle_id}' approaching {jx} on "
                            f"{merge_edge} "
                            f"({distance_to_junction_end:.1f} m from junction)"
                        )
                        print(
                            f"[V2V] '{best_aid}' is {best_distance:.1f} m "
                            f"from {jx}"
                        )
                        print(f"[V2V] Vehicle yielding at {jx}")

                        try:
                            traci.vehicle.setSpeed(vehicle_id, 0.0)
                            print(f"[V2V] '{vehicle_id}' stopped")
                        except traci.TraCIException:
                            pass

                        print("=" * 78)

                        intersection_holds[vehicle_id] = {
                            "jx": jx,
                            "ambulance_id": best_aid,
                        }

        # ----------------------------------------------------
        # RELEASE: for every vehicle held AT THIS junction,
        # release it once ITS OWN ambulance has either cleared
        # the junction (reached the downstream exit edge) or
        # simply left the simulation entirely (covers both the
        # normal "ambulance passed" case and, generalized from
        # v2x_integrated's special J5 rule, the terminal
        # junction where there is no downstream edge to watch).
        # ----------------------------------------------------

        exit_edge = MAIN_EXIT_EDGE.get(jx)

        held_here = [
            v for v, info in intersection_holds.items() if info["jx"] == jx
        ]

        for vehicle_id in held_here:

            info = intersection_holds[vehicle_id]
            aid = info["ambulance_id"]
            aid_snap = ambulance_snapshot.get(aid)

            passed = False

            if aid not in exists_cache:
                passed = True
            elif exit_edge is not None and aid_snap is not None and aid_snap["edge"] == exit_edge:
                passed = True

            if passed:

                if vehicle_id in exists_cache:

                    try:
                        traci.vehicle.setSpeed(vehicle_id, -1)
                    except traci.TraCIException:
                        pass

                    print()
                    print(f"[V2V] '{aid}' cleared intersection {jx}")
                    print(f"[V2V] '{vehicle_id}' resumed")

                del intersection_holds[vehicle_id]


# ============================================================
# V2I PREEMPTION MODULE (multi-junction, multi-ambulance)
# ============================================================

def process_v2i(current_time, ambulance_snapshot):

    for tls_id, runtime in tls_runtime.items():

        if runtime["emergency_phase"] is None:
            continue

        approach = runtime["approach"]
        exit_edge = runtime["exit"]

        junction_pos = junction_positions.get(tls_id)

        effective_distance = (
            I2I_PREARM_DISTANCE if runtime["i2i_armed"] else PREEMPTION_DISTANCE
        )

        # --------------------------------------------------
        # 1. NEW ACTIVATIONS: any ambulance that is on this
        #    junction's approach edge, within range, and not
        #    already being served by it.
        # --------------------------------------------------

        for aid, snap in ambulance_snapshot.items():

            if aid in runtime["active_ambulances"]:
                continue

            if snap["edge"] != approach:
                continue

            if junction_pos is not None:
                distance_to_tls = euclidean(snap["pos"], junction_pos)
            else:
                distance_to_tls = float("inf")

            if distance_to_tls > effective_distance:
                continue

            via = "I2I" if runtime["i2i_armed"] else "V2I"
            already_serving_someone = len(runtime["active_ambulances"]) > 0

            print()
            print("=" * 78)
            print(f"[{via}] EMERGENCY MESSAGE RECEIVED ({tls_id})")
            print(
                f"[{via}] '{aid}' detected {distance_to_tls:.1f} m "
                f"from {tls_id}"
                + (
                    f" (within I2I-armed range of {I2I_PREARM_DISTANCE:.0f} m)"
                    if via == "I2I"
                    else ""
                )
            )

            if already_serving_someone:

                print(
                    f"[{via}] {tls_id} already green for emergency traffic "
                    f"({sorted(runtime['active_ambulances'])}) - "
                    f"'{aid}' is joining the same window."
                )
                runtime["active_ambulances"].add(aid)

            else:

                print(
                    f"[{via}] Requesting emergency phase "
                    f"{runtime['emergency_phase']} at {tls_id}"
                )

                try:
                    traci.trafficlight.setPhase(
                        tls_id, runtime["emergency_phase"]
                    )
                    runtime["active"] = True
                    runtime["activated_via"] = via
                    runtime["active_ambulances"].add(aid)

                    print(f"[{via}] {tls_id} EMERGENCY GREEN ACTIVATED")

                except traci.TraCIException as error:
                    print(f"[{via}] ERROR activating emergency phase at {tls_id}:")
                    print(error)

            print("=" * 78)

        # --------------------------------------------------
        # 2. MAINTAIN / RELEASE: every ambulance currently
        #    being served by this junction.
        # --------------------------------------------------

        for aid in list(runtime["active_ambulances"]):

            if aid not in exists_cache:
                # Finished its whole route while still "on" this
                # junction's books - just drop it, nothing to release.
                runtime["active_ambulances"].discard(aid)
                continue

            snap = ambulance_snapshot.get(aid)

            if snap is None:
                continue

            edge = snap["edge"]

            if edge == approach:

                try:
                    current_phase = traci.trafficlight.getPhase(tls_id)

                    if current_phase != runtime["emergency_phase"]:
                        traci.trafficlight.setPhase(
                            tls_id, runtime["emergency_phase"]
                        )

                except traci.TraCIException:
                    pass

            elif edge == exit_edge:

                print()
                print("=" * 78)
                print(f"[V2I] '{aid}' PASSED {tls_id}")
                print(f"[V2I] '{aid}' is now on {exit_edge}")
                print("=" * 78)

                runtime["active_ambulances"].discard(aid)

            elif edge.startswith(":"):

                pass  # ambulance inside the junction, keep phase active

        # --------------------------------------------------
        # 3. REVERT: only once NOBODY is left being served.
        # --------------------------------------------------

        if runtime["active"] and not runtime["active_ambulances"]:

            print()
            print("=" * 78)
            print(f"[V2I] {tls_id} has no more emergency traffic to serve.")
            print("[V2I] Releasing emergency preemption...")

            try:
                traci.trafficlight.setPhase(
                    tls_id, runtime["normal_phase"]
                )
                runtime["active"] = False
                runtime["activated_via"] = None
                runtime["i2i_armed"] = False

                print(f"[V2I] {tls_id} returned to NORMAL OPERATION")

            except traci.TraCIException as error:
                print(f"[V2I] ERROR releasing emergency phase at {tls_id}:")
                print(error)

            print("=" * 78)


# ============================================================
# I2I MODULE (infrastructure-to-infrastructure chain)
# ============================================================

def process_i2i(current_time):
    """
    Unmodified from v2x_integrated v9: arms the NEXT junction down
    TLS_ORDER once the CURRENT one is actively serving ANY ambulance.
    This still works correctly with multiple ambulances in flight
    because it only ever reads tls_runtime[...]["active"], which is
    now backed by active_ambulances but kept in exact sync with it by
    process_v2i() (True the instant the set goes non-empty, False the
    instant it goes back to empty).
    """

    for i in range(len(TLS_ORDER) - 1):

        cur_id = TLS_ORDER[i]
        next_id = TLS_ORDER[i + 1]

        cur_runtime = tls_runtime.get(cur_id)
        next_runtime = tls_runtime.get(next_id)

        if cur_runtime is None or next_runtime is None:
            continue

        if not cur_runtime["active"]:
            continue

        if next_runtime["i2i_armed"] or next_runtime["active"]:
            continue

        if next_runtime["emergency_phase"] is None:
            continue

        next_runtime["i2i_armed"] = True

        print()
        print("=" * 78)
        print(f"[I2I] {cur_id} -> {next_id}")
        print("[I2I] EMERGENCY VEHICLE APPROACHING")
        print(
            f"[I2I] {next_id} ARMED - now watching its approach "
            f"({next_runtime['approach']}) out to "
            f"{I2I_PREARM_DISTANCE:.0f} m instead of the normal "
            f"{PREEMPTION_DISTANCE:.0f} m"
        )
        print(
            f"[I2I] {next_id} will activate its own emergency phase "
            f"once an ambulance is actually within that range - "
            f"not yet"
        )
        print("=" * 78)


# ============================================================
# MAIN LOOP
# ============================================================

exists_cache = []

try:

    while (
        traci.simulation.getMinExpectedNumber() > 0
        and traci.simulation.getTime() < SIMULATION_END
    ):

        traci.simulationStep()

        current_time = traci.simulation.getTime()
        exists_cache = traci.vehicle.getIDList()

        # DIAGNOSTIC ONLY (temporary, see BLOCKER_IDS above): record,
        # with certainty, the exact moment (if ever) SUMO actually
        # inserts each blocker, independent of anything the V2V module
        # does with it afterward.
        for _bid in BLOCKER_IDS:
            if not blocker_ever_seen[_bid] and _bid in exists_cache:
                blocker_ever_seen[_bid] = True
                print(f"[DEBUG] '{_bid}' INSERTED at t={current_time:.1f}s "
                      f"(pos={traci.vehicle.getLanePosition(_bid):.1f}, "
                      f"lane={traci.vehicle.getLaneIndex(_bid)})")

        # ----------------------------------------------------------
        # Lock out autonomous strategic/speed-gain/cooperative lane
        # changes on every non-ambulance vehicle, every step - same
        # fix proven necessary in v2x_integrated.
        # ----------------------------------------------------------

        for vehicle_id in exists_cache:

            if vehicle_id in AMBULANCE_IDS:
                continue

            try:
                traci.vehicle.setLaneChangeMode(vehicle_id, 512)
            except traci.TraCIException:
                pass

        # ----------------------------------------------------------
        # Build this step's ambulance snapshot (only ambulances that
        # currently exist), and track seen/finished per ambulance for
        # the one-time "completed its route" message. Unlike
        # v2x_integrated, there is NO early "continue" here when an
        # ambulance is absent - with 3 staggered ambulances, "no
        # ambulance present right now" is a perfectly normal state
        # (e.g. the gap between ambulance_2 finishing and ambulance_3
        # departing), and every module below already resolves its own
        # state cleanly against an empty or partial snapshot (that's
        # what the "ambulance_id not in exists_cache" safety nets
        # throughout this file are for).
        # ----------------------------------------------------------

        ambulance_snapshot = {}

        for aid in AMBULANCE_IDS:

            if aid in exists_cache:

                ambulance_seen[aid] = True

                try:
                    ambulance_snapshot[aid] = {
                        "lane": traci.vehicle.getLaneIndex(aid),
                        "edge": traci.vehicle.getRoadID(aid),
                        "speed": traci.vehicle.getSpeed(aid),
                        "pos": traci.vehicle.getPosition(aid),
                    }
                except traci.TraCIException:
                    pass

            else:

                if ambulance_seen[aid] and not ambulance_finished[aid]:

                    print()
                    print("=" * 78)
                    print(f"[SYSTEM] '{aid}' has completed its route.")
                    print("=" * 78)

                    ambulance_finished[aid] = True

        # ====================================================
        # STATUS OUTPUT (only while there's something to show,
        # otherwise a 600s run would print ~600 empty lines)
        # ====================================================

        active_tls = [t for t, r in tls_runtime.items() if r["active"]]
        armed_tls = [
            t for t, r in tls_runtime.items()
            if r["i2i_armed"] and not r["active"]
        ]

        anything_happening = (
            ambulance_snapshot or lane_change_states or intersection_holds
            or active_tls or armed_tls
        )

        if anything_happening and (
            last_status_time < 0 or current_time - last_status_time >= 1.0
        ):

            print()
            print("-" * 78)
            print(f"TIME          : {current_time:.1f} s")

            for aid, snap in ambulance_snapshot.items():
                print(
                    f"{aid:<14}: {snap['edge']} | "
                    f"lane {snap['lane']} | {snap['speed']:.2f} m/s"
                )

            if lane_change_states:
                summary = ", ".join(
                    f"{v}={s['state']}(->{s['ambulance_id']})"
                    for v, s in lane_change_states.items()
                )
                print(f"V2V LANE MGMT : {summary}")

            if intersection_holds:
                summary = ", ".join(
                    f"{v}@{info['jx']}(->{info['ambulance_id']})"
                    for v, info in intersection_holds.items()
                )
                print(f"V2V HOLDS     : {summary}")

            if active_tls:
                tagged = ", ".join(
                    f"{t}({tls_runtime[t]['activated_via']}:"
                    f"{sorted(tls_runtime[t]['active_ambulances'])})"
                    for t in active_tls
                )
                print(f"V2I/I2I ACTIVE: {tagged}")

            if armed_tls:
                print(
                    f"I2I ARMED     : {', '.join(armed_tls)} "
                    f"(watching at {I2I_PREARM_DISTANCE:.0f} m)"
                )

            print("[V2X] Corridor emergency coordination active")

            last_status_time = current_time

        # ====================================================
        # RUN THE FOUR MODULES
        # ====================================================

        for vehicle_id in exists_cache:

            if vehicle_id in AMBULANCE_IDS:
                continue

            process_lane_change(vehicle_id, current_time, ambulance_snapshot)

        process_intersections(current_time, ambulance_snapshot)

        process_v2i(current_time, ambulance_snapshot)

        process_i2i(current_time)


# ============================================================
# SHUTDOWN
# ============================================================

except traci.TraCIException as error:

    print()
    print("=" * 78)
    print("[SYSTEM] TraCI error:")
    print(error)
    print("=" * 78)

except traci.exceptions.FatalTraCIError as error:

    # NOTE: FatalTraCIError does NOT inherit from TraCIException - it
    # is a separate exception class entirely, so the clause above
    # never catches it (same class of gotcha as the earlier
    # traci.edge.getLength() AttributeError bug). Without this clause
    # it escapes as a raw, uncaught traceback AFTER the finally: block
    # below has already printed "TEST FINISHED" - which is exactly
    # what happened in testing.
    #
    # "Connection closed by SUMO" means SUMO itself hung up on this
    # script - almost always because the SUMO-GUI window was closed
    # (by hand, or by the OS) while the script was still running and
    # waiting out the rest of SIMULATION_END. It is not a bug in this
    # script. If every ambulance has already finished and only
    # background flow traffic is left, it's safe to close the window
    # early - just know that's why this message appears.
    print()
    print("=" * 78)
    print("[SYSTEM] SUMO closed the connection:")
    print(error)
    print(
        "[SYSTEM] This means the SUMO-GUI window was closed (or SUMO "
        "exited) while this script was still running - not a bug in "
        "this script. To see the script run all the way to its own "
        f"'TEST FINISHED' message (t={SIMULATION_END:.0f}s), leave "
        "the SUMO-GUI window open until the script exits on its own."
    )
    print("=" * 78)

except KeyboardInterrupt:

    print()
    print("[SYSTEM] Simulation stopped by user.")

finally:

    try:
        traci.close()
    except Exception:
        pass

    # DIAGNOSTIC ONLY (temporary, see BLOCKER_IDS above).
    print()
    print("-" * 78)
    print("[DEBUG] Blocker insertion summary:")
    for _bid in BLOCKER_IDS:
        status = "inserted at some point" if blocker_ever_seen[_bid] else "NEVER INSERTED"
        print(f"[DEBUG]   {_bid}: {status}")
    print("-" * 78)

    print()
    print("=" * 78)
    print("           V2X MULTI-AMBULANCE / 10-SIGNAL CORRIDOR TEST FINISHED")
    print("=" * 78)
