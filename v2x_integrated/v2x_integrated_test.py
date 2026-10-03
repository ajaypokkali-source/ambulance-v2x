"""
v2x_integrated_test.py

THE SINGLE INTEGRATED V2X STRESS TEST.

This lives in its own self-contained folder (C:\\ambulance_v2x\\v2x_integrated)
with its own edges/nodes/routes/network/config, so it never touches
v2v_stress_test, tls_simulation or simulation. It combines everything
that was previously proven separately across those folders:

    A. The ORIGINAL V2V same-lane, deterministic lane-change baseline
       (v2v_stress_test.py) - generalized so it can fire for ANY vehicle
       that ends up directly ahead of the ambulance in its lane (not
       just a single hardcoded "blocker_1"), which is how blocker_1,
       blocker_2 and blocker_3 all get their own overtake maneuver in
       this one run without touching the original algorithm.

    B. A real V2V intersection-yield module: vehicles merging onto the
       corridor from side roads (J1N/J1S .. J4N/J4S) are held at the
       junction while the ambulance is inside the local V2V range, and
       released the moment the ambulance has cleared that junction.

    C. The ORIGINAL V2I signal-preemption logic (v2i_signal_test.py),
       generalized from "just J2" to J2, J3 AND J4 - each junction
       activates and releases its own emergency phase independently,
       exactly like the proven J2 behavior.

    D. The safe-gap check from v2x_corridor_v1.py (front/rear gap
       around a candidate lane) is applied before any generalized
       lane-change is issued, so a vehicle is never pushed into an
       occupied lane.

    E. I2I (Infrastructure-to-Infrastructure): the ORIGINAL concept
       from i2i_corridor_test.py (J2 tells J3 to prepare before the
       ambulance ever gets close to J3), generalized to chain down
       the WHOLE managed corridor (J2 -> J3 -> J4) instead of just
       one hardcoded pair. See the I2I MODULE section below for the
       exact trigger rule and its timing trade-off.

REQUIRED ONE-TIME SETUP
------------------------
J2/J3/J4 must exist as real traffic-light junctions for the V2I/I2I
part of this script to do anything. Run this once first, from this
same folder:

    python build_network.py

That produces network.net.xml from nodes.nod.xml + edges.edg.xml via
netconvert.

This folder never modifies anything in v2v_stress_test, tls_simulation
or simulation - it is a fully separate scenario.
"""

import os
import sys
import traci


# ============================================================
# CONFIGURATION
# ============================================================

SUMO_CONFIG = r"C:\ambulance_v2x\v2x_integrated\simulation.sumocfg"

SUMO_BINARY = "sumo-gui"

AMBULANCE_ID = "ambulance"

SIMULATION_END = 300.0

# Bump this whenever you edit this file and re-run, so the printed
# banner tells you at a glance whether SUMO is actually running the
# version you think it is (this mattered once already: a re-run with
# no code changes reproduces an IDENTICAL trace, because SUMO uses a
# fixed default random seed unless you pass --seed/--random).
SCRIPT_VERSION = "v9 (I2I properly distance-gated via extended detection radius, not immediate cascade)"


# ------------------------------------------------------------
# B / D. V2V same-lane lane-change (generalized baseline)
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
# B. V2V intersection-yield (side-road merges)
# ------------------------------------------------------------

# NOTE: J1 is deliberately excluded here. The ambulance departs
# directly onto J1_J2 (i.e. it starts already just past J1), so it
# never actually "approaches" J1 as a junction to cross - treating it
# like the others would arm and immediately release a hold on the
# very first simulation step, which is not a real yield event.

INTERSECTION_MERGE_EDGES = {
    "J2": ["J2N_J2", "J2S_J2"],
    "J3": ["J3N_J3", "J3S_J3"],
    "J4": ["J4N_J4", "J4S_J4"],
    "J5": ["J5N_J5", "J5S_J5"],
}

# Edge a vehicle ends up on once it has actually merged past that
# junction onto the main corridor - used to know when the ambulance
# has cleared a junction and any hold there should be released.
MAIN_EXIT_EDGE = {
    "J2": "J2_J3",
    "J3": "J3_J4",
    "J4": "J4_J5",
    "J5": None,
}

# Ambulance must be within this distance of a junction for that
# junction's V2V yield zone to be "armed".
INTERSECTION_ZONE_DISTANCE = 80.0

# A merging vehicle is held once it is within this distance of the
# end of its merge lane (i.e. close to actually entering the corridor).
INTERSECTION_HOLD_TRIGGER_DISTANCE = 40.0


# ------------------------------------------------------------
# C. V2I signal preemption (multi-junction)
# ------------------------------------------------------------

TLS_CONFIG = {
    "J2": {"approach": "J1_J2", "exit": "J2_J3"},
    "J3": {"approach": "J2_J3", "exit": "J3_J4"},
    "J4": {"approach": "J3_J4", "exit": "J4_J5"},
}

PREEMPTION_DISTANCE = 100.0


# ------------------------------------------------------------
# E. I2I (Infrastructure-to-Infrastructure) chain
# ------------------------------------------------------------

# Sequential corridor order. Once TLS_ORDER[i] activates (by direct V2I
# detection, or because IT was itself I2I-armed), it immediately sends
# an I2I message to TLS_ORDER[i+1] - but that message only ARMS the
# next junction with an EXTENDED detection radius (I2I_PREARM_DISTANCE,
# below); it does NOT switch that junction's lights right away. The
# armed junction still waits for the ambulance to actually be on its
# approach edge and within range before it flips its phase - it just
# uses the bigger, I2I-extended range instead of the normal
# PREEMPTION_DISTANCE, so it reacts sooner than an un-notified junction
# would. That is the real-world point of I2I: infrastructure telling
# infrastructure "expect this vehicle soon" so the next node can react
# with a wider safety margin, WITHOUT guessing a fixed offset or firing
# blind the instant the upstream junction sees the ambulance (which is
# what an earlier version of this file did, and why J3/J4 used to get
# prepared ~30s before the ambulance was anywhere near them).
TLS_ORDER = ["J2", "J3", "J4"]

# Detection radius used ONLY for a junction that has been I2I-armed by
# its upstream neighbor. Must be bigger than PREEMPTION_DISTANCE (so
# I2I genuinely reacts earlier than an un-notified junction would) and
# comfortably smaller than the corridor's junction-to-junction spacing
# (~300m here) so it still only fires once the ambulance is genuinely
# on that junction's own approach edge, not from the moment it departs.
I2I_PREARM_DISTANCE = 220.0


# ============================================================
# SUMO
# ============================================================

if "SUMO_HOME" in os.environ:

    tools = os.path.join(os.environ["SUMO_HOME"], "tools")

    if tools not in sys.path:

        sys.path.append(tools)


print()
print("=" * 78)
print("                V2X INTEGRATED EMERGENCY CORRIDOR TEST")
print(f"                Controller build: {SCRIPT_VERSION}")
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

def exists(vehicle_id):

    return vehicle_id in traci.vehicle.getIDList()


def _edge_length(edge_id):
    """
    Length of a real (non-internal) edge, in meters.

    WHY THIS EXISTS: TraCI does not expose edge length as a per-edge
    command at all - traci.edge.getLength() simply does not exist
    (calling it raises AttributeError: 'EdgeDomain' object has no
    attribute 'getLength'; a real TraCI error class like
    traci.TraCIException is never involved, so wrapping the call in
    "except traci.TraCIException" does NOT catch this - it crashes the
    whole script). Length has to be read through the LANE API instead;
    every lane on an edge shares that edge's length in SUMO, so lane 0
    of the edge is a safe, always-available stand-in.
    """

    try:
        return traci.lane.getLength(f"{edge_id}_0")
    except traci.TraCIException:
        return 0.0


def route_distance(ambulance_id, other_id):
    """
    Distance from the ambulance to `other_id` measured FORWARD along
    the ambulance's own route (same function as the proven
    v2v_stress_test.py baseline, generalized to any vehicle id).

    Positive  -> other_id is ahead of the ambulance.
    Negative  -> other_id is behind the ambulance.
    None      -> not comparable (different routes / vehicle gone / one
                 of the two is transiting an internal junction lane
                 right now - see note below).

    IMPLEMENTATION NOTE (v8): earlier attempts tried to CORRECT for a
    vehicle currently on an internal junction lane (roadID like
    ":J4_14") by substituting some approximate position for it (first
    the internal lane's own getLanePosition(), then a hardcoded 0.0 on
    "the next edge"). Both produced the same class of bug in testing -
    a bogus ~40m separation reading for one step right as a vehicle
    crossed a junction, occasionally firing a false detection or a
    false "ambulance passed" event - just with different numbers,
    because getRouteIndex()'s exact behavior for a vehicle mid-junction
    isn't something to guess at twice and hope differently.

    The robust fix: stop guessing. While EITHER vehicle is on an
    internal lane, this returns None - "not comparable right now" -
    instead of approximating anything. Every caller of this function
    re-checks every simulation step anyway, and transiting a junction's
    internal lane normally takes a single 0.5s step, so this only ever
    delays a detection or a pass-event by up to one step. That is a
    total non-issue functionally, and it fully eliminates the class of
    bug that kept resurfacing at junction crossings.
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
            # Same edge - direct local-position comparison is valid.
            return other_pos - ambulance_pos

        if other_index < ambulance_index:
            # `other_id` is at least one whole edge behind the ambulance.
            # Their lane positions are on different edges' local
            # coordinate systems, so a raw subtraction would be
            # meaningless (and could accidentally look like a small
            # POSITIVE number, which would wrongly flag a vehicle that
            # is nowhere near the ambulance as "blocking ahead").
            # Report an unambiguous large negative value instead: it
            # correctly reads as "far behind" everywhere this function
            # is used (never triggers a forward detection, and clearly
            # satisfies "ambulance has passed" checks).
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
    Works out, from the ACTUAL compiled network, which phase index
    gives the approach edge a green light ("emergency phase"), and
    records whatever phase is currently running as the "normal"
    phase to restore to afterwards.

    This is done dynamically (instead of hardcoding a phase number)
    because network.net.xml is generated by netconvert, which is
    free to number phases however it likes.
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
            "activated_via": None,   # "V2I" or "I2I", set once active
            "i2i_armed": False,      # extended-range watch, set by I2I
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
        }

print("-" * 78)
print()


# ============================================================
# STATE
# ============================================================

lane_change_states = {}
# vehicle_id -> {
#     "state": "NORMAL" | "CHANGING" | "YIELDING" | "RETURNING" | "COMPLETE",
#     "original_lane": int,
#     "yield_lane": int,
#     "lane_change_time": float | None,
#     "ambulance_pass_time": float | None,
# }

intersection_holds = {}
# vehicle_id -> junction_id currently holding it

last_status_time = -1
ambulance_seen = False
ambulance_finished = False


# ============================================================
# V2V LANE-CHANGE MODULE
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

    # --------------------------------------------------------
    # NOT YET TRACKED: only start tracking if this vehicle is
    # actually a candidate (same lane as ambulance, ahead of it,
    # within detection range). This is the SAME geometric test
    # as the proven baseline - it never triggers on adjacent-lane
    # traffic.
    # --------------------------------------------------------

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

    # ----------------------------------------------------------
    # STATE: NORMAL -> try to find a safe adjacent lane and go
    # ----------------------------------------------------------

    if state == "NORMAL":

        distance = route_distance(AMBULANCE_ID, vehicle_id)

        if distance is None or distance <= 0:
            # Ambulance already passed it before it could react.
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
            # No adjacent lane exists at all - nothing we can do.
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
            # Not safe yet - stay NORMAL and re-check next step.
            return

        print()
        print("=" * 78)
        print("[V2V] EMERGENCY MESSAGE RECEIVED")
        print(f"[V2V] Ambulance detected behind vehicle '{vehicle_id}'")
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

            # ------------------------------------------------------
            # SAFETY NET: if the requested change hasn't completed
            # within LANE_CHANGE_DURATION + a grace period (traffic
            # shifted, or the vehicle's own lane-changing model briefly
            # contested the request), re-issue it instead of leaving
            # the vehicle stuck in CHANGING for the rest of the run.
            # ------------------------------------------------------

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
    # STATE: YIELDING -> hold position until ambulance clears
    # ----------------------------------------------------------

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
            print("[V2V] Emergency vehicle cleared the yielding vehicle.")
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
                # Hand control back to SUMO's own car-following model
                # instead of leaving the vehicle pinned at a manual speed.
                traci.vehicle.setSpeed(vehicle_id, -1)
            except traci.TraCIException:
                pass

    # ----------------------------------------------------------
    # STATE: COMPLETE -> stop tracking, hand back to normal driving
    # ----------------------------------------------------------

    elif state == "COMPLETE":

        del lane_change_states[vehicle_id]


# ============================================================
# V2V INTERSECTION-YIELD MODULE
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
                        print(
                            f"[V2V] '{vehicle_id}' approaching {jx} on "
                            f"{merge_edge} "
                            f"({distance_to_junction_end:.1f} m from junction)"
                        )
                        print(
                            f"[V2V] Ambulance is {ambulance_distance:.1f} m "
                            f"from {jx}"
                        )
                        print(f"[V2V] Vehicle yielding at {jx}")

                        try:
                            traci.vehicle.setSpeed(vehicle_id, 0.0)
                            print(f"[V2V] '{vehicle_id}' stopped")
                        except traci.TraCIException:
                            pass

                        print("=" * 78)

                        intersection_holds[vehicle_id] = jx

        # ----------------------------------------------------
        # RELEASE: ambulance has cleared this junction
        # ----------------------------------------------------

        exit_edge = MAIN_EXIT_EDGE.get(jx)

        passed = False

        if exit_edge is not None and ambulance_edge == exit_edge:
            passed = True
        elif jx == "J5" and ambulance_seen and not exists(AMBULANCE_ID):
            passed = True

        if passed:

            held_here = [
                v for v, held_jx in intersection_holds.items()
                if held_jx == jx
            ]

            for vehicle_id in held_here:

                if vehicle_id in exists_cache:

                    try:
                        traci.vehicle.setSpeed(vehicle_id, -1)
                    except traci.TraCIException:
                        pass

                    print()
                    print(f"[V2V] Ambulance cleared intersection {jx}")
                    print(f"[V2V] '{vehicle_id}' resumed")

                del intersection_holds[vehicle_id]


# ============================================================
# V2I PREEMPTION MODULE (multi-junction)
# ============================================================

def process_v2i(current_time, ambulance_pos, ambulance_edge):

    for tls_id, runtime in tls_runtime.items():

        if runtime["emergency_phase"] is None:
            continue

        approach = runtime["approach"]
        exit_edge = runtime["exit"]

        junction_pos = junction_positions.get(tls_id)

        if junction_pos is not None:
            distance_to_tls = euclidean(ambulance_pos, junction_pos)
        else:
            distance_to_tls = float("inf")

        effective_distance = (
            I2I_PREARM_DISTANCE if runtime["i2i_armed"] else PREEMPTION_DISTANCE
        )

        if (
            not runtime["active"]
            and ambulance_edge == approach
            and distance_to_tls <= effective_distance
        ):

            via = "I2I" if runtime["i2i_armed"] else "V2I"

            print()
            print("=" * 78)
            print(f"[{via}] EMERGENCY MESSAGE RECEIVED ({tls_id})")
            print(
                f"[{via}] Ambulance detected {distance_to_tls:.1f} m "
                f"from {tls_id}"
                + (
                    f" (within I2I-armed range of {I2I_PREARM_DISTANCE:.0f} m)"
                    if via == "I2I"
                    else ""
                )
            )
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

                print(f"[{via}] {tls_id} EMERGENCY GREEN ACTIVATED")

            except traci.TraCIException as error:
                print(f"[{via}] ERROR activating emergency phase at {tls_id}:")
                print(error)

            print("=" * 78)

        elif runtime["active"]:

            if ambulance_edge == approach:

                try:
                    current_phase = traci.trafficlight.getPhase(tls_id)

                    if current_phase != runtime["emergency_phase"]:
                        traci.trafficlight.setPhase(
                            tls_id, runtime["emergency_phase"]
                        )

                except traci.TraCIException:
                    pass

            elif ambulance_edge == exit_edge:

                print()
                print("=" * 78)
                print(f"[V2I] AMBULANCE PASSED {tls_id}")
                print(f"[V2I] Ambulance is now on {exit_edge}")
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

            elif ambulance_edge.startswith(":"):

                pass  # ambulance inside the junction, keep phase active


# ============================================================
# E. I2I MODULE (infrastructure-to-infrastructure chain)
# ============================================================

def process_i2i(current_time):
    """
    Arms the NEXT junction down TLS_ORDER with an extended detection
    radius once the CURRENT one activates - it does not switch anyone's
    lights itself.

    Every step, for each consecutive pair (cur, nxt) in TLS_ORDER: if
    `cur` is currently active (however it got activated - direct V2I
    detection, or a previous I2I hop) and `nxt` is not yet armed or
    active, mark `nxt` as I2I-armed. process_v2i() is what actually
    reads that flag: an armed junction watches for the ambulance using
    I2I_PREARM_DISTANCE instead of the normal (smaller) PREEMPTION_
    DISTANCE, so it reacts to the ambulance's approach sooner than an
    un-notified junction would - but it still only switches once the
    ambulance is actually on its approach edge and within that range,
    never blind or on a fixed timer.

    This naturally cascades the whole chain (J2 -> J3 -> J4) from a
    single upstream activation, and only fires once per junction (the
    "not nxt armed/active" guard becomes False as soon as it's armed,
    so it won't be re-triggered every step).
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
            f"once the ambulance is actually within that range - "
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

        # ----------------------------------------------------------
        # Lock out autonomous strategic/speed-gain/cooperative lane
        # changes on every non-ambulance vehicle, every step (this is
        # the exact fix v2x_corridor_v1.py already used, and it's
        # needed here too: without it, a vehicle's own lane-changing
        # model can silently contest or ignore a TraCI-commanded
        # changeLane(), which is what left blocker_3 stuck mid-change
        # in testing). Mode 512 keeps safety/collision checks active
        # and only disables the autonomous changes, so this never
        # touches the ambulance and never disables safety.
        # ----------------------------------------------------------

        for vehicle_id in exists_cache:

            if vehicle_id == AMBULANCE_ID:
                continue

            try:
                traci.vehicle.setLaneChangeMode(vehicle_id, 512)
            except traci.TraCIException:
                pass

        if AMBULANCE_ID in exists_cache:
            ambulance_seen = True

        # --------------------------------------------------
        # If the ambulance has left the simulation, release
        # any preemption / holds still active and stop.
        # --------------------------------------------------

        if ambulance_seen and AMBULANCE_ID not in exists_cache:

            for tls_id, runtime in tls_runtime.items():

                if runtime["active"] and runtime["emergency_phase"] is not None:

                    try:
                        traci.trafficlight.setPhase(
                            tls_id, runtime["normal_phase"]
                        )
                    except traci.TraCIException:
                        pass

                    runtime["active"] = False
                    runtime["activated_via"] = None
                    runtime["i2i_armed"] = False

                    print(f"[V2I] {tls_id} returned to NORMAL OPERATION "
                          f"(ambulance finished route)")

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

        # ====================================================
        # STATUS OUTPUT
        # ====================================================

        if last_status_time < 0 or current_time - last_status_time >= 1.0:

            print()
            print("-" * 78)
            print(f"TIME          : {current_time:.1f} s")
            print(
                f"AMBULANCE     : {ambulance_edge} | "
                f"lane {ambulance_lane} | {ambulance_speed:.2f} m/s"
            )

            if lane_change_states:
                summary = ", ".join(
                    f"{v}={s['state']}" for v, s in lane_change_states.items()
                )
                print(f"V2V LANE MGMT : {summary}")

            if intersection_holds:
                summary = ", ".join(
                    f"{v}@{jx}" for v, jx in intersection_holds.items()
                )
                print(f"V2V HOLDS     : {summary}")

            active_tls = [t for t, r in tls_runtime.items() if r["active"]]
            armed_tls = [
                t for t, r in tls_runtime.items()
                if r["i2i_armed"] and not r["active"]
            ]

            if active_tls:
                tagged = ", ".join(
                    f"{t}({tls_runtime[t]['activated_via']})"
                    for t in active_tls
                )
                print(f"V2I/I2I ACTIVE: {tagged}")

            if armed_tls:
                print(
                    f"I2I ARMED     : {', '.join(armed_tls)} "
                    f"(watching at {I2I_PREARM_DISTANCE:.0f} m)"
                )

            if lane_change_states or intersection_holds or active_tls or armed_tls:
                print("[V2X] Corridor emergency coordination active")

            last_status_time = current_time

        # ====================================================
        # RUN THE THREE MODULES
        # ====================================================

        for vehicle_id in exists_cache:

            if vehicle_id == AMBULANCE_ID:
                continue

            process_lane_change(vehicle_id, current_time, ambulance_lane)

        process_intersections(current_time, ambulance_pos, ambulance_edge)

        process_v2i(current_time, ambulance_pos, ambulance_edge)

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

except KeyboardInterrupt:

    print()
    print("[SYSTEM] Simulation stopped by user.")

finally:

    try:
        traci.close()
    except Exception:
        pass

    print()
    print("=" * 78)
    print("              V2X INTEGRATED TEST FINISHED")
    print("=" * 78)
