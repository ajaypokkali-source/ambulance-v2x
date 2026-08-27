import traci


# ============================================================
# V2V STRESS TEST
# ============================================================
#
# FEATURES
#
# 1. Ambulance emergency broadcast
# 2. Same-road V2V lane-change behavior
# 3. Intersection conflict detection
# 4. V2V emergency yield messages
# 5. Vehicles stop before conflicting intersections
# 6. Vehicles resume after ambulance clears
# 7. Multiple vehicles can yield simultaneously
# 8. Existing lane-change logic is preserved
#
# ============================================================


# ============================================================
# CONFIGURATION
# ============================================================

SIMULATION_END_TIME = 300.0

# How far ahead the controller begins preparing traffic
PREPARE_DISTANCE = 100.0

# Distance for same-road emergency behavior
EMERGENCY_DISTANCE = 25.0

# Intersection detection radius
INTERSECTION_DETECTION_DISTANCE = 100.0

# When the ambulance is this close to an intersection,
# conflicting vehicles receive the strongest emergency command.
INTERSECTION_EMERGENCY_DISTANCE = 70.0

# Safety gaps for lane changes
MIN_FRONT_GAP = 15.0
MIN_REAR_GAP = 15.0

# Lane-change duration
LANE_CHANGE_DURATION = 5.0

# Speed below which a vehicle is considered stopped
STOP_SPEED_THRESHOLD = 0.5


# ============================================================
# SUMO CONFIGURATION
# ============================================================

sumo_cmd = [
    "sumo-gui",
    "-c",
    r"C:\ambulance_v2x\v2v_stress_test\simulation.sumocfg"
]


# ============================================================
# INTERSECTION DEFINITIONS
# ============================================================
#
# Ambulance route:
#
# J1 -> J2 -> J3 -> J4 -> J5
#
# At each main intersection, traffic can approach from:
#
#   WEST
#   EAST
#   NORTH
#   SOUTH
#
# We explicitly describe the incoming edges.
#
# ============================================================

INTERSECTIONS = {

    "J1": {
        "x": 0.0,
        "incoming": [
            "J1N_J1",
            "J1S_J1",
            "J2_J1"
        ]
    },

    "J2": {
        "x": 300.0,
        "incoming": [
            "J1_J2",
            "J3_J2",
            "J2N_J2",
            "J2S_J2"
        ]
    },

    "J3": {
        "x": 600.0,
        "incoming": [
            "J2_J3",
            "J4_J3",
            "J3N_J3",
            "J3S_J3"
        ]
    },

    "J4": {
        "x": 900.0,
        "incoming": [
            "J3_J4",
            "J5_J4",
            "J4N_J4",
            "J4S_J4"
        ]
    },

    "J5": {
        "x": 1200.0,
        "incoming": [
            "J4_J5",
            "J5N_J5",
            "J5S_J5"
        ]
    }
}


# ============================================================
# START SUMO
# ============================================================

traci.start(sumo_cmd)

print()
print("=" * 70)
print("                  V2V STRESS TEST")
print("=" * 70)
print("Connected to SUMO!")
print()


# ============================================================
# STATE STORAGE
# ============================================================

known_vehicles = set()

pending_lane_changes = {}

# Vehicles currently being forced to yield for ambulance
yielding_vehicles = {}

# Tracks which intersection is currently being handled
active_intersection = None

# Tracks whether ambulance has already announced an intersection
announced_intersections = set()

# Tracks intersections already cleared
cleared_intersections = set()

# Prevents repeated emergency messages
emergency_broadcast_active = False


# ============================================================
# HELPER FUNCTIONS
# ============================================================


def get_vehicle_position(vehicle_id):

    try:
        return traci.vehicle.getPosition(vehicle_id)

    except traci.TraCIException:
        return None


def get_vehicle_speed(vehicle_id):

    try:
        return traci.vehicle.getSpeed(vehicle_id)

    except traci.TraCIException:
        return 0.0


def get_vehicle_road(vehicle_id):

    try:
        return traci.vehicle.getRoadID(vehicle_id)

    except traci.TraCIException:
        return ""


def get_vehicle_lane(vehicle_id):

    try:
        return traci.vehicle.getLaneID(vehicle_id)

    except traci.TraCIException:
        return ""


def get_vehicle_lane_index(vehicle_id):

    try:
        return traci.vehicle.getLaneIndex(vehicle_id)

    except traci.TraCIException:
        return 0


def get_next_intersection(ambulance_x):

    """
    Determine which main intersection the ambulance
    is approaching.

    J1 = x 0
    J2 = x 300
    J3 = x 600
    J4 = x 900
    J5 = x 1200
    """

    ordered = [
        ("J1", 0.0),
        ("J2", 300.0),
        ("J3", 600.0),
        ("J4", 900.0),
        ("J5", 1200.0)
    ]

    for junction_id, junction_x in ordered:

        distance = junction_x - ambulance_x

        if distance >= 0 and distance <= INTERSECTION_DETECTION_DISTANCE:

            return junction_id, distance

    return None, None


def is_conflicting_approach(vehicle_road, junction_id, ambulance_road):

    """
    Determine whether a vehicle is approaching the ambulance's
    intersection from an approach that should yield.

    Same-direction traffic on the ambulance's current road is
    handled separately by the lane-change logic.

    Traffic from:
        - opposite main road
        - north
        - south

    is treated as intersection traffic.
    """

    if junction_id not in INTERSECTIONS:
        return False

    incoming_edges = INTERSECTIONS[junction_id]["incoming"]

    if vehicle_road not in incoming_edges:
        return False

    # Same road as ambulance is handled by normal V2V logic.
    if vehicle_road == ambulance_road:
        return False

    return True


def restore_vehicle_speed(vehicle_id):

    """
    Return vehicle to normal SUMO speed control.
    """

    try:

        traci.vehicle.setSpeed(
            vehicle_id,
            -1
        )

    except traci.TraCIException:

        pass


def emergency_stop(vehicle_id):

    """
    Stop a vehicle for emergency V2V yielding.
    """

    try:

        traci.vehicle.setSpeed(
            vehicle_id,
            0.0
        )

        return True

    except traci.TraCIException:

        return False


def get_lane_count(road_id):

    try:

        return traci.edge.getLaneNumber(
            road_id
        )

    except traci.TraCIException:

        return 1


def find_safe_adjacent_lane(
    vehicle_id,
    vehicle_road,
    vehicle_x,
    vehicle_lane_index,
    vehicles
):

    """
    Find a safe adjacent lane.

    We preserve the original V2V idea:
    check front and rear gaps before changing lane.
    """

    lane_count = get_lane_count(
        vehicle_road
    )

    candidate_lanes = []

    # Prefer moving toward higher lane index.
    if vehicle_lane_index + 1 < lane_count:

        candidate_lanes.append(
            vehicle_lane_index + 1
        )

    # Also allow lower lane if available.
    if vehicle_lane_index - 1 >= 0:

        candidate_lanes.append(
            vehicle_lane_index - 1
        )

    for candidate_lane in candidate_lanes:

        candidate_lane_id = (
            vehicle_road
            + "_"
            + str(candidate_lane)
        )

        front_gap = float("inf")
        rear_gap = float("inf")

        for other_id in vehicles:

            if other_id == vehicle_id:
                continue

            if other_id == "ambulance":
                continue

            try:

                other_road = (
                    traci.vehicle.getRoadID(
                        other_id
                    )
                )

                other_lane = (
                    traci.vehicle.getLaneID(
                        other_id
                    )
                )

                other_x, other_y = (
                    traci.vehicle.getPosition(
                        other_id
                    )
                )

            except traci.TraCIException:

                continue

            if other_road != vehicle_road:
                continue

            if other_lane != candidate_lane_id:
                continue

            gap = other_x - vehicle_x

            if gap > 0:

                front_gap = min(
                    front_gap,
                    gap
                )

            elif gap < 0:

                rear_gap = min(
                    rear_gap,
                    abs(gap)
                )

        if (
            front_gap >= MIN_FRONT_GAP
            and
            rear_gap >= MIN_REAR_GAP
        ):

            return candidate_lane

    return None


# ============================================================
# MAIN SIMULATION LOOP
# ============================================================

while traci.simulation.getTime() < SIMULATION_END_TIME:

    # --------------------------------------------------------
    # ADVANCE SUMO
    # --------------------------------------------------------

    traci.simulationStep()

    current_time = traci.simulation.getTime()

    vehicles = traci.vehicle.getIDList()


    # ========================================================
    # DISABLE AUTONOMOUS STRATEGIC / TACTICAL LANE CHANGES
    # ========================================================

    for vehicle_id in vehicles:

        if vehicle_id == "ambulance":
            continue

        try:

            traci.vehicle.setLaneChangeMode(
                vehicle_id,
                512
            )

        except traci.TraCIException:

            pass


    # ========================================================
    # DETECT NEW VEHICLES
    # ========================================================

    for vehicle_id in vehicles:

        if vehicle_id not in known_vehicles:

            known_vehicles.add(
                vehicle_id
            )

            if vehicle_id != "ambulance":

                print(
                    f"[SYSTEM] New vehicle detected: "
                    f"{vehicle_id}"
                )


    # ========================================================
    # AMBULANCE CHECK
    # ========================================================

    if "ambulance" not in vehicles:

        continue


    # ========================================================
    # AMBULANCE STATE
    # ========================================================

    ambulance_x, ambulance_y = (
        traci.vehicle.getPosition(
            "ambulance"
        )
    )

    ambulance_speed = (
        traci.vehicle.getSpeed(
            "ambulance"
        )
    )

    ambulance_road = (
        traci.vehicle.getRoadID(
            "ambulance"
        )
    )

    ambulance_lane = (
        traci.vehicle.getLaneID(
            "ambulance"
        )
    )

    ambulance_lane_index = (
        traci.vehicle.getLaneIndex(
            "ambulance"
        )
    )


    # ========================================================
    # NEXT INTERSECTION
    # ========================================================

    next_junction, junction_distance = (
        get_next_intersection(
            ambulance_x
        )
    )


    # ========================================================
    # INTERSECTION V2V EMERGENCY SYSTEM
    # ========================================================

    if next_junction is not None:

        # ----------------------------------------------------
        # BROADCAST EMERGENCY
        # ----------------------------------------------------

        if (
            next_junction
            not in announced_intersections
        ):

            announced_intersections.add(
                next_junction
            )

            print()
            print("=" * 70)
            print("             V2V EMERGENCY BROADCAST")
            print("=" * 70)

            print(
                f"Time              : "
                f"{current_time:.1f} s"
            )

            print(
                f"Ambulance         : "
                f"ambulance"
            )

            print(
                f"Current road      : "
                f"{ambulance_road}"
            )

            print(
                f"Next intersection : "
                f"{next_junction}"
            )

            print(
                f"Distance          : "
                f"{junction_distance:.2f} m"
            )

            print(
                "[V2V] Emergency priority message sent."
            )


        # ----------------------------------------------------
        # PROCESS CONFLICTING VEHICLES
        # ----------------------------------------------------

        for vehicle_id in vehicles:

            if vehicle_id == "ambulance":
                continue

            try:

                vehicle_road = (
                    traci.vehicle.getRoadID(
                        vehicle_id
                    )
                )

                vehicle_x, vehicle_y = (
                    traci.vehicle.getPosition(
                        vehicle_id
                    )
                )

                vehicle_speed = (
                    traci.vehicle.getSpeed(
                        vehicle_id
                    )
                )

            except traci.TraCIException:

                continue


            # ------------------------------------------------
            # MUST BE ON AN APPROACH TO THIS INTERSECTION
            # ------------------------------------------------

            if not is_conflicting_approach(
                vehicle_road,
                next_junction,
                ambulance_road
            ):

                continue


            # ------------------------------------------------
            # DETERMINE DISTANCE TO INTERSECTION
            # ------------------------------------------------

            junction_x = (
                INTERSECTIONS[
                    next_junction
                ]["x"]
            )

            # Distance from vehicle to junction.
            vehicle_distance = abs(
                junction_x - vehicle_x
            )


            # ------------------------------------------------
            # ONLY CONTROL VEHICLES CLOSE ENOUGH
            # ------------------------------------------------

            if (
                vehicle_distance
                > INTERSECTION_DETECTION_DISTANCE
            ):

                continue


            # ------------------------------------------------
            # IF VEHICLE HAS PASSED JUNCTION,
            # DON'T CONTROL IT
            # ------------------------------------------------

            if (
                vehicle_distance < 8.0
                and
                vehicle_speed < STOP_SPEED_THRESHOLD
            ):

                continue


            # ------------------------------------------------
            # NEW YIELDING VEHICLE
            # ------------------------------------------------

            if vehicle_id not in yielding_vehicles:

                yielding_vehicles[
                    vehicle_id
                ] = next_junction

                print()
                print(
                    "[V2V] CONFLICT DETECTED"
                )

                print(
                    f"  Vehicle       : "
                    f"{vehicle_id}"
                )

                print(
                    f"  Approach      : "
                    f"{vehicle_road}"
                )

                print(
                    f"  Intersection  : "
                    f"{next_junction}"
                )

                print(
                    f"  Distance      : "
                    f"{vehicle_distance:.2f} m"
                )

                print(
                    "[V2V] EMERGENCY MESSAGE RECEIVED"
                )

                print(
                    f"[V2V] {vehicle_id} "
                    f"-> YIELD"
                )


            # ------------------------------------------------
            # EMERGENCY STOP
            # ------------------------------------------------

            if (
                junction_distance
                <= INTERSECTION_EMERGENCY_DISTANCE
            ):

                stopped = emergency_stop(
                    vehicle_id
                )

                if stopped:

                    print(
                        f"[V2V] {vehicle_id} "
                        f"-> STOPPING FOR AMBULANCE"
                    )


    # ========================================================
    # RELEASE VEHICLES AFTER INTERSECTION
    # ========================================================

    vehicles_to_release = []

    for vehicle_id, yielding_junction in (
        yielding_vehicles.items()
    ):

        if vehicle_id not in vehicles:

            vehicles_to_release.append(
                vehicle_id
            )

            continue


        junction_x = (
            INTERSECTIONS[
                yielding_junction
            ]["x"]
        )

        try:

            vehicle_x, vehicle_y = (
                traci.vehicle.getPosition(
                    vehicle_id
                )
            )

        except traci.TraCIException:

            vehicles_to_release.append(
                vehicle_id
            )

            continue


        # ----------------------------------------------------
        # Ambulance must have passed the intersection.
        # ----------------------------------------------------

        if (
            ambulance_x
            > junction_x + 15.0
        ):

            restore_vehicle_speed(
                vehicle_id
            )

            print()
            print(
                "[V2V] INTERSECTION CLEARED"
            )

            print(
                f"  Intersection : "
                f"{yielding_junction}"
            )

            print(
                f"  Vehicle      : "
                f"{vehicle_id}"
            )

            print(
                f"[V2V] {vehicle_id} "
                f"-> RESUME NORMAL TRAFFIC"
            )

            vehicles_to_release.append(
                vehicle_id
            )


    for vehicle_id in vehicles_to_release:

        yielding_vehicles.pop(
            vehicle_id,
            None
        )


    # ========================================================
    # PENDING LANE CHANGE MANAGEMENT
    # ========================================================

    completed_changes = []

    for vehicle_id, target_lane_index in (
        pending_lane_changes.items()
    ):

        if vehicle_id not in vehicles:

            completed_changes.append(
                vehicle_id
            )

            continue

        try:

            current_lane_index = (
                traci.vehicle.getLaneIndex(
                    vehicle_id
                )
            )

            current_lane = (
                traci.vehicle.getLaneID(
                    vehicle_id
                )
            )

        except traci.TraCIException:

            completed_changes.append(
                vehicle_id
            )

            continue


        if current_lane_index == target_lane_index:

            print()
            print(
                "[V2V] LANE CHANGE COMPLETED"
            )

            print(
                f"  Vehicle : "
                f"{vehicle_id}"
            )

            print(
                f"  New lane: "
                f"{current_lane}"
            )

            completed_changes.append(
                vehicle_id
            )


    for vehicle_id in completed_changes:

        pending_lane_changes.pop(
            vehicle_id,
            None
        )


    # ========================================================
    # NORMAL SAME-ROAD V2V LANE-CHANGE LOGIC
    # ========================================================

    for vehicle_id in vehicles:

        if vehicle_id == "ambulance":
            continue

        # Vehicles currently yielding at intersections
        # are handled by the intersection V2V system.
        if vehicle_id in yielding_vehicles:
            continue

        try:

            vehicle_x, vehicle_y = (
                traci.vehicle.getPosition(
                    vehicle_id
                )
            )

            vehicle_road = (
                traci.vehicle.getRoadID(
                    vehicle_id
                )
            )

            vehicle_lane = (
                traci.vehicle.getLaneID(
                    vehicle_id
                )
            )

            vehicle_lane_index = (
                traci.vehicle.getLaneIndex(
                    vehicle_id
                )
            )

            vehicle_speed = (
                traci.vehicle.getSpeed(
                    vehicle_id
                )
            )

        except traci.TraCIException:

            continue


        # ----------------------------------------------------
        # ONLY SAME ROAD
        # ----------------------------------------------------

        if vehicle_road != ambulance_road:
            continue


        # ----------------------------------------------------
        # LONGITUDINAL DISTANCE
        # ----------------------------------------------------

        longitudinal_distance = (
            vehicle_x
            - ambulance_x
        )


        # Vehicle behind ambulance
        if longitudinal_distance <= 0:
            continue


        # ----------------------------------------------------
        # ADJACENT LANE
        # ----------------------------------------------------

        if vehicle_lane != ambulance_lane:

            continue


        # ====================================================
        # EXISTING PENDING REQUEST
        # ====================================================

        if vehicle_id in pending_lane_changes:

            continue


        # ====================================================
        # PREPARE
        # ====================================================

        if longitudinal_distance > PREPARE_DISTANCE:

            continue


        # ====================================================
        # EMERGENCY LANE CHANGE
        # ====================================================

        if (
            longitudinal_distance
            <= EMERGENCY_DISTANCE
        ):

            print()
            print(
                "=" * 70
            )

            print(
                "        V2V EMERGENCY LANE MANEUVER"
            )

            print(
                "=" * 70
            )

            print(
                f"Vehicle          : "
                f"{vehicle_id}"
            )

            print(
                f"Road             : "
                f"{vehicle_road}"
            )

            print(
                f"Distance to EMS  : "
                f"{longitudinal_distance:.2f} m"
            )

            print(
                "[V2V] Emergency lane search..."
            )


        # ====================================================
        # NORMAL YIELD / PREPARE LANE SEARCH
        # ====================================================

        else:

            print()
            print(
                "[V2V] VEHICLE APPROACHING AMBULANCE"
            )

            print(
                f"  Vehicle : "
                f"{vehicle_id}"
            )

            print(
                f"  Distance: "
                f"{longitudinal_distance:.2f} m"
            )

            print(
                "  Action  : SEARCH SAFE LANE"
            )


        # ====================================================
        # FIND SAFE LANE
        # ====================================================

        target_lane = find_safe_adjacent_lane(
            vehicle_id,
            vehicle_road,
            vehicle_x,
            vehicle_lane_index,
            vehicles
        )


        # ====================================================
        # SAFE LANE FOUND
        # ====================================================

        if target_lane is not None:

            print(
                f"[V2V] SAFE LANE FOUND"
            )

            print(
                f"  Vehicle : "
                f"{vehicle_id}"
            )

            print(
                f"  From    : "
                f"{vehicle_lane}"
            )

            print(
                f"  To lane : "
                f"{target_lane}"
            )


            try:

                traci.vehicle.changeLane(
                    vehicle_id,
                    target_lane,
                    LANE_CHANGE_DURATION
                )

                pending_lane_changes[
                    vehicle_id
                ] = target_lane

                print(
                    "[V2V] LANE CHANGE REQUEST SENT"
                )

            except traci.TraCIException as error:

                print(
                    "[V2V] Lane change error:"
                )

                print(
                    error
                )


        # ====================================================
        # NO SAFE LANE
        # ====================================================

        else:

            print(
                "[V2V] NO SAFE LANE"
            )

            print(
                "[V2V] Vehicle will maintain "
                "normal SUMO safety behavior."
            )


# ============================================================
# CLOSE TRACI
# ============================================================

traci.close()

print()
print("=" * 70)
print("                 SIMULATION FINISHED")
print("=" * 70)