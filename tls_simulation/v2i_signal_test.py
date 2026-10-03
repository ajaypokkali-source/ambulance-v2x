import traci


# ============================================================
# CONFIGURATION
# ============================================================

SUMO_CONFIG = (
    r"C:\ambulance_v2x\tls_simulation\simulation.sumocfg"
)

# Target traffic light
TARGET_TLS = "J2"

# Distance at which V2I emergency request is triggered
PREEMPTION_DISTANCE = 100.0

# Phase that gives the ambulance green at J2
EMERGENCY_PHASE = 2

# Normal phase to return to after ambulance passes
NORMAL_PHASE = 0

# Road before J2
APPROACH_ROAD = "J1_J2"

# Road after J2
EXIT_ROAD = "J2_J3"


# ============================================================
# START SUMO
# ============================================================

sumo_cmd = [
    "sumo-gui",
    "-c",
    SUMO_CONFIG
]

traci.start(sumo_cmd)

print("Connected to SUMO!")


# ============================================================
# STATE
# ============================================================

preemption_active = False


# ============================================================
# MAIN SIMULATION LOOP
# ============================================================

while traci.simulation.getMinExpectedNumber() > 0:

    traci.simulationStep()

    vehicles = traci.vehicle.getIDList()

    current_time = traci.simulation.getTime()


    # ========================================================
    # CHECK FOR AMBULANCE
    # ========================================================

    if "ambulance" not in vehicles:

        continue


    # ========================================================
    # AMBULANCE INFORMATION
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


    # ========================================================
    # J2 POSITION
    # ========================================================

    tls_position = None

    try:

        controlled_lanes = (
            traci.trafficlight.getControlledLanes(
                TARGET_TLS
            )
        )

        if controlled_lanes:

            first_lane = controlled_lanes[0]

            lane_shape = (
                traci.lane.getShape(
                    first_lane
                )
            )

            if lane_shape:

                tls_position = lane_shape[-1]

    except traci.TraCIException:

        pass


    # ========================================================
    # CALCULATE DISTANCE TO J2
    # ========================================================

    if tls_position is not None:

        tls_x, tls_y = tls_position

        distance_to_tls = (
            (
                (tls_x - ambulance_x) ** 2
                +
                (tls_y - ambulance_y) ** 2
            )
            ** 0.5
        )

    else:

        distance_to_tls = float("inf")


    # ========================================================
    # CURRENT TRAFFIC LIGHT STATE
    # ========================================================

    current_phase = (
        traci.trafficlight.getPhase(
            TARGET_TLS
        )
    )

    signal_state = (
        traci.trafficlight.getRedYellowGreenState(
            TARGET_TLS
        )
    )

    next_switch = (
        traci.trafficlight.getNextSwitch(
            TARGET_TLS
        )
    )

    time_to_switch = (
        next_switch -
        current_time
    )


    # ========================================================
    # DISPLAY INFORMATION
    # ========================================================

    print("\n")
    print("=" * 65)
    print("              V2I EMERGENCY SIGNAL")
    print("=" * 65)

    print(
        f"Time            : "
        f"{current_time:.1f} s"
    )

    print(
        f"Ambulance road  : "
        f"{ambulance_road}"
    )

    print(
        f"Ambulance pos   : "
        f"({ambulance_x:.2f}, "
        f"{ambulance_y:.2f})"
    )

    print(
        f"Ambulance speed : "
        f"{ambulance_speed:.2f} m/s"
    )

    print(
        f"Distance to J2  : "
        f"{distance_to_tls:.2f} m"
    )

    print()

    print(
        f"Traffic Light   : "
        f"{TARGET_TLS}"
    )

    print(
        f"Current phase   : "
        f"{current_phase}"
    )

    print(
        f"Signal state    : "
        f"{signal_state}"
    )

    print(
        f"Next switch     : "
        f"{time_to_switch:.1f} s"
    )


    # ========================================================
    # STEP 1
    # DETECT APPROACHING AMBULANCE
    # ========================================================

    if (
        ambulance_road == APPROACH_ROAD
        and
        distance_to_tls <= PREEMPTION_DISTANCE
        and
        not preemption_active
    ):

        print()
        print("=" * 65)
        print("[V2I] EMERGENCY MESSAGE RECEIVED")
        print("=" * 65)

        print(
            f"[V2I] Ambulance detected "
            f"{distance_to_tls:.2f} m from J2"
        )

        print(
            f"[V2I] Ambulance road: "
            f"{ambulance_road}"
        )

        print(
            f"[V2I] Requesting emergency "
            f"phase {EMERGENCY_PHASE}"
        )


        # ----------------------------------------------------
        # ACTIVATE EMERGENCY PHASE
        # ----------------------------------------------------

        try:

            traci.trafficlight.setPhase(
                TARGET_TLS,
                EMERGENCY_PHASE
            )

            preemption_active = True

            print(
                "[V2I] J2 EMERGENCY GREEN ACTIVATED"
            )

            print(
                f"[V2I] J2 phase = "
                f"{EMERGENCY_PHASE}"
            )

        except traci.TraCIException as error:

            print(
                "[V2I] ERROR activating "
                "emergency phase:"
            )

            print(error)


    # ========================================================
    # STEP 2
    # KEEP EMERGENCY PHASE ACTIVE
    # ========================================================

    if preemption_active:

        # ----------------------------------------------------
        # AMBULANCE IS STILL APPROACHING J2
        # ----------------------------------------------------

        if ambulance_road == APPROACH_ROAD:

            try:

                current_phase = (
                    traci.trafficlight.getPhase(
                        TARGET_TLS
                    )
                )

                if current_phase != EMERGENCY_PHASE:

                    print()
                    print(
                        "[V2I] Restoring "
                        "emergency green phase"
                    )

                    traci.trafficlight.setPhase(
                        TARGET_TLS,
                        EMERGENCY_PHASE
                    )

            except traci.TraCIException as error:

                print(
                    "[V2I] ERROR maintaining "
                    "emergency phase:"
                )

                print(error)


            print()
            print(
                "[V2I] EMERGENCY PREEMPTION ACTIVE"
            )

            print(
                f"[V2I] Distance to J2: "
                f"{distance_to_tls:.2f} m"
            )

            print(
                f"[V2I] J2 phase: "
                f"{traci.trafficlight.getPhase(TARGET_TLS)}"
            )


        # ----------------------------------------------------
        # AMBULANCE HAS CROSSED J2
        # ----------------------------------------------------

        elif ambulance_road == EXIT_ROAD:

            print()
            print("=" * 65)
            print("[V2I] AMBULANCE PASSED J2")
            print("=" * 65)

            print(
                f"[V2I] Ambulance is now on "
                f"{EXIT_ROAD}"
            )

            print(
                "[V2I] Releasing emergency "
                "preemption..."
            )


            try:

                # Return J2 to normal operation
                traci.trafficlight.setPhase(
                    TARGET_TLS,
                    NORMAL_PHASE
                )

                preemption_active = False

                print(
                    "[V2I] J2 returned to "
                    "NORMAL OPERATION"
                )

                print(
                    f"[V2I] J2 phase = "
                    f"{NORMAL_PHASE}"
                )

            except traci.TraCIException as error:

                print(
                    "[V2I] ERROR releasing "
                    "emergency phase:"
                )

                print(error)


        # ----------------------------------------------------
        # AMBULANCE INSIDE JUNCTION
        # ----------------------------------------------------

        elif ambulance_road.startswith(":"):

            print()
            print(
                "[V2I] Ambulance is inside "
                "junction"
            )

            print(
                "[V2I] Emergency phase remains active"
            )


# ============================================================
# CLOSE SUMO / TRACI
# ============================================================

traci.close()

print()
print("=" * 65)
print("Simulation finished.")
print("=" * 65)