import traci


# ============================================================
# CONFIGURATION
# ============================================================

SUMO_CONFIG = (
    r"C:\ambulance_v2x\tls_simulation\simulation.sumocfg"
)

J2 = "J2"
J3 = "J3"

# Emergency phases
J2_EMERGENCY_PHASE = 2
J3_EMERGENCY_PHASE = 0

# Detection distances
J2_TRIGGER_DISTANCE = 100.0
J3_TRIGGER_DISTANCE = 150.0

# Roads
J2_APPROACH_ROAD = "J1_J2"
J2_EXIT_ROAD = "J2_J3"


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
# STATE VARIABLES
# ============================================================

j2_emergency_active = False
j3_emergency_active = False

# Becomes True when ambulance actually enters J3
ambulance_entered_j3 = False

# Prevents repeated messages
i2i_message_sent = False


# ============================================================
# FUNCTION: GET TRAFFIC LIGHT POSITION
# ============================================================

def get_tls_position(tls_id):

    try:

        controlled_lanes = (
            traci.trafficlight.getControlledLanes(
                tls_id
            )
        )

        if not controlled_lanes:
            return None

        # Try to find the lane approaching the junction.
        for lane_id in controlled_lanes:

            try:

                shape = traci.lane.getShape(
                    lane_id
                )

                if shape:
                    return shape[-1]

            except traci.TraCIException:
                continue

    except traci.TraCIException:
        pass

    return None


# ============================================================
# MAIN SIMULATION LOOP
# ============================================================

while traci.simulation.getMinExpectedNumber() > 0:

    traci.simulationStep()

    vehicles = traci.vehicle.getIDList()

    current_time = traci.simulation.getTime()


    # ========================================================
    # CHECK AMBULANCE
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


    # ========================================================
    # TRAFFIC LIGHT POSITIONS
    # ========================================================

    j2_position = get_tls_position(J2)
    j3_position = get_tls_position(J3)


    # ========================================================
    # DISTANCE TO J2
    # ========================================================

    if j2_position:

        j2_x, j2_y = j2_position

        distance_j2 = (
            (
                (j2_x - ambulance_x) ** 2
                +
                (j2_y - ambulance_y) ** 2
            )
            ** 0.5
        )

    else:

        distance_j2 = float("inf")


    # ========================================================
    # DISTANCE TO J3
    # ========================================================

    if j3_position:

        j3_x, j3_y = j3_position

        distance_j3 = (
            (
                (j3_x - ambulance_x) ** 2
                +
                (j3_y - ambulance_y) ** 2
            )
            ** 0.5
        )

    else:

        distance_j3 = float("inf")


    # ========================================================
    # GET SIGNAL STATES
    # ========================================================

    j2_phase = (
        traci.trafficlight.getPhase(
            J2
        )
    )

    j2_state = (
        traci.trafficlight.getRedYellowGreenState(
            J2
        )
    )

    j3_phase = (
        traci.trafficlight.getPhase(
            J3
        )
    )

    j3_state = (
        traci.trafficlight.getRedYellowGreenState(
            J3
        )
    )


    # ========================================================
    # DISPLAY
    # ========================================================

    print("\n")
    print("=" * 70)
    print("             V2X / I2I EMERGENCY CORRIDOR")
    print("=" * 70)

    print(
        f"Time            : "
        f"{current_time:.1f} s"
    )

    print(
        f"Ambulance road  : "
        f"{ambulance_road}"
    )

    print(
        f"Ambulance speed : "
        f"{ambulance_speed:.2f} m/s"
    )

    print(
        f"Distance to J2  : "
        f"{distance_j2:.2f} m"
    )

    print(
        f"Distance to J3  : "
        f"{distance_j3:.2f} m"
    )


    print("\nJ2")

    print(
        f"  Phase : "
        f"{j2_phase}"
    )

    print(
        f"  State : "
        f"{j2_state}"
    )


    print("\nJ3")

    print(
        f"  Phase : "
        f"{j3_phase}"
    )

    print(
        f"  State : "
        f"{j3_state}"
    )


    # ========================================================
    # STEP 1
    # J2 DETECTS AMBULANCE
    # ========================================================

    if (
        ambulance_road == J2_APPROACH_ROAD
        and
        distance_j2 <= J2_TRIGGER_DISTANCE
        and
        not j2_emergency_active
    ):

        print("\n")
        print("=" * 70)
        print("[V2I] AMBULANCE DETECTED BY J2")
        print("=" * 70)

        print(
            f"[V2I] Distance to J2: "
            f"{distance_j2:.2f} m"
        )

        print(
            f"[V2I] Requesting J2 "
            f"emergency phase {J2_EMERGENCY_PHASE}"
        )


        # ----------------------------------------------------
        # Activate J2 emergency phase
        # ----------------------------------------------------

        traci.trafficlight.setPhase(
            J2,
            J2_EMERGENCY_PHASE
        )

        j2_emergency_active = True


        print(
            "[V2I] J2 EMERGENCY GREEN ACTIVE"
        )


        # ====================================================
        # I2I MESSAGE FROM J2 TO J3
        # ====================================================

        if not i2i_message_sent:

            print("\n")
            print(
                "[I2I] J2 -> J3"
            )

            print(
                "[I2I] EMERGENCY VEHICLE APPROACHING"
            )

            print(
                "[I2I] Requesting J3 corridor phase"
            )


            # ------------------------------------------------
            # Prepare J3
            # ------------------------------------------------

            traci.trafficlight.setPhase(
                J3,
                J3_EMERGENCY_PHASE
            )

            j3_emergency_active = True

            i2i_message_sent = True


            print(
                "[I2I] J3 EMERGENCY PHASE PREPARED"
            )

            print(
                f"[I2I] J3 phase = "
                f"{J3_EMERGENCY_PHASE}"
            )


    # ========================================================
    # STEP 2
    # MAINTAIN J2 EMERGENCY PHASE
    # ========================================================

    if j2_emergency_active:

        # ----------------------------------------------------
        # Ambulance still approaching J2
        # ----------------------------------------------------

        if ambulance_road == J2_APPROACH_ROAD:

            current_j2_phase = (
                traci.trafficlight.getPhase(
                    J2
                )
            )

            if current_j2_phase != J2_EMERGENCY_PHASE:

                traci.trafficlight.setPhase(
                    J2,
                    J2_EMERGENCY_PHASE
                )

            print(
                "[V2I] J2 emergency phase active"
            )


        # ----------------------------------------------------
        # Ambulance inside J2
        # ----------------------------------------------------

        elif ambulance_road.startswith(":"):

            print(
                "[V2I] Ambulance inside junction"
            )


        # ----------------------------------------------------
        # Ambulance has left J2
        # ----------------------------------------------------

        elif ambulance_road == J2_EXIT_ROAD:

            print("\n")
            print(
                "[V2I] AMBULANCE PASSED J2"
            )

            print(
                "[V2I] Releasing J2 emergency preemption"
            )


            # Return J2 to normal operation
            traci.trafficlight.setPhase(
                J2,
                0
            )

            j2_emergency_active = False


            print(
                "[V2I] J2 returned to normal operation"
            )


    # ========================================================
    # STEP 3
    # J3 PREPARATION
    # ========================================================

    if j3_emergency_active:

        # ----------------------------------------------------
        # Ambulance has NOT reached J3 yet
        #
        # Keep J3 on the emergency corridor phase.
        # ----------------------------------------------------

        if ambulance_road == J2_EXIT_ROAD:

            current_j3_phase = (
                traci.trafficlight.getPhase(
                    J3
                )
            )

            if current_j3_phase != J3_EMERGENCY_PHASE:

                print(
                    "[I2I] Restoring J3 emergency phase"
                )

                traci.trafficlight.setPhase(
                    J3,
                    J3_EMERGENCY_PHASE
                )


            print(
                "[I2I] J3 emergency corridor ACTIVE"
            )

            print(
                f"[I2I] J3 phase = "
                f"{J3_EMERGENCY_PHASE}"
            )


        # ----------------------------------------------------
        # Ambulance is inside a junction
        #
        # We only mark it as entering J3 if it is already
        # travelling on J2_J3 and is close to J3.
        # ----------------------------------------------------

        elif (
            ambulance_road.startswith(":")
            and
            distance_j3 <= J3_TRIGGER_DISTANCE
        ):

            ambulance_entered_j3 = True

            print(
                "[I2I] AMBULANCE ENTERING J3"
            )

            print(
                "[I2I] Emergency phase remains active"
            )


        # ----------------------------------------------------
        # Ambulance has entered J3 and then leaves it
        # ----------------------------------------------------

        elif (
            ambulance_entered_j3
            and
            ambulance_road != J2_EXIT_ROAD
            and
            not ambulance_road.startswith(":")
        ):

            print("\n")
            print("=" * 70)
            print("[I2I] AMBULANCE PASSED J3")
            print("=" * 70)

            print(
                "[I2I] Releasing J3 emergency phase"
            )


            # Return J3 to normal operation
            traci.trafficlight.setPhase(
                J3,
                0
            )

            j3_emergency_active = False

            print(
                "[I2I] J3 returned to normal operation"
            )


# ============================================================
# CLOSE
# ============================================================

traci.close()

print("\n")
print("=" * 70)
print("Simulation finished.")
print("=" * 70)