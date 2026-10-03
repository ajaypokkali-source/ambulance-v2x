import os
import sys
import traci


# ============================================================
# CONFIGURATION
# ============================================================

SUMO_CONFIG = r"C:\ambulance_v2x\v2v_stress_test\simulation.sumocfg"

SUMO_BINARY = "sumo-gui"

AMBULANCE_ID = "ambulance"

BLOCKER_ID = "blocker_1"

ORIGINAL_LANE = 0

YIELD_LANE = 1

DETECTION_DISTANCE = 100.0

PASS_DISTANCE = -10.0

RETURN_DISTANCE = -25.0

BLOCKER_NORMAL_SPEED = 8.0

BLOCKER_YIELD_SPEED = 7.0

SIMULATION_END = 300.0


# ============================================================
# SUMO
# ============================================================

if "SUMO_HOME" in os.environ:

    tools = os.path.join(
        os.environ["SUMO_HOME"],
        "tools"
    )

    if tools not in sys.path:

        sys.path.append(tools)


print()
print("=" * 75)
print("              V2V EMERGENCY VEHICLE TEST")
print("=" * 75)
print()

print("[SYSTEM] Starting SUMO...")


sumo_cmd = [
    SUMO_BINARY,
    "-c",
    SUMO_CONFIG,
    "--step-length",
    "0.5"
]


traci.start(sumo_cmd)


print("[SYSTEM] Connected to SUMO!")
print()


# ============================================================
# STATE
# ============================================================

state = "NORMAL"

lane_change_time = None

ambulance_pass_time = None

return_command_sent = False

last_status_time = -1


# ============================================================
# HELPER
# ============================================================

def exists(vehicle_id):

    return vehicle_id in traci.vehicle.getIDList()


def route_distance(ambulance_id, blocker_id):

    try:

        ambulance_route = traci.vehicle.getRoute(
            ambulance_id
        )

        blocker_route = traci.vehicle.getRoute(
            blocker_id
        )

        ambulance_index = traci.vehicle.getRouteIndex(
            ambulance_id
        )

        blocker_index = traci.vehicle.getRouteIndex(
            blocker_id
        )

        ambulance_pos = traci.vehicle.getLanePosition(
            ambulance_id
        )

        blocker_pos = traci.vehicle.getLanePosition(
            blocker_id
        )

        if ambulance_index < 0 or blocker_index < 0:

            return None


        # Different routes

        if ambulance_route != blocker_route:

            return None


        # Blocker behind ambulance

        if blocker_index < ambulance_index:

            return blocker_pos - ambulance_pos


        # Same edge

        if blocker_index == ambulance_index:

            return blocker_pos - ambulance_pos


        # Remaining distance on ambulance edge

        ambulance_lane_id = traci.vehicle.getLaneID(
            ambulance_id
        )

        remaining = (
            traci.lane.getLength(
                ambulance_lane_id
            )
            - ambulance_pos
        )


        distance = remaining


        # Edges between vehicles

        for i in range(
            ambulance_index + 1,
            blocker_index
        ):

            try:

                distance += traci.edge.getLength(
                    ambulance_route[i]
                )

            except traci.TraCIException:

                pass


        distance += blocker_pos

        return distance


    except traci.TraCIException:

        return None


# ============================================================
# MAIN LOOP
# ============================================================

try:

    while (

        traci.simulation.getMinExpectedNumber() > 0

        and

        traci.simulation.getTime() < SIMULATION_END

    ):

        traci.simulationStep()


        current_time = traci.simulation.getTime()


        # ====================================================
        # AMBULANCE
        # ====================================================

        if not exists(AMBULANCE_ID):

            continue


        ambulance_lane = traci.vehicle.getLaneIndex(
            AMBULANCE_ID
        )

        ambulance_edge = traci.vehicle.getRoadID(
            AMBULANCE_ID
        )

        ambulance_speed = traci.vehicle.getSpeed(
            AMBULANCE_ID
        )

        ambulance_pos = traci.vehicle.getLanePosition(
            AMBULANCE_ID
        )


        # ====================================================
        # BLOCKER
        # ====================================================

        if not exists(BLOCKER_ID):

            continue


        blocker_lane = traci.vehicle.getLaneIndex(
            BLOCKER_ID
        )

        blocker_edge = traci.vehicle.getRoadID(
            BLOCKER_ID
        )

        blocker_speed = traci.vehicle.getSpeed(
            BLOCKER_ID
        )

        blocker_pos = traci.vehicle.getLanePosition(
            BLOCKER_ID
        )


        distance = route_distance(
            AMBULANCE_ID,
            BLOCKER_ID
        )


        # ====================================================
        # STATUS OUTPUT
        # ====================================================

        if (

            last_status_time < 0

            or

            current_time - last_status_time >= 1.0

        ):

            print()
            print("-" * 75)

            print(
                f"TIME              : "
                f"{current_time:.1f} s"
            )

            print(
                f"AMBULANCE         : "
                f"{ambulance_edge} | "
                f"lane {ambulance_lane} | "
                f"{ambulance_pos:.1f} m | "
                f"{ambulance_speed:.2f} m/s"
            )

            print(
                f"BLOCKER           : "
                f"{blocker_edge} | "
                f"lane {blocker_lane} | "
                f"{blocker_pos:.1f} m | "
                f"{blocker_speed:.2f} m/s"
            )

            if distance is not None:

                print(
                    f"SEPARATION        : "
                    f"{distance:.1f} m"
                )

            print(
                f"V2V STATE         : "
                f"{state}"
            )

            last_status_time = current_time


        # ====================================================
        # STATE 1
        # NORMAL
        # ====================================================

        if state == "NORMAL":

            if (

                distance is not None

                and

                distance > 0

                and

                distance <= DETECTION_DISTANCE

                and

                blocker_lane == ORIGINAL_LANE

            ):

                print()
                print("=" * 75)

                print(
                    "[V2V] EMERGENCY MESSAGE RECEIVED"
                )

                print(
                    "[V2V] Ambulance detected behind "
                    "vehicle blocker_1"
                )

                print(
                    f"[V2V] Separation: "
                    f"{distance:.1f} m"
                )

                print(
                    "[V2V] Vehicle is blocking "
                    "the emergency lane."
                )

                print(
                    "[V2V] Checking adjacent lane..."
                )

                print(
                    "[V2V] Lane 1 available."
                )

                print(
                    "[V2V] Vehicle instructed to yield."
                )

                print(
                    "[V2V] Preparing lane change: "
                    "0 -> 1"
                )

                print("=" * 75)


                # ============================================
                # SLOW THE VEHICLE BEFORE CHANGING LANES
                # ============================================

                try:

                    traci.vehicle.setSpeed(
                        BLOCKER_ID,
                        BLOCKER_YIELD_SPEED
                    )

                except traci.TraCIException:

                    pass


                # ============================================
                # PERFORM LANE CHANGE
                # ============================================

                try:

                    traci.vehicle.changeLane(
                        BLOCKER_ID,
                        YIELD_LANE,
                        3.0
                    )

                    state = "CHANGING"

                    lane_change_time = current_time

                    print()
                    print(
                        "[V2V] Lane-change command sent."
                    )

                except traci.TraCIException as error:

                    print()
                    print(
                        "[V2V] Lane-change command failed:"
                    )

                    print(error)


        # ====================================================
        # STATE 2
        # CHANGING
        # ====================================================

        elif state == "CHANGING":

            if blocker_lane == YIELD_LANE:

                state = "YIELDING"

                print()
                print("=" * 75)

                print(
                    "[V2V] *** VEHICLE SUCCESSFULLY CHANGED LANES ***"
                )

                print(
                    "[V2V] blocker_1: lane 0 -> lane 1"
                )

                print(
                    "[V2V] Emergency lane is now clear."
                )

                if lane_change_time is not None:

                    print(
                        f"[V2V] Lane-change time: "
                        f"{current_time - lane_change_time:.1f} s"
                    )

                print("=" * 75)


        # ====================================================
        # STATE 3
        # YIELDING
        # ====================================================

        elif state == "YIELDING":

            # Keep blocker slower than ambulance

            try:

                traci.vehicle.setSpeed(
                    BLOCKER_ID,
                    BLOCKER_YIELD_SPEED
                )

            except traci.TraCIException:

                pass


            # Ambulance has passed

            if (

                distance is not None

                and

                distance < PASS_DISTANCE

                and

                ambulance_pass_time is None

            ):

                ambulance_pass_time = current_time

                print()
                print("=" * 75)

                print(
                    "[V2V] *** AMBULANCE HAS PASSED ***"
                )

                print(
                    "[V2V] Emergency vehicle cleared "
                    "the yielding vehicle."
                )

                print(
                    "[V2V] Preparing vehicle to return "
                    "to original lane."
                )

                print("=" * 75)


                # ============================================
                # RETURN TO ORIGINAL LANE
                # ============================================

                try:

                    traci.vehicle.changeLane(
                        BLOCKER_ID,
                        ORIGINAL_LANE,
                        4.0
                    )

                    return_command_sent = True

                    state = "RETURNING"

                except traci.TraCIException as error:

                    print(
                        "[V2V] Return lane-change failed:"
                    )

                    print(error)


        # ====================================================
        # STATE 4
        # RETURNING
        # ====================================================

        elif state == "RETURNING":

            if blocker_lane == ORIGINAL_LANE:

                state = "COMPLETE"

                print()
                print("=" * 75)

                print(
                    "[V2V] *** VEHICLE RETURNED TO ORIGINAL LANE ***"
                )

                print(
                    "[V2V] blocker_1: lane 1 -> lane 0"
                )

                print(
                    "[V2V] V2V emergency maneuver complete."
                )

                print("=" * 75)


        # ====================================================
        # STATE 5
        # COMPLETE
        # ====================================================

        elif state == "COMPLETE":

            try:

                traci.vehicle.setSpeed(
                    BLOCKER_ID,
                    BLOCKER_NORMAL_SPEED
                )

            except traci.TraCIException:

                pass


# ============================================================
# SHUTDOWN
# ============================================================

except traci.TraCIException as error:

    print()
    print("=" * 75)

    print(
        "[SYSTEM] TraCI error:"
    )

    print(error)

    print("=" * 75)


except KeyboardInterrupt:

    print()
    print(
        "[SYSTEM] Simulation stopped by user."
    )


finally:

    try:

        traci.close()

    except:

        pass

    print()
    print("=" * 75)
    print("                V2V TEST FINISHED")
    print("=" * 75)