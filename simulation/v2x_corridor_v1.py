import traci


# ============================================================
# CONFIGURATION
# ============================================================

PREPARE_DISTANCE = 100.0
YIELD_DISTANCE = 60.0
EMERGENCY_DISTANCE = 25.0

MIN_FRONT_GAP = 15.0
MIN_REAR_GAP = 15.0

LANE_CHANGE_DURATION = 5.0


# ============================================================
# SUMO CONFIGURATION
# ============================================================

sumo_cmd = [
    "sumo-gui",
    "-c",
    r"C:\ambulance_v2x\simulation\simulation.sumocfg"
]


# ============================================================
# START SUMO
# ============================================================

traci.start(sumo_cmd)

print("Connected to SUMO!")


# ============================================================
# LANE CHANGE STATE
#
# Stores vehicles for which OUR controller has already
# requested a lane change.
#
# Example:
#
# {
#     "car2": 1
# }
#
# means:
# car2 has been instructed by V2X to move to lane 1.
# ============================================================

pending_lane_changes = {}


# ============================================================
# TRACK VEHICLES ALREADY SEEN
#
# Used so we can identify newly inserted vehicles.
# ============================================================

known_vehicles = set()


# ============================================================
# MAIN SIMULATION LOOP
# ============================================================

while traci.simulation.getMinExpectedNumber() > 0:

    # ========================================================
    # ADVANCE SUMO
    # ========================================================

    traci.simulationStep()

    vehicles = traci.vehicle.getIDList()


    # ========================================================
    # DISABLE AUTONOMOUS STRATEGIC / TACTICAL
    # LANE CHANGING
    #
    # IMPORTANT:
    # This is applied every simulation step because vehicles
    # can enter the simulation after traci.start().
    #
    # 512 = disable autonomous strategic/tactical lane
    # changes while retaining safety-related behavior.
    # ========================================================

    for vehicle_id in vehicles:

        # Never modify ambulance lane-change behavior
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

    ambulance_x, ambulance_y = traci.vehicle.getPosition(
        "ambulance"
    )

    ambulance_speed = traci.vehicle.getSpeed(
        "ambulance"
    )

    ambulance_lane = traci.vehicle.getLaneID(
        "ambulance"
    )

    ambulance_road = traci.vehicle.getRoadID(
        "ambulance"
    )

    ambulance_lane_index = traci.vehicle.getLaneIndex(
        "ambulance"
    )

    current_time = traci.simulation.getTime()


    # ========================================================
    # V2X EMERGENCY BROADCAST
    # ========================================================

    print("\n")
    print("=" * 65)
    print("                 V2X AMBULANCE BROADCAST")
    print("=" * 65)

    print(
        f"Time            : "
        f"{current_time:.1f} s"
    )

    print(
        f"Ambulance X     : "
        f"{ambulance_x:.2f}"
    )

    print(
        f"Ambulance speed : "
        f"{ambulance_speed:.2f} m/s"
    )

    print(
        f"Ambulance road  : "
        f"{ambulance_road}"
    )

    print(
        f"Ambulance lane  : "
        f"{ambulance_lane}"
    )

    print(
        f"Lane index      : "
        f"{ambulance_lane_index}"
    )


    # ========================================================
    # PENDING LANE CHANGES
    # ========================================================

    completed_changes = []


    for vehicle_id, target_lane_index in (
        pending_lane_changes.items()
    ):

        # Vehicle may have left simulation

        if vehicle_id not in vehicles:

            completed_changes.append(
                vehicle_id
            )

            continue


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


        # ====================================================
        # CHECK WHETHER LANE CHANGE FINISHED
        # ====================================================

        if current_lane_index == target_lane_index:

            print("\n")
            print(
                "[V2X] LANE CHANGE COMPLETED"
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


    # ========================================================
    # REMOVE COMPLETED REQUESTS
    # ========================================================

    for vehicle_id in completed_changes:

        if vehicle_id in pending_lane_changes:

            del pending_lane_changes[
                vehicle_id
            ]


    # ========================================================
    # DISPLAY PENDING CHANGES
    # ========================================================

    if pending_lane_changes:

        print("\n")
        print(
            "--- PENDING V2X LANE CHANGES ---"
        )

        for vehicle_id, target_lane in (
            pending_lane_changes.items()
        ):

            print(
                f"  {vehicle_id} "
                f"-> lane {target_lane}"
            )


    # ========================================================
    # PROCESS ALL VEHICLES
    # ========================================================

    for vehicle_id in vehicles:

        # ----------------------------------------------------
        # NEVER CONTROL AMBULANCE
        # ----------------------------------------------------

        if vehicle_id == "ambulance":

            continue


        # ====================================================
        # VEHICLE STATE
        # ====================================================

        vehicle_x, vehicle_y = (
            traci.vehicle.getPosition(
                vehicle_id
            )
        )

        vehicle_lane = (
            traci.vehicle.getLaneID(
                vehicle_id
            )
        )

        vehicle_road = (
            traci.vehicle.getRoadID(
                vehicle_id
            )
        )

        vehicle_speed = (
            traci.vehicle.getSpeed(
                vehicle_id
            )
        )

        vehicle_lane_index = (
            traci.vehicle.getLaneIndex(
                vehicle_id
            )
        )


        # ====================================================
        # ONLY SAME ROAD
        # ====================================================

        if vehicle_road != ambulance_road:

            continue


        # ====================================================
        # LONGITUDINAL DISTANCE
        #
        # This is ONLY used for vehicles on the SAME road.
        # ====================================================

        longitudinal_distance = (
            vehicle_x -
            ambulance_x
        )


        # ====================================================
        # VEHICLE BEHIND
        # ====================================================

        if longitudinal_distance <= 0:

            continue


        # ====================================================
        # ADJACENT / OTHER LANE
        #
        # IMPORTANT:
        #
        # OUR CONTROLLER DOES ABSOLUTELY NOTHING TO THIS
        # VEHICLE.
        #
        # No:
        #   setSpeed()
        #   slowDown()
        #   changeLane()
        #
        # ====================================================

        if vehicle_lane != ambulance_lane:

            print("\n")
            print(
                f"{vehicle_id}"
            )

            print(
                f"  Lane        : "
                f"{vehicle_lane}"
            )

            print(
                f"  Position    : "
                f"AHEAD"
            )

            print(
                f"  Status      : "
                f"ADJACENT LANE"
            )

            print(
                f"  V2X ACTION  : "
                f"IGNORE"
            )

            continue


        # ====================================================
        # SAME-LANE VEHICLE
        # ====================================================

        print("\n")
        print(
            f"{vehicle_id}"
        )

        print(
            f"  Lane        : "
            f"{vehicle_lane}"
        )

        print(
            f"  Position    : "
            f"AHEAD"
        )

        print(
            f"  Long. gap   : "
            f"{longitudinal_distance:.2f} m"
        )

        print(
            f"  Speed       : "
            f"{vehicle_speed:.2f} m/s"
        )


        # ====================================================
        # ALREADY HAS A V2X LANE-CHANGE REQUEST
        # ====================================================

        if vehicle_id in pending_lane_changes:

            target_lane_index = (
                pending_lane_changes[
                    vehicle_id
                ]
            )

            print(
                f"  State       : "
                f"LANE CHANGE PENDING"
            )

            print(
                f"  Target lane : "
                f"{target_lane_index}"
            )

            print(
                f"  Current lane: "
                f"{vehicle_lane_index}"
            )

            print(
                f"  V2X ACTION  : "
                f"WAIT"
            )

            continue


        # ====================================================
        # PREPARE
        # ====================================================

        if longitudinal_distance > PREPARE_DISTANCE:

            print(
                f"  State       : "
                f"PREPARE"
            )

            print(
                f"  V2X ACTION  : "
                f"PREPARE"
            )

            continue


        # ====================================================
        # YIELD
        # ====================================================

        if (
            longitudinal_distance
            > EMERGENCY_DISTANCE
        ):

            print(
                f"  State       : "
                f"YIELD"
            )

            print(
                f"  V2X ACTION  : "
                f"SEARCH SAFE LANE"
            )


            # =================================================
            # NUMBER OF LANES
            # =================================================

            try:

                lane_count = (
                    traci.edge.getLaneNumber(
                        ambulance_road
                    )
                )

            except:

                lane_count = 1


            # =================================================
            # CURRENT LANE
            # =================================================

            current_lane_index = (
                traci.vehicle.getLaneIndex(
                    vehicle_id
                )
            )


            target_lane = None


            # =================================================
            # CHECK LEFT ADJACENT LANE
            # =================================================

            if (
                current_lane_index + 1
                < lane_count
            ):

                candidate_lane = (
                    current_lane_index + 1
                )

                candidate_lane_id = (
                    ambulance_road
                    + "_"
                    + str(candidate_lane)
                )


                front_gap = float("inf")

                rear_gap = float("inf")


                # =============================================
                # SEARCH TARGET LANE
                # =============================================

                for other_id in vehicles:

                    if other_id == vehicle_id:

                        continue

                    if other_id == "ambulance":

                        continue


                    other_lane = (
                        traci.vehicle.getLaneID(
                            other_id
                        )
                    )

                    other_road = (
                        traci.vehicle.getRoadID(
                            other_id
                        )
                    )


                    if other_road != ambulance_road:

                        continue


                    if other_lane != candidate_lane_id:

                        continue


                    other_x, other_y = (
                        traci.vehicle.getPosition(
                            other_id
                        )
                    )


                    gap = (
                        other_x -
                        vehicle_x
                    )


                    # -----------------------------------------
                    # Vehicle ahead
                    # -----------------------------------------

                    if gap > 0:

                        if gap < front_gap:

                            front_gap = gap


                    # -----------------------------------------
                    # Vehicle behind
                    # -----------------------------------------

                    elif gap < 0:

                        if abs(gap) < rear_gap:

                            rear_gap = abs(gap)


                print(
                    f"  Candidate lane: "
                    f"{candidate_lane_id}"
                )

                print(
                    f"  Front gap     : "
                    f"{front_gap:.2f} m"
                )

                print(
                    f"  Rear gap      : "
                    f"{rear_gap:.2f} m"
                )


                # =============================================
                # SAFETY CHECK
                # =============================================

                if (
                    front_gap >= MIN_FRONT_GAP
                    and
                    rear_gap >= MIN_REAR_GAP
                ):

                    target_lane = (
                        candidate_lane
                    )

                    print(
                        f"  Target lane   : "
                        f"SAFE"
                    )

                else:

                    print(
                        f"  Target lane   : "
                        f"UNSAFE"
                    )


            # =================================================
            # REQUEST LANE CHANGE
            # =================================================

            if target_lane is not None:

                print("\n")

                print(
                    "[V2X] LANE CHANGE REQUEST"
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


                    # -----------------------------------------
                    # STORE REQUEST
                    # -----------------------------------------

                    pending_lane_changes[
                        vehicle_id
                    ] = target_lane


                    print(
                        "[V2X] REQUEST STORED"
                    )


                except traci.TraCIException as error:

                    print(
                        "[V2X] Lane change error:"
                    )

                    print(
                        error
                    )


            else:

                print(
                    "[V2X] NO SAFE LANE"
                )

                print(
                    "       No artificial "
                    "braking applied."
                )


            continue


        # ====================================================
        # EMERGENCY DISTANCE
        # ====================================================

        print(
            f"  State       : "
            f"EMERGENCY"
        )

        print(
            f"  V2X ACTION  : "
            f"EMERGENCY LANE SEARCH"
        )


        # ====================================================
        # NUMBER OF LANES
        # ====================================================

        try:

            lane_count = (
                traci.edge.getLaneNumber(
                    ambulance_road
                )
            )

        except:

            lane_count = 1


        current_lane_index = (
            traci.vehicle.getLaneIndex(
                vehicle_id
            )
        )


        target_lane = None


        # ====================================================
        # EMERGENCY LEFT LANE
        # ====================================================

        if (
            current_lane_index + 1
            < lane_count
        ):

            candidate_lane = (
                current_lane_index + 1
            )

            candidate_lane_id = (
                ambulance_road
                + "_"
                + str(candidate_lane)
            )


            front_gap = float("inf")

            rear_gap = float("inf")


            # =================================================
            # SEARCH TARGET LANE
            # =================================================

            for other_id in vehicles:

                if other_id in [
                    vehicle_id,
                    "ambulance"
                ]:

                    continue


                other_lane = (
                    traci.vehicle.getLaneID(
                        other_id
                    )
                )

                other_road = (
                    traci.vehicle.getRoadID(
                        other_id
                    )
                )


                if other_road != ambulance_road:

                    continue


                if other_lane != candidate_lane_id:

                    continue


                other_x, other_y = (
                    traci.vehicle.getPosition(
                        other_id
                    )
                )


                gap = (
                    other_x -
                    vehicle_x
                )


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


            print(
                f"  Emergency target: "
                f"{candidate_lane_id}"
            )

            print(
                f"  Front gap: "
                f"{front_gap:.2f} m"
            )

            print(
                f"  Rear gap : "
                f"{rear_gap:.2f} m"
            )


            # =================================================
            # EMERGENCY SAFETY CHECK
            # =================================================

            if (
                front_gap >= MIN_FRONT_GAP
                and
                rear_gap >= MIN_REAR_GAP
            ):

                target_lane = (
                    candidate_lane
                )


        # ====================================================
        # EMERGENCY LANE CHANGE
        # ====================================================

        if target_lane is not None:

            print("\n")

            print(
                "[V2X] EMERGENCY LANE CHANGE"
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
                    2.0
                )


                pending_lane_changes[
                    vehicle_id
                ] = target_lane


                print(
                    "[V2X] EMERGENCY REQUEST STORED"
                )


            except traci.TraCIException as error:

                print(
                    "[V2X] Emergency lane "
                    "change error:"
                )

                print(
                    error
                )


        else:

            print(
                "[V2X] NO SAFE EMERGENCY LANE"
            )

            print(
                "       No artificial braking."
            )


# ============================================================
# CLOSE TRACI
# ============================================================

traci.close()

print("\nSimulation finished.")