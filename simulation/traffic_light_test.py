import traci


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
# DISCOVER TRAFFIC LIGHTS
# ============================================================

traffic_lights = traci.trafficlight.getIDList()

print("\n")
print("=" * 60)
print("             TRAFFIC LIGHT DISCOVERY")
print("=" * 60)

print(
    f"Traffic lights found: "
    f"{len(traffic_lights)}"
)


if len(traffic_lights) == 0:

    print(
        "\nNo traffic lights were found."
    )

else:

    print("\nTraffic-light IDs:")

    for tls_id in traffic_lights:

        print(
            f"  - {tls_id}"
        )


# ============================================================
# SIMULATION LOOP
# ============================================================

while traci.simulation.getMinExpectedNumber() > 0:

    traci.simulationStep()

    # --------------------------------------------------------
    # Print traffic-light information once per second
    # --------------------------------------------------------

    simulation_time = traci.simulation.getTime()

    if simulation_time % 1 != 0:
        continue

    print("\n")
    print(
        f"========== TIME {simulation_time:.0f} s =========="
    )


    for tls_id in traffic_lights:

        # ----------------------------------------------------
        # Current phase
        # ----------------------------------------------------

        phase = traci.trafficlight.getPhase(
            tls_id
        )

        # ----------------------------------------------------
        # Current phase duration
        # ----------------------------------------------------

        phase_duration = traci.trafficlight.getPhaseDuration(
            tls_id
        )

        # ----------------------------------------------------
        # Remaining time in current phase
        # ----------------------------------------------------

        next_switch = traci.trafficlight.getNextSwitch(
            tls_id
        )

        remaining = (
            next_switch -
            simulation_time
        )

        # ----------------------------------------------------
        # Full signal state
        # ----------------------------------------------------

        signal_state = traci.trafficlight.getRedYellowGreenState(
            tls_id
        )


        print(
            f"\nTraffic Light: {tls_id}"
        )

        print(
            f"  Phase           : "
            f"{phase}"
        )

        print(
            f"  Phase duration  : "
            f"{phase_duration:.1f} s"
        )

        print(
            f"  Time to switch  : "
            f"{remaining:.1f} s"
        )

        print(
            f"  Signal state    : "
            f"{signal_state}"
        )


# ============================================================
# CLOSE SUMO
# ============================================================

traci.close()

print("\nSimulation finished.")