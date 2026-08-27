import traci

sumo_cmd = [
    "sumo-gui",
    "-n",
    r"C:\ambulance_v2x\tls_simulation\network.net.xml"
]

traci.start(sumo_cmd)

print("Connected to SUMO!")

traffic_lights = traci.trafficlight.getIDList()

print("\n" + "=" * 60)
print("TRAFFIC LIGHT DISCOVERY")
print("=" * 60)

print(
    f"Traffic lights found: "
    f"{len(traffic_lights)}"
)

for tls_id in traffic_lights:

    print(f"\nTraffic Light: {tls_id}")

    print(
        f"  Current phase: "
        f"{traci.trafficlight.getPhase(tls_id)}"
    )

    print(
        f"  Signal state: "
        f"{traci.trafficlight.getRedYellowGreenState(tls_id)}"
    )


# Run a few simulation steps just to verify
# that the traffic lights are active.

for step in range(20):

    traci.simulationStep()

    print(
        f"\nTime: "
        f"{traci.simulation.getTime():.1f} s"
    )

    for tls_id in traffic_lights:

        phase = traci.trafficlight.getPhase(
            tls_id
        )

        next_switch = traci.trafficlight.getNextSwitch(
            tls_id
        )

        print(
            f"  {tls_id}: "
            f"phase={phase}, "
            f"next switch={next_switch:.1f}s"
        )


traci.close()

print("\nTraffic-light test finished.")