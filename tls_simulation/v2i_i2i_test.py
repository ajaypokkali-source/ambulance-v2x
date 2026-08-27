import traci

sumo_cmd = [
    "sumo-gui",
    "-c",
    r"C:\ambulance_v2x\tls_simulation\simulation.sumocfg"
]

traci.start(sumo_cmd)

print("Connected to SUMO!")

print("\n" + "=" * 60)
print("                 J3 PHASE DISCOVERY")
print("=" * 60)

logic = traci.trafficlight.getAllProgramLogics("J3")[0]

phases = logic.getPhases()

print(
    f"\nJ3 has {len(phases)} phases."
)

for index, phase in enumerate(phases):

    print(f"\nPhase {index}")

    print(
        f"  Duration : {phase.duration}"
    )

    print(
        f"  State    : {phase.state}"
    )

traci.close()

print("\nFinished.")