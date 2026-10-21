import csv
import os
import traci

# ============================================================
# CONFIGURATION
# ============================================================

SUMO_CONFIG = "simulation.sumocfg"
OUTPUT_FILE = "mobility_trace.csv"

# Sample interval in seconds.
# SUMO will still advance using its configured step length.
SAMPLE_INTERVAL = 0.1

# We want the ambulance plus all normal vehicles.
TRACK_ALL_VEHICLES = True


# ============================================================
# START SUMO
# ============================================================

sumo_cmd = [
    "sumo",
    "-c",
    SUMO_CONFIG,
]

print("=" * 70)
print("SUMO MOBILITY EXPORT")
print("=" * 70)
print(f"Config : {SUMO_CONFIG}")
print(f"Output : {OUTPUT_FILE}")
print()

traci.start(sumo_cmd)

print("Connected to SUMO")
print()


# ============================================================
# OPEN CSV
# ============================================================

with open(OUTPUT_FILE, "w", newline="") as csv_file:

    writer = csv.writer(csv_file)

    writer.writerow([
        "time",
        "vehicle_id",
        "x",
        "y",
        "speed",
        "angle",
        "road_id",
        "lane_id",
        "lane_index"
    ])

    next_sample_time = 0.0
    sample_count = 0

    # ========================================================
    # MAIN SIMULATION LOOP
    # ========================================================

    while traci.simulation.getMinExpectedNumber() > 0:

        traci.simulationStep()

        sim_time = traci.simulation.getTime()

        # Only write at our desired sampling interval.
        if sim_time + 1e-9 < next_sample_time:
            continue

        vehicle_ids = traci.vehicle.getIDList()

        for vehicle_id in vehicle_ids:

            if not TRACK_ALL_VEHICLES:
                if vehicle_id != "ambulance":
                    continue

            try:
                x, y = traci.vehicle.getPosition(vehicle_id)
                speed = traci.vehicle.getSpeed(vehicle_id)
                angle = traci.vehicle.getAngle(vehicle_id)
                road_id = traci.vehicle.getRoadID(vehicle_id)
                lane_id = traci.vehicle.getLaneID(vehicle_id)
                lane_index = traci.vehicle.getLaneIndex(vehicle_id)

                writer.writerow([
                    f"{sim_time:.2f}",
                    vehicle_id,
                    f"{x:.3f}",
                    f"{y:.3f}",
                    f"{speed:.3f}",
                    f"{angle:.3f}",
                    road_id,
                    lane_id,
                    lane_index
                ])

            except traci.TraCIException:
                # Vehicle may disappear between getIDList()
                # and the state query.
                continue

        sample_count += 1

        if sample_count % 100 == 0:
            print(
                f"Time = {sim_time:7.2f} s | "
                f"Vehicles = {len(vehicle_ids):3d} | "
                f"Samples = {sample_count}"
            )

        next_sample_time += SAMPLE_INTERVAL


# ============================================================
# CLOSE SUMO
# ============================================================

traci.close()

print()
print("=" * 70)
print("MOBILITY EXPORT COMPLETE")
print("=" * 70)
print(f"Saved: {os.path.abspath(OUTPUT_FILE)}")
print("=" * 70)
