import csv
from collections import defaultdict

INPUT = "mobility_trace.csv"
OUTPUT = "ns3_mobility_trace.csv"
VEHICLES = ["ambulance", "car11", "car15", "car7", "car17"]

# Read the SUMO export.
rows = defaultdict(list)
with open(INPUT, newline="") as f:
    reader = csv.DictReader(f)
    for r in reader:
        vid = r["vehicle_id"]
        if vid in VEHICLES:
            rows[vid].append({
                "time": float(r["time"]),
                "x": float(r["x"]),
                "y": float(r["y"]),
            })

for vid in VEHICLES:
    rows[vid].sort(key=lambda p: p["time"])
    if not rows[vid]:
        raise SystemExit(f"No mobility data found for {vid}")

# Start the ns-3 replay when the ambulance first appears.
start_time = rows["ambulance"][0]["time"]

with open(OUTPUT, "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["time", "vehicle_id", "x", "y"])

    for vid in VEHICLES:
        selected = [p for p in rows[vid] if p["time"] >= start_time]
        if not selected:
            raise SystemExit(f"No samples for {vid} at/after ambulance start time {start_time}")

        # Make the first sample for every selected vehicle occur at t=0.
        first = selected[0]
        writer.writerow(["0.000", vid, f"{first['x']:.3f}", f"{first['y']:.3f}"])

        last_out_time = 0.0
        for p in selected[1:]:
            t = p["time"] - start_time
            if t <= last_out_time + 1e-9:
                continue
            writer.writerow([f"{t:.3f}", vid, f"{p['x']:.3f}", f"{p['y']:.3f}"])
            last_out_time = t

print(f"Created {OUTPUT}")
print(f"Replay start time in original SUMO trace: {start_time:.2f} s")
for i, vid in enumerate(VEHICLES):
    print(f"UE{i}: {vid:9s} -> {len(rows[vid])} original samples")
