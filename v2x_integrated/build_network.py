"""
build_network.py

One-time build script for this self-contained V2X-integrated scenario
folder (C:\\ambulance_v2x\\v2x_integrated).

WHY THIS EXISTS
----------------
This scenario needs J2, J3 and J4 to be REAL traffic-light junctions
(so V2I/I2I emergency-signal preemption has something to act on),
while J1 and J5 stay plain "priority" junctions - same corridor shape
as the v2v_stress_test baseline, just with signals added at J2/J3/J4.

You cannot hand-edit a compiled SUMO network.net.xml safely (junction
requests, internal lanes and <tlLogic> link indices all have to stay in
sync). The correct, safe way to do this is to let SUMO's own
`netconvert` tool regenerate the network from the node/edge definitions
- so that's what this script does.

USAGE
-----
Run this once, from this same folder, on the machine where SUMO is
installed:

    python build_network.py

It looks for netconvert in this order:
    1. %SUMO_HOME%\\bin\\netconvert.exe
    2. D:\\Softwares\\SUMO\\bin\\netconvert.exe   (your known SUMO install)
    3. "netconvert" on PATH

Output:
    C:\\ambulance_v2x\\v2x_integrated\\network.net.xml

This folder is fully self-contained - nothing in v2v_stress_test,
tls_simulation or simulation is read or written by this script.
"""

import os
import shutil
import subprocess
import sys

PROJECT_DIR = r"C:\ambulance_v2x\v2x_integrated"

NODES_FILE = os.path.join(PROJECT_DIR, "nodes.nod.xml")
EDGES_FILE = os.path.join(PROJECT_DIR, "edges.edg.xml")
OUTPUT_NET = os.path.join(PROJECT_DIR, "network.net.xml")

FALLBACK_SUMO_HOME = r"D:\Softwares\SUMO"


def find_netconvert():

    sumo_home = os.environ.get("SUMO_HOME")

    candidates = []

    if sumo_home:
        candidates.append(os.path.join(sumo_home, "bin", "netconvert.exe"))

    candidates.append(
        os.path.join(FALLBACK_SUMO_HOME, "bin", "netconvert.exe")
    )

    for candidate in candidates:
        if os.path.isfile(candidate):
            return candidate

    on_path = shutil.which("netconvert")

    if on_path:
        return on_path

    return None


def main():

    print("=" * 70)
    print("  BUILDING network.net.xml (J2 / J3 / J4 = traffic_light)")
    print("=" * 70)

    if not os.path.isfile(NODES_FILE):
        print(f"[ERROR] Missing file: {NODES_FILE}")
        sys.exit(1)

    if not os.path.isfile(EDGES_FILE):
        print(f"[ERROR] Missing file: {EDGES_FILE}")
        sys.exit(1)

    netconvert = find_netconvert()

    if netconvert is None:
        print(
            "[ERROR] Could not find netconvert.exe.\n"
            "        Set the SUMO_HOME environment variable, or edit\n"
            "        FALLBACK_SUMO_HOME at the top of this script."
        )
        sys.exit(1)

    print(f"[INFO] Using netconvert: {netconvert}")
    print(f"[INFO] Nodes file      : {NODES_FILE}")
    print(f"[INFO] Edges file      : {EDGES_FILE}")
    print(f"[INFO] Output network  : {OUTPUT_NET}")
    print()

    cmd = [
        netconvert,
        "--node-files", NODES_FILE,
        "--edge-files", EDGES_FILE,
        "--output-file", OUTPUT_NET,
        "--tls.default-type", "static",
    ]

    # NOTE: --tls.guess is deliberately NOT used. The traffic lights we
    # want (J2, J3, J4) are already declared explicitly in nodes.nod.xml
    # via type="traffic_light" - letting netconvert "guess" additional
    # ones could add a traffic light at J1 (it has 3 incoming edges:
    # J1N_J1, J1S_J1, J2_J1), which we do NOT want, since the ambulance
    # departs already just past J1.

    result = subprocess.run(cmd, capture_output=True, text=True)

    print(result.stdout)

    if result.returncode != 0:
        print("[ERROR] netconvert failed:")
        print(result.stderr)
        sys.exit(1)

    if result.stderr.strip():
        # netconvert prints warnings to stderr even on success
        print("[netconvert warnings]")
        print(result.stderr)

    if os.path.isfile(OUTPUT_NET):
        print()
        print("=" * 70)
        print(f"  SUCCESS: {OUTPUT_NET}")
        print("=" * 70)
    else:
        print("[ERROR] netconvert did not produce the expected output file.")
        sys.exit(1)


if __name__ == "__main__":
    main()
