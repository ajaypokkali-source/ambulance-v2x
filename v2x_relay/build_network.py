"""
build_network.py

One-time build script for the v2x_relay scenario folder
(C:\\ambulance_v2x\\v2x_relay).

Identical in approach to v2x_integrated/build_network.py (same corridor,
same J2/J3/J4 traffic-light setup) - see that file for the full
rationale. This copy just points at this folder's own nodes/edges/output
so v2x_relay stays fully self-contained and never touches v2x_integrated,
v2x_multi, ambulance_v2x_city or any earlier scenario.

USAGE
-----
Run this once, from this same folder, on the machine where SUMO is
installed:

    python build_network.py

Output:
    C:\\ambulance_v2x\\v2x_relay\\network.net.xml
"""

import os
import shutil
import subprocess
import sys

PROJECT_DIR = r"C:\ambulance_v2x\v2x_relay"

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

    result = subprocess.run(cmd, capture_output=True, text=True)

    print(result.stdout)

    if result.returncode != 0:
        print("[ERROR] netconvert failed:")
        print(result.stderr)
        sys.exit(1)

    if result.stderr.strip():
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
