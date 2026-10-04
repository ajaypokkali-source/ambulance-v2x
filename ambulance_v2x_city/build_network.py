"""
build_network.py

Builds the 10-signal city grid and its background traffic.

    J1  - J2  - J3  - J4  - J5      (row 0, y =   0)
    |     |     |     |     |
    J6  - J7  - J8  - J9  - J10     (row 1, y = 300)

Only J1..J10 are traffic lights. The border nodes (W/E/S/N) are plain
entry/exit points, so SUMO never adds extra signals to them.

Run once (re-run to regenerate):

    .venv/bin/python build_network.py
"""

import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SUMO_HOME = os.environ.get("SUMO_HOME", "/usr/share/sumo")

COLS = 5
SPACING = 300.0
BORDER = 150.0

# Background traffic (runs for most of the scenario)
TRAFFIC_END = 640
TRAFFIC_PERIOD = 1.2
SEED = 42

# Ambulance scenario: (id, entry edge, exit edge, depart time s, rgb colour)
# Three waves. Wave 1 crosses at J3/J8; later waves use other corridors,
# including long diagonal trips through many signals.
AMBULANCES = [
    ("amb1", "W0_J1", "J5_E0", 20, "1,0,0"),
    ("amb2", "E1_J10", "J6_W1", 30, "0,0.4,1"),
    ("amb3", "S3_J3", "J10_E1", 40, "1,0,1"),
    ("amb4", "N5_J10", "J1_W0", 170, "1,0.5,0"),
    ("amb5", "W1_J6", "J5_S5", 190, "0,0.7,0.3"),
    ("amb6", "S1_J1", "J10_N5", 330, "0.6,0,1"),
    ("amb7", "E0_J5", "J6_W1", 350, "0,0.8,0.8"),
    ("amb8", "N3_J8", "J1_S1", 370, "1,0.8,0"),
]


def jid(row, col):
    return f"J{row * COLS + col + 1}"


def build_nodes_edges():
    nodes = []
    edges = []

    def node(nid, x, y, kind):
        nodes.append(f'    <node id="{nid}" x="{x}" y="{y}" type="{kind}"/>')

    def edge(a, b):
        edges.append(
            f'    <edge id="{a}_{b}" from="{a}" to="{b}" numLanes="2" '
            f'speed="13.89" priority="2"/>'
        )

    # Signalised junctions
    for row in range(2):
        for col in range(COLS):
            node(jid(row, col), col * SPACING, row * SPACING, "traffic_light")

    # Border entry/exit nodes
    for row in range(2):
        node(f"W{row}", -BORDER, row * SPACING, "priority")
        node(f"E{row}", (COLS - 1) * SPACING + BORDER, row * SPACING, "priority")
    for col in range(COLS):
        node(f"S{col + 1}", col * SPACING, -BORDER, "priority")
        node(f"N{col + 1}", col * SPACING, SPACING + BORDER, "priority")

    # Horizontal streets (both directions)
    for row in range(2):
        for col in range(COLS - 1):
            a, b = jid(row, col), jid(row, col + 1)
            edge(a, b)
            edge(b, a)
        edge(f"W{row}", jid(row, 0))
        edge(jid(row, 0), f"W{row}")
        edge(f"E{row}", jid(row, COLS - 1))
        edge(jid(row, COLS - 1), f"E{row}")

    # Vertical streets (both directions)
    for col in range(COLS):
        a, b = jid(0, col), jid(1, col)
        edge(a, b)
        edge(b, a)
        edge(f"S{col + 1}", a)
        edge(a, f"S{col + 1}")
        edge(f"N{col + 1}", b)
        edge(b, f"N{col + 1}")

    with open(os.path.join(HERE, "nodes.nod.xml"), "w") as f:
        f.write("<nodes>\n" + "\n".join(nodes) + "\n</nodes>\n")
    with open(os.path.join(HERE, "edges.edg.xml"), "w") as f:
        f.write("<edges>\n" + "\n".join(edges) + "\n</edges>\n")


def build_ambulances():
    """Writes ambulances.rou.xml with shortest-path routes from sumolib."""
    import sumolib

    net = sumolib.net.readNet(os.path.join(HERE, "network.net.xml"))
    out = [
        "<routes>",
        "",
        "    <!-- Emergency vehicle type. NOTE: SUMO's blue-light device is",
        "         switched OFF on purpose. With it on, SUMO ambulances simply",
        "         drive through red lights, so there would be nothing for",
        "         V2I/I2I preemption to improve. Here the ambulance obeys",
        "         signals and only gets a green via V2X. -->",
        '    <vType id="ambulance" vClass="emergency" guiShape="emergency"',
        '           accel="3.5" decel="5.0" sigma="0.1" length="6" minGap="2.0"',
        '           maxSpeed="16.67" speedFactor="1.15" lcStrategic="0.0"',
        '           lcCooperative="1.0" lcSpeedGain="3.0">',
        '        <param key="has.bluelight.device" value="false"/>',
        "    </vType>",
        "",
    ]
    for vid, src, dst, depart, rgb in AMBULANCES:
        path, _ = net.getShortestPath(net.getEdge(src), net.getEdge(dst))
        if not path:
            sys.exit(f"[ERROR] no route {src} -> {dst} for {vid}")
        edges = " ".join(e.getID() for e in path)
        out += [
            f'    <vehicle id="{vid}" type="ambulance" depart="{depart}" '
            f'color="{rgb}">',
            f'        <route edges="{edges}"/>',
            "    </vehicle>",
            "",
        ]
    out.append("</routes>")
    with open(os.path.join(HERE, "ambulances.rou.xml"), "w") as f:
        f.write("\n".join(out) + "\n")


def run(cmd):
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        print(result.stdout)
        print(result.stderr)
        sys.exit(f"[ERROR] failed: {' '.join(cmd)}")
    return result


def main():
    build_nodes_edges()
    print("[OK] nodes.nod.xml / edges.edg.xml written")

    run([
        "netconvert",
        "--node-files", os.path.join(HERE, "nodes.nod.xml"),
        "--edge-files", os.path.join(HERE, "edges.edg.xml"),
        "--output-file", os.path.join(HERE, "network.net.xml"),
        "--tls.default-type", "static",
    ])
    print("[OK] network.net.xml built")

    build_ambulances()
    print(f"[OK] ambulances.rou.xml ({len(AMBULANCES)} ambulances)")

    run([
        sys.executable,
        os.path.join(SUMO_HOME, "tools", "randomTrips.py"),
        "-n", os.path.join(HERE, "network.net.xml"),
        "-o", os.path.join(HERE, "traffic.trips.xml"),
        "-r", os.path.join(HERE, "traffic.rou.xml"),
        "-b", "0", "-e", str(TRAFFIC_END),
        "-p", str(TRAFFIC_PERIOD),
        "--fringe-factor", "10",
        "--prefix", "car",
        "--seed", str(SEED),
    ])
    print("[OK] traffic.rou.xml (background traffic) generated")


if __name__ == "__main__":
    main()
