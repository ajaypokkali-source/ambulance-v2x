#!/usr/bin/env python3

import os
import socket
import subprocess
import sys
import time
from collections import deque

import traci


# ============================================================
# CONFIGURATION
# ============================================================

HERE = os.path.dirname(os.path.abspath(__file__))

SUMO_CFG = os.path.join(HERE, "simulation.sumocfg")

NS3_STATE_IP = "127.0.0.1"
NS3_STATE_PORT = 5000

NS3_RX_IP = "127.0.0.1"
NS3_RX_PORT = 5001

MAX_UE = 600

AMBULANCE_COUNT = 8
INTERSECTION_COUNT = 10

# Fixed UE allocation
AMB_BASE = 0
INTERSECTION_BASE = 8
CAR_BASE = 18


# ============================================================
# FIXED UE MAPPING
# ============================================================

ambulance_to_ue = {
    f"amb{i}": AMB_BASE + (i - 1)
    for i in range(1, 9)
}

ue_to_ambulance = {
    ue: amb
    for amb, ue in ambulance_to_ue.items()
}

intersection_to_ue = {
    f"J{i}": INTERSECTION_BASE + (i - 1)
    for i in range(1, 11)
}

ue_to_intersection = {
    ue: jid
    for jid, ue in intersection_to_ue.items()
}


# Dynamic car mapping
vehicle_to_ue = {}
ue_to_vehicle = {}

free_car_ues = deque(
    range(
        CAR_BASE,
        MAX_UE
    )
)


# ============================================================
# SOCKETS
# ============================================================

state_sock = socket.socket(
    socket.AF_INET,
    socket.SOCK_DGRAM
)

rx_sock = socket.socket(
    socket.AF_INET,
    socket.SOCK_DGRAM
)

rx_sock.bind(
    (NS3_RX_IP, NS3_RX_PORT)
)

rx_sock.setblocking(False)


# ============================================================
# UTILITY FUNCTIONS
# ============================================================

def send_state(
    step,
    ue_slot,
    active,
    x,
    y
):
    """
    Send SUMO node state to ns-3.

    Protocol:
        STATE step slot active x y
    """

    msg = (
        f"STATE "
        f"{step} "
        f"{ue_slot} "
        f"{int(active)} "
        f"{x:.3f} "
        f"{y:.3f}"
    )

    state_sock.sendto(
        msg.encode(),
        (NS3_STATE_IP, NS3_STATE_PORT)
    )


def send_stop():
    state_sock.sendto(
        b"STOP",
        (NS3_STATE_IP, NS3_STATE_PORT)
    )


def allocate_car(car_id):

    if car_id in vehicle_to_ue:
        return vehicle_to_ue[car_id]

    if not free_car_ues:
        raise RuntimeError(
            "No free UE slots for cars"
        )

    ue = free_car_ues.popleft()

    vehicle_to_ue[car_id] = ue
    ue_to_vehicle[ue] = car_id

    return ue


def release_car(car_id):

    ue = vehicle_to_ue.pop(
        car_id,
        None
    )

    if ue is None:
        return

    ue_to_vehicle.pop(
        ue,
        None
    )

    free_car_ues.append(ue)


# ============================================================
# SEND INTERSECTION STATES
# ============================================================

def send_intersection_states(step):

    for jid, ue in intersection_to_ue.items():

        try:
            x, y = traci.junction.getPosition(jid)

        except traci.TraCIException:
            continue

        send_state(
            step,
            ue,
            True,
            x,
            y
        )


# ============================================================
# SEND AMBULANCE STATES
# ============================================================

def send_ambulance_states(step):

    for amb_id, ue in ambulance_to_ue.items():

        active = amb_id in traci.vehicle.getIDList()

        if active:

            x, y = traci.vehicle.getPosition(
                amb_id
            )

        else:

            x, y = 0.0, 0.0

        send_state(
            step,
            ue,
            active,
            x,
            y
        )


# ============================================================
# SEND CAR STATES
# ============================================================

def send_car_states(step):

    current_ids = set(
        traci.vehicle.getIDList()
    )

    # Remove cars that disappeared from SUMO
    old_ids = set(vehicle_to_ue.keys())

    for car_id in old_ids - current_ids:
        release_car(car_id)

    # Send current cars
    for car_id in current_ids:

        # Skip ambulances
        if car_id in ambulance_to_ue:
            continue

        ue = allocate_car(car_id)

        x, y = traci.vehicle.getPosition(
            car_id
        )

        send_state(
            step,
            ue,
            True,
            x,
            y
        )

# ============================================================
# RECEIVE NS-3 RX EVENTS
# ================================================x_events():

def process_rx_events():

    events = []

    while True:

        try:
            data, _ = rx_sock.recvfrom(4096)

        except BlockingIOError:
            break

        msg = data.decode(
            errors="ignore"
        ).strip()

        # ns-3 startup message
        if msg == "READY":
            continue

        if not msg.startswith("RX"):
            continue

        parts = msg.split()

        # RX step sender receiver ns3_time
        if len(parts) != 5:
            continue

        try:
            step = int(parts[1])
            sender_ue = int(parts[2])
            receiver_ue = int(parts[3])
            ns3_time = float(parts[4])
        except ValueError:
            continue

        if sender_ue in ue_to_ambulance:
            sender_name = ue_to_ambulance[sender_ue]
        elif sender_ue in ue_to_vehicle:
            sender_name = ue_to_vehicle[sender_ue]
        elif sender_ue in ue_to_intersection:
            sender_name = ue_to_intersection[sender_ue]
        else:
            sender_name = f"UE{sender_ue}"

        if receiver_ue in ue_to_vehicle:
            receiver_name = ue_to_vehicle[receiver_ue]
        elif receiver_ue in ue_to_ambulance:
            receiver_name = ue_to_ambulance[receiver_ue]
        elif receiver_ue in ue_to_intersection:
            receiver_name = ue_to_intersection[receiver_ue]
        else:
            receiver_name = f"UE{receiver_ue}"

        events.append({
            "step": step,
            "sender_ue": sender_ue,
            "receiver_ue": receiver_ue,
            "sender": sender_name,
            "receiver": receiver_name,
            "ns3_time": ns3_time,
        })

    return events    


# ============================================================
# MAIN
# ============================================================

def main():

    print("=" * 70)
    print("CITY NR-V2X CONTROLLER")
    print("=" * 70)

    print(
        f"Ambulances   : {len(ambulance_to_ue)}"
    )

    print(
        f"Intersections: {len(intersection_to_ue)}"
    )

    print(
        f"Max UEs      : {MAX_UE}"
    )

    print()

    # Start ns-3
    ns3_dir = os.path.expanduser(
        "~/v2x-project/ns-3-v2x"
    )

    ns3_cmd = [
        "./ns3",
        "run",
        "scratch/cv2x_sumo_live"
    ]

    print("[NS3] Starting...")
    ns3_process = subprocess.Popen(
        ns3_cmd,
        cwd=ns3_dir
    )

    # Give ns-3 a moment to bind sockets
    time.sleep(2)

    print("[SUMO] Starting...")

    traci.start(
        [
            "sumo-gui",
            "-c",
            SUMO_CFG
        ]
    )

    print("[BRIDGE] SUMO connected")

    step = 0

    try:

        while traci.simulation.getMinExpectedNumber() > 0:

            traci.simulationStep()

            # Send complete city state
            send_ambulance_states(step)
            send_intersection_states(step)
            send_car_states(step)

            # Read real ns-3 RX events
            events = process_rx_events()

            for event in events:

    		print(
        		f"[NR-V2X RX] "
        		f"{event['sender']} "
        		f"(UE{event['sender_ue']}) -> "
        		f"{event['receiver']} "
        		f"(UE{event['receiver_ue']}) "
        		f"step={event['step']} "
        		f"ns3_t={event['ns3_time']:.6f}"
    		)

            step += 1

            # Small real-time pacing
            time.sleep(0.01)

    except KeyboardInterrupt:

        print("\n[CTRL] Interrupted")

    finally:

        send_stop()

        try:
            traci.close()
        except Exception:
            pass

        try:
            ns3_process.terminate()
            ns3_process.wait(timeout=5)
        except Exception:
            ns3_process.kill()

        print("[CTRL] Shutdown complete")


if __name__ == "__main__":
    main()
