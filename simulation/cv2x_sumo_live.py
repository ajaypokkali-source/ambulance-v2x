#!/usr/bin/env python3

import math
import os
import socket
import time

import traci


# ============================================================
# USER SETTINGS
# ============================================================

SUMO_CONFIG = os.path.join(
    os.path.dirname(__file__),
    "simulation.sumocfg"
)

HOST = "127.0.0.1"

# Python -> ns-3
NS3_INPUT_PORT = 5000

# ns-3 -> Python
NS3_RESULT_PORT = 5001

# One ambulance UE + four vehicle UEs for the first integration.
NUM_VEHICLE_UES = 4

# SUMO vehicle states are selected on the same road and ahead
# of the ambulance whenever possible.
MAX_STEP_HISTORY = 20


# ============================================================
# SOCKETS
# ============================================================

tx_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

rx_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
rx_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
rx_socket.bind((HOST, NS3_RESULT_PORT))
rx_socket.setblocking(False)


# ============================================================
# WAIT FOR ns-3
# ============================================================

print("=" * 70)
print("SUMO <-> NR-V2X LIVE BRIDGE")
print("=" * 70)
print("Waiting for ns-3 NR-V2X bridge on UDP port 5001...")

ready = False

deadline = time.monotonic() + 30.0

while time.monotonic() < deadline:
    try:
        data, _ = rx_socket.recvfrom(1024)
        msg = data.decode("utf-8", errors="ignore").strip()

        if msg == "READY":
            ready = True
            break

    except BlockingIOError:
        time.sleep(0.05)

if not ready:
    print("ERROR: ns-3 bridge was not found.")
    print("Start cv2x_sumo_live first, then run this script.")
    tx_socket.close()
    rx_socket.close()
    raise SystemExit(1)

print("ns-3 bridge is READY.")


# ============================================================
# SUMO
# ============================================================

sumo_cmd = [
    "sumo-gui",
    "-c",
    SUMO_CONFIG
]

print(f"SUMO config: {SUMO_CONFIG}")

traci.start(sumo_cmd)

print("SUMO connected.")


# ============================================================
# HELPERS
# ============================================================

step_counter = 0
slot_history = {}


def send_state(step_id, slot, active, x=0.0, y=0.0):
    msg = (
        f"STATE {step_id} {slot} {1 if active else 0} "
        f"{x:.6f} {y:.6f}"
    )

    tx_socket.sendto(
        msg.encode("utf-8"),
        (HOST, NS3_INPUT_PORT)
    )


def drain_nr_v2x_rx():
    while True:
        try:
            data, _ = rx_socket.recvfrom(1024)

        except BlockingIOError:
            break

        text = data.decode("utf-8", errors="ignore").strip()
        fields = text.split()

        if len(fields) != 4 or fields[0] != "RX":
            continue

        try:
            step_id = int(fields[1])
            slot = int(fields[2])
            ns3_time = float(fields[3])

        except ValueError:
            continue

        mapping = slot_history.get(step_id)

        if mapping is None:
            continue

        vehicle_id = mapping.get(slot)

        if vehicle_id is not None:
            print(
                f"[C-V2X RX] {vehicle_id} received "
                f"NR-V2X packet at ns-3 t={ns3_time:.6f}s"
            )


def choose_vehicles(vehicles, ambulance_id):
    if ambulance_id not in vehicles:
        return []

    try:
        ambulance_x, ambulance_y = traci.vehicle.getPosition(
            ambulance_id
        )

        ambulance_road = traci.vehicle.getRoadID(
            ambulance_id
        )

    except traci.TraCIException:
        return []

    same_road = []

    for vehicle_id in vehicles:
        if vehicle_id == ambulance_id:
            continue

        try:
            road = traci.vehicle.getRoadID(vehicle_id)

            if road != ambulance_road:
                continue

            x, y = traci.vehicle.getPosition(vehicle_id)

            # For a first emergency-warning demo, only consider
            # vehicles in front of the ambulance.
            longitudinal = x - ambulance_x

            if longitudinal <= 0:
                continue

            distance = math.hypot(
                x - ambulance_x,
                y - ambulance_y
            )

            same_road.append(
                (distance, vehicle_id, x, y)
            )

        except traci.TraCIException:
            continue

    same_road.sort(key=lambda item: item[0])

    return same_road[:NUM_VEHICLE_UES]


# ============================================================
# MAIN LOOP
# ============================================================

try:

    while traci.simulation.getMinExpectedNumber() > 0:

        traci.simulationStep()

        step_counter += 1

        vehicles = traci.vehicle.getIDList()

        if "ambulance" not in vehicles:
            # No ambulance currently active.
            send_state(step_counter, 0, False)

            for slot in range(1, NUM_VEHICLE_UES + 1):
                send_state(step_counter, slot, False)

            drain_nr_v2x_rx()

            time.sleep(0.05)
            continue

        selected = choose_vehicles(
            vehicles,
            "ambulance"
        )

        mapping = {
            0: "ambulance"
        }

        for slot, item in enumerate(
            selected,
            start=1
        ):
            _, vehicle_id, _, _ = item
            mapping[slot] = vehicle_id

        slot_history[step_counter] = mapping

        # Keep only recent mappings.
        old_steps = [
            old_step
            for old_step in slot_history
            if old_step < step_counter - MAX_STEP_HISTORY
        ]

        for old_step in old_steps:
            del slot_history[old_step]

        # ----------------------------------------------------
        # Ambulance
        # ----------------------------------------------------

        try:
            x, y = traci.vehicle.getPosition("ambulance")
            send_state(
                step_counter,
                0,
                True,
                x,
                y
            )

        except traci.TraCIException:
            send_state(step_counter, 0, False)

        # ----------------------------------------------------
        # Vehicles
        # ----------------------------------------------------

        for slot in range(1, NUM_VEHICLE_UES + 1):

            vehicle_id = mapping.get(slot)

            if vehicle_id is None:
                send_state(
                    step_counter,
                    slot,
                    False
                )
                continue

            try:
                x, y = traci.vehicle.getPosition(
                    vehicle_id
                )

                send_state(
                    step_counter,
                    slot,
                    True,
                    x,
                    y
                )

            except traci.TraCIException:
                send_state(
                    step_counter,
                    slot,
                    False
                )

        # Read any NR-V2X reception results.
        drain_nr_v2x_rx()

        # Keep SUMO approximately synchronized with ns-3 realtime mode.
        step_length = traci.simulation.getDeltaT()
        time.sleep(step_length)


finally:

    try:
        tx_socket.sendto(
            b"STOP",
            (HOST, NS3_INPUT_PORT)
        )

    except OSError:
        pass

    tx_socket.close()
    rx_socket.close()

    try:
        traci.close()

    except Exception:
        pass

print("=" * 70)
print("SUMO <-> NR-V2X bridge finished")
print("=" * 70)
