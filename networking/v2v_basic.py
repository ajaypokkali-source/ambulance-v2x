import os
import sys
import random
import math
from pathlib import Path

import traci


# ============================================================
# CONFIGURATION
# ============================================================

SIMULATION_END_TIME = 300

COMMUNICATION_RANGE = 300.0      # meters
PACKET_LOSS_PROBABILITY = 0.10   # 10%
NETWORK_LATENCY = 0.05           # 50 ms
BROADCAST_INTERVAL = 0.10        # 100 ms

AMBULANCE_ID = "ambulance"


# ============================================================
# PATHS
# ============================================================

# networking/
#     v2v_basic.py
#
# repo/
#     v2v_stress_test/
#         simulation.sumocfg

REPO_ROOT = Path(__file__).resolve().parent.parent

SUMO_CONFIG = (
    REPO_ROOT
    / "v2v_stress_test"
    / "simulation.sumocfg"
)


# ============================================================
# STATISTICS
# ============================================================

packets_sent = 0
packets_received = 0
packets_lost = 0

latencies = []


# ============================================================
# VEHICLE DISTANCE
# ============================================================

def get_distance(vehicle_a, vehicle_b):
    """Return Euclidean distance between two vehicles."""

    x1, y1 = traci.vehicle.getPosition(vehicle_a)
    x2, y2 = traci.vehicle.getPosition(vehicle_b)

    return math.sqrt(
        (x2 - x1) ** 2 +
        (y2 - y1) ** 2
    )


# ============================================================
# CREATE EMERGENCY PACKET
# ============================================================

def create_emergency_packet(sim_time):

    x, y = traci.vehicle.getPosition(AMBULANCE_ID)

    speed = traci.vehicle.getSpeed(AMBULANCE_ID)

    road = traci.vehicle.getRoadID(AMBULANCE_ID)

    packet = {
        "sender": AMBULANCE_ID,
        "type": "EMERGENCY",
        "timestamp": sim_time,
        "x": x,
        "y": y,
        "speed": speed,
        "road": road,
    }

    return packet


# ============================================================
# BROADCAST PACKET
# ============================================================

def broadcast_packet(packet, vehicles, sim_time):

    global packets_sent
    global packets_received
    global packets_lost

    for vehicle_id in vehicles:

        # Don't send the ambulance's packet to itself
        if vehicle_id == AMBULANCE_ID:
            continue

        distance = get_distance(
            AMBULANCE_ID,
            vehicle_id
        )

        # ----------------------------------------------------
        # RANGE CHECK
        # ----------------------------------------------------

        if distance > COMMUNICATION_RANGE:
            continue

        packets_sent += 1

        # ----------------------------------------------------
        # PACKET LOSS
        # ----------------------------------------------------

        if random.random() < PACKET_LOSS_PROBABILITY:

            packets_lost += 1

            print(
                f"[{sim_time:6.2f}s] "
                f"PACKET LOST  "
                f"ambulance -> {vehicle_id}  "
                f"distance={distance:6.1f}m"
            )

            continue

        # ----------------------------------------------------
        # PACKET RECEIVED
        # ----------------------------------------------------

        packets_received += 1

        receive_time = sim_time + NETWORK_LATENCY

        latency_ms = NETWORK_LATENCY * 1000

        latencies.append(latency_ms)

        print(
            f"[{receive_time:6.2f}s] "
            f"V2V RECEIVED  "
            f"ambulance -> {vehicle_id}  "
            f"distance={distance:6.1f}m  "
            f"latency={latency_ms:.0f}ms"
        )


# ============================================================
# MAIN
# ============================================================

def main():

    global packets_sent
    global packets_received
    global packets_lost

    print("=" * 60)
    print(" BASIC V2V COMMUNICATION SIMULATION")
    print("=" * 60)

    print(f"SUMO config : {SUMO_CONFIG}")
    print(f"Range       : {COMMUNICATION_RANGE} m")
    print(f"Packet loss : {PACKET_LOSS_PROBABILITY * 100:.0f}%")
    print(f"Latency     : {NETWORK_LATENCY * 1000:.0f} ms")
    print()

    if not SUMO_CONFIG.exists():

        print("ERROR: SUMO configuration file not found.")
        print(SUMO_CONFIG)
        return

    # --------------------------------------------------------
    # START SUMO
    # --------------------------------------------------------

    sumo_binary = "sumo-gui"

    sumo_cmd = [
        sumo_binary,
        "-c",
        str(SUMO_CONFIG),
        "--start",
    ]

    traci.start(sumo_cmd)

    print("SUMO started.")
    print()

    last_broadcast_time = -BROADCAST_INTERVAL

    try:

        while traci.simulation.getTime() < SIMULATION_END_TIME:

            traci.simulationStep()

            sim_time = traci.simulation.getTime()

            vehicles = traci.vehicle.getIDList()

            # ------------------------------------------------
            # CHECK AMBULANCE
            # ------------------------------------------------

            if AMBULANCE_ID not in vehicles:

                continue

            # ------------------------------------------------
            # PERIODIC EMERGENCY BROADCAST
            # ------------------------------------------------

            if (
                sim_time - last_broadcast_time
                >= BROADCAST_INTERVAL
            ):

                packet = create_emergency_packet(sim_time)

                print()
                print(
                    f"[{sim_time:6.2f}s] "
                    f"EMERGENCY BROADCAST"
                )

                print(
                    f"             "
                    f"position=({packet['x']:.1f}, "
                    f"{packet['y']:.1f})  "
                    f"speed={packet['speed']:.2f}m/s"
                )

                broadcast_packet(
                    packet,
                    vehicles,
                    sim_time
                )

                last_broadcast_time = sim_time

    finally:

        traci.close()

    # ========================================================
    # RESULTS
    # ========================================================

    print()
    print("=" * 60)
    print(" V2V COMMUNICATION RESULTS")
    print("=" * 60)

    print(f"Packets sent     : {packets_sent}")
    print(f"Packets received : {packets_received}")
    print(f"Packets lost     : {packets_lost}")

    if packets_sent > 0:

        delivery_ratio = (
            packets_received / packets_sent
        ) * 100

        loss_ratio = (
            packets_lost / packets_sent
        ) * 100

        print(
            f"Delivery ratio   : {delivery_ratio:.2f}%"
        )

        print(
            f"Loss ratio       : {loss_ratio:.2f}%"
        )

    if latencies:

        average_latency = (
            sum(latencies) / len(latencies)
        )

        print(
            f"Average latency  : "
            f"{average_latency:.2f} ms"
        )

    print("=" * 60)


# ============================================================
# ENTRY POINT
# ============================================================

if __name__ == "__main__":
    main()