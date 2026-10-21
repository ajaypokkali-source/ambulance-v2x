#!/usr/bin/env python3

import math
import os
import socket
import time
from collections import deque

import traci

# ============================================================
# SUMO <-> ns-3 NR-V2X STRESS-TEST CONTROLLER
# ============================================================
# SUMO remains responsible for traffic/mobility.
# ns-3 + 5G-LENA/NR-V2X remains responsible for communication.
# This Python file only bridges the two and applies a traffic action
# AFTER ns-3 reports that the emergency packet was actually received.
# ============================================================

SUMO_CONFIG = os.path.join(os.path.dirname(__file__), "simulation.sumocfg")

HOST = "127.0.0.1"
NS3_INPUT_PORT = 5000
NS3_RESULT_PORT = 5001

# Must match the C++ bridge's ueNum.
# UE0 is the ambulance, UE1..UE39 are vehicle slots.
MAX_UE = 40

SIMULATION_END_TIME = 300.0

# Application-level emergency warning distance.
# This is NOT the PHY radio limit; NR-V2X decides whether a packet is received.
C_V2X_APPLICATION_RANGE = 400.0

# A received emergency message remains actionable for this long.
MESSAGE_VALIDITY = 10.0

# Existing corridor behavior parameters.
PREPARE_DISTANCE = 100.0
EMERGENCY_DISTANCE = 25.0
INTERSECTION_DETECTION_DISTANCE = 100.0
INTERSECTION_EMERGENCY_DISTANCE = 70.0
MIN_FRONT_GAP = 15.0
MIN_REAR_GAP = 15.0
LANE_CHANGE_DURATION = 5.0
STOP_SPEED_THRESHOLD = 0.5

# Ambulance route/intersections used by the existing stress-test scenario.
INTERSECTION_ORDER = ["J1", "J2", "J3", "J4", "J5"]
INTERSECTION_X = {
    "J1": 0.0,
    "J2": 300.0,
    "J3": 600.0,
    "J4": 900.0,
    "J5": 1200.0,
}

INTERSECTIONS = {
    "J1": {"incoming": ["J1N_J1", "J1S_J1", "J2_J1"]},
    "J2": {"incoming": ["J1_J2", "J3_J2", "J2N_J2", "J2S_J2"]},
    "J3": {"incoming": ["J2_J3", "J4_J3", "J3N_J3", "J3S_J3"]},
    "J4": {"incoming": ["J3_J4", "J5_J4", "J4N_J4", "J4S_J4"]},
    "J5": {"incoming": ["J4_J5", "J5N_J5", "J5S_J5"]},
}

# ============================================================
# UDP SOCKETS
# ============================================================

tx_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

rx_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
rx_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
rx_socket.bind((HOST, NS3_RESULT_PORT))
rx_socket.setblocking(False)


# ============================================================
# WAIT FOR ns-3
# ============================================================

print("=" * 75)
print("SUMO <-> 5G NR-V2X STRESS TEST")
print("=" * 75)
print("Waiting for ns-3 bridge...")

ready = False
deadline = time.monotonic() + 30.0

while time.monotonic() < deadline:
    try:
        data, _ = rx_socket.recvfrom(1024)
        if data.decode("utf-8", errors="ignore").strip() == "READY":
            ready = True
            break
    except BlockingIOError:
        time.sleep(0.05)

if not ready:
    raise SystemExit("ERROR: ns-3 bridge was not found on UDP 5001.")

print("[BRIDGE] ns-3 is READY.")


# ============================================================
# START SUMO GUI
# ============================================================

traci.start(["sumo-gui", "-c", SUMO_CONFIG])
print(f"[SUMO] Connected: {SUMO_CONFIG}")


# ============================================================
# STABLE VEHICLE <-> UE MAPPING
# ============================================================

vehicle_to_ue = {"ambulance": 0}
ue_to_vehicle = {0: "ambulance"}

# Reuse the fixed ns-3 UE slots when SUMO vehicles leave the scenario.
free_ues = deque(range(1, MAX_UE))

# Track which slots were active in the previous SUMO step.
previous_active_slots = set()

# Keep a recent per-step mapping so an RX event can be mapped back
# to the SUMO vehicle that occupied that slot at that simulation step.
step_slot_to_vehicle = {}

# vehicle_id -> expiry time of last actual NR-V2X RX.
emergency_until = {}

# Prevent repeated SUMO actions for every periodic packet.
last_action_time = {}

# vehicle_id -> target lane index for an already-requested lane change.
pending_lane_changes = {}

# vehicle_id -> intersection ID while it is yielding/stopped.
yielding_vehicles = {}


def allocate_ue(vehicle_id):
    if vehicle_id in vehicle_to_ue:
        return vehicle_to_ue[vehicle_id]

    if not free_ues:
        raise RuntimeError(
            f"No free NR-V2X UE slots remain. "
            f"MAX_UE={MAX_UE}, active vehicles={len(vehicle_to_ue)}"
        )

    slot = free_ues.popleft()
    vehicle_to_ue[vehicle_id] = slot
    ue_to_vehicle[slot] = vehicle_id

    print(f"[MAP] {vehicle_id:20s} -> UE{slot}")
    return slot


def release_ue(vehicle_id):
    if vehicle_id == "ambulance":
        return

    slot = vehicle_to_ue.pop(vehicle_id, None)
    if slot is None:
        return

    ue_to_vehicle.pop(slot, None)
    free_ues.append(slot)

    emergency_until.pop(vehicle_id, None)
    last_action_time.pop(vehicle_id, None)
    pending_lane_changes.pop(vehicle_id, None)
    yielding_vehicles.pop(vehicle_id, None)

    print(f"[MAP] {vehicle_id:20s} LEFT -> UE{slot} released")


# ============================================================
# BRIDGE HELPERS
# ============================================================

def send_state(step_id, slot, active, x=0.0, y=0.0):
    msg = f"STATE {step_id} {slot} {1 if active else 0} {x:.6f} {y:.6f}"
    tx_socket.sendto(msg.encode("utf-8"), (HOST, NS3_INPUT_PORT))


def junction_distance_from_vehicle(vehicle_id, junction_id):
    """Use the real SUMO junction position when available."""
    try:
        jx, jy = traci.junction.getPosition(junction_id)
        vx, vy = traci.vehicle.getPosition(vehicle_id)
        return math.hypot(jx - vx, jy - vy)
    except traci.TraCIException:
        try:
            vx, _ = traci.vehicle.getPosition(vehicle_id)
            return abs(INTERSECTION_X[junction_id] - vx)
        except Exception:
            return float("inf")


def next_intersection_for_ambulance(ambulance_x):
    for junction_id in INTERSECTION_ORDER:
        d = INTERSECTION_X[junction_id] - ambulance_x
        if 0.0 <= d <= INTERSECTION_DETECTION_DISTANCE:
            return junction_id, d
    return None, None


def safe_adjacent_lane(vehicle_id, vehicle_road, vehicle_lane_index, vehicles):
    try:
        lane_count = traci.edge.getLaneNumber(vehicle_road)
    except traci.TraCIException:
        lane_count = 1

    candidates = []
    if vehicle_lane_index + 1 < lane_count:
        candidates.append(vehicle_lane_index + 1)
    if vehicle_lane_index - 1 >= 0:
        candidates.append(vehicle_lane_index - 1)

    try:
        vehicle_lane_pos = traci.vehicle.getLanePosition(vehicle_id)
    except traci.TraCIException:
        return None

    for candidate_lane in candidates:
        candidate_lane_id = f"{vehicle_road}_{candidate_lane}"
        front_gap = float("inf")
        rear_gap = float("inf")

        for other_id in vehicles:
            if other_id in (vehicle_id, "ambulance"):
                continue

            try:
                if traci.vehicle.getRoadID(other_id) != vehicle_road:
                    continue
                if traci.vehicle.getLaneID(other_id) != candidate_lane_id:
                    continue

                other_pos = traci.vehicle.getLanePosition(other_id)
            except traci.TraCIException:
                continue

            gap = other_pos - vehicle_lane_pos
            if gap > 0:
                front_gap = min(front_gap, gap)
            elif gap < 0:
                rear_gap = min(rear_gap, abs(gap))

        if front_gap >= MIN_FRONT_GAP and rear_gap >= MIN_REAR_GAP:
            return candidate_lane

    return None


def restore_speed(vehicle_id):
    try:
        traci.vehicle.setSpeed(vehicle_id, -1)
    except traci.TraCIException:
        pass


def stop_vehicle(vehicle_id):
    try:
        traci.vehicle.setSpeed(vehicle_id, 0.0)
        return True
    except traci.TraCIException:
        return False


def process_nr_v2x_rx(current_time, vehicles, ambulance_position):
    """Consume actual RX events produced by ns-3."""
    received_now = []

    while True:
        try:
            data, _ = rx_socket.recvfrom(1024)
        except BlockingIOError:
            break

        fields = data.decode("utf-8", errors="ignore").strip().split()
        if len(fields) != 4 or fields[0] != "RX":
            continue

        try:
            _step = int(fields[1])
            slot = int(fields[2])
            ns3_time = float(fields[3])
        except ValueError:
            continue

        # Use the mapping associated with the reported simulation step.
        # Fall back to the current slot mapping if the bridge event is delayed.
        vehicle_id = step_slot_to_vehicle.get(_step, {}).get(slot)
        if vehicle_id is None:
            vehicle_id = ue_to_vehicle.get(slot)

        if vehicle_id is None or vehicle_id == "ambulance":
            continue
        if vehicle_id not in vehicles:
            continue

        try:
            vx, vy = traci.vehicle.getPosition(vehicle_id)
            distance = math.hypot(vx - ambulance_position[0], vy - ambulance_position[1])
        except traci.TraCIException:
            continue

        if distance > C_V2X_APPLICATION_RANGE:
            continue

        emergency_until[vehicle_id] = current_time + MESSAGE_VALIDITY
        received_now.append(vehicle_id)

        print(
            f"[C-V2X RX] {vehicle_id} <- ambulance "
            f"distance={distance:.1f} m ns3_t={ns3_time:.3f}s"
        )

    return received_now


# ============================================================
# MAIN LOOP
# ============================================================

step_counter = 0

try:
    while traci.simulation.getTime() < SIMULATION_END_TIME and traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        step_counter += 1
        current_time = traci.simulation.getTime()
        vehicles = list(traci.vehicle.getIDList())

        # Disable autonomous strategic/tactical lane changes exactly as in the original stress test.
        for vehicle_id in vehicles:
            if vehicle_id == "ambulance":
                continue
            try:
                traci.vehicle.setLaneChangeMode(vehicle_id, 512)
            except traci.TraCIException:
                pass

        # ----------------------------------------------------
        # RELEASE UE SLOTS FOR VEHICLES THAT LEFT SUMO
        # ----------------------------------------------------
        current_vehicle_set = set(vehicles)

        departed = [
            vehicle_id
            for vehicle_id in list(vehicle_to_ue.keys())
            if vehicle_id != "ambulance" and vehicle_id not in current_vehicle_set
        ]

        for vehicle_id in departed:
            release_ue(vehicle_id)

        # Allocate a reusable UE slot for each currently active vehicle.
        for vehicle_id in vehicles:
            allocate_ue(vehicle_id)

        # ----------------------------------------------------
        # SEND COMPLETE SUMO STATE TO ns-3
        # ----------------------------------------------------
        active_slots = set()
        current_step_mapping = {}

        for vehicle_id in vehicles:
            slot = vehicle_to_ue[vehicle_id]
            current_step_mapping[slot] = vehicle_id

            try:
                x, y = traci.vehicle.getPosition(vehicle_id)
                send_state(step_counter, slot, True, x, y)
                active_slots.add(slot)
            except traci.TraCIException:
                send_state(step_counter, slot, False)

        step_slot_to_vehicle[step_counter] = current_step_mapping

        # Keep only recent mappings.
        old_steps = [
            old_step
            for old_step in step_slot_to_vehicle
            if old_step < step_counter - 30
        ]
        for old_step in old_steps:
            del step_slot_to_vehicle[old_step]

        # Mark slots that disappeared from SUMO as inactive in ns-3.
        for slot in previous_active_slots - active_slots:
            send_state(step_counter, slot, False)

        previous_active_slots = active_slots

        # ----------------------------------------------------
        # AMBULANCE STATE
        # ----------------------------------------------------
        if "ambulance" not in vehicles:
            time.sleep(traci.simulation.getDeltaT())
            continue

        ambulance_x, ambulance_y = traci.vehicle.getPosition("ambulance")
        ambulance_position = (ambulance_x, ambulance_y)
        ambulance_road = traci.vehicle.getRoadID("ambulance")

        # ----------------------------------------------------
        # FIRST: accept only actual NR-V2X RX events
        # ----------------------------------------------------
        process_nr_v2x_rx(current_time, set(vehicles), ambulance_position)

        # ----------------------------------------------------
        # DETERMINE NEXT INTERSECTION
        # ----------------------------------------------------
        next_junction, _ = next_intersection_for_ambulance(ambulance_x)

        # ----------------------------------------------------
        # ACTIONS ONLY FOR VEHICLES THAT RECEIVED NR-V2X
        # ----------------------------------------------------
        for vehicle_id in vehicles:
            if vehicle_id == "ambulance":
                continue

            if emergency_until.get(vehicle_id, -1.0) < current_time:
                continue

            try:
                vehicle_road = traci.vehicle.getRoadID(vehicle_id)
                vehicle_lane = traci.vehicle.getLaneID(vehicle_id)
                vehicle_lane_index = traci.vehicle.getLaneIndex(vehicle_id)
            except traci.TraCIException:
                continue

            # ================================================
            # SAME-ROAD TRAFFIC: CREATE EMERGENCY LANE SPACE
            # ================================================
            if vehicle_road == ambulance_road and vehicle_id not in pending_lane_changes:
                try:
                    ambulance_lane_pos = traci.vehicle.getLanePosition("ambulance")
                    vehicle_lane_pos = traci.vehicle.getLanePosition(vehicle_id)
                    longitudinal_gap = vehicle_lane_pos - ambulance_lane_pos
                except traci.TraCIException:
                    longitudinal_gap = -1.0

                if 0.0 < longitudinal_gap <= PREPARE_DISTANCE and vehicle_lane == traci.vehicle.getLaneID("ambulance"):
                    target_lane = safe_adjacent_lane(
                        vehicle_id,
                        vehicle_road,
                        vehicle_lane_index,
                        vehicles,
                    )

                    if target_lane is not None:
                        try:
                            traci.vehicle.changeLane(
                                vehicle_id,
                                target_lane,
                                LANE_CHANGE_DURATION,
                            )
                            pending_lane_changes[vehicle_id] = target_lane
                            print(
                                f"[C-V2X ACTION] {vehicle_id} YIELD -> "
                                f"lane {target_lane} (gap={longitudinal_gap:.1f}m)"
                            )
                        except traci.TraCIException as exc:
                            print(f"[C-V2X ACTION] lane change failed for {vehicle_id}: {exc}")

            # ================================================
            # INTERSECTION TRAFFIC: STOP WHEN AMBULANCE IS CLOSE
            # ================================================
            if next_junction is not None:
                if vehicle_road in INTERSECTIONS[next_junction]["incoming"] and vehicle_road != ambulance_road:
                    vehicle_distance = junction_distance_from_vehicle(vehicle_id, next_junction)
                    ambulance_distance = junction_distance_from_vehicle("ambulance", next_junction)

                    if vehicle_distance <= INTERSECTION_DETECTION_DISTANCE:
                        yielding_vehicles[vehicle_id] = next_junction

                        if ambulance_distance <= INTERSECTION_EMERGENCY_DISTANCE:
                            if stop_vehicle(vehicle_id):
                                print(
                                    f"[C-V2X ACTION] {vehicle_id} STOP at {next_junction} "
                                    f"(vehicle={vehicle_distance:.1f}m, ambulance={ambulance_distance:.1f}m)"
                                )

        # ----------------------------------------------------
        # COMPLETE LANE CHANGE TRACKING
        # ----------------------------------------------------
        completed = []
        for vehicle_id, target_lane in pending_lane_changes.items():
            if vehicle_id not in vehicles:
                completed.append(vehicle_id)
                continue
            try:
                if traci.vehicle.getLaneIndex(vehicle_id) == target_lane:
                    print(f"[C-V2X ACTION] {vehicle_id} lane change completed")
                    completed.append(vehicle_id)
            except traci.TraCIException:
                completed.append(vehicle_id)

        for vehicle_id in completed:
            pending_lane_changes.pop(vehicle_id, None)

        # ----------------------------------------------------
        # RELEASE INTERSECTION VEHICLES AFTER AMBULANCE PASSES
        # ----------------------------------------------------
        release = []
        for vehicle_id, junction_id in yielding_vehicles.items():
            if vehicle_id not in vehicles:
                release.append(vehicle_id)
                continue

            try:
                junction_x = INTERSECTION_X[junction_id]
                if ambulance_x > junction_x + 15.0:
                    restore_speed(vehicle_id)
                    print(f"[C-V2X ACTION] {vehicle_id} RESUME after {junction_id} cleared")
                    release.append(vehicle_id)
            except (KeyError, traci.TraCIException):
                release.append(vehicle_id)

        for vehicle_id in release:
            yielding_vehicles.pop(vehicle_id, None)

        # Keep SUMO roughly aligned with ns-3 realtime simulation.
        time.sleep(max(0.0, traci.simulation.getDeltaT()))

finally:
    try:
        tx_socket.sendto(b"STOP", (HOST, NS3_INPUT_PORT))
    except OSError:
        pass

    tx_socket.close()
    rx_socket.close()

    try:
        traci.close()
    except Exception:
        pass

print("=" * 75)
print("SUMO <-> 5G NR-V2X STRESS TEST FINISHED")
print("=" * 75)
