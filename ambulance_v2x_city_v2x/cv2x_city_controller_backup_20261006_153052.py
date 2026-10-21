#!/usr/bin/env python3

import argparse
import math
import os
import socket
import subprocess
import time
from collections import deque


import traci


# ============================================================
# CONFIGURATION
# ============================================================

HERE = os.path.dirname(os.path.abspath(__file__))

SUMO_CFG = os.path.join(
    HERE,
    "simulation.sumocfg"
)

NS3_DIR = os.path.expanduser(
    "~/v2x-project/ns-3-v2x"
)

NS3_PROGRAM = "scratch/cv2x_sumo_live"

NS3_STATE_IP = "127.0.0.1"
NS3_STATE_PORT = 5000

NS3_RX_IP = "127.0.0.1"
NS3_RX_PORT = 5001

MAX_UE = 600

AMBULANCE_COUNT = 8
INTERSECTION_COUNT = 10

AMB_BASE = 0
INTERSECTION_BASE = 8
CAR_BASE = 18


# ============================================================
# SIGNAL PARAMETERS
# ============================================================

BEACON_TIMEOUT = 2.0
YELLOW_TIME = 3.0


# ============================================================
# GUI COMMUNICATION VISUALIZATION
# ============================================================

PACKET_DISPLAY_TIME = 0.5

PACKET_LINE_PREFIX = "v2x_packet_"


# ============================================================
# UE MAPPINGS
# ============================================================

ambulance_to_ue = {
    f"amb{i}": AMB_BASE + i - 1
    for i in range(1, AMBULANCE_COUNT + 1)
}

ue_to_ambulance = {
    ue: amb
    for amb, ue in ambulance_to_ue.items()
}


intersection_to_ue = {
    f"J{i}": INTERSECTION_BASE + i - 1
    for i in range(1, INTERSECTION_COUNT + 1)
}

ue_to_intersection = {
    ue: jid
    for jid, ue in intersection_to_ue.items()
}


# Dynamic car mappings
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

rx_sock.setsockopt(
    socket.SOL_SOCKET,
    socket.SO_REUSEADDR,
    1
)

rx_sock.bind(
    (
        NS3_RX_IP,
        NS3_RX_PORT
    )
)

rx_sock.setblocking(False)


# ============================================================
# STEP HISTORY
# ============================================================

MAX_STEP_HISTORY = 100

step_history = {}


def save_step_mapping(step):
    mapping = {}

    for ue, vehicle in ue_to_vehicle.items():
        mapping[ue] = vehicle

    for ue, ambulance in ue_to_ambulance.items():
        mapping[ue] = ambulance

    for ue, junction in ue_to_intersection.items():
        mapping[ue] = junction

    step_history[step] = mapping

    while len(step_history) > MAX_STEP_HISTORY:

        oldest = min(
            step_history
        )

        del step_history[
            oldest
        ]


def resolve_ue(ue, step):

    mapping = step_history.get(
        step
    )

    if mapping is not None:

        if ue in mapping:
            return mapping[ue]

    if ue in ue_to_ambulance:
        return ue_to_ambulance[ue]

    if ue in ue_to_intersection:
        return ue_to_intersection[ue]

    if ue in ue_to_vehicle:
        return ue_to_vehicle[ue]

    return f"UE{ue}"


# ============================================================
# SUMO -> ns-3 STATE
# ============================================================

def send_state(
    step,
    ue_slot,
    active,
    x,
    y
):

    message = (
        f"STATE "
        f"{step} "
        f"{ue_slot} "
        f"{1 if active else 0} "
        f"{x:.6f} "
        f"{y:.6f}"
    )

    state_sock.sendto(
        message.encode(),
        (
            NS3_STATE_IP,
            NS3_STATE_PORT
        )
    )


def send_stop():

    try:

        state_sock.sendto(
            b"STOP",
            (
                NS3_STATE_IP,
                NS3_STATE_PORT
            )
        )

    except OSError:
        pass


# ============================================================
# CAR UE MANAGEMENT
# ============================================================

def allocate_car(car_id):

    if car_id in vehicle_to_ue:
        return vehicle_to_ue[car_id]

    if not free_car_ues:

        raise RuntimeError(
            "No free NR-V2X UE slots remain for cars"
        )

    ue = free_car_ues.popleft()

    vehicle_to_ue[car_id] = ue
    ue_to_vehicle[ue] = car_id

    print(
        f"[MAP] {car_id} -> UE{ue}"
    )

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

    free_car_ues.append(
        ue
    )


# ============================================================
# STATE SENDERS
# ============================================================

def send_intersection_states(step):

    for junction_id, ue in intersection_to_ue.items():

        try:

            x, y = traci.junction.getPosition(
                junction_id
            )

        except traci.TraCIException:

            continue

        send_state(
            step,
            ue,
            True,
            x,
            y
        )


def send_ambulance_states(step):

    active_ids = set(
        traci.vehicle.getIDList()
    )

    for ambulance_id, ue in ambulance_to_ue.items():

        if ambulance_id in active_ids:

            try:

                x, y = traci.vehicle.getPosition(
                    ambulance_id
                )

            except traci.TraCIException:

                continue

            send_state(
                step,
                ue,
                True,
                x,
                y
            )

        else:

            send_state(
                step,
                ue,
                False,
                0.0,
                0.0
            )


def send_car_states(step):

    current_ids = set(
        traci.vehicle.getIDList()
    )

    old_ids = set(
        vehicle_to_ue.keys()
    )

    for car_id in old_ids - current_ids:

        release_car(
            car_id
        )

    for car_id in current_ids:

        if car_id in ambulance_to_ue:
            continue

        try:

            ue = allocate_car(
                car_id
            )

            x, y = traci.vehicle.getPosition(
                car_id
            )

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
# GUI PACKET VISUALIZATION
# ============================================================

packet_visuals = {}


def get_entity_position(name):

    if name in ambulance_to_ue:

        try:
            return traci.vehicle.getPosition(name)
        except traci.TraCIException:
            return None

    if name in vehicle_to_ue:

        try:
            return traci.vehicle.getPosition(name)
        except traci.TraCIException:
            return None

    if name in intersection_to_ue:

        try:
            return traci.junction.getPosition(name)
        except traci.TraCIException:
            return None

    return None


def create_packet_line(
    packet_id,
    sender_name,
    receiver_name
):

    sender_pos = get_entity_position(
        sender_name
    )

    receiver_pos = get_entity_position(
        receiver_name
    )

    if sender_pos is None or receiver_pos is None:
        return

    x1, y1 = sender_pos
    x2, y2 = receiver_pos

    dx = x2 - x1
    dy = y2 - y1

    length = math.hypot(
        dx,
        dy
    )

    if length < 1.0:

        # Small square around a stationary receiver.
        shape = [
            (x1 - 4, y1 - 4),
            (x1 + 4, y1 - 4),
            (x1 + 4, y1 + 4),
            (x1 - 4, y1 + 4)
        ]

    else:

        width = 1.8

        nx = -dy / length
        ny = dx / length

        shape = [
            (
                x1 + nx * width,
                y1 + ny * width
            ),
            (
                x2 + nx * width,
                y2 + ny * width
            ),
            (
                x2 - nx * width,
                y2 - ny * width
            ),
            (
                x1 - nx * width,
                y1 - ny * width
            )
        ]

    try:

        traci.polygon.add(
            packet_id,
            shape,
            (0, 170, 255, 210),
            fill=True,
            layer=100
        )

        packet_visuals[
            packet_id
        ] = traci.simulation.getTime()

    except traci.TraCIException:
        pass


def remove_old_packet_visuals():

    now = traci.simulation.getTime()

    expired = []

    for packet_id, start_time in packet_visuals.items():

        if (
            now - start_time
            >= PACKET_DISPLAY_TIME
        ):

            expired.append(
                packet_id
            )

    for packet_id in expired:

        try:

            traci.polygon.remove(
                packet_id
            )

        except traci.TraCIException:
            pass

        packet_visuals.pop(
            packet_id,
            None
        )


def add_tx_visual(event):

    sender = event["sender"]

    # TX is a broadcast from an ambulance.
    if sender in ambulance_to_ue:

        position = get_entity_position(
            sender
        )

        if position is None:
            return

        x, y = position

        # Draw a small expanding-looking square around the
        # ambulance's current position.
        size = 10.0

        shape = [
            (x - size, y - size),
            (x + size, y - size),
            (x + size, y + size),
            (x - size, y + size)
        ]

        packet_id = (
            f"{PACKET_LINE_PREFIX}"
            f"tx_{event['step']}_{event['sender_ue']}"
        )

        try:

            traci.polygon.add(
                packet_id,
                shape,
                (0, 150, 255, 110),
                fill=False,
                layer=99
            )

            packet_visuals[
                packet_id
            ] = traci.simulation.getTime()

        except traci.TraCIException:
            pass


def add_rx_visual(event):

    sender = event["sender"]
    receiver = event["receiver"]

    packet_id = (
        f"{PACKET_LINE_PREFIX}"
        f"rx_{event['step']}_"
        f"{event['sender_ue']}_"
        f"{event['receiver_ue']}"
    )

    create_packet_line(
        packet_id,
        sender,
        receiver
    )


# ============================================================
# SIGNAL CONTROLLER
# ============================================================

class SignalController:

    NORMAL = "NORMAL"
    CLEARING = "CLEARING"
    GREEN = "GREEN"

    def __init__(
        self,
        junction_id
    ):

        self.id = junction_id

        self.program = traci.trafficlight.getProgram(
            junction_id
        )

        self.links = traci.trafficlight.getControlledLinks(
            junction_id
        )

        self.heard = {}

        self.owner = None

        self.mode = self.NORMAL

        self.until = 0.0

        self.target = None

        self.approach_edge = None

        self.since = 0.0

    def target_state(
        self,
        link_index
    ):

        if (
            link_index < 0
            or link_index >= len(self.links)
        ):
            return None

        group = self.links[
            link_index
        ]

        if not group or not group[0]:
            return None

        incoming_lane = group[0][0]

        incoming_edge = incoming_lane.rsplit(
            "_",
            1
        )[0]

        state = [
            "r"
            for _ in self.links
        ]

        for i, connection_group in enumerate(
            self.links
        ):

            if not connection_group:
                continue

            if not connection_group[0]:
                continue

            lane_id = connection_group[0][0]

            edge = lane_id.rsplit(
                "_",
                1
            )[0]

            if edge == incoming_edge:

                state[i] = "G"

        return "".join(
            state
        )

    def hear(
        self,
        now,
        ambulance_id,
        link_index,
        distance
    ):

        self.heard[
            ambulance_id
        ] = {
            "time": now,
            "link": link_index,
            "distance": distance
        }

    def requesters(
        self,
        now
    ):

        return {
            amb: info
            for amb, info in self.heard.items()
            if now - info["time"] <= BEACON_TIMEOUT
        }

    def start(
        self,
        now,
        ambulance_id
    ):

        info = self.heard.get(
            ambulance_id
        )

        if info is None:
            return False

        target = self.target_state(
            info["link"]
        )

        if target is None:
            return False

        current = list(
            traci.trafficlight.getRedYellowGreenState(
                self.id
            )
        )

        if len(current) != len(target):
            print(
                f"[WARN] {self.id}: "
                f"signal state mismatch"
            )
            return False

        transition = list(
            current
        )

        needs_yellow = False

        for i, value in enumerate(
            current
        ):

            if (
                value in "Gg"
                and target[i] == "r"
            ):

                transition[i] = "y"
                needs_yellow = True

        self.owner = ambulance_id

        self.target = target

        self.approach_edge = (
            self.links[
                info["link"]
            ][0][0].rsplit(
                "_",
                1
            )[0]
        )

        self.since = now

        if needs_yellow:

            traci.trafficlight.setRedYellowGreenState(
                self.id,
                "".join(
                    transition
                )
            )

            self.mode = self.CLEARING

            self.until = (
                now
                + YELLOW_TIME
            )

            print(
                f"[V2I] {self.id}: "
                f"{ambulance_id} -> CLEARING"
            )

        else:

            traci.trafficlight.setRedYellowGreenState(
                self.id,
                target
            )

            self.mode = self.GREEN

            print(
                f"[V2I] {self.id}: "
                f"{ambulance_id} -> GREEN"
            )

        return True

    def ambulance_inside(self):

        if self.owner is None:
            return False

        try:

            road_id = traci.vehicle.getRoadID(
                self.owner
            )

        except traci.TraCIException:

            return False

        return road_id.startswith(
            f":{self.id}_"
        )

    def ambulance_cleared(self):

        if self.owner is None:
            return False

        try:

            road_id = traci.vehicle.getRoadID(
                self.owner
            )

        except traci.TraCIException:

            return True

        if road_id.startswith(
            f":{self.id}_"
        ):
            return False

        if (
            self.approach_edge is not None
            and road_id == self.approach_edge
        ):
            return False

        return True

    def release(
        self,
        now
    ):

        print(
            f"[V2I] {self.id}: "
            f"{self.owner} cleared "
            f"-> RESTORE"
        )

        self.heard.pop(
            self.owner,
            None
        )

        self.owner = None

        self.mode = self.NORMAL

        self.until = 0.0

        self.target = None

        self.approach_edge = None

        self.since = 0.0

    def restore(self):

        traci.trafficlight.setProgram(
            self.id,
            self.program
        )

    def update(
        self,
        now,
        alive
    ):

        if self.mode == self.NORMAL:

            requests = self.requesters(
                now
            )

            if not requests:
                return

            selected = min(
                requests,
                key=lambda amb:
                    requests[amb]["distance"]
            )

            self.start(
                now,
                selected
            )

            return

        if self.mode == self.CLEARING:

            if self.owner not in alive:

                self.release(
                    now
                )

                self.restore()

                return

            if now >= self.until:

                if self.target is not None:

                    traci.trafficlight.setRedYellowGreenState(
                        self.id,
                        self.target
                    )

                self.mode = self.GREEN

                print(
                    f"[V2I] {self.id}: "
                    f"GREEN for {self.owner}"
                )

            return

        if self.mode == self.GREEN:

            if self.owner not in alive:

                self.release(
                    now
                )

                self.restore()

                return

            if self.ambulance_inside():
                return

            if self.ambulance_cleared():

                self.release(
                    now
                )

                waiting = self.requesters(
                    now
                )

                if waiting:

                    selected = min(
                        waiting,
                        key=lambda amb:
                            waiting[amb]["distance"]
                    )

                    self.start(
                        now,
                        selected
                    )

                else:

                    self.restore()

                return


# ============================================================
# SIGNAL SETUP
# ============================================================

signals = {}


def initialize_signals():

    global signals

    signals = {}

    for junction_id in intersection_to_ue:

        try:

            signals[junction_id] = SignalController(
                junction_id
            )

        except traci.TraCIException as exc:

            print(
                f"[WARN] Could not initialize "
                f"{junction_id}: {exc}"
            )

    print(
        f"[TLS] Initialized "
        f"{len(signals)} traffic signals"
    )


# ============================================================
# V2I DECISION
# ============================================================

def process_v2i_rx(
    event
):

    ambulance_id = event[
        "sender"
    ]

    junction_id = event[
        "receiver"
    ]

    if ambulance_id not in ambulance_to_ue:
        return

    if junction_id not in intersection_to_ue:
        return

    try:

        upcoming = traci.vehicle.getNextTLS(
            ambulance_id
        )

    except traci.TraCIException:

        return

    if not upcoming:

        return

    matching = [
        entry
        for entry in upcoming
        if entry[0] == junction_id
    ]

    if not matching:

        # This RX was physically received, but this is not
        # the next signal on the ambulance's route.
        return

    closest = min(
        matching,
        key=lambda entry: entry[2]
    )

    _, link_index, distance, _ = closest

    signal = signals.get(
        junction_id
    )

    if signal is None:
        return

    now = traci.simulation.getTime()

    signal.hear(
        now,
        ambulance_id,
        link_index,
        distance
    )

    print(
        f"[V2I RX] "
        f"{ambulance_id} -> {junction_id} "
        f"distance={distance:.1f}m"
    )


# ============================================================
# NS-3 EVENTS
# ============================================================

def process_ns3_events():

    events = []

    while True:

        try:

            data, _ = rx_sock.recvfrom(
                4096
            )

        except BlockingIOError:

            break

        except OSError:

            break

        message = data.decode(
            errors="ignore"
        ).strip()

        if not message:
            continue

        # ----------------------------------------------------
        # Actual transmission event
        #
        # TX step senderUE ns3_time
        # ----------------------------------------------------

        if message.startswith("TX"):

            parts = message.split()

            if len(parts) != 4:
                continue

            try:

                step = int(parts[1])
                sender_ue = int(parts[2])
                ns3_time = float(parts[3])

            except ValueError:

                continue

            event = {
                "kind": "TX",
                "step": step,
                "sender_ue": sender_ue,
                "sender": resolve_ue(
                    sender_ue,
                    step
                ),
                "ns3_time": ns3_time
            }

            events.append(
                event
            )

            continue

        # ----------------------------------------------------
        # Actual reception event
        #
        # RX step senderUE receiverUE ns3_time
        # ----------------------------------------------------

        if message.startswith("RX"):

            parts = message.split()

            if len(parts) != 5:
                continue

            try:

                step = int(parts[1])
                sender_ue = int(parts[2])
                receiver_ue = int(parts[3])
                ns3_time = float(parts[4])

            except ValueError:

                continue

            event = {
                "kind": "RX",
                "step": step,
                "sender_ue": sender_ue,
                "receiver_ue": receiver_ue,
                "sender": resolve_ue(
                    sender_ue,
                    step
                ),
                "receiver": resolve_ue(
                    receiver_ue,
                    step
                ),
                "ns3_time": ns3_time
            }

            events.append(
                event
            )

    return events


# ============================================================
# NS-3 READY
# ============================================================

def wait_for_ns3_ready(
    timeout=20.0
):

    print(
        "[NS3] Waiting for READY..."
    )

    deadline = (
        time.monotonic()
        + timeout
    )

    while time.monotonic() < deadline:

        try:

            data, _ = rx_sock.recvfrom(
                4096
            )

        except BlockingIOError:

            time.sleep(
                0.05
            )

            continue

        message = data.decode(
            errors="ignore"
        ).strip()

        if message == "READY":

            print(
                "[NS3] READY received"
            )

            return

    raise RuntimeError(
        "ns-3 did not send READY"
    )


# ============================================================
# COMMAND LINE
# ============================================================

def parse_arguments():

    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--duration",
        type=float,
        default=180.0
    )

    parser.add_argument(
        "--pacing",
        type=float,
        default=0.45
    )

    return parser.parse_args()


# ============================================================
# MAIN
# ============================================================

def main():

    args = parse_arguments()

    print("=" * 70)
    print("SUMO <-> ns-3 NR-V2X CITY")
    print("=" * 70)

    ns3_process = None

    sumo_started = False

    try:

        # ----------------------------------------------------
        # Start ns-3
        # ----------------------------------------------------

        ns3_command = [
            "./ns3",
            "run",
            f"{NS3_PROGRAM} "
            f"--simTime={args.duration}"
        ]

        print(
            "[NS3] Starting:",
            " ".join(ns3_command)
        )

        ns3_process = subprocess.Popen(
            ns3_command,
            cwd=NS3_DIR
        )

        wait_for_ns3_ready()

        # ----------------------------------------------------
        # Start SUMO
        # ----------------------------------------------------

        print(
            "[SUMO] Starting..."
        )

        traci.start(
            [
                "sumo-gui",
                "-c",
                SUMO_CFG
            ]
        )

        sumo_started = True

        print(
            "[SUMO] Connected"
        )

        initialize_signals()

        step = 0

        # ----------------------------------------------------
        # Main loop
        # ----------------------------------------------------

        while True:

            sim_time = traci.simulation.getTime()

            if sim_time >= args.duration:
                break

            if (
                traci.simulation.getMinExpectedNumber()
                <= 0
            ):
                break

            traci.simulationStep()

            step += 1

            # ------------------------------------------------
            # SUMO -> ns-3
            # ------------------------------------------------

            send_ambulance_states(
                step
            )

            send_intersection_states(
                step
            )

            send_car_states(
                step
            )

            save_step_mapping(
                step
            )

            # ------------------------------------------------
            # ns-3 -> Python
            # ------------------------------------------------

            events = process_ns3_events()

            # ------------------------------------------------
            # Process communication
            # ------------------------------------------------

            for event in events:

                if event["kind"] == "TX":

                    print(
                        f"[NR-V2X TX] "
                        f"{event['sender']} "
                        f"(UE{event['sender_ue']}) "
                        f"ns3_t={event['ns3_time']:.6f}"
                    )

                    add_tx_visual(
                        event
                    )

                elif event["kind"] == "RX":

                    print(
                        f"[NR-V2X RX] "
                        f"{event['sender']} "
                        f"(UE{event['sender_ue']}) -> "
                        f"{event['receiver']} "
                        f"(UE{event['receiver_ue']}) "
                        f"ns3_t={event['ns3_time']:.6f}"
                    )

                    add_rx_visual(
                        event
                    )

                    process_v2i_rx(
                        event
                    )

            # ------------------------------------------------
            # Update signal state machines
            # ------------------------------------------------

            alive = set(
                traci.vehicle.getIDList()
            )

            now = traci.simulation.getTime()

            for signal in signals.values():

                signal.update(
                    now,
                    alive
                )

            # ------------------------------------------------
            # Remove old communication overlays
            # ------------------------------------------------

            remove_old_packet_visuals()

            # ------------------------------------------------
            # Check ns-3 is alive
            # ------------------------------------------------

            if (
                ns3_process is not None
                and ns3_process.poll() is not None
            ):

                raise RuntimeError(
                    "ns-3 terminated unexpectedly"
                )

            time.sleep(
                max(
                    0.0,
                    args.pacing
                )
            )

    except KeyboardInterrupt:

        print(
            "\n[CTRL] Interrupted"
        )

    finally:

        print(
            "\n[CTRL] Shutting down..."
        )

        # Restore signals.
        for signal in signals.values():

            try:

                if signal.mode != SignalController.NORMAL:
                    signal.restore()

            except Exception:
                pass

        send_stop()

        if sumo_started:

            try:
                traci.close()
            except Exception:
                pass

        try:
            state_sock.close()
        except Exception:
            pass

        try:
            rx_sock.close()
        except Exception:
            pass

        if ns3_process is not None:

            try:

                ns3_process.wait(
                    timeout=8
                )

            except subprocess.TimeoutExpired:

                ns3_process.terminate()

                try:

                    ns3_process.wait(
                        timeout=3
                    )

                except subprocess.TimeoutExpired:

                    ns3_process.kill()

        print(
            "[CTRL] Shutdown complete"
        )


if __name__ == "__main__":
    main()
