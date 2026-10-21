#!/usr/bin/env python3

import argparse
import os
import socket
import subprocess
import time
from collections import deque

import traci


# ============================================================
# PATHS
# ============================================================

HERE = os.path.dirname(
    os.path.abspath(__file__)
)

SUMO_CFG = os.path.join(
    HERE,
    "simulation.sumocfg"
)

NS3_DIR = os.path.expanduser(
    "~/v2x-project/ns-3-v2x"
)

NS3_PROGRAM = "scratch/cv2x_sumo_live"


# ============================================================
# UDP BRIDGE
# ============================================================

STATE_IP = "127.0.0.1"
STATE_PORT = 5000

RX_IP = "127.0.0.1"
RX_PORT = 5001


# ============================================================
# CITY / UE CONFIGURATION
# ============================================================

MAX_UE = 600

AMBULANCE_COUNT = 8
INTERSECTION_COUNT = 10

AMB_BASE = 0
INTERSECTION_BASE = 8
CAR_BASE = 18


# ============================================================
# EMERGENCY SIGNAL PARAMETERS
# ============================================================

YELLOW_TIME = 3.0

# How long a V2I reception remains considered fresh.
BEACON_TIMEOUT = 2.0

# I2I warning lifetime.
ARM_TTL = 120.0

# Do not repeatedly send the same I2I request every
# single simulation step.
I2I_REPEAT_GUARD = 5.0


# ============================================================
# FIXED UE MAPPINGS
# ============================================================

ambulance_to_ue = {
    f"amb{i}": AMB_BASE + (i - 1)
    for i in range(1, AMBULANCE_COUNT + 1)
}

ue_to_ambulance = {
    ue: amb
    for amb, ue in ambulance_to_ue.items()
}


intersection_to_ue = {
    f"J{i}": INTERSECTION_BASE + (i - 1)
    for i in range(1, INTERSECTION_COUNT + 1)
}

ue_to_intersection = {
    ue: junction
    for junction, ue in intersection_to_ue.items()
}


# ============================================================
# DYNAMIC CAR MAPPING
# ============================================================

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
        RX_IP,
        RX_PORT
    )
)

rx_sock.setblocking(
    False
)


# ============================================================
# RX STEP HISTORY
# ============================================================

STEP_HISTORY_LIMIT = 100

step_history = {}


def save_step_mapping(
    step
):
    mapping = {}

    for ue, amb in ue_to_ambulance.items():
        mapping[ue] = amb

    for ue, junction in ue_to_intersection.items():
        mapping[ue] = junction

    for ue, vehicle in ue_to_vehicle.items():
        mapping[ue] = vehicle

    step_history[
        step
    ] = mapping

    while len(step_history) > STEP_HISTORY_LIMIT:

        oldest = min(
            step_history
        )

        del step_history[
            oldest
        ]


def resolve_ue(
    ue,
    step
):

    historical = step_history.get(
        step
    )

    if historical is not None:

        if ue in historical:
            return historical[ue]

    if ue in ue_to_ambulance:
        return ue_to_ambulance[ue]

    if ue in ue_to_intersection:
        return ue_to_intersection[ue]

    if ue in ue_to_vehicle:
        return ue_to_vehicle[ue]

    return f"UE{ue}"


# ============================================================
# SUMO -> NS-3 STATE
# ============================================================

def send_state(
    step,
    ue,
    active,
    x,
    y
):

    msg = (
        f"STATE "
        f"{step} "
        f"{ue} "
        f"{1 if active else 0} "
        f"{x:.3f} "
        f"{y:.3f}"
    )

    state_sock.sendto(
        msg.encode(),
        (
            STATE_IP,
            STATE_PORT
        )
    )


def send_stop():

    try:

        state_sock.sendto(
            b"STOP",
            (
                STATE_IP,
                STATE_PORT
            )
        )

    except OSError:
        pass


# ============================================================
# CAR UE MANAGEMENT
# ============================================================

def allocate_car(
    car_id
):

    if car_id in vehicle_to_ue:

        return vehicle_to_ue[
            car_id
        ]

    if not free_car_ues:

        raise RuntimeError(
            "No free UE slots remain for cars"
        )

    ue = free_car_ues.popleft()

    vehicle_to_ue[
        car_id
    ] = ue

    ue_to_vehicle[
        ue
    ] = car_id

    return ue


def release_car(
    car_id
):

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
# SEND INTERSECTION STATE
# ============================================================

def send_intersection_states(
    step
):

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


# ============================================================
# SEND AMBULANCE STATE
# ============================================================

def send_ambulance_states(
    step
):

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


# ============================================================
# SEND CAR STATE
# ============================================================

def send_car_states(
    step
):

    current_ids = set(
        traci.vehicle.getIDList()
    )

    old_ids = set(
        vehicle_to_ue.keys()
    )

    # Release disappeared cars.
    for car_id in old_ids - current_ids:

        release_car(
            car_id
        )

    # Add/update active cars.
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

        self.program = (
            traci.trafficlight.getProgram(
                junction_id
            )
        )

        self.links = (
            traci.trafficlight.getControlledLinks(
                junction_id
            )
        )

        # ambulance -> {
        #     "time": reception time,
        #     "link": incoming link index,
        #     "distance": distance
        # }
        self.heard = {}

        # Current emergency owner.
        self.owner = None

        self.mode = (
            self.NORMAL
        )

        self.until = 0.0

        self.target = None

        self.approach_edge = None

        # Route index on which the ambulance approached this junction.
        # Cleared only after SUMO reports that the ambulance has crossed it.
        self.approach_route_index = None

        self.since = 0.0

        # I2I information received by this junction.
        self.armed = {}

    # --------------------------------------------------------
    # Build emergency green state
    # --------------------------------------------------------

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

        if not group:
            return None

        if not group[0]:
            return None

        incoming_lane = group[0][0]

        incoming_edge = (
            incoming_lane.rsplit(
                "_",
                1
            )[0]
        )

        state = [
            "r"
            for _ in self.links
        ]

        for i, group_i in enumerate(
            self.links
        ):

            if not group_i:
                continue

            if not group_i[0]:
                continue

            lane_id = group_i[0][0]

            edge = (
                lane_id.rsplit(
                    "_",
                    1
                )[0]
            )

            if edge == incoming_edge:

                state[i] = "G"

        return "".join(
            state
        )

    # --------------------------------------------------------
    # Record V2I reception
    # --------------------------------------------------------

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

    # --------------------------------------------------------
    # Return fresh V2I requests
    # --------------------------------------------------------

    def requesters(
        self,
        now
    ):

        valid = {}

        for amb, info in self.heard.items():

            if (
                now - info["time"]
                <= BEACON_TIMEOUT
            ):

                valid[
                    amb
                ] = info

        return valid

    # --------------------------------------------------------
    # Record I2I arm
    # --------------------------------------------------------

    def arm(
        self,
        ambulance_id,
        previous_junction,
        now
    ):

        self.armed[
            ambulance_id
        ] = {
            "time": now,
            "from": previous_junction
        }

        print(
            f"[I2I] {self.id}: "
            f"ALERT RECEIVED from "
            f"{previous_junction} "
            f"for {ambulance_id}"
        )

    # --------------------------------------------------------
    # Is an I2I arm valid?
    # --------------------------------------------------------

    def is_armed(
        self,
        ambulance_id,
        now
    ):

        info = self.armed.get(
            ambulance_id
        )

        if info is None:
            return False

        return (
            now - info["time"]
            <= ARM_TTL
        )

    # --------------------------------------------------------
    # Start emergency priority
    # --------------------------------------------------------

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
                f"signal length mismatch"
            )

            return False

        transition = list(
            current
        )

        needs_yellow = False

        for i, current_state in enumerate(
            current
        ):

            if (
                current_state in "Gg"
                and target[i] == "r"
            ):

                transition[i] = "y"

                needs_yellow = True

        self.owner = (
            ambulance_id
        )

        self.target = (
            target
        )

        self.approach_edge = (
            self.links[
                info["link"]
            ][0][0]
            .rsplit(
                "_",
                1
            )[0]
        )

        try:
            self.approach_route_index = traci.vehicle.getRouteIndex(
                ambulance_id
            )
        except traci.TraCIException:
            self.approach_route_index = None

        self.since = now

        if needs_yellow:

            traci.trafficlight.setRedYellowGreenState(
                self.id,
                "".join(
                    transition
                )
            )

            self.mode = (
                self.CLEARING
            )

            self.until = (
                now
                + YELLOW_TIME
            )

            print(
                f"[V2I] {self.id}: "
                f"{ambulance_id} "
                f"requested priority "
                f"({info['distance']:.1f} m) "
                f"-> CLEARING"
            )

        else:

            traci.trafficlight.setRedYellowGreenState(
                self.id,
                target
            )

            self.mode = (
                self.GREEN
            )

            print(
                f"[V2I] {self.id}: "
                f"{ambulance_id} "
                f"requested priority "
                f"({info['distance']:.1f} m) "
                f"-> GREEN"
            )

        return True

    # --------------------------------------------------------
    # Is ambulance inside the physical junction?
    # --------------------------------------------------------

    def ambulance_inside(
        self
    ):

        if self.owner is None:

            return False

        try:

            road_id = (
                traci.vehicle.getRoadID(
                    self.owner
                )
            )

        except traci.TraCIException:

            return False

        return road_id.startswith(
            f":{self.id}_"
        )

    # --------------------------------------------------------
    # Has ambulance cleared the junction?
    # --------------------------------------------------------

    def ambulance_cleared(
        self
    ):

        if self.owner is None:
            return False

        # If the route index wasn't captured, do not guess that the
        # vehicle cleared the junction.
        if self.approach_route_index is None:
            return False

        try:
            road_id = traci.vehicle.getRoadID(self.owner)
            current_route_index = traci.vehicle.getRouteIndex(self.owner)
        except traci.TraCIException:
            return False

        # Never release while the ambulance is on an internal junction edge.
        if road_id.startswith(f":{self.id}_"):
            return False

        # Remaining on the incoming route edge means it has not crossed
        # the signal, even if getNextTLS() temporarily stops listing it.
        if current_route_index <= self.approach_route_index:
            return False

        # SUMO's actual route index advanced beyond the incoming edge.
        return True

    # --------------------------------------------------------
    # Release
    # --------------------------------------------------------

    def release(
        self,
        now
    ):

        old_owner = (
            self.owner
        )

        print(
            f"[V2I] {self.id}: "
            f"{old_owner} cleared "
            f"-> RESTORE"
        )

        self.heard.pop(
            old_owner,
            None
        )

        self.owner = None

        self.mode = (
            self.NORMAL
        )

        self.until = 0.0

        self.target = None

        self.approach_edge = None

        self.approach_route_index = None

        self.since = 0.0

    # --------------------------------------------------------
    # Restore normal signal program
    # --------------------------------------------------------

    def restore(self):

        traci.trafficlight.setProgram(
            self.id,
            self.program
        )

    # --------------------------------------------------------
    # Signal state machine
    # --------------------------------------------------------

    def update(
        self,
        now,
        alive
    ):

        # ====================================================
        # NORMAL
        # ====================================================

        if self.mode == self.NORMAL:

            requests = self.requesters(
                now
            )

            if not requests:

                return

            # If two ambulances request the same junction,
            # the one nearest to the stop line wins.
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

        # ====================================================
        # CLEARING
        # ====================================================

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

                self.mode = (
                    self.GREEN
                )

                print(
                    f"[V2I] {self.id}: "
                    f"GREEN for "
                    f"{self.owner}"
                )

            return

        # ====================================================
        # GREEN
        # ====================================================

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
# SIGNAL OBJECTS
# ============================================================

signals = {}


def initialize_signals():

    signals.clear()

    for junction_id in intersection_to_ue:

        try:

            signals[
                junction_id
            ] = SignalController(
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
# GET ONLY THE NEXT SIGNAL
# ============================================================

def get_next_tls(
    ambulance_id
):

    try:

        upcoming = (
            traci.vehicle.getNextTLS(
                ambulance_id
            )
        )

    except traci.TraCIException:

        return None

    if not upcoming:

        return None

    # IMPORTANT:
    #
    # Only the FIRST upcoming traffic signal is allowed to
    # respond directly to an ambulance's V2I alert.
    #
    # We deliberately do NOT scan the whole list.
    #
    tls_id, link_index, distance, state = (
        upcoming[0]
    )

    if tls_id not in signals:

        return None

    return {
        "junction": tls_id,
        "link": link_index,
        "distance": distance,
        "state": state
    }


# ============================================================
# GET NEXT SIGNAL AFTER A GIVEN JUNCTION
# ============================================================

def get_signal_after(
    ambulance_id,
    current_junction
):

    try:

        upcoming = (
            traci.vehicle.getNextTLS(
                ambulance_id
            )
        )

    except traci.TraCIException:

        return None

    found_current = False

    for tls_id, link_index, distance, state in upcoming:

        if not found_current:

            if tls_id == current_junction:

                found_current = True

            continue

        if tls_id not in signals:

            continue

        return tls_id

    return None


# ============================================================
# I2I SEND REQUEST
# ============================================================

last_i2i_sent = {}


def request_i2i(
    source_junction,
    target_junction,
    ambulance_id
):

    if (
        source_junction not in intersection_to_ue
        or target_junction not in intersection_to_ue
    ):

        return

    now = (
        traci.simulation.getTime()
    )

    key = (
        ambulance_id,
        source_junction,
        target_junction
    )

    previous = last_i2i_sent.get(
        key,
        -1e9
    )

    if (
        now - previous
        < I2I_REPEAT_GUARD
    ):

        return

    sender_ue = (
        intersection_to_ue[
            source_junction
        ]
    )

    target_ue = (
        intersection_to_ue[
            target_junction
        ]
    )

    command = (
        f"I2I "
        f"{sender_ue} "
        f"{target_ue}"
    )

    try:

        state_sock.sendto(
            command.encode(),
            (
                STATE_IP,
                STATE_PORT
            )
        )

        last_i2i_sent[
            key
        ] = now

        print(
            f"[I2I REQUEST] "
            f"{source_junction} -> "
            f"{target_junction} "
            f"for {ambulance_id}"
        )

    except OSError:

        print(
            f"[WARN] Could not send "
            f"I2I request "
            f"{source_junction} -> "
            f"{target_junction}"
        )


# ============================================================
# I2I SOURCE OWNERS
# ============================================================
#
# Used so an I2I packet can be associated with the ambulance
# whose corridor the source intersection is currently serving.
# ============================================================

corridor_owner = {}


# ============================================================
# V2I RX
# ============================================================

def process_v2i_rx(
    event
):

    ambulance_id = (
        event["sender"]
    )

    receiving_junction = (
        event["receiver"]
    )

    if ambulance_id not in ambulance_to_ue:

        return

    if receiving_junction not in intersection_to_ue:

        return

    # --------------------------------------------------------
    # CRITICAL FIX:
    #
    # Ask SUMO for ONLY the first upcoming signal.
    # --------------------------------------------------------

    next_tls = get_next_tls(
        ambulance_id
    )

    if next_tls is None:

        return

    if (
        next_tls["junction"]
        != receiving_junction
    ):

        # The packet really was received by this junction,
        # but it is NOT the ambulance's immediate next signal.
        #
        # Therefore:
        #
        #   no signal preemption
        #   no I2I propagation
        #
        return

    link_index = (
        next_tls["link"]
    )

    distance = (
        next_tls["distance"]
    )

    signal = signals.get(
        receiving_junction
    )

    if signal is None:

        return

    now = (
        traci.simulation.getTime()
    )

    signal.hear(
        now,
        ambulance_id,
        link_index,
        distance
    )

    # This junction is now associated with this ambulance
    # for I2I propagation.
    corridor_owner[
        receiving_junction
    ] = ambulance_id

    print(
        f"[V2I RX] "
        f"{ambulance_id} -> "
        f"{receiving_junction} "
        f"distance={distance:.1f}m "
        f"(NEXT TLS)"
    )

    # --------------------------------------------------------
    # One-hop I2I
    # --------------------------------------------------------

    next_junction = get_signal_after(
        ambulance_id,
        receiving_junction
    )

    if next_junction is not None:

        request_i2i(
            receiving_junction,
            next_junction,
            ambulance_id
        )


# ============================================================
# I2I RX
# ============================================================

def process_i2i_rx(
    event
):

    source_junction = (
        event["sender"]
    )

    target_junction = (
        event["receiver"]
    )

    if source_junction not in intersection_to_ue:

        return

    if target_junction not in intersection_to_ue:

        return

    ambulance_id = corridor_owner.get(
        source_junction
    )

    if ambulance_id is None:

        return

    # --------------------------------------------------------
    # Verify that target is EXACTLY the next signal.
    # --------------------------------------------------------

    expected_next = get_signal_after(
        ambulance_id,
        source_junction
    )

    if expected_next != target_junction:

        return

    target_signal = signals.get(
        target_junction
    )

    if target_signal is None:

        return

    now = (
        traci.simulation.getTime()
    )

    target_signal.arm(
        ambulance_id,
        source_junction,
        now
    )

    print(
        f"[I2I RX] "
        f"{source_junction} -> "
        f"{target_junction} "
        f"for {ambulance_id}"
    )


# ============================================================
# NS-3 RX EVENTS
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
        # TX
        #
        # We receive these from ns-3 for diagnostics only.
        # ----------------------------------------------------

        if message.startswith(
            "TX"
        ):

            parts = message.split()

            if len(parts) != 4:

                continue

            try:

                step = int(parts[1])
                sender_ue = int(parts[2])
                ns3_time = float(parts[3])

            except ValueError:

                continue

            events.append({
                "kind": "TX",
                "step": step,
                "sender_ue": sender_ue,
                "sender": resolve_ue(
                    sender_ue,
                    step
                ),
                "ns3_time": ns3_time
            })

            continue

        # ----------------------------------------------------
        # RX
        #
        # RX step senderUE receiverUE ns3_time
        # ----------------------------------------------------

        if message.startswith(
            "RX"
        ):

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

            events.append({
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
            })

    return events


# ============================================================
# DUPLICATE RX FILTER
# ============================================================

recent_rx = deque(
    maxlen=2000
)

recent_rx_set = set()


def rx_is_duplicate(
    event
):

    key = (
        event["step"],
        event["sender_ue"],
        event["receiver_ue"],
        round(
            event["ns3_time"],
            6
        )
    )

    if key in recent_rx_set:

        return True

    recent_rx.append(
        key
    )

    recent_rx_set.add(
        key
    )

    # Keep set synchronized with bounded deque.
    while len(recent_rx_set) > len(recent_rx):

        recent_rx_set.clear()

        for item in recent_rx:

            recent_rx_set.add(
                item
            )

    return False


# ============================================================
# WAIT FOR ns-3 READY
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
# ARGUMENTS
# ============================================================

def parse_args():

    parser = argparse.ArgumentParser(
        description=(
            "SUMO <-> ns-3 NR-V2X "
            "emergency corridor"
        )
    )

    parser.add_argument(
        "--duration",
        type=float,
        default=120.0,
        help=(
            "Simulation duration in seconds"
        )
    )

    parser.add_argument(
        "--pacing",
        type=float,
        default=0.5,
        help=(
            "Wall-clock delay per SUMO step"
        )
    )

    return parser.parse_args()


# ============================================================
# MAIN
# ============================================================

def main():

    args = parse_args()

    print("=" * 70)
    print(
        "SUMO <-> ns-3 NR-V2X "
        "EMERGENCY CORRIDOR"
    )
    print("=" * 70)

    print(
        f"Ambulances    : {AMBULANCE_COUNT}"
    )

    print(
        f"Intersections : {INTERSECTION_COUNT}"
    )

    print(
        f"Max UEs       : {MAX_UE}"
    )

    print(
        "UE mapping    : "
        "UE0-7 ambulances | "
        "UE8-17 intersections | "
        "UE18-599 cars"
    )

    print(
        f"Duration      : {args.duration:.1f}s"
    )

    print()

    ns3_process = None
    sumo_started = False

    try:

        # ====================================================
        # START ns-3
        # ====================================================

        ns3_command = [
            "./ns3",
            "run",
            f"{NS3_PROGRAM} "
            f"--simTime={args.duration}"
        ]

        print(
            "[NS3] Starting:"
        )

        print(
            " ".join(
                ns3_command
            )
        )

        ns3_process = subprocess.Popen(
            ns3_command,
            cwd=NS3_DIR
        )

        wait_for_ns3_ready()

        # ====================================================
        # START SUMO
        # ====================================================

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

        # ====================================================
        # MAIN CO-SIMULATION LOOP
        # ====================================================

        while True:

            sim_time = (
                traci.simulation.getTime()
            )

            if sim_time >= args.duration:

                break

            if (
                traci.simulation.getMinExpectedNumber()
                <= 0
            ):

                break

            # ------------------------------------------------
            # Advance SUMO.
            # ------------------------------------------------

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
            # Process RX events.
            # ------------------------------------------------

            for event in events:

                if event["kind"] != "RX":

                    continue

                # Ignore exact duplicate RX records.
                if rx_is_duplicate(
                    event
                ):

                    continue

                sender = event[
                    "sender"
                ]

                receiver = event[
                    "receiver"
                ]

                sumo_now = traci.simulation.getTime()
                print(
                    f"[NR-V2X RX] "
                    f"{sender} (UE"
                    f"{event['sender_ue']}) -> "
                    f"{receiver} (UE"
                    f"{event['receiver_ue']}) "
                    f"sumo_t={sumo_now:.2f} "
                    f"ns3_t={event['ns3_time']:.6f}"
                )

                # ------------------------------------------------
                # Ambulance -> intersection = V2I
                # ------------------------------------------------

                if (
                    sender in ambulance_to_ue
                    and receiver in intersection_to_ue
                ):

                    process_v2i_rx(
                        event
                    )

                # ------------------------------------------------
                # Intersection -> intersection = I2I
                # ------------------------------------------------

                elif (
                    sender in intersection_to_ue
                    and receiver in intersection_to_ue
                ):

                    process_i2i_rx(
                        event
                    )

            # ------------------------------------------------
            # Update signals.
            # ------------------------------------------------

            alive = set(
                traci.vehicle.getIDList()
            )

            now = (
                traci.simulation.getTime()
            )

            for signal in signals.values():

                signal.update(
                    now,
                    alive
                )

                # Remove stale corridor ownership after
                # the signal no longer serves that ambulance.
                if (
                    signal.mode
                    == SignalController.NORMAL
                    and signal.id in corridor_owner
                    and signal.owner is None
                ):

                    # Do not immediately delete the mapping:
                    # I2I may still be useful while the next
                    # signal is clearing the ambulance corridor.
                    pass

            # ------------------------------------------------
            # Check ns-3 process.
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

    except Exception as exc:

        print(
            f"\n[CTRL] ERROR: {exc}"
        )

        raise

    finally:

        print(
            "\n[CTRL] Shutting down..."
        )

        # Restore all signals.
        for signal in signals.values():

            try:

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
