#!/usr/bin/env python3

import argparse
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

# Fixed UE allocation
#
# UE 0..7     -> ambulances
# UE 8..17    -> intersections
# UE 18..599  -> SUMO cars
#
AMB_BASE = 0
INTERSECTION_BASE = 8
CAR_BASE = 18


# ============================================================
# V2I SIGNAL PARAMETERS
# ============================================================

# How long an ambulance request remains valid if another
# emergency packet is not received.
BEACON_TIMEOUT = 2.0

# Safety clearance before switching conflicting green
# movements to the ambulance approach.
YELLOW_TIME = 3.0


# ============================================================
# UE MAPPINGS
# ============================================================

ambulance_to_ue = {
    f"amb{i}": AMB_BASE + (i - 1)
    for i in range(1, AMBULANCE_COUNT + 1)
}

ue_to_ambulance = {
    ue: ambulance
    for ambulance, ue in ambulance_to_ue.items()
}


intersection_to_ue = {
    f"J{i}": INTERSECTION_BASE + (i - 1)
    for i in range(1, INTERSECTION_COUNT + 1)
}

ue_to_intersection = {
    ue: junction
    for junction, ue in intersection_to_ue.items()
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
# STATE HISTORY
# ============================================================
#
# RX events contain the ns-3 bridge step number. Keeping a
# small history allows us to resolve dynamic car UE mappings
# using the mapping that existed at the RX step.
#

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
        oldest = min(step_history)
        del step_history[oldest]


def resolve_ue(ue, step):
    """
    Resolve a UE using the historical mapping for the RX step
    first, then fall back to the current mapping.
    """

    mapping = step_history.get(step)

    if mapping is not None and ue in mapping:
        return mapping[ue]

    if ue in ue_to_ambulance:
        return ue_to_ambulance[ue]

    if ue in ue_to_intersection:
        return ue_to_intersection[ue]

    if ue in ue_to_vehicle:
        return ue_to_vehicle[ue]

    return f"UE{ue}"


# ============================================================
# SUMO -> ns-3
# ============================================================

def send_state(
    step,
    ue_slot,
    active,
    x,
    y
):
    """
    Synchronization protocol:

        STATE step slot active x y
    """

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
    """
    Tell ns-3 to stop its realtime bridge.
    """

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
    """
    Assign a persistent UE to a currently active SUMO car.
    """

    if car_id in vehicle_to_ue:
        return vehicle_to_ue[car_id]

    if not free_car_ues:
        raise RuntimeError(
            "No free NR-V2X UE slots remain for cars"
        )

    ue = free_car_ues.popleft()

    vehicle_to_ue[car_id] = ue
    ue_to_vehicle[ue] = car_id

    return ue


def release_car(car_id):
    """
    Release a car's UE when the vehicle disappears from SUMO.
    """

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
# SUMO STATE TRANSMISSION
# ============================================================

def send_intersection_states(step):
    """
    Intersections are fixed NR-V2X nodes.

        UE8  -> J1
        UE9  -> J2
        ...
        UE17 -> J10
    """

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
    """
    Synchronize all eight ambulance positions.
    """

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

            # Move inactive ambulance UE far away in ns-3.
            send_state(
                step,
                ue,
                False,
                0.0,
                0.0
            )


def send_car_states(step):
    """
    Synchronize all active SUMO cars.
    """

    current_ids = set(
        traci.vehicle.getIDList()
    )

    # --------------------------------------------------------
    # Release cars that disappeared.
    # --------------------------------------------------------

    previous_car_ids = set(
        vehicle_to_ue.keys()
    )

    for car_id in previous_car_ids - current_ids:
        release_car(car_id)

    # --------------------------------------------------------
    # Allocate/update all active cars.
    # --------------------------------------------------------

    for car_id in current_ids:

        # Ambulances have fixed UE slots.
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

    def __init__(self, junction_id):

        self.id = junction_id

        # Remember original SUMO program.
        self.program = traci.trafficlight.getProgram(
            junction_id
        )

        # Controlled link groups.
        self.links = traci.trafficlight.getControlledLinks(
            junction_id
        )

        # Emergency requests:
        #
        # ambulance ->
        #     timestamp
        #     link index
        #     distance
        #
        self.heard = {}

        # Current emergency owner.
        self.owner = None

        # State machine.
        self.mode = self.NORMAL

        # CLEARING expiry time.
        self.until = 0.0

        # Target signal state for emergency movement.
        self.target = None

        # Incoming approach edge.
        self.approach_edge = None

        # Start time of current priority.
        self.since = 0.0

    # --------------------------------------------------------
    # Build emergency signal state
    # --------------------------------------------------------

    def target_state(self, link_index):
        """
        Give GREEN to all controlled links belonging to the
        same incoming road edge as the ambulance.

        Everything else becomes RED.
        """

        if (
            link_index < 0 or
            link_index >= len(self.links)
        ):
            return None

        group = self.links[link_index]

        if not group:
            return None

        if not group[0]:
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

        return "".join(state)

    # --------------------------------------------------------
    # Save/update a V2I request
    # --------------------------------------------------------

    def hear(
        self,
        now,
        ambulance_id,
        link_index,
        distance
    ):
        self.heard[ambulance_id] = {
            "time": now,
            "link": link_index,
            "distance": distance
        }

    # --------------------------------------------------------
    # Current valid requests
    # --------------------------------------------------------

    def requesters(self, now):

        valid = {}

        for ambulance_id, info in self.heard.items():

            if (
                now - info["time"]
                <= BEACON_TIMEOUT
            ):
                valid[ambulance_id] = info

        return valid

    # --------------------------------------------------------
    # Start priority
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
                f"[WARN] {self.id}: signal-state length "
                f"mismatch current={len(current)} "
                f"target={len(target)}"
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
            self.links[info["link"]][0][0]
            .rsplit("_", 1)[0]
        )

        self.since = now

        if needs_yellow:

            traci.trafficlight.setRedYellowGreenState(
                self.id,
                "".join(transition)
            )

            self.mode = self.CLEARING

            self.until = (
                now
                + YELLOW_TIME
            )

            print(
                f"[V2I] {self.id}: "
                f"{ambulance_id} requested priority "
                f"({info['distance']:.1f} m) "
                f"-> CLEARING"
            )

        else:

            traci.trafficlight.setRedYellowGreenState(
                self.id,
                target
            )

            self.mode = self.GREEN

            print(
                f"[V2I] {self.id}: "
                f"{ambulance_id} requested priority "
                f"({info['distance']:.1f} m) "
                f"-> GREEN"
            )

        return True

    # --------------------------------------------------------
    # Is ambulance physically inside this junction?
    # --------------------------------------------------------

    def ambulance_inside(self):
        """
        SUMO uses internal road IDs beginning with ':Jx_' while
        the vehicle is physically traversing the intersection.
        """

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

    # --------------------------------------------------------
    # Has ambulance cleared the junction?
    # --------------------------------------------------------

    def ambulance_cleared(self):
        """
        Once the ambulance leaves the incoming approach edge
        and is no longer on an internal junction edge, it has
        crossed the intersection.
        """

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

    # --------------------------------------------------------
    # Release owner
    # --------------------------------------------------------

    def release(
        self,
        now
    ):

        ambulance_id = self.owner

        duration = (
            now
            - self.since
        )

        print(
            f"[V2I] {self.id}: "
            f"{ambulance_id} cleared "
            f"-> RESTORE "
            f"(priority {duration:.1f}s)"
        )

        self.heard.pop(
            ambulance_id,
            None
        )

        self.owner = None

        self.mode = self.NORMAL

        self.until = 0.0

        self.target = None

        self.approach_edge = None

        self.since = 0.0

    # --------------------------------------------------------
    # Restore normal SUMO program
    # --------------------------------------------------------

    def restore(self):

        traci.trafficlight.setProgram(
            self.id,
            self.program
        )

    # --------------------------------------------------------
    # State-machine update
    # --------------------------------------------------------

    def update(
        self,
        now,
        alive
    ):

        # ----------------------------------------------------
        # NORMAL
        # ----------------------------------------------------

        if self.mode == self.NORMAL:

            requests = self.requesters(
                now
            )

            if not requests:
                return

            # Nearest ambulance gets priority.
            selected = min(
                requests,
                key=lambda ambulance: requests[
                    ambulance
                ]["distance"]
            )

            self.start(
                now,
                selected
            )

            return

        # ----------------------------------------------------
        # CLEARING
        # ----------------------------------------------------

        if self.mode == self.CLEARING:

            # Owner vanished from SUMO.
            if self.owner not in alive:

                self.release(now)
                self.restore()
                return

            # Yellow clearance complete.
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

        # ----------------------------------------------------
        # GREEN
        # ----------------------------------------------------

        if self.mode == self.GREEN:

            if self.owner not in alive:

                self.release(now)
                self.restore()
                return

            # While ambulance is physically crossing the
            # intersection, keep emergency green regardless
            # of packet freshness.
            if self.ambulance_inside():

                return

            # Once it has left the approach, release.
            if self.ambulance_cleared():

                self.release(now)

                waiting = self.requesters(
                    now
                )

                if waiting:

                    next_owner = min(
                        waiting,
                        key=lambda ambulance:
                            waiting[ambulance]["distance"]
                    )

                    self.start(
                        now,
                        next_owner
                    )

                else:

                    self.restore()

                return

            return


# ============================================================
# CREATE SIGNAL MANAGERS
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
# V2I RX DECISION
# ============================================================

def process_v2i_rx(event):
    """
    A V2I action is allowed ONLY when ns-3 reports an actual
    packet reception:

        ambulance UE -> intersection UE

    We then verify that the receiving intersection is the
    ambulance's NEXT traffic signal according to SUMO.
    """

    sender = event["sender"]

    receiver = event["receiver"]

    if sender not in ambulance_to_ue:
        return

    if receiver not in intersection_to_ue:
        return

    ambulance_id = sender
    junction_id = receiver

    try:
        upcoming = traci.vehicle.getNextTLS(
            ambulance_id
        )
    except traci.TraCIException:
        return

    if not upcoming:
        return

    # --------------------------------------------------------
    # First upcoming signal on the ambulance route.
    # --------------------------------------------------------

    next_tls_id = upcoming[0][0]

    if next_tls_id != junction_id:

        # The packet was received by an intersection, but that
        # intersection is not the ambulance's next signal.
        return

    # There can be multiple controlled links for the same TLS.
    # Pick the closest matching one.
    matching = [
        item
        for item in upcoming
        if item[0] == junction_id
    ]

    if not matching:
        return

    selected = min(
        matching,
        key=lambda item: item[2]
    )

    _, link_index, distance, _ = selected

    now = traci.simulation.getTime()

    signal = signals.get(
        junction_id
    )

    if signal is None:
        return

    signal.hear(
        now,
        ambulance_id,
        link_index,
        distance
    )

    print(
        f"[V2I RX] "
        f"{ambulance_id} -> {junction_id} "
        f"distance={distance:.1f}m "
        f"step={event['step']} "
        f"ns3_t={event['ns3_time']:.6f}"
    )


# ============================================================
# NS-3 RX EVENTS
# ============================================================

def process_rx_events():

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

        if message == "READY":
            continue

        if not message.startswith("RX"):
            continue

        parts = message.split()

        # Expected:
        #
        # RX step senderUE receiverUE ns3_time
        #

        if len(parts) != 5:
            print(
                f"[WARN] Invalid RX message: "
                f"{message}"
            )
            continue

        try:

            step = int(
                parts[1]
            )

            sender_ue = int(
                parts[2]
            )

            receiver_ue = int(
                parts[3]
            )

            ns3_time = float(
                parts[4]
            )

        except ValueError:

            print(
                f"[WARN] Could not parse RX message: "
                f"{message}"
            )

            continue

        sender_name = resolve_ue(
            sender_ue,
            step
        )

        receiver_name = resolve_ue(
            receiver_ue,
            step
        )

        event = {
            "step": step,
            "sender_ue": sender_ue,
            "receiver_ue": receiver_ue,
            "sender": sender_name,
            "receiver": receiver_name,
            "ns3_time": ns3_time,
        }

        events.append(
            event
        )

    return events


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

    while (
        time.monotonic()
        < deadline
    ):

        try:

            data, _ = rx_sock.recvfrom(
                4096
            )

        except BlockingIOError:

            time.sleep(
                0.05
            )

            continue

        except OSError as exc:

            raise RuntimeError(
                f"RX socket error while "
                f"waiting for ns-3: {exc}"
            )

        message = data.decode(
            errors="ignore"
        ).strip()

        if message == "READY":

            print(
                "[NS3] READY received"
            )

            return

        # Do not throw away a genuine early RX.
        if message.startswith("RX"):

            print(
                f"[NS3] Early RX received: "
                f"{message}"
            )

    raise RuntimeError(
        "ns-3 did not send READY "
        f"within {timeout:.1f}s"
    )


# ============================================================
# COMMAND-LINE ARGUMENTS
# ============================================================

def parse_arguments():

    parser = argparse.ArgumentParser(
        description=(
            "SUMO <-> ns-3 NR-V2X emergency "
            "corridor controller"
        )
    )

    parser.add_argument(
        "--duration",
        type=float,
        default=900.0,
        help=(
            "Simulation duration in seconds "
            "(default: 900)"
        )
    )

    parser.add_argument(
        "--pacing",
        type=float,
        default=0.45,
        help=(
            "Wall-clock delay after each SUMO step "
            "(default: 0.45 s)"
        )
    )

    return parser.parse_args()


# ============================================================
# MAIN
# ============================================================

def main():

    args = parse_arguments()

    duration = max(
        1.0,
        args.duration
    )

    pacing = max(
        0.0,
        args.pacing
    )

    print("=" * 70)
    print("SUMO <-> ns-3 NR-V2X EMERGENCY CORRIDOR")
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
        f"Duration      : {duration:.1f}s"
    )

    print(
        f"Step pacing   : {pacing:.2f}s"
    )

    print()

    ns3_process = None

    sumo_started = False

    try:

        # ====================================================
        # Start ns-3
        # ====================================================

        ns3_command = [
            "./ns3",
            "run",
            f"{NS3_PROGRAM} "
            f"--simTime={duration}",
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
        # Start SUMO
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

        # ====================================================
        # Initialize the signal managers after SUMO is live.
        # ====================================================

        initialize_signals()

        step = 0

        # ====================================================
        # Main co-simulation loop
        # ====================================================

        while True:

            sim_time = (
                traci.simulation.getTime()
            )

            if sim_time >= duration:
                break

            if (
                traci.simulation.getMinExpectedNumber()
                <= 0
            ):
                break

            # ------------------------------------------------
            # Advance SUMO by one step.
            # ------------------------------------------------

            traci.simulationStep()

            step += 1

            # ------------------------------------------------
            # SUMO -> ns-3 positions.
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

            # ------------------------------------------------
            # Save the mapping used for this step.
            # ------------------------------------------------

            save_step_mapping(
                step
            )

            # ------------------------------------------------
            # ns-3 -> Python.
            # ------------------------------------------------

            events = process_rx_events()

            # ------------------------------------------------
            # Process actual NR-V2X RX events.
            # ------------------------------------------------

            for event in events:

                process_v2i_rx(
                    event
                )

            # ------------------------------------------------
            # Update all V2I signal state machines.
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
            # Check whether ns-3 died unexpectedly.
            # ------------------------------------------------

            if (
                ns3_process is not None
                and ns3_process.poll() is not None
            ):

                raise RuntimeError(
                    "ns-3 process terminated unexpectedly"
                )

            # ------------------------------------------------
            # Realtime pacing.
            # SUMO step length is 0.5 seconds.
            # ------------------------------------------------

            if pacing > 0:

                time.sleep(
                    pacing
                )

    except KeyboardInterrupt:

        print(
            "\n[CTRL] Interrupted by user"
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

        # ----------------------------------------------------
        # Restore any signals still under emergency control.
        # ----------------------------------------------------

        for signal in signals.values():

            try:

                if signal.mode != SignalController.NORMAL:

                    signal.restore()

            except Exception:
                pass

        # ----------------------------------------------------
        # Tell ns-3 to stop.
        # ----------------------------------------------------

        send_stop()

        # ----------------------------------------------------
        # Close SUMO.
        # ----------------------------------------------------

        if sumo_started:

            try:
                traci.close()
            except Exception:
                pass

        # ----------------------------------------------------
        # Close sockets.
        # ----------------------------------------------------

        try:
            state_sock.close()
        except Exception:
            pass

        try:
            rx_sock.close()
        except Exception:
            pass

        # ----------------------------------------------------
        # Wait for ns-3.
        # ----------------------------------------------------

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
