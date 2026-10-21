#!/usr/bin/env python3

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

SUMO_CFG = os.path.join(HERE, "simulation.sumocfg")

NS3_DIR = os.path.expanduser(
    "~/v2x-project/ns-3-v2x"
)

NS3_PROGRAM = "scratch/cv2x_sumo_live"

# Run the network simulation for the full SUMO scenario.
NS3_SIM_TIME = 900

NS3_STATE_IP = "127.0.0.1"
NS3_STATE_PORT = 5000

NS3_RX_IP = "127.0.0.1"
NS3_RX_PORT = 5001

MAX_UE = 600

AMBULANCE_COUNT = 8
INTERSECTION_COUNT = 10

# ------------------------------------------------------------
# Fixed UE allocation
#
# UE 0..7     -> ambulances
# UE 8..17    -> intersections J1..J10
# UE 18..599  -> cars
# ------------------------------------------------------------

AMB_BASE = 0
INTERSECTION_BASE = 8
CAR_BASE = 18


# ============================================================
# UE MAPPINGS
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


# Dynamic mapping for SUMO cars
vehicle_to_ue = {}
ue_to_vehicle = {}

free_car_ues = deque(
    range(CAR_BASE, MAX_UE)
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
    (NS3_RX_IP, NS3_RX_PORT)
)

rx_sock.setblocking(False)


# ============================================================
# SUMO / NS-3 BRIDGE
# ============================================================

def send_state(step, ue_slot, active, x, y):
    """
    Send SUMO state to ns-3.

    Current protocol:

        STATE <step> <slot> <active> <x> <y>
    """

    message = (
        f"STATE "
        f"{step} "
        f"{ue_slot} "
        f"{int(active)} "
        f"{x:.3f} "
        f"{y:.3f}"
    )

    state_sock.sendto(
        message.encode(),
        (NS3_STATE_IP, NS3_STATE_PORT)
    )


def send_stop():
    """
    Tell ns-3 to terminate its realtime simulation.
    """

    try:
        state_sock.sendto(
            b"STOP",
            (NS3_STATE_IP, NS3_STATE_PORT)
        )
    except OSError:
        pass


# ============================================================
# CAR UE MANAGEMENT
# ============================================================

def allocate_car(car_id):
    """
    Give a SUMO car a persistent UE slot while it is active.
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

    print(
        f"[MAP] {car_id:<12} -> UE{ue}"
    )

    return ue


def release_car(car_id):
    """
    Release a UE slot when a SUMO car disappears.
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

    free_car_ues.append(ue)


# ============================================================
# SEND INTERSECTION POSITIONS
# ============================================================

def send_intersection_states(step):
    """
    Intersections are static NR-V2X nodes.

    UE8..UE17 correspond to J1..J10.
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


# ============================================================
# SEND AMBULANCE POSITIONS
# ============================================================

def send_ambulance_states(step):
    """
    Synchronize all eight ambulance UEs with SUMO.

    If an ambulance has not departed yet, or has already
    disappeared from SUMO, its ns-3 position is moved to
    the inactive/far-away state.
    """

    active_vehicle_ids = set(
        traci.vehicle.getIDList()
    )

    for ambulance_id, ue in ambulance_to_ue.items():

        active = ambulance_id in active_vehicle_ids

        if active:

            x, y = traci.vehicle.getPosition(
                ambulance_id
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
# SEND CAR POSITIONS
# ============================================================

def send_car_states(step):
    """
    Synchronize all currently active SUMO cars.
    """

    current_ids = set(
        traci.vehicle.getIDList()
    )

    # --------------------------------------------------------
    # Release UE slots belonging to cars that disappeared.
    # --------------------------------------------------------

    old_car_ids = set(
        vehicle_to_ue.keys()
    )

    for car_id in old_car_ids - current_ids:
        release_car(car_id)

    # --------------------------------------------------------
    # Allocate / update current cars.
    # --------------------------------------------------------

    for car_id in current_ids:

        # Ambulances have fixed UE slots.
        if car_id in ambulance_to_ue:
            continue

        ue = allocate_car(car_id)

        try:
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
# PROCESS NS-3 RX EVENTS
# ============================================================

def process_rx_events():
    """
    Read real packet reception events from ns-3.

    Current ns-3 protocol:

        RX <step> <senderUE> <receiverUE> <ns3_time>
    """

    events = []

    while True:

        try:
            data, _ = rx_sock.recvfrom(4096)

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
            # READY is handled separately during startup.
            continue

        if not message.startswith("RX"):
            continue

        parts = message.split()

        if len(parts) != 5:
            print(
                f"[WARN] Unexpected RX message: {message}"
            )
            continue

        try:
            step = int(parts[1])
            sender_ue = int(parts[2])
            receiver_ue = int(parts[3])
            ns3_time = float(parts[4])

        except ValueError:
            print(
                f"[WARN] Invalid RX message: {message}"
            )
            continue

        # ----------------------------------------------------
        # Resolve sender identity.
        # ----------------------------------------------------

        if sender_ue in ue_to_ambulance:

            sender_name = ue_to_ambulance[
                sender_ue
            ]

        elif sender_ue in ue_to_vehicle:

            sender_name = ue_to_vehicle[
                sender_ue
            ]

        elif sender_ue in ue_to_intersection:

            sender_name = ue_to_intersection[
                sender_ue
            ]

        else:

            sender_name = f"UE{sender_ue}"

        # ----------------------------------------------------
        # Resolve receiver identity.
        # ----------------------------------------------------

        if receiver_ue in ue_to_vehicle:

            receiver_name = ue_to_vehicle[
                receiver_ue
            ]

        elif receiver_ue in ue_to_ambulance:

            receiver_name = ue_to_ambulance[
                receiver_ue
            ]

        elif receiver_ue in ue_to_intersection:

            receiver_name = ue_to_intersection[
                receiver_ue
            ]

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
# WAIT FOR NS-3 READY
# ============================================================

def wait_for_ns3_ready(timeout=15.0):
    """
    Wait until ns-3 confirms that the bridge is ready.

    This avoids guessing that two seconds is enough.
    """

    deadline = time.monotonic() + timeout

    print("[NS3] Waiting for READY...")

    while time.monotonic() < deadline:

        try:
            data, _ = rx_sock.recvfrom(4096)

        except BlockingIOError:

            time.sleep(0.05)
            continue

        except OSError as exc:

            raise RuntimeError(
                f"RX socket failure: {exc}"
            )

        message = data.decode(
            errors="ignore"
        ).strip()

        if message == "READY":

            print("[NS3] READY received")
            return

        # It should not normally be possible to get RX data
        # before SUMO starts, but don't silently lose it.
        if message.startswith("RX"):

            print(
                f"[NS3] Received early event: {message}"
            )

    raise RuntimeError(
        "ns-3 did not send READY within "
        f"{timeout:.1f} seconds"
    )


# ============================================================
# MAIN
# ============================================================

def main():

    print("=" * 70)
    print("SUMO <-> 5G NR-V2X CITY BRIDGE")
    print("=" * 70)

    print(
        f"Ambulances    : {AMBULANCE_COUNT}"
    )

    print(
        f"Intersections : {INTERSECTION_COUNT}"
    )

    print(
        f"Max NR UEs    : {MAX_UE}"
    )

    print(
        "UE mapping    : "
        "AMB 0-7 | J 8-17 | CAR 18-599"
    )

    print(
        f"SUMO config   : {SUMO_CFG}"
    )

    print(
        f"ns-3 duration : {NS3_SIM_TIME} s"
    )

    print()

    ns3_process = None
    sumo_started = False

    try:

        # ----------------------------------------------------
        # Start ns-3.
        # ----------------------------------------------------

        ns3_command = [
            "./ns3",
            "run",
            f"{NS3_PROGRAM} --simTime={NS3_SIM_TIME}",
        ]

        print(
            "[NS3] Starting:",
            " ".join(ns3_command)
        )

        ns3_process = subprocess.Popen(
            ns3_command,
            cwd=NS3_DIR
        )

        # ----------------------------------------------------
        # Wait for the bridge handshake.
        # ----------------------------------------------------

        wait_for_ns3_ready()

        # ----------------------------------------------------
        # Start SUMO only after ns-3 is ready.
        # ----------------------------------------------------

        print("[SUMO] Starting...")

        traci.start(
            [
                "sumo-gui",
                "-c",
                SUMO_CFG
            ]
        )

        sumo_started = True

        print("[SUMO] Connected")

        step = 0

        # ----------------------------------------------------
        # Main co-simulation loop.
        # ----------------------------------------------------

        while (
            traci.simulation.getMinExpectedNumber() > 0
        ):

            traci.simulationStep()

            # SUMO -> ns-3
            send_ambulance_states(step)
            send_intersection_states(step)
            send_car_states(step)

            # ns-3 -> Python
            events = process_rx_events()

            # ------------------------------------------------
            # Communication-only test.
            #
            # No lane-change or signal logic yet.
            # We are only verifying the real NR-V2X RX path.
            # ------------------------------------------------

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

            # Small pacing delay for the realtime co-simulation.
            time.sleep(0.01)

    except KeyboardInterrupt:

        print("\n[CTRL] Interrupted by user")

    except Exception as exc:

        print(
            f"\n[CTRL] ERROR: {exc}"
        )
        raise

    finally:

        print("\n[CTRL] Shutting down...")

        # Tell ns-3 to stop.
        send_stop()

        # Close SUMO.
        if sumo_started:

            try:
                traci.close()
            except Exception:
                pass

        # Close sockets.
        try:
            state_sock.close()
        except Exception:
            pass

        try:
            rx_sock.close()
        except Exception:
            pass

        # Stop ns-3 process.
        if ns3_process is not None:

            try:

                ns3_process.wait(
                    timeout=5
                )

            except subprocess.TimeoutExpired:

                ns3_process.terminate()

                try:
                    ns3_process.wait(
                        timeout=3
                    )
                except subprocess.TimeoutExpired:
                    ns3_process.kill()

        print("[CTRL] Shutdown complete")


if __name__ == "__main__":
    main()
