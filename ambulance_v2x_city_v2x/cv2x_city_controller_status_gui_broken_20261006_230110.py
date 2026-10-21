#!/usr/bin/env python3

"""
GUI wrapper for the existing CITY NR-V2X controller.

IMPORTANT:
    The original cv2x_city_controller.py is NOT modified.

This wrapper adds one static text/status panel near every
intersection J1..J10.

The panel is implemented using an invisible SUMO POI whose
type contains the displayed text.

Example:

    J1
    V2I: RX amb1
    I2I: TX -> J2
"""

import cv2x_city_controller as base


# ============================================================
# GUI STATUS CONFIGURATION
# ============================================================

STATUS_LAYER = 1000

# Position of the status panel relative to each junction.
#
# The city intersections are spaced widely enough that this
# offset keeps the panel away from the actual junction.
BOX_OFFSET_X = 65.0
BOX_OFFSET_Y = 55.0


# ============================================================
# STATUS STORAGE
# ============================================================

status = {}

poi_ids = {}


# ============================================================
# INITIAL STATUS
# ============================================================

def make_initial_status():

    for junction_id in base.intersection_to_ue:

        status[
            junction_id
        ] = {
            "v2i": "WAITING",
            "i2i": "WAITING",
            "amb": "--",
        }


# ============================================================
# FORMAT STATUS TEXT
# ============================================================

def format_status(
    junction_id
):

    info = status[
        junction_id
    ]

    return (
        f"{junction_id}\n"
        f"V2I: {info['v2i']}\n"
        f"I2I: {info['i2i']}\n"
        f"AMB: {info['amb']}"
    )


# ============================================================
# UPDATE ONE STATUS BOX
# ============================================================

def update_status_box(
    junction_id
):

    poi_id = poi_ids.get(
        junction_id
    )

    if poi_id is None:
        return

    text = format_status(
        junction_id
    )

    try:

        base.traci.poi.setType(
            poi_id,
            text
        )

    except base.traci.TraCIException:

        pass


# ============================================================
# CREATE ALL TEN STATUS POIs
# ============================================================

def create_status_boxes():

    make_initial_status()

    existing = set(
        base.traci.poi.getIDList()
    )

    created = 0

    for junction_id in base.intersection_to_ue:

        try:

            x, y = (
                base.traci.junction.getPosition(
                    junction_id
                )
            )

        except base.traci.TraCIException:

            continue

        px = (
            x + BOX_OFFSET_X
        )

        py = (
            y + BOX_OFFSET_Y
        )

        poi_id = (
            f"v2x_status_{junction_id}"
        )

        poi_ids[
            junction_id
        ] = poi_id

        # Invisible POI. The POI TYPE itself is used as the
        # displayed text in SUMO-GUI.
        #
        # Current TraCI POI API:
        # add(id, x, y, color, type, layer, image, width,
        #     height, angle, icon)
        if poi_id not in existing:

            try:

                base.traci.poi.add(
                    poi_id,
                    px,
                    py,
                    (0, 0, 0, 0),
                    format_status(
                        junction_id
                    ),
                    STATUS_LAYER,
                    "",
                    1.0,
                    1.0,
                    0.0
                )

                created += 1

            except base.traci.TraCIException as exc:

                print(
                    f"[GUI] Could not create "
                    f"{junction_id}: {exc}"
                )

        else:

            update_status_box(
                junction_id
            )

    print(
        f"[GUI] Created "
        f"{created} intersection status panels"
    )


# ============================================================
# V2I STATUS
# ============================================================

def wrapped_process_v2i_rx(
    event
):

    # Preserve the original V2I decision logic.
    base.process_v2i_rx(
        event
    )

    sender = event[
        "sender"
    ]

    receiver = event[
        "receiver"
    ]

    # We only display an ambulance V2I event for an
    # intersection that is actually the ambulance's FIRST
    # upcoming traffic signal.
    if sender not in base.ambulance_to_ue:
        return

    if receiver not in base.intersection_to_ue:
        return

    try:

        next_tls = (
            base.get_next_tls(
                sender
            )
        )

    except Exception:

        return

    if next_tls is None:
        return

    if (
        next_tls["junction"]
        != receiver
    ):

        # Farther signal physically received the broadcast,
        # but it is not the immediate V2I target.
        return

    status[
        receiver
    ]["v2i"] = (
        f"RX {sender}"
    )

    status[
        receiver
    ]["amb"] = sender

    # Show where the I2I message is intended to go.
    try:

        next_junction = (
            base.get_signal_after(
                sender,
                receiver
            )
        )

    except Exception:

        next_junction = None

    if next_junction is None:

        status[
            receiver
        ]["i2i"] = "NONE"

    else:

        status[
            receiver
        ]["i2i"] = (
            f"TX -> {next_junction}"
        )

    update_status_box(
        receiver
    )


# ============================================================
# TRACK REQUESTED I2I TRANSMISSIONS
# ============================================================
#
# Python asks ns-3:
#
#     I2I senderUE targetUE
#
# We remember the target so that when the actual ns-3 TX event
# arrives we can update the source status panel.
# ============================================================

pending_i2i = {}


original_request_i2i = (
    base.request_i2i
)


def wrapped_request_i2i(
    source_junction,
    target_junction,
    ambulance_id
):

    original_request_i2i(
        source_junction,
        target_junction,
        ambulance_id
    )

    sender_ue = (
        base.intersection_to_ue[
            source_junction
        ]
    )

    target_ue = (
        base.intersection_to_ue[
            target_junction
        ]
    )

    pending_i2i[
        sender_ue
    ] = {
        "source": source_junction,
        "target": target_junction,
        "ambulance": ambulance_id,
    }


# ============================================================
# REAL ns-3 I2I TX VISUAL STATUS
# ============================================================

original_process_ns3_events = (
    base.process_ns3_events
)


def wrapped_process_ns3_events():

    events = (
        original_process_ns3_events()
    )

    for event in events:

        if event.get("kind") != "TX":
            continue

        sender_ue = (
            event[
                "sender_ue"
            ]
        )

        i2i = pending_i2i.pop(
            sender_ue,
            None
        )

        if i2i is None:
            continue

        source = i2i[
            "source"
        ]

        target = i2i[
            "target"
        ]

        ambulance = i2i[
            "ambulance"
        ]

        status[
            source
        ]["i2i"] = (
            f"TX -> {target}"
        )

        status[
            source
        ]["amb"] = ambulance

        update_status_box(
            source
        )

        print(
            f"[GUI] {source} panel: "
            f"I2I TX -> {target} "
            f"for {ambulance}"
        )

    return events


# ============================================================
# REAL ns-3 I2I RX STATUS
# ============================================================

original_process_i2i_rx = (
    base.process_i2i_rx
)


def wrapped_process_i2i_rx(
    event
):

    source = event[
        "sender"
    ]

    target = event[
        "receiver"
    ]

    if source not in base.intersection_to_ue:
        return

    if target not in base.intersection_to_ue:
        return

    target_signal = (
        base.signals.get(
            target
        )
    )

    if target_signal is None:
        return

    # Remember the target's I2I state before the original
    # controller processes this packet.
    before = dict(
        getattr(
            target_signal,
            "armed",
            {}
        )
    )

    # Let the actual controller process the RX.
    original_process_i2i_rx(
        event
    )

    # Check whether the original controller accepted this
    # I2I reception.
    after = dict(
        getattr(
            target_signal,
            "armed",
            {}
        )
    )

    new_arms = [
        amb
        for amb in after
        if (
            amb not in before
            or after[amb] != before[amb]
        )
    ]

    if not new_arms:
        return

    ambulance = new_arms[
        0
    ]

    status[
        target
    ]["i2i"] = (
        f"RX <- {source}"
    )

    status[
        target
    ]["amb"] = ambulance

    update_status_box(
        target
    )

    print(
        f"[GUI] {target} panel: "
        f"I2I RX <- {source} "
        f"for {ambulance}"
    )


# ============================================================
# PATCH THE ORIGINAL CONTROLLER
# ============================================================

base.process_v2i_rx = (
    wrapped_process_v2i_rx
)

base.request_i2i = (
    wrapped_request_i2i
)

base.process_ns3_events = (
    wrapped_process_ns3_events
)

base.process_i2i_rx = (
    wrapped_process_i2i_rx
)


# ============================================================
# INTERCEPT SUMO START
# ============================================================

original_traci_start = (
    base.traci.start
)


def wrapped_traci_start(
    command,
    *args,
    **kwargs
):

    result = (
        original_traci_start(
            command,
            *args,
            **kwargs
        )
    )

    # SUMO is now connected, so create the ten fixed panels.
    create_status_boxes()

    return result


base.traci.start = (
    wrapped_traci_start
)


# ============================================================
# RUN ORIGINAL CONTROLLER
# ============================================================

if __name__ == "__main__":

    print("=" * 70)
    print("NR-V2X CITY CONTROLLER + INTERSECTION STATUS GUI")
    print("=" * 70)

    print(
        "[GUI] Static J1..J10 status panels enabled"
    )

    print(
        "[GUI] No packet lines or packet animations"
    )

    base.main()
