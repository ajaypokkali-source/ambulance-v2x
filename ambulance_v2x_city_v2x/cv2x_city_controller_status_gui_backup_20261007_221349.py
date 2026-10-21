#!/usr/bin/env python3

"""
GUI wrapper around the existing NR-V2X city controller.

The original cv2x_city_controller.py is NOT modified.

This wrapper adds one fixed information panel near every
intersection and displays:

    V2I: RX ambX
    I2I: TX -> Jx
    I2I: RX <- Jy
    AMB: ambX

There are NO packet lines or packet animations.
"""

import cv2x_city_controller as base


# ============================================================
# GUI CONFIGURATION
# ============================================================

STATUS_LAYER = 1000

BOX_WIDTH = 110.0
BOX_HEIGHT = 55.0

BOX_OFFSET_X = 70.0
BOX_OFFSET_Y = 60.0


COLOR_NORMAL = (
    45,
    45,
    45,
    230
)

COLOR_V2I = (
    150,
    45,
    45,
    235
)

COLOR_I2I = (
    45,
    90,
    160,
    235
)

COLOR_PRIORITY = (
    45,
    130,
    70,
    235
)


# ============================================================
# STATUS STORAGE
# ============================================================

status = {}

box_ids = {}
text_ids = {}


# ============================================================
# IMPORTANT:
# SAVE ORIGINAL FUNCTIONS BEFORE MONKEY-PATCHING THEM
#
# This prevents recursion.
# ============================================================

original_process_v2i_rx = (
    base.process_v2i_rx
)

original_request_i2i = (
    base.request_i2i
)

original_process_i2i_rx = (
    base.process_i2i_rx
)

original_process_ns3_events = (
    base.process_ns3_events
)

original_traci_start = (
    base.traci.start
)


# ============================================================
# INITIAL STATUS
# ============================================================

def initialize_status():

    status.clear()

    for junction_id in base.intersection_to_ue:

        status[
            junction_id
        ] = {
            "v2i": "WAITING",
            "i2i": "WAITING",
            "amb": "--",
            "color": COLOR_NORMAL
        }


# ============================================================
# BOX SHAPE
# ============================================================

def make_box_shape(
    x,
    y
):

    half_w = BOX_WIDTH / 2.0
    half_h = BOX_HEIGHT / 2.0

    return [
        (
            x - half_w,
            y - half_h
        ),
        (
            x + half_w,
            y - half_h
        ),
        (
            x + half_w,
            y + half_h
        ),
        (
            x - half_w,
            y + half_h
        )
    ]


# ============================================================
# CREATE ONE TEXT POI
# ============================================================

def create_text_poi(
    poi_id,
    x,
    y,
    text
):

    try:

        base.traci.poi.add(
            poi_id,
            x,
            y,
            (255, 255, 255, 255),
            text,
            STATUS_LAYER + 1,
            "",
            1.0,
            1.0,
            0.0
        )

        return True

    except base.traci.TraCIException as exc:

        print(
            f"[GUI] Could not create POI "
            f"{poi_id}: {exc}"
        )

        return False


# ============================================================
# UPDATE TEXT LINES
# ============================================================

def update_status_box(
    junction_id
):

    info = status.get(
        junction_id
    )

    if info is None:
        return

    ids = text_ids.get(
        junction_id
    )

    if ids is None:
        return

    lines = [
        junction_id,
        f"V2I: {info['v2i']}",
        f"I2I: {info['i2i']}",
        f"AMB: {info['amb']}"
    ]

    for index, poi_id in enumerate(
        ids
    ):

        try:

            base.traci.poi.setType(
                poi_id,
                lines[index]
            )

        except base.traci.TraCIException:

            pass

    try:

        base.traci.polygon.setColor(
            box_ids[junction_id],
            info["color"]
        )

    except base.traci.TraCIException:

        pass


# ============================================================
# CREATE ALL TEN STATIC BOXES
# ============================================================

def create_status_boxes():

    initialize_status()

    existing_polygons = set(
        base.traci.polygon.getIDList()
    )

    existing_pois = set(
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

        cx = (
            x + BOX_OFFSET_X
        )

        cy = (
            y + BOX_OFFSET_Y
        )

        # ----------------------------------------------------
        # Background box
        # ----------------------------------------------------

        polygon_id = (
            f"v2x_info_box_{junction_id}"
        )

        box_ids[
            junction_id
        ] = polygon_id

        if polygon_id not in existing_polygons:

            try:

                base.traci.polygon.add(
                    polygon_id,
                    make_box_shape(
                        cx,
                        cy
                    ),
                    COLOR_NORMAL,
                    True,
                    "v2x_status",
                    STATUS_LAYER,
                    1.0
                )

            except base.traci.TraCIException as exc:

                print(
                    f"[GUI] Could not create box "
                    f"{junction_id}: {exc}"
                )

        # ----------------------------------------------------
        # Four separate text POIs
        #
        # We deliberately use four POIs rather than newline
        # characters so SUMO-GUI does not collapse the entire
        # status into one line.
        # ----------------------------------------------------

        text_positions = [
            (
                cx - 47.0,
                cy + 17.0
            ),
            (
                cx - 47.0,
                cy + 4.0
            ),
            (
                cx - 47.0,
                cy - 9.0
            ),
            (
                cx - 47.0,
                cy - 22.0
            )
        ]

        ids = []

        for index, (
            px,
            py
        ) in enumerate(
            text_positions
        ):

            poi_id = (
                f"v2x_info_"
                f"{junction_id}_"
                f"{index}"
            )

            ids.append(
                poi_id
            )

            if poi_id not in existing_pois:

                create_text_poi(
                    poi_id,
                    px,
                    py,
                    ""
                )

        text_ids[
            junction_id
        ] = ids

        update_status_box(
            junction_id
        )

        created += 1

    print(
        f"[GUI] Status panels ready: "
        f"{created}"
    )


# ============================================================
# V2I WRAPPER
#
# IMPORTANT:
# Call original_process_v2i_rx(), NOT
# base.process_v2i_rx(), otherwise recursion occurs.
# ============================================================

def wrapped_process_v2i_rx(
    event
):

    sender = event[
        "sender"
    ]

    receiver = event[
        "receiver"
    ]

    # Let the real controller do the V2I decision.
    original_process_v2i_rx(
        event
    )

    # Only ambulance -> intersection is relevant here.
    if sender not in base.ambulance_to_ue:

        return

    if receiver not in base.intersection_to_ue:

        return

    # Make sure this really is the immediate next TLS.
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

        return

    # --------------------------------------------------------
    # Update visual state.
    # --------------------------------------------------------

    status[
        receiver
    ]["v2i"] = (
        f"RX {sender}"
    )

    status[
        receiver
    ]["amb"] = sender

    # Find the one-hop I2I destination.
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

    status[
        receiver
    ]["color"] = COLOR_V2I

    update_status_box(
        receiver
    )


# ============================================================
# TRACK I2I REQUESTS
# ============================================================

pending_i2i = {}


# ============================================================
# I2I REQUEST WRAPPER
# ============================================================

def wrapped_request_i2i(
    source_junction,
    target_junction,
    ambulance_id
):

    # Call ORIGINAL function.
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

    pending_i2i[
        sender_ue
    ] = {
        "source": source_junction,
        "target": target_junction,
        "ambulance": ambulance_id
    }


# ============================================================
# NS-3 EVENT WRAPPER
# ============================================================

def wrapped_process_ns3_events():

    return (
        original_process_ns3_events()
    )


# ============================================================
# I2I RX WRAPPER
# ============================================================

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

    before = dict(
        getattr(
            target_signal,
            "armed",
            {}
        )
    )

    # Call ORIGINAL I2I handler.
    original_process_i2i_rx(
        event
    )

    after = dict(
        getattr(
            target_signal,
            "armed",
            {}
        )
    )

    # Identify newly armed ambulance.
    new_ambulances = [
        amb
        for amb in after
        if (
            amb not in before
            or after[amb] != before[amb]
        )
    ]

    if not new_ambulances:

        return

    ambulance = (
        new_ambulances[0]
    )

    status[
        target
    ]["v2i"] = "WAITING"

    status[
        target
    ]["i2i"] = (
        f"RX <- {source}"
    )

    status[
        target
    ]["amb"] = ambulance

    status[
        target
    ]["color"] = COLOR_I2I

    update_status_box(
        target
    )

    print(
        f"[GUI] {target}: "
        f"I2I RX <- {source} "
        f"for {ambulance}"
    )


# ============================================================
# WRAP SUMO START
# ============================================================

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

    create_status_boxes()

    return result


# ============================================================
# INSTALL WRAPPERS
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

base.traci.start = (
    wrapped_traci_start
)


# ============================================================
# RUN ORIGINAL CONTROLLER
# ============================================================

if __name__ == "__main__":

    print("=" * 70)
    print(
        "NR-V2X CITY + INTERSECTION STATUS GUI"
    )
    print("=" * 70)

    print(
        "[GUI] Static J1..J10 status panels enabled"
    )

    print(
        "[GUI] Packet animations disabled"
    )

    base.main()
