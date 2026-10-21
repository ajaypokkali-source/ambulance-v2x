#!/usr/bin/env python3

"""
NR-V2X city GUI wrapper.

The original:
    cv2x_city_controller.py

is NOT modified.

This wrapper adds:

    1. Static V2I/I2I status panels at J1..J10
    2. Automatic panel reset after the ambulance passes
    3. Moving labels above active ambulances

There are no packet lines or packet animations.
"""

import cv2x_city_controller as base


# ============================================================
# GUI CONFIGURATION
# ============================================================

STATUS_LAYER = 1000
AMBULANCE_LABEL_LAYER = 1100

# Information-box geometry
BOX_WIDTH = 100.0
BOX_HEIGHT = 52.0

BOX_OFFSET_X = 60.0
BOX_OFFSET_Y = 52.0

# Ambulance label offset
AMB_LABEL_OFFSET_X = 0.0
AMB_LABEL_OFFSET_Y = 7.0


# ============================================================
# BOX COLORS
# ============================================================

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
    165,
    235
)


# ============================================================
# GUI STATE
# ============================================================

status = {}

box_ids = {}
box_text_ids = {}

ambulance_label_ids = {}


# ============================================================
# SAVE ORIGINAL FUNCTIONS BEFORE PATCHING
#
# IMPORTANT:
# This prevents recursive wrapper calls.
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

original_simulation_step = (
    base.traci.simulationStep
)


# ============================================================
# STATUS INITIALIZATION
# ============================================================

def reset_all_status():

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
# MAKE BOX GEOMETRY
# ============================================================

def make_box_shape(
    cx,
    cy
):

    half_w = BOX_WIDTH / 2.0
    half_h = BOX_HEIGHT / 2.0

    return [
        (
            cx - half_w,
            cy - half_h
        ),
        (
            cx + half_w,
            cy - half_h
        ),
        (
            cx + half_w,
            cy + half_h
        ),
        (
            cx - half_w,
            cy + half_h
        )
    ]


# ============================================================
# UPDATE ONE STATUS PANEL
# ============================================================

def update_status_panel(
    junction_id
):

    info = status.get(
        junction_id
    )

    if info is None:
        return

    text_ids = box_text_ids.get(
        junction_id
    )

    if text_ids is None:
        return

    lines = [
        junction_id,
        f"V2I: {info['v2i']}",
        f"I2I: {info['i2i']}",
        f"AMB: {info['amb']}"
    ]

    for index, poi_id in enumerate(
        text_ids
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
# CREATE STATUS PANELS
# ============================================================

def create_status_panels():

    reset_all_status()

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
        # Background rectangle
        # ----------------------------------------------------

        polygon_id = (
            f"v2x_status_box_{junction_id}"
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
                    f"[GUI] Failed to create "
                    f"box for {junction_id}: "
                    f"{exc}"
                )

        # ----------------------------------------------------
        # Separate text POIs.
        #
        # Four POIs are used because SUMO-GUI renders POI type
        # text more reliably as separate labels than as a
        # multiline string.
        # ----------------------------------------------------

        text_positions = [
            (
                cx - 46.0,
                cy + 16.0
            ),
            (
                cx - 46.0,
                cy + 3.0
            ),
            (
                cx - 46.0,
                cy - 10.0
            ),
            (
                cx - 46.0,
                cy - 23.0
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
                f"v2x_status_text_"
                f"{junction_id}_"
                f"{index}"
            )

            ids.append(
                poi_id
            )

            if poi_id not in existing_pois:

                try:

                    # Transparent POI marker. Only its type/text
                    # should be visible.
                    base.traci.poi.add(
                        poi_id,
                        px,
                        py,
                        (0, 0, 0, 0),
                        "",
                        STATUS_LAYER + 1,
                        "",
                        1.0,
                        1.0,
                        0.0
                    )

                except base.traci.TraCIException as exc:

                    print(
                        f"[GUI] Failed to create "
                        f"text POI "
                        f"{poi_id}: {exc}"
                    )

        box_text_ids[
            junction_id
        ] = ids

        update_status_panel(
            junction_id
        )

        created += 1

    print(
        f"[GUI] Created "
        f"{created} intersection panels"
    )


# ============================================================
# AMBULANCE LABELS
# ============================================================

def create_ambulance_labels():

    existing = set(
        base.traci.poi.getIDList()
    )

    created = 0

    for ambulance_id in base.ambulance_to_ue:

        poi_id = (
            f"v2x_ambulance_label_{ambulance_id}"
        )

        ambulance_label_ids[
            ambulance_id
        ] = poi_id

        if poi_id in existing:

            continue

        try:

            base.traci.poi.add(
                poi_id,
                0.0,
                0.0,
                (0, 0, 0, 0),
                ambulance_id,
                AMBULANCE_LABEL_LAYER,
                "",
                1.0,
                1.0,
                0.0
            )

            created += 1

        except base.traci.TraCIException as exc:

            print(
                f"[GUI] Failed to create "
                f"label for {ambulance_id}: "
                f"{exc}"
            )

    print(
        f"[GUI] Created "
        f"{created} ambulance labels"
    )


def update_ambulance_labels():

    alive = set(
        base.traci.vehicle.getIDList()
    )

    for ambulance_id, poi_id in ambulance_label_ids.items():

        if ambulance_id not in alive:

            # Empty text makes the label invisible until that
            # ambulance enters SUMO.
            try:

                base.traci.poi.setType(
                    poi_id,
                    ""
                )

            except base.traci.TraCIException:

                pass

            continue

        try:

            x, y = (
                base.traci.vehicle.getPosition(
                    ambulance_id
                )
            )

            base.traci.poi.setPosition(
                poi_id,
                x + AMB_LABEL_OFFSET_X,
                y + AMB_LABEL_OFFSET_Y
            )

            base.traci.poi.setType(
                poi_id,
                ambulance_id
            )

        except base.traci.TraCIException:

            pass


# ============================================================
# CLEAR A SINGLE INTERSECTION PANEL
# ============================================================

def clear_status_panel(
    junction_id
):

    if junction_id not in status:
        return

    status[
        junction_id
    ] = {
        "v2i": "WAITING",
        "i2i": "WAITING",
        "amb": "--",
        "color": COLOR_NORMAL
    }

    update_status_panel(
        junction_id
    )


# ============================================================
# RESET PANELS AFTER AMBULANCE PASSES
# ============================================================

def reset_passed_intersections():

    alive = set(
        base.traci.vehicle.getIDList()
    )

    for junction_id, info in list(
        status.items()
    ):

        ambulance_id = (
            info["amb"]
        )

        if ambulance_id == "--":

            continue

        # --------------------------------------------
        # Ambulance disappeared from SUMO.
        # --------------------------------------------

        if ambulance_id not in alive:

            clear_status_panel(
                junction_id
            )

            continue

        # --------------------------------------------
        # Check whether this junction is still ahead
        # of the ambulance.
        # --------------------------------------------

        try:

            upcoming = (
                base.traci.vehicle.getNextTLS(
                    ambulance_id
                )
            )

        except base.traci.TraCIException:

            continue

        upcoming_ids = {
            item[0]
            for item in upcoming
        }

        # --------------------------------------------
        # If the ambulance is physically inside the
        # junction, keep the status.
        # --------------------------------------------

        try:

            road_id = (
                base.traci.vehicle.getRoadID(
                    ambulance_id
                )
            )

        except base.traci.TraCIException:

            continue

        inside_current_junction = (
            road_id.startswith(
                f":{junction_id}_"
            )
        )

        # --------------------------------------------
        # Once it is neither approaching nor inside
        # this junction, reset the panel.
        # --------------------------------------------

        if (
            junction_id not in upcoming_ids
            and not inside_current_junction
        ):

            print(
                f"[GUI] {junction_id}: "
                f"{ambulance_id} passed -> "
                f"reset"
            )

            clear_status_panel(
                junction_id
            )


# ============================================================
# V2I RX WRAPPER
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

    # --------------------------------------------
    # Let the actual controller process the RX.
    # --------------------------------------------

    original_process_v2i_rx(
        event
    )

    # --------------------------------------------
    # Only ambulance -> intersection.
    # --------------------------------------------

    if sender not in base.ambulance_to_ue:

        return

    if receiver not in base.intersection_to_ue:

        return

    # --------------------------------------------
    # Verify this is really the ambulance's first
    # upcoming TLS.
    # --------------------------------------------

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

    # --------------------------------------------
    # Valid V2I reception.
    # --------------------------------------------

    status[
        receiver
    ]["v2i"] = (
        f"RX {sender}"
    )

    status[
        receiver
    ]["amb"] = sender

    # --------------------------------------------
    # Determine the I2I one-hop destination.
    # --------------------------------------------

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

    update_status_panel(
        receiver
    )


# ============================================================
# TRACK I2I REQUESTS
# ============================================================

pending_i2i = []


# ============================================================
# I2I REQUEST WRAPPER
# ============================================================

def wrapped_request_i2i(
    source_junction,
    target_junction,
    ambulance_id
):

    # Call the real networking request.
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

    pending_i2i.append({
        "sender_ue": sender_ue,
        "target_ue": target_ue,
        "source": source_junction,
        "target": target_junction,
        "ambulance": ambulance_id
    })


# ============================================================
# PROCESS NS-3 EVENTS
# ============================================================

def wrapped_process_ns3_events():

    events = (
        original_process_ns3_events()
    )

    # --------------------------------------------------------
    # Look for actual intersection TX events.
    # --------------------------------------------------------

    for event in events:

        if event.get(
            "kind"
        ) != "TX":

            continue

        sender_ue = event[
            "sender_ue"
        ]

        matches = [
            item
            for item in pending_i2i
            if item["sender_ue"] == sender_ue
        ]

        if not matches:

            continue

        # The oldest pending transmission from this
        # intersection is the one we associate with this TX.
        i2i = matches[0]

        pending_i2i.remove(
            i2i
        )

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

        status[
            source
        ]["color"] = COLOR_I2I

        update_status_panel(
            source
        )

        print(
            f"[GUI] {source}: "
            f"actual I2I TX -> {target} "
            f"for {ambulance}"
        )

    return events


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

    # Call real I2I processing.
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

    new_arms = [
        ambulance
        for ambulance in after
        if (
            ambulance not in before
            or after[ambulance]
            != before[ambulance]
        )
    ]

    if not new_arms:

        return

    ambulance = new_arms[
        0
    ]

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

    update_status_panel(
        target
    )

    print(
        f"[GUI] {target}: "
        f"actual I2I RX <- {source} "
        f"for {ambulance}"
    )


# ============================================================
# SUMO START WRAPPER
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

    create_status_panels()
    create_ambulance_labels()

    return result


# ============================================================
# SUMO SIMULATION STEP WRAPPER
# ============================================================
#
# This is called automatically because the base controller
# calls traci.simulationStep() every loop.
# ============================================================

def wrapped_simulation_step(
    *args,
    **kwargs
):

    result = (
        original_simulation_step(
            *args,
            **kwargs
        )
    )

    # Update labels after vehicles have moved.
    update_ambulance_labels()

    # Reset panels whose ambulance has passed.
    reset_passed_intersections()

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

base.traci.simulationStep = (
    wrapped_simulation_step
)


# ============================================================
# MAIN
# ============================================================

if __name__ == "__main__":

    print("=" * 70)
    print(
        "NR-V2X CITY + STATUS PANELS + AMBULANCE LABELS"
    )
    print("=" * 70)

    print(
        "[GUI] J1..J10 status panels enabled"
    )

    print(
        "[GUI] Ambulance labels enabled"
    )

    print(
        "[GUI] Packet animations disabled"
    )

    print(
        "[GUI] Panels automatically reset "
        "after ambulance passes"
    )

    base.main()
