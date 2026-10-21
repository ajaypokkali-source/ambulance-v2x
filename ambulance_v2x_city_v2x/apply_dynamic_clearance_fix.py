#!/usr/bin/env python3

from pathlib import Path
from datetime import datetime
import shutil


BASE = Path("cv2x_city_controller.py")
GUI = Path("cv2x_city_controller_status_gui.py")

timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

base_backup = BASE.with_name(
    f"cv2x_city_controller_backup_dynamic_clearance_{timestamp}.py"
)

gui_backup = GUI.with_name(
    f"cv2x_city_controller_status_gui_backup_dynamic_clearance_{timestamp}.py"
)


# ============================================================
# SAFETY
# ============================================================

if not BASE.exists():
    raise SystemExit("ERROR: cv2x_city_controller.py not found")

if not GUI.exists():
    raise SystemExit(
        "ERROR: cv2x_city_controller_status_gui.py not found"
    )


base_text = BASE.read_text()
gui_text = GUI.read_text()


# ============================================================
# BACKUPS
# ============================================================

shutil.copy2(BASE, base_backup)
shutil.copy2(GUI, gui_backup)

print(f"[BACKUP] {base_backup}")
print(f"[BACKUP] {gui_backup}")


# ============================================================
# BASE CONTROLLER
# ============================================================

# ------------------------------------------------------------
# 1. Store route index of the incoming approach edge
# ------------------------------------------------------------

old_init = '''        self.approach_edge = None

        self.since = 0.0
'''

new_init = '''        self.approach_edge = None

        # SUMO route index of the ambulance's incoming
        # approach edge at the time priority starts.
        self.approach_route_index = None

        self.since = 0.0
'''

if base_text.count(old_init) != 1:
    raise SystemExit(
        "ERROR: SignalController __init__ block not found uniquely"
    )

base_text = base_text.replace(
    old_init,
    new_init,
    1
)


# ------------------------------------------------------------
# 2. Capture route index when priority starts
# ------------------------------------------------------------

old_start = '''        self.approach_edge = (
            self.links[
                info["link"]
            ][0][0]
            .rsplit(
                "_",
                1
            )[0]
        )

        self.since = now
'''

new_start = '''        self.approach_edge = (
            self.links[
                info["link"]
            ][0][0]
            .rsplit(
                "_",
                1
            )[0]
        )

        # Remember where this junction sits on the ambulance's
        # actual SUMO route. No junction sequence is hardcoded.
        try:

            self.approach_route_index = (
                traci.vehicle.getRouteIndex(
                    ambulance_id
                )
            )

        except traci.TraCIException:

            self.approach_route_index = None

        self.since = now
'''

if base_text.count(old_start) != 1:
    raise SystemExit(
        "ERROR: SignalController start() block not found uniquely"
    )

base_text = base_text.replace(
    old_start,
    new_start,
    1
)


# ------------------------------------------------------------
# 3. Replace ambulance_cleared()
# ------------------------------------------------------------

old_cleared = '''    def ambulance_cleared(
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
'''

new_cleared = '''    def ambulance_cleared(
        self
    ):

        if self.owner is None:

            return False

        # If we do not know the route position at the time
        # the request started, do not guess that the junction
        # has been cleared.
        if self.approach_route_index is None:

            return False

        try:

            road_id = (
                traci.vehicle.getRoadID(
                    self.owner
                )
            )

            current_route_index = (
                traci.vehicle.getRouteIndex(
                    self.owner
                )
            )

        except traci.TraCIException:

            return False

        # Still inside SUMO's internal junction.
        if road_id.startswith(
            f":{self.id}_"
        ):

            return False

        # Still on the same incoming route edge.
        if current_route_index <= self.approach_route_index:

            return False

        # The ambulance has advanced to a later edge in its
        # actual SUMO route. Therefore it has crossed this
        # junction.
        return True
'''

if base_text.count(old_cleared) != 1:
    raise SystemExit(
        "ERROR: ambulance_cleared() block not found uniquely"
    )

base_text = base_text.replace(
    old_cleared,
    new_cleared,
    1
)


# ------------------------------------------------------------
# 4. Reset route index on release
# ------------------------------------------------------------

old_release = '''        self.approach_edge = None

        self.since = 0.0
'''

new_release = '''        self.approach_edge = None

        self.approach_route_index = None

        self.since = 0.0
'''

# This pattern now occurs exactly once after the first replacement.
if base_text.count(old_release) != 1:
    raise SystemExit(
        "ERROR: release() reset block not found uniquely"
    )

base_text = base_text.replace(
    old_release,
    new_release,
    1
)


BASE.write_text(base_text)


# ============================================================
# GUI WRAPPER
# ============================================================

# ------------------------------------------------------------
# 5. Save original SignalController methods
# ------------------------------------------------------------

old_originals = '''original_process_ns3_events = (
    base.process_ns3_events
)

original_traci_start = (
    base.traci.start
)
'''

new_originals = '''original_process_ns3_events = (
    base.process_ns3_events
)

original_signal_start = (
    base.SignalController.start
)

original_signal_release = (
    base.SignalController.release
)

original_traci_start = (
    base.traci.start
)
'''

if gui_text.count(old_originals) != 1:
    raise SystemExit(
        "ERROR: GUI original-function block not found uniquely"
    )

gui_text = gui_text.replace(
    old_originals,
    new_originals,
    1
)


# ------------------------------------------------------------
# 6. Replace the GUI's independent passed-junction logic
# ------------------------------------------------------------

start_marker = '''# ============================================================
# RESET PANELS AFTER AMBULANCE PASSES
# ============================================================

def reset_passed_intersections():
'''

end_marker = '''# ============================================================
# V2I RX WRAPPER
# ============================================================
'''

start_pos = gui_text.find(start_marker)
end_pos = gui_text.find(end_marker)

if start_pos == -1 or end_pos == -1 or end_pos <= start_pos:
    raise SystemExit(
        "ERROR: Could not locate GUI reset_passed_intersections()"
    )

new_gui_section = '''# ============================================================
# SIGNAL CONTROLLER GUI SYNCHRONIZATION
# ============================================================

def wrapped_signal_start(
    signal,
    now,
    ambulance_id
):

    result = (
        original_signal_start(
            signal,
            now,
            ambulance_id
        )
    )

    if not result:
        return result

    # The actual controller has accepted this ambulance
    # as the emergency owner.
    status[
        signal.id
    ]["amb"] = ambulance_id

    status[
        signal.id
    ]["v2i"] = (
        f"RX {ambulance_id}"
    )

    status[
        signal.id
    ]["color"] = COLOR_V2I

    update_status_panel(
        signal.id
    )

    return result


def wrapped_signal_release(
    signal,
    now
):

    old_owner = signal.owner

    original_signal_release(
        signal,
        now
    )

    # The real controller has now decided that the ambulance
    # physically cleared the junction.
    #
    # Therefore the GUI resets here rather than independently
    # trying to infer passage using getNextTLS().
    clear_status_panel(
        signal.id
    )

    if old_owner is not None:

        print(
            f"[GUI] {signal.id}: "
            f"{old_owner} cleared -> "
            f"panel reset"
        )


# ============================================================
# V2I RX WRAPPER
# ============================================================
'''

gui_text = (
    gui_text[:start_pos]
    + new_gui_section
    + gui_text[end_pos + len(end_marker):]
)


# ------------------------------------------------------------
# 7. Remove old reset call from simulation-step wrapper
# ------------------------------------------------------------

old_step = '''    # Update labels after vehicles have moved.
    update_ambulance_labels()

    # Reset panels whose ambulance has passed.
    reset_passed_intersections()

    return result
'''

new_step = '''    # Update labels after vehicles have moved.
    update_ambulance_labels()

    # Panel reset is handled by the real
    # SignalController.release() event.

    return result
'''

if gui_text.count(old_step) != 1:
    raise SystemExit(
        "ERROR: GUI simulation-step block not found uniquely"
    )

gui_text = gui_text.replace(
    old_step,
    new_step,
    1
)


# ------------------------------------------------------------
# 8. Install SignalController wrappers
# ------------------------------------------------------------

old_install = '''base.process_v2i_rx = (
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
'''

new_install = '''base.process_v2i_rx = (
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

base.SignalController.start = (
    wrapped_signal_start
)

base.SignalController.release = (
    wrapped_signal_release
)

base.traci.start = (
    wrapped_traci_start
)
'''

if gui_text.count(old_install) != 1:
    raise SystemExit(
        "ERROR: GUI wrapper installation block not found uniquely"
    )

gui_text = gui_text.replace(
    old_install,
    new_install,
    1
)


GUI.write_text(gui_text)


print()
print("[OK] Dynamic junction-clearance fix applied.")
print()
print("Modified:")
print(f"  {BASE}")
print(f"  {GUI}")
print()
print("Backups:")
print(f"  {base_backup}")
print(f"  {gui_backup}")
