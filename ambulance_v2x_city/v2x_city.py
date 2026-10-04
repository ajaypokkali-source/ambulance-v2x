#!/usr/bin/env python3
"""
v2x_city.py

V2I + I2I signal preemption for several ambulances on the 10-signal grid
(network.net.xml, built by build_network.py).

HOW IT WORKS
------------
Nothing here is hardcoded to particular junctions. Each step, for every
ambulance, SUMO's getNextTLS() gives the signals still ahead on its route,
the distance to each stop line, and the exact signal link it will use.

  V2I  An ambulance within V2I_RADIUS of a signal sends that signal a beacon.
       The signal gives the ambulance's whole approach edge a green and
       everything else red (after a yellow clearance), then returns to its
       normal program once the ambulance has cleared the junction.

  I2I  A signal that is serving an ambulance beacons the NEXT signal on the
       ambulance's route. That signal is then "armed": it listens out to
       I2I_RADIUS instead of V2I_RADIUS, so it reacts earlier.

  Several ambulances at one signal: the nearest to the stop line is served
  first; the others wait their turn.

  V2V  A car directly ahead of an ambulance in the same lane (within
       V2V_RANGE) receives the ambulance's message and moves into a free
       adjacent lane, if that lane is safe and still reaches the car's next
       road. It is released again once the ambulance has passed.

All messages go through a Channel with a configurable latency and loss
probability (ideal by default). That is the single place to replace with real
ns-3 NR-V2X delay / delivery results later.

USAGE
-----
    python v2x_city.py                  # GUI, V2X on
    python v2x_city.py --baseline       # GUI, no preemption (for comparison)
    python v2x_city.py --nogui          # headless
    python v2x_city.py --no-v2v         # signals only (V2I + I2I)
    python v2x_city.py --track auto     # camera follows the active ambulance
    python v2x_city.py --latency 0.05 --loss 0.1
"""

import argparse
import csv
import os
import random
import shutil
import sys

import traci

import overlay

HERE = os.path.dirname(os.path.abspath(__file__))

# ------------------------------------------------------------
# Parameters
# ------------------------------------------------------------

V2I_RADIUS = 100.0        # m, ambulance -> signal, normal detection range
I2I_RADIUS = 220.0        # m, detection range of a signal armed by I2I
YELLOW_TIME = 3.0         # s, clearance before the ambulance gets green
BEACON_TIMEOUT = 2.0      # s, a request expires if not refreshed
ARM_TTL = 120.0           # s, an I2I arm stays valid (queues can delay an ambulance)

V2V_RANGE = 70.0          # m, cars this far ahead of an ambulance yield
V2V_GAP_FRONT = 15.0      # m, free space needed ahead in the target lane
V2V_GAP_REAR = 12.0       # m, free space needed behind in the target lane


def load_ambulances():
    """Ambulance ids, in departure order, from ambulances.rou.xml."""
    import xml.etree.ElementTree as ET
    root = ET.parse(os.path.join(HERE, "ambulances.rou.xml")).getroot()
    vehicles = [(float(v.get("depart")), v.get("id"), v.get("color", "1,0,0"))
                for v in root.iter("vehicle")]
    vehicles.sort()
    return ([v[1] for v in vehicles],
            {v[1]: tuple(int(float(c) * 255) for c in v[2].split(","))
             for v in vehicles})


# ------------------------------------------------------------
# Communication channel
# ------------------------------------------------------------

class Channel:
    """Delivers messages after `latency` seconds, dropping a fraction `loss`."""

    def __init__(self, latency, loss, seed):
        self.latency = latency
        self.loss = loss
        self.rng = random.Random(seed)
        self.queue = []
        self.sent = 0
        self.dropped = 0

    def send(self, now, kind, src, dst, payload):
        self.sent += 1
        if self.rng.random() < self.loss:
            self.dropped += 1
            return
        self.queue.append((now + self.latency, kind, src, dst, payload))

    def deliver(self, now):
        due = [m for m in self.queue if m[0] <= now]
        self.queue = [m for m in self.queue if m[0] > now]
        return due


# ------------------------------------------------------------
# Traffic signal controller
# ------------------------------------------------------------

class Signal:
    NORMAL, CLEARING, GREEN = "NORMAL", "CLEARING", "GREEN"

    def __init__(self, tid):
        self.id = tid
        self.program = traci.trafficlight.getProgram(tid)
        self.links = traci.trafficlight.getControlledLinks(tid)
        self.heard = {}     # amb -> (time, link, dist)
        self.armed = {}     # amb -> time of last I2I message
        self.owner = None
        self.mode = Signal.NORMAL
        self.until = 0.0
        self.target = None
        self.via = {}       # amb -> "V2I" | "I2I"
        self.since = 0.0
        self.ring = f"ring_{tid}"
        self.ring_color = None

    def target_state(self, link):
        """Green for every link on the ambulance's approach edge, red else."""
        n = len(self.links)
        in_edge = self.links[link][0][0].rsplit("_", 1)[0]
        state = ["r"] * n
        for i, group in enumerate(self.links):
            if group and group[0][0].rsplit("_", 1)[0] == in_edge:
                state[i] = "G"
        return state

    def requesters(self, now):
        return {a: v for a, v in self.heard.items()
                if now - v[0] <= BEACON_TIMEOUT}

    def is_armed(self, amb, now):
        return now - self.armed.get(amb, -1e9) <= ARM_TTL

    def start(self, now, amb, log):
        _, link, _ = self.heard[amb]
        current = list(traci.trafficlight.getRedYellowGreenState(self.id))
        self.target = self.target_state(link)
        self.owner = amb
        self.since = now
        transition = list(current)
        needs_yellow = False
        for i, c in enumerate(current):
            if c in "Gg" and self.target[i] == "r":
                transition[i] = "y"
                needs_yellow = True
        if needs_yellow:
            traci.trafficlight.setRedYellowGreenState(
                self.id, "".join(transition))
            self.mode = Signal.CLEARING
            self.until = now + YELLOW_TIME
        else:
            traci.trafficlight.setRedYellowGreenState(
                self.id, "".join(self.target))
            self.mode = Signal.GREEN
        log(f"[V2I] {self.id}: {amb} requested priority "
            f"({self.via[amb]}, {self.heard[amb][2]:.0f} m) "
            f"-> clearing" if needs_yellow else
            f"[V2I] {self.id}: {amb} requested priority "
            f"({self.via[amb]}, {self.heard[amb][2]:.0f} m) -> GREEN")

    def release(self, now, log, events):
        events.append((self.id, self.owner, self.via.get(self.owner, "V2I"),
                       self.since, now))
        log(f"[V2I] {self.id}: {self.owner} cleared -> released")
        self.heard.pop(self.owner, None)
        self.owner = None
        self.mode = Signal.NORMAL

    def restore(self):
        traci.trafficlight.setProgram(self.id, self.program)


# ------------------------------------------------------------
# GUI helpers
# ------------------------------------------------------------

RING_COLORS = {
    "armed": (255, 165, 0, 110),
    "clearing": (255, 230, 0, 130),
    "green": (0, 220, 60, 130),
    None: (0, 0, 0, 0),
}


def make_ring(sig):
    x, y = traci.junction.getPosition(sig.id)
    import math
    shape = [(x + 16 * math.cos(a / 12 * 2 * math.pi),
              y + 16 * math.sin(a / 12 * 2 * math.pi)) for a in range(12)]
    traci.polygon.add(sig.ring, shape, RING_COLORS[None], fill=True, layer=10)


def paint_ring(sig, kind):
    if kind != sig.ring_color:
        traci.polygon.setColor(sig.ring, RING_COLORS[kind])
        sig.ring_color = kind


def route_shape(amb):
    """Centre line of the ambulance's remaining route (for the GUI ribbon)."""
    route = traci.vehicle.getRoute(amb)
    idx = traci.vehicle.getRouteIndex(amb)
    pts = []
    for edge in route[max(idx, 0):]:
        pts.extend(traci.lane.getShape(f"{edge}_0"))
    return pts


def gui_ambulance_start(amb, colors):
    traci.vehicle.highlight(amb, colors[amb] + (255,), size=14)
    x, y = traci.vehicle.getPosition(amb)
    path, w, h = overlay.label(amb.upper(), colors[amb])
    traci.poi.add(f"lbl_{amb}", x, y + 26, WHITE, "", 30,
                  path, w, h)
    traci.polygon.add(f"route_{amb}", route_shape(amb), colors[amb] + (90,),
                      fill=False, layer=2, lineWidth=4)


def gui_ambulance_update(amb, text, color, edge_changed):
    x, y = traci.vehicle.getPosition(amb)
    path, w, h = overlay.label(text, color)
    traci.poi.setPosition(f"lbl_{amb}", x, y + 26)
    traci.poi.setImageFile(f"lbl_{amb}", path)
    traci.poi.setWidth(f"lbl_{amb}", w)
    traci.poi.setHeight(f"lbl_{amb}", h)
    if edge_changed:
        pts = route_shape(amb)
        if len(pts) >= 2:
            traci.polygon.setShape(f"route_{amb}", pts)


def gui_ambulance_end(amb):
    for kind, ident in ((traci.poi, f"lbl_{amb}"),
                        (traci.polygon, f"route_{amb}")):
        try:
            kind.remove(ident)
        except traci.TraCIException:
            pass


def gui_junction_tags(signals):
    for tid in signals:
        x, y = traci.junction.getPosition(tid)
        path, w, h = overlay.junction_tag(tid)
        traci.poi.add(f"tag_{tid}", x + 24, y + 24, WHITE, "", 25,
                      path, w, h)


# POI images are multiplied by the POI colour, so keep it white
WHITE = (255, 255, 255, 255)
BOARD_POS = (600.0, 660.0)
BOARD_HEAD = ["SIGNAL RINGS:  ORANGE = warned by I2I   YELLOW = clearing   "
              "GREEN = ambulance priority",
              "LINES: coloured line = I2I warning passed from one signal "
              "to the next"]


def gui_board(feed):
    lines = BOARD_HEAD + list(feed)[-4:]
    path, w, h = overlay.board(lines)
    if "board" in traci.poi.getIDList():
        traci.poi.setImageFile("board", path)
    else:
        traci.poi.add("board", BOARD_POS[0], BOARD_POS[1], WHITE,
                      "", 40, path, w, h)


def i2i_line(src_tid, dst_tid, amb, color, lines):
    key = (src_tid, dst_tid, amb)
    if key in lines:
        return
    a = traci.junction.getPosition(src_tid)
    b = traci.junction.getPosition(dst_tid)
    pid = f"i2i_{src_tid}_{dst_tid}_{amb}"
    traci.polygon.add(pid, [a, b], color + (230,), fill=False, layer=15,
                      lineWidth=7)
    lines[key] = pid


def i2i_prune(lines, alive, ahead):
    """Drop a warning line once its ambulance has passed the target signal."""
    for key, pid in list(lines.items()):
        _, dst, amb = key
        if amb not in alive or dst not in ahead.get(amb, ()):
            try:
                traci.polygon.remove(pid)
            except traci.TraCIException:
                pass
            del lines[key]


# ------------------------------------------------------------
# V2V: cars ahead of an ambulance make way
# ------------------------------------------------------------

def lane_ids(edge, idx):
    return traci.lane.getLastStepVehicleIDs(f"{edge}_{idx}")


def pick_lane(car, edge, lane, n_lanes, cpos):
    """A free adjacent lane that still reaches the car's next road, or None."""
    route = traci.vehicle.getRoute(car)
    ridx = traci.vehicle.getRouteIndex(car)
    need = route[ridx + 1] if ridx + 1 < len(route) else None
    for target in (lane + 1, lane - 1):
        if not 0 <= target < n_lanes:
            continue
        if any(-V2V_GAP_REAR <= traci.vehicle.getLanePosition(v) - cpos
               <= V2V_GAP_FRONT for v in lane_ids(edge, target) if v != car):
            continue
        if need is not None:
            links = traci.lane.getLinks(f"{edge}_{target}")
            if not any(lk[0].rsplit("_", 1)[0] == need for lk in links):
                continue
        return target
    return None


def v2v_step(alive, ambulances, yielding, log):
    """Ask cars ahead of each ambulance to move over; release those it passed."""
    amb_set = set(ambulances)
    moved = 0

    for amb in ambulances:
        if amb not in alive:
            continue
        edge = traci.vehicle.getRoadID(amb)
        if not edge or edge.startswith(":"):
            continue
        n_lanes = traci.edge.getLaneNumber(edge)
        if n_lanes < 2:
            continue
        lane = traci.vehicle.getLaneIndex(amb)
        apos = traci.vehicle.getLanePosition(amb)
        for car in lane_ids(edge, lane):
            if car in amb_set or car in yielding:
                continue
            cpos = traci.vehicle.getLanePosition(car)
            if not 0 < cpos - apos <= V2V_RANGE:
                continue
            target = pick_lane(car, edge, lane, n_lanes, cpos)
            if target is None:
                continue
            traci.vehicle.setLaneChangeMode(car, 512)
            traci.vehicle.changeLane(car, target, 8.0)
            yielding[car] = (amb, edge)
            moved += 1
            log(f"[V2V] {car}: {amb} is {cpos - apos:.0f} m behind -> "
                f"lane {lane} -> {target}")

    for car, (amb, edge) in list(yielding.items()):
        if (car not in alive or amb not in alive
                or traci.vehicle.getRoadID(car) != edge
                or traci.vehicle.getRoadID(amb) != edge
                or traci.vehicle.getLanePosition(amb)
                > traci.vehicle.getLanePosition(car) + 8):
            if car in alive:
                traci.vehicle.setLaneChangeMode(car, 1621)
            del yielding[car]
    return moved


# ------------------------------------------------------------
# Main
# ------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--nogui", action="store_true")
    ap.add_argument("--baseline", action="store_true",
                    help="no V2I/I2I: signals run their normal programs")
    ap.add_argument("--latency", type=float, default=0.0,
                    help="V2I/I2I message latency in s")
    ap.add_argument("--loss", type=float, default=0.0,
                    help="V2I/I2I message loss probability 0..1")
    ap.add_argument("--beacon", type=float, default=0.5,
                    help="beacon interval in s")
    ap.add_argument("--delay", type=int, default=100,
                    help="GUI delay per step in ms")
    ap.add_argument("--no-v2v", action="store_true",
                    help="signals only: skip V2V lane clearing")
    ap.add_argument("--track", default=None,
                    help="GUI: lock the camera on a vehicle (e.g. amb1) or "
                         "'auto' for whichever ambulance is active. While "
                         "locked the view re-centres every step, so you "
                         "cannot pan or zoom freely. Default: free camera")
    ap.add_argument("--focus", default=None, metavar="J3",
                    help="GUI: start centred on this junction (e.g. J3)")
    ap.add_argument("--zoom", type=float, default=450,
                    help="GUI: zoom used with --focus (100 = whole map)")
    ap.add_argument("--pause", action="store_true",
                    help="GUI: start paused, press Play to begin")
    ap.add_argument("--seed", type=int, default=1)
    args = ap.parse_args()

    binary = shutil.which("sumo" if args.nogui else "sumo-gui")
    if binary is None:
        sys.exit("[ERROR] SUMO not found on PATH (sudo apt install sumo)")

    cmd = [binary, "-c", os.path.join(HERE, "simulation.sumocfg"),
           "--seed", str(args.seed)]
    if not args.nogui:
        cmd += ["--delay", str(args.delay)]
        if not args.pause:
            cmd += ["--start"]
    traci.start(cmd)

    ambulances, colors = load_ambulances()
    if args.baseline:
        mode, tag = "BASELINE (no V2X)", "baseline"
    elif args.no_v2v:
        mode, tag = "V2X signals only (V2I + I2I)", "signals"
    else:
        mode, tag = "FULL V2X (V2V + V2I + I2I)", "full"
    print("=" * 74)
    print(f"  10-SIGNAL EMERGENCY CORRIDOR   {len(ambulances)} ambulances")
    print(f"  mode: {mode}")
    if not args.baseline:
        print(f"  channel: latency {args.latency * 1000:.0f} ms, "
              f"loss {args.loss * 100:.0f} %, beacon {args.beacon} s")
    print("=" * 74)

    feed = []            # recent key events, shown on the GUI board
    feed_dirty = [True]

    def log(msg):
        t = traci.simulation.getTime()
        print(f"[{t:6.1f}s] {msg}")
        if any(k in msg for k in ("[I2I]", "GREEN for", "arrived")):
            short = msg.replace("[V2I] ", "").replace("[SYSTEM] ", "")
            feed.append(f"{t:5.0f}s  {short}"[:100])
            feed_dirty[0] = True

    signals = {t: Signal(t) for t in traci.trafficlight.getIDList()}
    print(f"[SYSTEM] {len(signals)} traffic signals: "
          f"{', '.join(sorted(signals, key=lambda s: int(s[1:])))}")

    if not args.nogui:
        traci.gui.setSchema("View #0", "real world")
        traci.gui.setBoundary("View #0", -220, -220, 1420, 760)
        for sig in signals.values():
            make_ring(sig)
        gui_junction_tags(signals)
        if args.focus in signals:
            fx, fy = traci.junction.getPosition(args.focus)
            traci.gui.setZoom("View #0", args.zoom)
            traci.gui.setOffset("View #0", fx, fy)

    channel = Channel(args.latency, args.loss, args.seed)
    last_beacon = {}
    events = []

    depart = {}
    arrive = {}
    stopped_time = {a: 0.0 for a in ambulances}
    stops = {a: 0 for a in ambulances}
    was_stopped = {a: False for a in ambulances}
    last_edge = {}
    yielding = {}
    i2i_lines = {}
    ahead = {}
    v2v_moves = 0
    step_len = traci.simulation.getDeltaT()
    tracked = None

    while (traci.simulation.getMinExpectedNumber() > 0
           and len(arrive) < len(ambulances)):
        traci.simulationStep()
        now = traci.simulation.getTime()
        alive = set(traci.vehicle.getIDList())

        if args.track and not args.nogui:
            want = None
            if args.track == "auto":
                want = next((a for a in ambulances if a in alive), None)
            elif args.track in alive:
                want = args.track
            if want and want != tracked:
                traci.gui.trackVehicle("View #0", want)
                traci.gui.setZoom("View #0", 120)
                tracked = want

        # ---- ambulance bookkeeping --------------------------------
        for amb in ambulances:
            if amb in alive:
                if amb not in depart and not args.nogui:
                    gui_ambulance_start(amb, colors)
                depart.setdefault(amb, now)
                if traci.vehicle.getSpeed(amb) < 0.1:
                    stopped_time[amb] += step_len
                    if not was_stopped[amb]:
                        stops[amb] += 1
                    was_stopped[amb] = True
                else:
                    was_stopped[amb] = False
            elif amb in depart and amb not in arrive:
                if not args.nogui:
                    gui_ambulance_end(amb)
                arrive[amb] = now
                log(f"[SYSTEM] {amb} arrived "
                    f"(travel time {now - depart[amb]:.1f} s)")

        if not args.nogui:
            for amb in ambulances:
                if amb in alive and amb in depart:
                    nxt = traci.vehicle.getNextTLS(amb)
                    ahead[amb] = {t[0] for t in nxt}
                    where = (f"  >  {nxt[0][0]}  {round(nxt[0][2] / 20) * 20:.0f} m"
                             if nxt else "")
                    kmh = round(traci.vehicle.getSpeed(amb) * 3.6 / 10) * 10
                    edge = traci.vehicle.getRoadID(amb)
                    gui_ambulance_update(
                        amb, f"{amb.upper()}  {kmh:.0f} km/h{where}",
                        colors[amb],
                        edge != last_edge.get(amb) and not edge.startswith(":"))
                    last_edge[amb] = edge
            i2i_prune(i2i_lines, alive, ahead)
            if feed_dirty[0]:
                gui_board(feed)
                feed_dirty[0] = False

        if args.baseline:
            continue

        # ---- V2V lane clearing --------------------------------------
        if not args.no_v2v:
            v2v_moves += v2v_step(alive, ambulances, yielding, log)

        # ---- deliver messages -------------------------------------
        for _, kind, src, dst, payload in channel.deliver(now):
            sig = signals[dst]
            if kind == "V2I":
                if src not in sig.heard:
                    sig.via[src] = "I2I" if payload[1] > V2I_RADIUS else "V2I"
                sig.heard[src] = (now, payload[0], payload[1])
            else:  # I2I
                if not sig.is_armed(src, now):
                    log(f"[I2I] {payload} -> {dst}: {src} approaching, "
                        f"{dst} ARMED (listening to {I2I_RADIUS:.0f} m)")
                    if not args.nogui:
                        i2i_line(payload, dst, src, colors[src], i2i_lines)
                sig.armed[src] = now

        # ---- ambulances transmit ----------------------------------
        for amb in ambulances:
            if amb not in alive:
                continue
            upcoming = traci.vehicle.getNextTLS(amb)
            for i, (tid, link, dist, _) in enumerate(upcoming):
                if tid not in signals:
                    continue
                sig = signals[tid]
                radius = I2I_RADIUS if sig.is_armed(amb, now) else V2I_RADIUS
                key = (amb, tid)
                if (dist <= radius
                        and now - last_beacon.get(key, -1e9) >= args.beacon):
                    channel.send(now, "V2I", amb, tid, (link, dist))
                    last_beacon[key] = now
                # I2I: the signal serving this ambulance arms the next one
                if sig.owner == amb and i + 1 < len(upcoming):
                    nxt = upcoming[i + 1][0]
                    if nxt in signals:
                        k2 = (tid, nxt, amb)
                        if now - last_beacon.get(k2, -1e9) >= args.beacon:
                            channel.send(now, "I2I", amb, nxt, tid)
                            last_beacon[k2] = now

        # ---- signal state machines --------------------------------
        for sig in signals.values():
            reqs = sig.requesters(now)

            if sig.mode == Signal.NORMAL:
                if reqs:
                    amb = min(reqs, key=lambda a: reqs[a][2])
                    sig.start(now, amb, log)

            elif sig.mode == Signal.CLEARING:
                if now >= sig.until:
                    traci.trafficlight.setRedYellowGreenState(
                        sig.id, "".join(sig.target))
                    sig.mode = Signal.GREEN
                    log(f"[V2I] {sig.id}: GREEN for {sig.owner}")

            elif sig.mode == Signal.GREEN:
                owner = sig.owner
                inside = (owner in alive and
                          traci.vehicle.getRoadID(owner).startswith(f":{sig.id}_"))
                if owner not in reqs and not inside:
                    sig.release(now, log, events)
                    waiting = sig.requesters(now)
                    if waiting:
                        nxt_amb = min(waiting, key=lambda a: waiting[a][2])
                        sig.start(now, nxt_amb, log)
                    else:
                        sig.restore()

            # GUI rings: orange = armed by I2I, yellow = clearing, green = preempted
            if not args.nogui:
                if sig.mode == Signal.GREEN:
                    paint_ring(sig, "green")
                elif sig.mode == Signal.CLEARING:
                    paint_ring(sig, "clearing")
                elif any(sig.is_armed(a, now) and a in alive
                         for a in ambulances):
                    paint_ring(sig, "armed")
                else:
                    paint_ring(sig, None)

    # ---- summary ----------------------------------------------------
    now = traci.simulation.getTime()
    print()
    print("=" * 74)
    print(f"  SUMMARY   mode: {mode}   (sim time {now:.0f} s)")
    print("=" * 74)
    print(f"{'ambulance':<10}{'depart':>8}{'arrive':>9}{'travel':>9}"
          f"{'stopped':>9}{'stops':>7}")
    rows = []
    for amb in ambulances:
        if amb in arrive:
            travel = arrive[amb] - depart[amb]
            print(f"{amb:<10}{depart[amb]:>8.1f}{arrive[amb]:>9.1f}"
                  f"{travel:>9.1f}{stopped_time[amb]:>9.1f}{stops[amb]:>7d}")
            rows.append([amb, depart[amb], arrive[amb], round(travel, 1),
                         round(stopped_time[amb], 1), stops[amb]])
        else:
            print(f"{amb:<10}  did not finish")
    if rows:
        print(f"{'TOTAL':<10}{'':>8}{'':>9}"
              f"{sum(r[3] for r in rows):>9.1f}"
              f"{sum(r[4] for r in rows):>9.1f}{sum(r[5] for r in rows):>7d}")
    if not args.baseline:
        n_i2i = sum(1 for e in events if e[2] == "I2I")
        print(f"\npreemptions: {len(events)} "
              f"({n_i2i} triggered early via I2I)")
        print(f"messages   : {channel.sent} sent, {channel.dropped} lost")
        if not args.no_v2v:
            print(f"V2V        : {v2v_moves} cars moved over for ambulances")

    os.makedirs(os.path.join(HERE, "results"), exist_ok=True)
    with open(os.path.join(HERE, "results", f"ambulances_{tag}.csv"),
              "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["ambulance", "depart_s", "arrive_s", "travel_s",
                    "stopped_s", "stops"])
        w.writerows(rows)
    if not args.baseline:
        with open(os.path.join(HERE, "results", f"preemptions_{tag}.csv"),
                  "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["signal", "ambulance", "trigger", "start_s", "end_s"])
            w.writerows(events)

    if not args.nogui and sys.stdin.isatty():
        input("\nSimulation finished. Press Enter to close SUMO-GUI...")
    traci.close()


if __name__ == "__main__":
    main()
