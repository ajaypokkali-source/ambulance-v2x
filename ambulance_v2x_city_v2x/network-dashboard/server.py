#!/usr/bin/env python3
"""
Real-Time Network Visualization Dashboard Server.

Provides:
- HTTP server for dashboard web frontend
- Server-Sent Events (SSE) stream (/api/events/stream) for zero-latency event push
- UDP listener (127.0.0.1:5005) ingesting live simulation telemetry
- Road topology extractor from SUMO network.net.xml and ambulances.rou.xml
- Node & packet state cache for instant browser synchronization
"""

import argparse
import json
import os
import queue
import socket
import sys
import threading
import time
import xml.etree.ElementTree as ET
from collections import deque
from http import HTTPStatus
from http.server import HTTPServer, SimpleHTTPRequestHandler
from socketserver import ThreadingMixIn

DASHBOARD_DIR = os.path.dirname(os.path.abspath(__file__))
SIM_DIR = os.path.dirname(DASHBOARD_DIR)
NET_XML_PATH = os.path.join(SIM_DIR, "network.net.xml")
ROU_XML_PATH = os.path.join(SIM_DIR, "ambulances.rou.xml")

TELEMETRY_PORT = 5005
HTTP_PORT = 8080


class ThreadingHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True


class NetworkTopology:
    """Parses SUMO network and ambulance definitions."""

    def __init__(self, net_path=NET_XML_PATH, rou_path=ROU_XML_PATH):
        self.net_path = net_path
        self.rou_path = rou_path
        self.junctions = {}
        self.edges = []
        self.ambulances = {}
        self.bounds = {"min_x": 0.0, "max_x": 1500.0, "min_y": 0.0, "max_y": 600.0}
        self.load()

    def load(self):
        # 1. Parse network.net.xml
        if os.path.exists(self.net_path):
            try:
                tree = ET.parse(self.net_path)
                root = tree.getroot()

                # Get bounds if available
                loc = root.find("location")
                if loc is not None and "convBoundary" in loc.attrib:
                    parts = [float(x) for x in loc.attrib["convBoundary"].split(",")]
                    if len(parts) == 4:
                        self.bounds = {
                            "min_x": parts[0],
                            "min_y": parts[1],
                            "max_x": parts[2],
                            "max_y": parts[3],
                        }

                # Parse junctions
                for j in root.findall("junction"):
                    jid = j.get("id")
                    if jid.startswith(":"):
                        continue
                    self.junctions[jid] = {
                        "id": jid,
                        "x": float(j.get("x")),
                        "y": float(j.get("y")),
                        "type": j.get("type"),
                        "is_tls": jid in [f"J{i}" for i in range(1, 11)],
                        "ue": (int(jid[1:]) + 7) if (jid.startswith("J") and jid[1:].isdigit() and 1 <= int(jid[1:]) <= 10) else None
                    }

                # Parse normal road edges and lane geometry
                for e in root.findall("edge"):
                    eid = e.get("id")
                    if eid.startswith(":"):
                        continue
                    lanes = []
                    for lane in e.findall("lane"):
                        shape_str = lane.get("shape", "")
                        points = [
                            [float(c) for c in pt.split(",")]
                            for pt in shape_str.strip().split()
                            if "," in pt
                        ]
                        lanes.append({
                            "id": lane.get("id"),
                            "speed": float(lane.get("speed", 13.89)),
                            "length": float(lane.get("length", 0.0)),
                            "shape": points,
                        })
                    self.edges.append({
                        "id": eid,
                        "from": e.get("from"),
                        "to": e.get("to"),
                        "lanes": lanes,
                    })
            except Exception as exc:
                print(f"[WARN] Error parsing network.net.xml: {exc}")

        # 2. Parse ambulances.rou.xml
        if os.path.exists(self.rou_path):
            try:
                tree = ET.parse(self.rou_path)
                root = tree.getroot()
                for v in root.findall("vehicle"):
                    vid = v.get("id")
                    depart = float(v.get("depart", 0))
                    color = v.get("color", "1,0,0")
                    route_elem = v.find("route")
                    edges_str = route_elem.get("edges", "") if route_elem is not None else ""
                    self.ambulances[vid] = {
                        "id": vid,
                        "depart": depart,
                        "color": color,
                        "route": edges_str.split(),
                        "ue": int(vid.replace("amb", "")) - 1 if vid.startswith("amb") and vid[3:].isdigit() else None
                    }
            except Exception as exc:
                print(f"[WARN] Error parsing ambulances.rou.xml: {exc}")

    def to_dict(self):
        return {
            "junctions": self.junctions,
            "edges": self.edges,
            "ambulances": self.ambulances,
            "bounds": self.bounds,
            "radio_config": {
                "technology": "3GPP NR-V2X Rel 16 Sidelink (5G-LENA v3.1 / CTTC)",
                "band": "n47 (5.89 GHz)",
                "channel_bandwidth": "40 MHz",
                "numerology": 2,
                "scs": "60 kHz",
                "v2i_mode": "Sidelink Groupcast (L2 ID 255, 225.0.0.0:8000)",
                "i2i_mode": "Target-Specific Sidelink Groupcast (L2 ID 108..117, 225.0.1.x:8001)",
                "phy_mac_tracing": "Traced to SQLite DB upon completion; application events streamed live over UDP"
            }
        }


class TelemetryState:
    """Thread-safe state store for simulation telemetry and connected clients."""

    def __init__(self, topology):
        self.topology = topology
        self.lock = threading.Lock()
        self.is_connected = False
        self.last_telemetry_time = 0.0
        self.sim_time = 0.0
        self.sim_step = 0
        self.sim_duration = 120.0
        self.pacing = 0.5
        
        # Live entities
        self.ambulances = {}
        self.cars = []
        self.signals = {}
        
        # Recent event history buffer
        self.history = deque(maxlen=2000)
        self.sse_queues = []
        
        # Metrics
        self.metrics = {
            "v2i_tx": 0,
            "v2i_rx": 0,
            "v2i_accepted": 0,
            "i2i_tx": 0,
            "i2i_rx": 0,
            "total_app_tx": 0,
            "total_app_rx": 0,
            "active_ambulances": 0,
            "active_cars": 0,
            "recent_deliveries": deque(maxlen=100),
        }

    def register_client(self):
        q = queue.Queue(maxsize=1000)
        with self.lock:
            self.sse_queues.append(q)
            # Send initial state snapshot
            initial_msg = {
                "type": "snapshot",
                "is_connected": self.is_connected,
                "sim_time": self.sim_time,
                "sim_step": self.sim_step,
                "sim_duration": self.sim_duration,
                "pacing": self.pacing,
                "ambulances": self.ambulances,
                "signals": self.signals,
                "metrics": self.get_metrics_summary(),
                "history": list(self.history)[-100:]  # Last 100 events
            }
            q.put(initial_msg)
        return q

    def unregister_client(self, q):
        with self.lock:
            if q in self.sse_queues:
                self.sse_queues.remove(q)

    def broadcast(self, event):
        with self.lock:
            # Store event in history
            self.history.append(event)
            # Push to all connected SSE clients
            dead_queues = []
            for q in self.sse_queues:
                try:
                    q.put_nowait(event)
                except queue.Full:
                    dead_queues.append(q)
            for q in dead_queues:
                if q in self.sse_queues:
                    self.sse_queues.remove(q)

    def get_metrics_summary(self):
        v2i_tx = self.metrics["v2i_tx"]
        v2i_rx = self.metrics["v2i_rx"]
        i2i_tx = self.metrics["i2i_tx"]
        i2i_rx = self.metrics["i2i_rx"]
        
        # Calculation of correlated delivery ratio for V2I
        v2i_pdr = (v2i_rx / v2i_tx) if v2i_tx > 0 else None
        i2i_pdr = (i2i_rx / i2i_tx) if i2i_tx > 0 else None

        return {
            "v2i_tx": v2i_tx,
            "v2i_rx": v2i_rx,
            "v2i_accepted": self.metrics["v2i_accepted"],
            "v2i_pdr": round(v2i_pdr, 3) if v2i_pdr is not None else "N/A",
            "i2i_tx": i2i_tx,
            "i2i_rx": i2i_rx,
            "i2i_pdr": round(i2i_pdr, 3) if i2i_pdr is not None else "N/A",
            "total_app_tx": self.metrics["total_app_tx"],
            "total_app_rx": self.metrics["total_app_rx"],
            "active_ambulances": self.metrics["active_ambulances"],
            "active_cars": self.metrics["active_cars"],
            "phy_mac_note": "Traced to SQLite; not streamed over live UDP"
        }

    def clear_history(self):
        with self.lock:
            self.history.clear()
            self.metrics["v2i_tx"] = 0
            self.metrics["v2i_rx"] = 0
            self.metrics["v2i_accepted"] = 0
            self.metrics["i2i_tx"] = 0
            self.metrics["i2i_rx"] = 0
            self.metrics["total_app_tx"] = 0
            self.metrics["total_app_rx"] = 0
        self.broadcast({"type": "history_cleared"})

    def handle_telemetry(self, raw_data):
        now = time.time()
        self.last_telemetry_time = now
        self.is_connected = True

        try:
            msg = json.loads(raw_data)
        except Exception:
            return

        etype = msg.get("type")
        data = msg.get("data", {})

        with self.lock:
            if etype == "init":
                self.sim_duration = data.get("duration", 120.0)
                self.pacing = data.get("pacing", 0.5)

            elif etype == "step":
                self.sim_step = data.get("step", 0)
                self.sim_time = data.get("sim_time", 0.0)
                self.ambulances = data.get("ambulances", {})
                self.cars = data.get("cars", [])
                self.signals = data.get("signals", {})
                self.metrics["active_ambulances"] = len(self.ambulances)
                self.metrics["active_cars"] = len(self.cars)

            elif etype == "packet_tx":
                proto = data.get("protocol")
                self.metrics["total_app_tx"] += 1
                if proto == "V2I":
                    self.metrics["v2i_tx"] += 1
                elif proto == "I2I":
                    self.metrics["i2i_tx"] += 1

            elif etype == "packet_rx":
                proto = data.get("protocol")
                self.metrics["total_app_rx"] += 1
                if proto == "V2I":
                    self.metrics["v2i_rx"] += 1
                    if data.get("accepted"):
                        self.metrics["v2i_accepted"] += 1
                elif proto == "I2I":
                    self.metrics["i2i_rx"] += 1

            elif etype == "sim_stopped":
                self.is_connected = False

        # Broadcast event to web clients
        event_obj = {
            "type": etype,
            "ts": msg.get("ts", now),
            "seq": msg.get("seq", 0),
            "data": data,
            "metrics": self.get_metrics_summary() if etype in ("packet_tx", "packet_rx", "step", "init", "sim_stopped") else None
        }
        self.broadcast(event_obj)

    def check_liveness(self):
        """Monitors heartbeat from simulation."""
        now = time.time()
        with self.lock:
            if self.is_connected and (now - self.last_telemetry_time > 2.5):
                self.is_connected = False
                st_event = {"type": "connection_state", "is_connected": False}
                # Dispatch disconnect notification
                for q in self.sse_queues:
                    try:
                        q.put_nowait(st_event)
                    except queue.Full:
                        pass


class TelemetryReceiver(threading.Thread):
    """Background UDP socket listener on 127.0.0.1:5005."""

    def __init__(self, state, port=TELEMETRY_PORT):
        super().__init__(daemon=True)
        self.state = state
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("127.0.0.1", self.port))
        self.running = True

    def run(self):
        print(f"[TELEMETRY] Listening for simulation events on UDP 127.0.0.1:{self.port}")
        self.sock.settimeout(0.5)
        while self.running:
            try:
                data, _ = self.sock.recvfrom(65535)
                self.state.handle_telemetry(data.decode("utf-8", errors="ignore"))
            except socket.timeout:
                self.state.check_liveness()
            except Exception as exc:
                if self.running:
                    time.sleep(0.1)

    def stop(self):
        self.running = False
        self.sock.close()


def create_handler(state, topology):
    class DashboardRequestHandler(SimpleHTTPRequestHandler):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, directory=DASHBOARD_DIR, **kwargs)

        def log_message(self, format, *args):
            # Suppress normal GET spam in console for clean CLI experience
            if self.path.startswith("/api/events/stream"):
                return
            super().log_message(format, *args)

        def do_GET(self):
            parsed_path = self.path.split("?")[0]

            if parsed_path == "/api/network":
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "application/json")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers()
                self.wfile.write(json.dumps(topology.to_dict()).encode("utf-8"))
                return

            elif parsed_path == "/api/state":
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "application/json")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers()
                with state.lock:
                    snapshot = {
                        "is_connected": state.is_connected,
                        "sim_time": state.sim_time,
                        "sim_step": state.sim_step,
                        "sim_duration": state.sim_duration,
                        "pacing": state.pacing,
                        "ambulances": state.ambulances,
                        "signals": state.signals,
                        "metrics": state.get_metrics_summary(),
                        "history": list(state.history)[-200:]
                    }
                self.wfile.write(json.dumps(snapshot).encode("utf-8"))
                return

            elif parsed_path == "/api/events/stream":
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "text/event-stream")
                self.send_header("Cache-Control", "no-cache")
                self.send_header("Connection", "keep-alive")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers()

                client_q = state.register_client()
                try:
                    while True:
                        try:
                            item = client_q.get(timeout=2.0)
                            msg = f"data: {json.dumps(item)}\n\n"
                            self.wfile.write(msg.encode("utf-8"))
                            self.wfile.flush()
                        except queue.Empty:
                            # Keep-alive heartbeat comment
                            self.wfile.write(b": keepalive\n\n")
                            self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError):
                    pass
                finally:
                    state.unregister_client(client_q)
                return

            # Serve static files
            super().do_GET()

        def do_POST(self):
            if self.path == "/api/clear":
                state.clear_history()
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "application/json")
                self.end_headers()
                self.wfile.write(b'{"status":"cleared"}')
                return
            self.send_error(HTTPStatus.NOT_FOUND)

    return DashboardRequestHandler


def main():
    parser = argparse.ArgumentParser(description="NR-V2X Network Visualization Dashboard Server")
    parser.add_argument("--port", type=int, default=HTTP_PORT, help=f"HTTP web server port (default: {HTTP_PORT})")
    parser.add_argument("--telemetry-port", type=int, default=TELEMETRY_PORT, help=f"UDP telemetry port (default: {TELEMETRY_PORT})")
    args = parser.parse_args()

    print("=" * 70)
    print("NR-V2X REAL-TIME NETWORK VISUALIZATION DASHBOARD SERVER")
    print("=" * 70)

    # 1. Load road network topology
    topology = NetworkTopology()
    print(f"[NET] Loaded {len(topology.junctions)} junctions, {len(topology.edges)} road segments, {len(topology.ambulances)} ambulances")

    # 2. State & Telemetry receiver
    state = TelemetryState(topology)
    receiver = TelemetryReceiver(state, port=args.telemetry_port)
    receiver.start()

    # 3. HTTP Server
    handler_class = create_handler(state, topology)
    httpd = ThreadingHTTPServer(("0.0.0.0", args.port), handler_class)

    print(f"[HTTP] Web dashboard ready at: http://localhost:{args.port}/")
    print(f"[HTTP] Press Ctrl+C to stop.")

    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n[HTTP] Stopping server...")
    finally:
        receiver.stop()
        httpd.server_close()
        print("[HTTP] Server stopped.")


if __name__ == "__main__":
    main()
