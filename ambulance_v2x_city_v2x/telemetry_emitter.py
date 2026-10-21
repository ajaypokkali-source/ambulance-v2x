#!/usr/bin/env python3
"""
Telemetry Emitter for NR-V2X Simulation.

Publishes non-invasive real-time simulation events to the Network Dashboard via:
1. UDP loopback to 127.0.0.1:5005 (zero-overhead, non-blocking)
2. Append-only JSONL log (telemetry_events.jsonl) for replay and persistent audit
"""

import os
import json
import socket
import time

TELEMETRY_PORT = 5005
TELEMETRY_HOST = "127.0.0.1"
LOG_FILENAME = "telemetry_events.jsonl"


class TelemetryEmitter:
    _instance = None

    @classmethod
    def get_instance(cls):
        if cls._instance is None:
            cls._instance = cls()
        return cls._instance

    def __init__(self, log_dir=None):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)
        self.target_addr = (TELEMETRY_HOST, TELEMETRY_PORT)
        
        if log_dir is None:
            log_dir = os.path.dirname(os.path.abspath(__file__))
        self.log_path = os.path.join(log_dir, LOG_FILENAME)
        
        try:
            self.log_file = open(self.log_path, "w", encoding="utf-8")
        except OSError:
            self.log_file = None

        self._seq = 0

    def emit(self, event_type, payload):
        self._seq += 1
        msg = {
            "seq": self._seq,
            "type": event_type,
            "ts": time.time(),
            "data": payload
        }
        
        try:
            data = json.dumps(msg).encode("utf-8")
            self.sock.sendto(data, self.target_addr)
        except OSError:
            # UDP drop is fine if dashboard is not listening; never block or crash simulation
            pass

        if self.log_file:
            try:
                self.log_file.write(json.dumps(msg) + "\n")
                self.log_file.flush()
            except OSError:
                pass

    def emit_init(self, sim_duration, pacing, junctions, ambulances):
        self.emit("init", {
            "duration": sim_duration,
            "pacing": pacing,
            "junctions": junctions,
            "ambulances": ambulances,
        })

    def emit_sim_step(self, step, sim_time, ambulances, cars, signals):
        self.emit("step", {
            "step": step,
            "sim_time": round(sim_time, 2),
            "ambulances": ambulances,
            "cars": cars,
            "signals": signals,
        })

    def emit_packet_tx(self, protocol, sender, sender_ue, intended_dest, target_ue,
                       ns3_time, sumo_time, step, extra=None):
        payload = {
            "protocol": protocol,
            "sender": sender,
            "sender_ue": sender_ue,
            "intended_dest": intended_dest,
            "target_ue": target_ue,
            "ns3_time": round(ns3_time, 6),
            "sumo_time": round(sumo_time, 2),
            "step": step,
        }
        if extra:
            payload.update(extra)
        self.emit("packet_tx", payload)

    def emit_packet_rx(self, protocol, sender, sender_ue, receiver, receiver_ue,
                       accepted, ns3_time, sumo_time, step, extra=None):
        payload = {
            "protocol": protocol,
            "sender": sender,
            "sender_ue": sender_ue,
            "receiver": receiver,
            "receiver_ue": receiver_ue,
            "accepted": accepted,
            "ns3_time": round(ns3_time, 6),
            "sumo_time": round(sumo_time, 2),
            "step": step,
        }
        if extra:
            payload.update(extra)
        self.emit("packet_rx", payload)

    def emit_signal_event(self, junction_id, event_type, ambulance_id, mode,
                          state_str, distance=None, sumo_time=0.0):
        self.emit("signal_event", {
            "junction": junction_id,
            "event_type": event_type,
            "ambulance": ambulance_id,
            "mode": mode,
            "state": state_str,
            "distance": round(distance, 1) if distance is not None else None,
            "sumo_time": round(sumo_time, 2),
        })

    def close(self):
        try:
            self.emit("sim_stopped", {"reason": "normal_shutdown"})
        except Exception:
            pass
        if self.log_file:
            try:
                self.log_file.close()
            except OSError:
                pass
        try:
            self.sock.close()
        except OSError:
            pass
