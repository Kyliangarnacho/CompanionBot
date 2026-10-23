"""Finite software UART stress: real rclcpp/DDS/PTY plus the shared C MCU core.

The PTY is explicitly byte-rate shaped in both directions. This measures host
transport behaviour, not USB/UART electronics, MCU CPU speed or hardware DMA.
The fixture knows its MCU clock exactly; no deployment clock mapping is implied.
"""
from pathlib import Path
import argparse
from collections import Counter
import ctypes as C
import fcntl
import hashlib
import json
import math
import os
import pty
import signal
import struct
import subprocess
import threading
import time
import tty

FIXTURE_SHA256 = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()


class RateLimitedMcu:
    def __init__(self, master, baud, output, sent):
        from companionbot_control.runtime import native as n
        from companionbot_control.runtime.embedded_link import EmbeddedParser
        self.n, self.master, self.baud, self.output, self.sent = n, master, baud, output, sent
        self.mcu = n.NativeMCU(n.McuConfig(n.BrakeConfig(.002, .5, .8, 1), .15, .012, 0, .2, .8))
        self.parser = EmbeddedParser()
        self.start = time.monotonic()
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self.run, name="software_rate_limited_MCU")
        self.started = False
        self.first_start = None
        self.active = False
        self.status_enabled = True
        self.read_enabled = True
        self.ingress = bytearray()
        self.egress = bytearray()
        self.stats = Counter()
        self.accepted = Counter()
        self.rejected = Counter()
        self.latencies = []
        self.events = []
        self.error = None

    @property
    def tick(self):
        return int((time.monotonic() - self.start) / .002)

    def run(self):
        from companionbot_control.runtime.embedded_link import EmbeddedFrame
        last = time.monotonic()
        rx_credit = tx_credit = 0.
        step = -1
        next_status = status_sequence = 0
        previous_state = None
        try:
            with (self.output / "wire_events.jsonl").open("w") as log:
                while not self.stop.is_set():
                    now = time.monotonic()
                    delta, last = now - last, now
                    self.stats["fixture_max_loop_gap_us"] = max(self.stats["fixture_max_loop_gap_us"], int(delta * 1e6))
                    # Credit is bounded: host scheduling pauses cannot result in
                    # an arbitrarily large instantaneous serial delivery burst.
                    rx_credit = min(rx_credit + delta * self.baud / 10, self.baud / 1000)
                    tx_credit = min(tx_credit + delta * self.baud / 10, self.baud / 1000)
                    tick = self.tick
                    while step < tick:
                        step += 1
                        if self.first_start is not None and step >= self.first_start:
                            out = self.mcu.step(step, 0, 0, 0)
                            self.started = True
                            state = (out.safety_state, out.reason, out.epoch)
                            if state != previous_state:
                                event = {"state": state[0], "reason": state[1], "epoch": state[2],
                                         "tick": step, "host_s": now, "during_production": self.active}
                                self.events.append(event)
                                log.write(json.dumps({"mcu_event": event}) + "\n")
                                previous_state = state
                            if self.active:
                                self.stats["active_control_ticks"] += 1
                                self.stats["active_non_normal_ticks"] += int(out.safety_state != 0)
                    if self.read_enabled:
                        try:
                            data = os.read(self.master, 4096 - len(self.ingress)) if len(self.ingress) < 4096 else b""
                        except BlockingIOError:
                            data = b""
                        if data and not self.ingress:
                            # An idle line cannot bank past time and spend it
                            # to deliver the first future frame instantaneously.
                            rx_credit = 0.
                        self.ingress.extend(data)
                        self.stats["kernel_accepted_bytes_observed"] += len(data)
                    self.stats["max_shaper_backlog_bytes"] = max(self.stats["max_shaper_backlog_bytes"], len(self.ingress))
                    assert len(self.ingress) <= 4096
                    count = min(len(self.ingress), int(rx_credit))
                    if count:
                        chunk = self.ingress[:count]
                        del self.ingress[:count]
                        rx_credit -= count
                        self.stats["wire_rx_bytes"] += count
                        for frame in self.parser.feed(chunk):
                            raw = frame.c_frame()
                            result = self.n.library().cb_mcu_dispatch(self.mcu.state, C.byref(raw), 77, tick)
                            key = str(frame.kind)
                            (self.rejected if result else self.accepted)[key + ":" + str(result)] += 1
                            if frame.kind == 1 and result == 0 and self.first_start is None:
                                self.first_start = struct.unpack_from("<I", frame.payload, 12)[0]
                            published = self.sent.get((frame.kind, frame.sequence_id))
                            latency = None if published is None else (now - published) * 1000
                            if latency is not None:
                                self.latencies.append(latency)
                            log.write(json.dumps({"kind": frame.kind, "sequence": frame.sequence_id,
                                "tick": tick, "dispatch_result": result, "publish_to_dispatch_ms": latency}) + "\n")
                    if tick >= next_status:
                        next_status = tick + 10
                        if self.status_enabled and not self.egress:
                            out = self.mcu.output
                            integers = (out.epoch, out.safety_state, out.reason, 32, 0,
                                        sum(self.rejected.values()), 0, 0)
                            values = [0.] * 18 + [math.nan, math.nan]
                            payload = struct.pack("<8I20f8I", *integers, *values, *([0] * 8))
                            self.egress.extend(EmbeddedFrame(128, 77, status_sequence, tick, payload).to_wire())
                            tx_credit = 0.
                            status_sequence += 1
                        else:
                            self.stats["status_skipped_busy_or_disabled"] += 1
                    count = min(len(self.egress), int(tx_credit))
                    if count:
                        try:
                            count = os.write(self.master, self.egress[:count])
                        except BlockingIOError:
                            count = 0
                        del self.egress[:count]
                        tx_credit -= count
                        self.stats["wire_tx_bytes"] += count
                    time.sleep(.0005)
        except BaseException as error:
            self.error = str(error)


def main():
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument("--output-dir", type=Path, required=True)
    cli.add_argument("--duration-s", type=float, default=120)
    cli.add_argument("--scenario", choices=("nominal", "load", "overload", "deficit", "burst",
                                           "producer_loss", "heartbeat_loss", "status_loss", "backpressure", "writer_stall",
                                           "slow_baud"), default="nominal")
    cli.add_argument("--preload", type=Path)
    cli.add_argument("--baud", type=int, default=921600)
    args = cli.parse_args()
    if args.duration_s < 2:
        cli.error("duration must be at least 2 seconds")
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=True)
    os.environ["ROS_DOMAIN_ID"] = "97"
    os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    os.environ["ROS_LOG_DIR"] = str(output / "ros_logs")
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from ament_index_python.packages import get_package_prefix
    from diagnostic_msgs.msg import DiagnosticArray
    from companionbot_interfaces.msg import McuFrame
    from companionbot_contracts.packets import ReferenceBlockPacket
    from companionbot_control.runtime.embedded_link import EmbeddedFrame
    rclpy.init()
    probe = rclpy.create_node("uart_stress_fixture", namespace="/uart_stress")
    publisher = probe.create_publisher(McuFrame, "mcu/tx", 8)
    diagnostics = []
    received = []
    subscriptions = [probe.create_subscription(DiagnosticArray, "diagnostics", lambda msg: diagnostics.extend(msg.status), 10),
                     probe.create_subscription(McuFrame, "mcu/rx", lambda msg: received.append(msg.sequence_id), qos_profile_sensor_data)]
    master, slave = pty.openpty()
    tty.setraw(slave)
    fcntl.fcntl(master, fcntl.F_SETFL, os.O_NONBLOCK)
    sent = {}
    wire_baud = 115200 if args.scenario == "slow_baud" else args.baud
    peer = RateLimitedMcu(master, wire_baud, output, sent)
    peer.thread.start()
    binary = Path(get_package_prefix("companionbot_control")) / "lib/companionbot_control/mcu_bridge"
    env = dict(os.environ)
    if args.preload:
        env["LD_PRELOAD"] = str(args.preload.resolve())
        env["CB_SERIAL_PROBE_OUTPUT"] = str(output / "serial_syscalls.json")
    process = None
    checks = {}
    try:
        with (output / "bridge.log").open("w") as log:
            process = subprocess.Popen([str(binary), "--ros-args", "-r", "__ns:=/uart_stress", "-p",
                "device:=" + os.ttyname(slave), "-p", "baud:=" + str(wire_baud), "-p", "allow_command_tx:=true"],
                stdout=log, stderr=subprocess.STDOUT, env=env)
            deadline = time.monotonic() + 10
            while (publisher.get_subscription_count() != 1 or not received) and time.monotonic() < deadline:
                rclpy.spin_once(probe, timeout_sec=.01)
            if not received:
                raise RuntimeError("DDS/status discovery failed")
            warmup_until = time.monotonic() + 1.1
            while time.monotonic() < warmup_until:
                rclpy.spin_once(probe, timeout_sec=.01)
            started = time.monotonic()
            stop_at = started + args.duration_s
            peer.active = True
            sequences = Counter()
            published = Counter()
            rates = {1: 20, 2: 20, 3: 20}
            if args.scenario == "load": rates = {1: 20, 2: 100, 3: 100}
            if args.scenario == "overload": rates = {1: 200, 2: 200, 3: 200}
            next_at = {kind: started for kind in rates}
            next_start = peer.tick + 25
            last_progress = started
            burst_sent = False
            while time.monotonic() < stop_at and process.poll() is None:
                now = time.monotonic()
                elapsed = now - started
                if args.scenario == "deficit" and elapsed > args.duration_s / 2:
                    # Effective service bandwidth falls while configured baud
                    # remains fixed, e.g. a deliberately stalled USB transport.
                    peer.baud = 115200
                if args.scenario == "status_loss" and elapsed > args.duration_s / 2:
                    peer.status_enabled = False
                if args.scenario == "backpressure" and elapsed > args.duration_s / 3:
                    peer.read_enabled = False
                producing = args.scenario != "producer_loss" or elapsed <= args.duration_s / 2
                if producing:
                    for kind, rate in rates.items():
                        if kind == 2 and args.scenario == "heartbeat_loss" and elapsed > args.duration_s / 2:
                            continue
                        if now < next_at[kind]:
                            continue
                        next_at[kind] += 1 / rate
                        sequence = sequences[kind]
                        sequences[kind] += 1
                        payload = b""
                        if kind == 1:
                            payload = ReferenceBlockPacket.from_rows(reference_epoch=0, sequence_id=sequence,
                                start_control_tick=next_start, generated_time_s=0, dt_s=.002,
                                yaw_rate_target_rad_s=0, rows=[(0., 0., 0., 0., 0., 0.)] * 25).to_bytes()
                            next_start += 25
                        elif kind == 3:
                            payload = struct.pack("<d", 0.)
                        message = EmbeddedFrame(kind, 77, sequence, peer.tick + 100,
                            payload, time.monotonic_ns() + 90000000).to_ros()
                        sent[(kind, sequence)] = time.monotonic()
                        publisher.publish(message)
                        published[str(kind)] += 1
                    if args.scenario == "burst" and elapsed > args.duration_s / 2 and not burst_sent:
                        # Extra individually valid reference intentions stress
                        # the queue; receiver capacity/epoch still decides.
                        for _ in range(8):
                            sequence = sequences[1]; sequences[1] += 1
                            payload = ReferenceBlockPacket.from_rows(reference_epoch=0, sequence_id=sequence,
                                start_control_tick=next_start + 25 * _, generated_time_s=0, dt_s=.002,
                                yaw_rate_target_rad_s=0, rows=[(0., 0., 0., 0., 0., 0.)] * 25).to_bytes()
                            sent[(1, sequence)] = time.monotonic()
                            publisher.publish(EmbeddedFrame(1, 77, sequence, peer.tick + 100, payload,
                                time.monotonic_ns() + 90000000).to_ros())
                            published["1"] += 1
                        next_at[1] = now + .05
                        sent[(4, 0)] = time.monotonic()
                        publisher.publish(EmbeddedFrame(4, 77, 0, peer.tick + 100, struct.pack("<BH", 0, 1),
                            time.monotonic_ns() + 90000000).to_ros())
                        published["4"] += 1
                        burst_sent = True
                rclpy.spin_once(probe, timeout_sec=.0005)
                if peer.error:
                    raise RuntimeError(peer.error)
                if now - last_progress > 10:
                    print(json.dumps({"scenario": args.scenario, "elapsed_s": elapsed,
                        "published": published, "accepted": peer.accepted,
                        "backlog": len(peer.ingress), "mcu_state": int(peer.mcu.output.safety_state)}), flush=True)
                    last_progress = now
            peer.active = False
            exit_during_stress = process.poll()
            # Finite drain, not a longer command deadline or an automatic resume.
            until = time.monotonic() + 1.1
            while time.monotonic() < until and process.poll() is None:
                rclpy.spin_once(probe, timeout_sec=.01)
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
            returncode = process.wait(timeout=5)
        peer.stop.set(); peer.thread.join(timeout=5)
        values = {kv.key: kv.value for status in diagnostics for kv in status.values}
        checks["fixture_completed"] = peer.error is None and not peer.thread.is_alive()
        checks["bounded_application_queue"] = int(values.get("queued", 0)) <= 8
        checks["no_physical_execution_claim"] = values.get("hardware_execution_ready") == "false"
        if args.scenario in ("nominal", "load"):
            checks["real_control_core_started"] = peer.started
            checks["bridge_survived"] = exit_during_stress is None and returncode == 0
            checks["mcu_remained_normal_during_production"] = peer.stats["active_non_normal_ticks"] == 0
            checks["all_reference_frames_received"] = peer.accepted["1:0"] == published["1"]
            checks["no_mcu_rejection"] = sum(peer.rejected.values()) == 0
            checks["all_generated_commands_received"] = sum(peer.accepted.values()) == sum(published.values())
            checks["no_bridge_queue_rejection"] = int(values.get("rejected_queue_full", 0)) == 0
        else:
            checks["degraded_or_startup_inhibited"] = any(event["state"] != 0 for event in peer.events) or not peer.started
            if args.scenario in ("writer_stall", "slow_baud"):
                # An incomplete/late first block must never start the core.
                checks["startup_inhibited"] = not peer.started
            if args.scenario in ("backpressure", "writer_stall"):
                checks["blocked_writer_exited_nonzero"] = returncode != 0
            if args.scenario == "status_loss":
                checks["stale_status_gate_visible"] = int(values.get("rejected_device_stale", 0)) > 0
            if args.scenario == "heartbeat_loss":
                checks["heartbeat_timeout_reason_preserved"] = any(event["reason"] == 2 for event in peer.events)
            if args.scenario == "burst":
                checks["explicit_safety_received"] = peer.accepted["4:0"] == 1
                checks["queued_intentions_superseded"] = int(values.get("rejected_superseded", 0)) > 0
                checks["explicit_safety_reason_preserved"] = any(event["reason"] == 1 for event in peer.events)
        latencies = sorted(peer.latencies)
        summary = {"scenario": args.scenario, "requested_duration_s": args.duration_s,
            "fixture_code_sha256": FIXTURE_SHA256,
            "line_idle_credit_accumulation_allowed": False,
            "baud_8n1": wire_baud, "rate_shaped_pty_not_hardware": True,
            "fixture_clock_mapping_known_only_in_test": True,
            "published": published, "mcu_accepted": peer.accepted, "mcu_rejected": peer.rejected,
            "stats": peer.stats, "state_events": peer.events, "diagnostics": values,
            "core_started": peer.started,
            "bridge_exit_during_stress": exit_during_stress, "bridge_returncode": returncode,
            "publish_to_dispatch_ms": {"count": len(latencies),
                "p50": latencies[len(latencies)//2] if latencies else None,
                "p99": latencies[min(len(latencies)-1, int(len(latencies)*.99))] if latencies else None,
                "max": max(latencies, default=None)},
            "checks": checks, "passed": all(checks.values())}
        (output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
        print(json.dumps(summary), flush=True)
        if not summary["passed"]:
            raise SystemExit(1)
    except Exception as error:
        (output / "failure.json").write_text(json.dumps({"error": str(error),
            "stats": peer.stats, "mcu_accepted": peer.accepted, "mcu_rejected": peer.rejected,
            "events": peer.events}, indent=2) + "\n")
        raise
    finally:
        if process is not None and process.poll() is None:
            process.kill(); process.wait()
        peer.stop.set(); peer.thread.join(timeout=5)
        os.close(master); os.close(slave)
        probe.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
