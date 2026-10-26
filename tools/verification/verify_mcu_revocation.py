"""Host software MCU race injection; C99 output only, no board/plant truth.

Use an explicit COMPANIONBOT_CORE_LIBRARY to compare the retained pre-fix and
current builds. The unsafe expectation is for historical reproduction only.
"""
import argparse
import json
from pathlib import Path

from companionbot_contracts.packets import (
    HeartbeatPacket, ReferenceBlockPacket, ResumeReferenceCommandPacket,
    SafetyCommandPacket, YawCommandPacket,
)
from companionbot_control.runtime import native as n


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--expect", choices=("safe", "unsafe"), default="safe")
    args = parser.parse_args()
    out = args.output_dir.resolve()
    out.mkdir(parents=True, exist_ok=True)
    mcu = n.NativeMCU(n.McuConfig(n.BrakeConfig(.002, .5, .8, 1.), .15, .012, 0., .2, .8))
    history, ingress = [], []

    def send(channel, packet, tick):
        result = mcu.receive(channel, packet.to_bytes(), tick)
        ingress.append(dict(tick=tick, channel=channel, sequence=packet.sequence_id, result=result))
        return result

    def block(epoch, sequence, start, tick, velocity=0.):
        return send("reference", ReferenceBlockPacket.from_rows(
            reference_epoch=epoch, sequence_id=sequence, start_control_tick=start,
            generated_time_s=tick * .002, dt_s=.002, yaw_rate_target_rad_s=0.,
            rows=[(i * .002 * velocity, velocity, 0., 0., 0., 0.) for i in range(25)]), tick)

    def step(tick):
        command = mcu.step(tick, 0., 0., 0.)
        history.append(dict(tick=tick, state=int(command.safety_state),
                            epoch=int(command.epoch), reason=int(command.reason),
                            velocity_reference_m_s=command.reference.v))
        return command

    block(0, 0, 0, 0)
    block(0, 1, 25, 0)
    send("heartbeat", HeartbeatPacket(1, 0, 0.), 0)
    send("yaw", YawCommandPacket(1, 0, 0., 0.), 0)
    step(0)
    send("safety", SafetyCommandPacket(1, 0, .002, "HARD_STOP", "FIRST_STOP"), 1)
    for tick in range(1, 601):
        if tick % 20 == 0:
            send("heartbeat", HeartbeatPacket(1, tick, tick * .002), tick)
        state = step(tick)
    assert state.safety_state == 3
    original_epoch = state.epoch
    send("yaw", YawCommandPacket(1, 601, 1.202, 0.), 601)
    assert block(original_epoch, 0, 610, 601, .1) == 0
    assert send("resume", ResumeReferenceCommandPacket(1, 0, 1.202, original_epoch, "HARD"), 601) == 0
    step(601)
    stop_result = send("safety", SafetyCommandPacket(1, 1, 1.204, "HARD_STOP", "CANCEL_PENDING"), 602)
    for tick in range(602, 621):
        step(tick)
    after = [row for row in history if row["tick"] >= 602]
    safe = (stop_result == 0 and all(row["state"] == 3 and row["epoch"] == original_epoch + 1
                                   and row["velocity_reference_m_s"] == 0. for row in after))
    unsafe_reproduced = any(row["state"] == 0 and row["velocity_reference_m_s"] > .09 for row in after)
    passed = safe if args.expect == "safe" else unsafe_reproduced
    summary = dict(scope="host C99 software MCU; no hardware certification",
                   core_library=n.library()._name, expectation=args.expect,
                   stop_result=stop_result, accepted_hard_stop_revokes=safe,
                   old_resume_reactivated=unsafe_reproduced, passed=passed,
                   hardware_execution_ready=False)
    (out / "history.json").write_text(json.dumps(history, indent=2) + "\n")
    (out / "ingress.json").write_text(json.dumps(ingress, indent=2) + "\n")
    (out / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary), flush=True)
    if not passed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
