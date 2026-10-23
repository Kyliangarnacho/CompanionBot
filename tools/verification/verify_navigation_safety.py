"""Physical reference/MCU fault fixtures below ROS, explicitly not Nav2 Actions.

Uses measured state flags and a declared simulation clock. No truth/pose enters
the intent producer. This complements, and never substitutes for, DDS faults.
"""
import argparse
import json
from pathlib import Path

from companionbot_follow.runtime.navigation_motion import NavigationMotionAdapter
from companionbot_simulation.runtime.spatial_session import SpatialSession
from verify_spatial_foundation import control_gates


class FaultDriver:
    def __init__(self, fault):
        self.time_s, self.fault = 0., fault
        self.adapter = NavigationMotionAdapter(map_id="bookstore_fixture", map_version="software_fault_v1", clock=lambda:self.time_s)
        self.task, self.sequence, self.granted = "first", 0, False
        self.fired, self.restarted, self.last_cmd, self.last_renew = False, False, -1., -1.
        self.initial_attempted, self.restart_attempted, self.final_stopped = False, False, False
        self.events, self.states = [], []
        self.cancel_resume_at = None

    def control(self, command):
        self.sequence += 1
        answer=self.adapter.control(task_id=self.task,source_epoch="physical_fault_fixture",sequence=self.sequence,
            command=command,lease_duration_s=.8,map_id=self.adapter.map_id,map_version=self.adapter.map_version,now_s=self.time_s)
        self.events.append(dict(time_s=self.time_s,command=command,task_id=self.task,**answer))
        return answer["accepted"]

    def __call__(self, frame):
        state=frame["state"];self.time_s=state.time_s
        health=state.imu_valid and not state.imu_stale and state.encoder_valid
        if self.fault in ("scan_health_loss","localization_health_loss") and 6.<=self.time_s<7.2:
            health=False
        self.adapter.update_health(health,"fixture_"+self.fault)
        if self.time_s>=1. and not self.initial_attempted:
            self.initial_attempted=True
            self.granted=self.control("ACQUIRE")
        if self.time_s>=6. and not self.fired:
            if self.fault in ("STOP","WAIT","CANCEL","cancel_during_resume"):
                self.control("WAIT" if self.fault=="WAIT" else "STOP")
            self.fired=True
        if self.granted and self.time_s-self.last_renew>=.2:
            if not (self.fault=="lease_expiry" and 6.<=self.time_s<12.):
                self.control("RENEW")
            self.last_renew=self.time_s
        if self.granted and self.time_s-self.last_cmd>=.1:
            if not (self.fault=="command_expiry" and 6.<=self.time_s<12.):
                accepted=self.adapter.receive_cmd(.2,0.,source_time_s=self.time_s,now_s=self.time_s,source_id="diagnostic_fixture")
                if accepted and self.restarted and self.fault=="cancel_during_resume" and self.cancel_resume_at is None:
                    self.cancel_resume_at=self.time_s+.02
            self.last_cmd=self.time_s
        if self.time_s>=12. and not self.restart_attempted:
            self.restart_attempted=True
            self.task="explicit_second";self.sequence=0
            self.restarted=self.control("ACQUIRE")
        if self.time_s>=16. and self.restarted and not self.final_stopped:
            self.final_stopped=True
            self.control("STOP");self.restarted=False;self.granted=False
        if self.cancel_resume_at is not None and self.time_s>=self.cancel_resume_at and not self.final_stopped:
            self.final_stopped=True
            self.control("STOP");self.restarted=False;self.granted=False
        self.states.append(dict(time_s=self.time_s,velocity_hat_m_s=state.velocity_m_s,
                                authorized=self.adapter.snapshot()["authorized"]))


def main():
    cli=argparse.ArgumentParser(description=__doc__);cli.add_argument("--output-dir",type=Path,required=True)
    cli.add_argument("--faults",nargs="+",default=["STOP","WAIT","CANCEL","command_expiry","lease_expiry","scan_health_loss","localization_health_loss","cancel_during_resume"])
    args=cli.parse_args()
    out=args.output_dir.resolve();out.mkdir(parents=True,exist_ok=True);summaries=[]
    for fault in args.faults:
        driver=FaultDriver(fault);path=out/fault;summary=dict(fault=fault,passed=False,scope="software ingress fault + actual frozen MuJoCo/reference/MCU chain; no ROS Action")
        try:
            session=SpatialSession(path,scenario="navigation",world="bookstore",duration_s=20.,motion_adapter=driver.adapter,on_frame=driver)
            result=session.run();summary["control_gates"]=control_gates(result,20.)
            hold=[s for s in driver.states if 10.5<=s["time_s"]<11.5]
            later=[s for s in driver.states if 14.<=s["time_s"]<15.]
            history=result["history_50hz"]
            early_hold=[r for r in history if 8.<=r["t"]<8.8]
            before=min(history,key=lambda r:abs(r["t"]-6.))
            braking=[r for r in history if 6.<=r["t"]<12.]
            summary["early_hold_gt_velocity_peak_m_s"]=max(abs(r["v_GT_m_s"]) for r in early_hold)
            summary["braking_forward_distance_posthoc_m"]=max(r["p_GT_m"] for r in braking)-before["p_GT_m"]
            summary["settled_hold_window_s"]=[10.5,11.5]
            summary["hold_velocity_peak_m_s"]=max(abs(s["velocity_hat_m_s"]) for s in hold)
            summary["old_authority_revoked"]=all(not s["authorized"] for s in hold)
            summary["explicit_restart_velocity_peak_m_s"]=max(abs(s["velocity_hat_m_s"]) for s in later)
            authority=json.loads((path/"motion_authority.json").read_text())
            summary["unexpected_takeovers"]=[e for e in authority["safety_events"] if e["event"]=="safety_takeover"
                and not e["reason"].startswith(("navigation_","fixture_"))]
            summary["cancel_resume_at_s"]=driver.cancel_resume_at
            restart_passed=(summary["explicit_restart_velocity_peak_m_s"]<.03 and driver.cancel_resume_at is not None
                if fault=="cancel_during_resume" else summary["explicit_restart_velocity_peak_m_s"]>.1)
            summary["passed"]=(summary["control_gates"]["passed"] and summary["old_authority_revoked"]
                and not summary["unexpected_takeovers"] and summary["hold_velocity_peak_m_s"]<.03 and restart_passed)
        except BaseException as error:
            summary["error"]=repr(error)
        finally:
            path.mkdir(parents=True,exist_ok=True)
            (path/"fixture_ingress.json").write_text(json.dumps(driver.events,indent=2)+"\n")
            (path/"fixture_state.json").write_text(json.dumps(driver.states)+"\n")
            (path/"adapter.json").write_text(json.dumps(driver.adapter.snapshot(),indent=2)+"\n")
            (path/"summary.json").write_text(json.dumps(summary,indent=2)+"\n")
            summaries.append(summary);print(json.dumps(summary),flush=True)
    (out/"summary.json").write_text(json.dumps(dict(passed=all(s["passed"] for s in summaries),cases=summaries),indent=2)+"\n")
    if not all(s["passed"] for s in summaries):raise SystemExit(1)


if __name__=="__main__":main()
