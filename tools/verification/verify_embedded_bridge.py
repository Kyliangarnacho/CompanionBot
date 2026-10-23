"""Finite PTY + real rclcpp/DDS transport acceptance. No physical MCU claim."""
from pathlib import Path
import argparse
import fcntl
import json
import os
import pty
import select
import signal
import struct
import subprocess
import time
import tty

def main():
    cli=argparse.ArgumentParser(description=__doc__)
    cli.add_argument("--output-dir",type=Path,required=True)
    args=cli.parse_args(); output=args.output_dir.resolve(); output.mkdir(parents=True,exist_ok=True)
    os.environ["ROS_DOMAIN_ID"]="96"
    os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"]="LOCALHOST"
    os.environ["ROS_LOG_DIR"]=str(output/"ros_logs")
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from diagnostic_msgs.msg import DiagnosticArray
    from companionbot_interfaces.msg import McuFrame
    from companionbot_control.runtime.embedded_link import EmbeddedFrame,EmbeddedParser
    from ament_index_python.packages import get_package_prefix
    binary=Path(get_package_prefix("companionbot_control"))/"lib/companionbot_control/mcu_bridge"
    rclpy.init(); probe=rclpy.create_node("embedded_bridge_probe",namespace="/embedded_acceptance")
    pub=probe.create_publisher(McuFrame,"mcu/tx",8)
    received=[]; diagnostics=[]
    subscriptions=[probe.create_subscription(McuFrame,"mcu/rx",lambda msg:received.append(msg),qos_profile_sensor_data),
                   probe.create_subscription(DiagnosticArray,"diagnostics",lambda msg:diagnostics.extend(msg.status),10)]
    checks={}; processes=[]
    try:
        for enabled in (False,True):
            master,slave=pty.openpty(); path=os.ttyname(slave)
            tty.setraw(slave)  # no line-discipline echo before the C++ port opens
            fcntl.fcntl(master,fcntl.F_SETFL,os.O_NONBLOCK)
            parser=EmbeddedParser(); transmitted=[]; raw_received=bytearray(); start=time.monotonic(); last_status=0.; tick=0; seq=0
            log=(output/f"bridge_tx_{enabled}.log").open("w")
            process=subprocess.Popen([str(binary),"--ros-args","-r","__ns:=/embedded_acceptance",
                "-p",f"device:={path}","-p",f"allow_command_tx:={str(enabled).lower()}"],stdout=log,stderr=subprocess.STDOUT)
            processes.append(process)
            def pump(duration,send_status=True,session=77):
                nonlocal last_status,tick,seq
                until=time.monotonic()+duration
                while time.monotonic()<until:
                    if process.poll() is not None: raise RuntimeError(f"rclcpp bridge exited: {process.returncode}")
                    current=time.monotonic()
                    if send_status and current-last_status>=.02:
                        tick=int((current-start)/.002); seq+=1
                        payload=struct.pack("<8I20f8I",0,0,0,35,0,0,0,0,*([0.]*18+[float("nan")]*2),*([0]*8))
                        os.write(master,EmbeddedFrame(128,session,seq,tick,payload).to_wire()); last_status=current
                    rclpy.spin_once(probe,timeout_sec=.005)
                    if select.select([master],[],[],0)[0]:
                        try: data=os.read(master,4096)
                        except BlockingIOError: data=b""
                        raw_received.extend(data)
                        transmitted.extend(parser.feed(data))
            def send(kind=2,payload=b"",session=77,deadline=None,valid_tick=None,sequence=10,clock_id=None):
                message=EmbeddedFrame(kind,session,sequence,tick+100 if valid_tick is None else valid_tick,
                    payload,time.monotonic_ns()+90000000 if deadline is None else deadline).to_ros()
                if clock_id is not None: message.host_clock_id=clock_id
                pub.publish(message)
            previous_count=len(received)
            discovery_deadline=time.monotonic()+10
            while (pub.get_subscription_count()!=1 or len(received)==previous_count) and time.monotonic()<discovery_deadline:
                pump(.05)
            pump(.10)
            checks[f"discovery_{enabled}"]=pub.get_subscription_count()==1 and any(m.session_id==77 for m in received)
            send();pump(.30)
            checks[f"command_gate_{enabled}"]=len(transmitted)==int(enabled)
            print(json.dumps({"enabled":enabled,"checks":checks,"transmitted":len(transmitted),
                "received":len(received),"host_monotonic_ns":time.monotonic_ns(),"tick":tick,
                "wire_bytes":raw_received.hex(),"parser_rejected":parser.state.rejected,
                "diagnostics":[{kv.key:kv.value for kv in s.values} for s in diagnostics[-2:]]}),flush=True)
            if enabled:
                if not transmitted:
                    pump(1.05)
                    raise RuntimeError("no heartbeat reached PTY; raw="+raw_received.hex()+"; rejected="+str(parser.state.rejected)+": " + str([
                        {kv.key:kv.value for kv in s.values} for s in diagnostics[-2:]]))
                assert transmitted[0].kind==2 and transmitted[0].session_id==77
                count=len(transmitted)
                send(session=78);send(deadline=time.monotonic_ns()-1);send(valid_tick=max(0,tick-10));send(clock_id="different-host")
                send(kind=3,payload=struct.pack("<d",float("nan")));pump(.15)
                checks["invalid_commands_rejected"]=len(transmitted)==count
                # Corruption and recovery use the same PTY/real parser, no Node mock.
                wire=bytearray(EmbeddedFrame(128,77,seq+1,tick,bytes(144)).to_wire())
                wire[8]^=1;os.write(master,wire);pump(.1)
                checks["crc_recovery"]=any(m.sequence_id>=seq-2 for m in received)
                pump(.32,send_status=False)
                count=len(transmitted);send();pump(.10,send_status=False)
                checks["stale_device_gate"]=len(transmitted)==count
                pump(.12,session=88);send(session=77);pump(.08,session=88)
                checks["old_session_rejected"]=len(transmitted)==count
                send(session=88,sequence=11);pump(.12,session=88)
                checks["new_session_explicit_command"]=len(transmitted)==count+1 and transmitted[-1].session_id==88
                pump(1.05,session=88)
                checks["no_synthetic_heartbeat"]=len(transmitted)==count+1
            process.send_signal(signal.SIGINT); code=process.wait(timeout=5)
            checks[f"clean_shutdown_{enabled}"]=code==0
            log.close();os.close(master);os.close(slave)
        # Device loss must propagate out of the worker and end the process with
        # failure, allowing required-process launch supervision to report it.
        master,slave=pty.openpty();tty.setraw(slave)
        with (output/"bridge_disconnect.log").open("w") as log:
            process=subprocess.Popen([str(binary),"--ros-args","-r","__ns:=/embedded_disconnect",
                "-p",f"device:={os.ttyname(slave)}"],stdout=log,stderr=subprocess.STDOUT)
            processes.append(process);time.sleep(.30)
            os.close(master);os.close(slave)
            checks["disconnect_exits_nonzero"]=process.wait(timeout=5)!=0
        for label,extra in (("missing_device",["-p","device:=/tmp/companionbot-nonexistent-port"]),
                            ("sim_time",["-p","use_sim_time:=true","-p","device:=/tmp/companionbot-nonexistent-port"])):
            with (output/f"bridge_{label}.log").open("w") as log:
                result=subprocess.run([str(binary),"--ros-args",*extra],stdout=log,stderr=subprocess.STDOUT,timeout=5)
            checks[label+"_rejected"]=result.returncode!=0
        values={kv.key:kv.value for status in diagnostics for kv in status.values}
        checks["diagnostics_have_counters"]={"rx","tx","rejected","framing_crc_rejected","queued"}<=values.keys()
        checks["crc_rejection_visible"]=int(values.get("framing_crc_rejected","0"))>0
        checks["clock_identity_rejection_visible"]=int(values.get("rejected_host_clock","0"))>0
        checks["physical_ready_false"]=values.get("hardware_execution_ready")=="false"
        report={"kind":"software_PTY_real_rclcpp_DDS", "hardware_tested":False,
            "checks":checks,"received_statuses":len(received),"diagnostic_values":values,"passed":all(checks.values())}
        (output/"summary.json").write_text(json.dumps(report,indent=2));print(json.dumps(report),flush=True)
        assert report["passed"],"Inspect PTY evidence; no hardware claim"
    except BaseException as exc:
        (output/"failure.json").write_text(json.dumps({"error":str(exc),"checks":checks,
            "received":len(received),"diagnostics":[{"message":s.message,
                "values":{kv.key:kv.value for kv in s.values}} for s in diagnostics]},indent=2))
        raise
    finally:
        for process in processes:
            if process.poll() is None:
                process.terminate()
                try: process.wait(timeout=5)
                except subprocess.TimeoutExpired: process.kill();process.wait()
        probe.destroy_node();rclpy.shutdown()

if __name__=="__main__":main()
