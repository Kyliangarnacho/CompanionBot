"""Actual ROS map reload, authority/failure and cleanup checks; no motion commands."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import numpy as np
from PIL import Image
import psutil
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
import yaml


def main():
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument('--map-prefix', type=Path, required=True)
    cli.add_argument('--output-dir', type=Path, required=True)
    args = cli.parse_args()
    out = args.output_dir.resolve();out.mkdir(parents=True,exist_ok=True)
    prefix = args.map_prefix.resolve()
    os.environ['ROS_DOMAIN_ID']='72'
    os.environ['ROS_LOG_DIR']=str(out/'ros_logs')
    rclpy.init();node=Node('slam_lifecycle_probe')
    maps, children = [], set()
    qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(OccupancyGrid,'/map',lambda m:maps.append(m),qos)
    node.create_subscription(TFMessage,'/tf',lambda m:children.update(t.child_frame_id for t in m.transforms),100)
    processes,handles=[],[]
    records={'passed':False}

    def launch(label, filename, options):
        maps.clear();children.clear()
        handle=(out/(label+'.txt')).open('w');handles.append(handle)
        p=subprocess.Popen([sys.executable,'/opt/ros/jazzy/bin/ros2','launch','companionbot_bringup',
            filename,*options],stdout=handle,stderr=subprocess.STDOUT,start_new_session=True)
        processes.append(p)
        return p

    def spin_until(predicate, timeout=25):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.02)
            if predicate():return
        raise TimeoutError('SLAM lifecycle deadline')

    def finish(p, label, *, failure=False, interrupt=False):
        if interrupt and p.poll() is None:os.killpg(p.pid,signal.SIGINT)
        spin_until(lambda:p.poll() is not None)
        text=(out/(label+'.txt')).read_text()
        assert (p.returncode!=0) if failure else (p.returncode==0), text
        spin_until(lambda: not any(name in ('slam_toolbox','spatial_sim','spatial_state','tf_authority',
            'robot_state_publisher','map_server','map_review_lifecycle') for name,ns in node.get_node_names_and_namespaces()))
        records[label]={'exit_code':p.returncode,'clean_graph':True}
        return text

    try:
        p=launch('static_reload','map_review.launch.py',['map_yaml:='+str(prefix.with_suffix('.yaml'))])
        spin_until(lambda:bool(maps) or p.poll() is not None)
        assert maps, (out/'static_reload.txt').read_text()
        m=maps[-1]
        metadata=yaml.safe_load(prefix.with_suffix('.yaml').read_text())
        image=np.array(Image.open(prefix.parent/metadata['image']))[::-1]
        occupancy=1.-image.astype(float)/255
        expected=np.full(image.shape,-1)
        expected[occupancy<metadata['free_thresh']]=0
        expected[occupancy>metadata['occupied_thresh']]=100
        actual=np.asarray(m.data).reshape(m.info.height,m.info.width)
        assert np.array_equal(expected,actual), 'standard map_server differs from saved trinary image'
        assert not children and not node.get_publishers_info_by_topic('/tf'), 'static map server must not localize'
        finish(p,'static_reload',interrupt=True)
        records['static_reload'].update({'cells_exact':int(actual.size),'tf_published':False})

        p=launch('graph_reload','slam_simulation.launch.py',['scenario:=balance','duration_s:=4.0',
            'map_file_name:='+str(prefix),'output_dir:='+str(out/'graph_reload'),'finish_grace_s:=-1.0'])
        spin_until(lambda:bool(maps) and 'odom' in children)
        assert maps[-1].info.width>=100 and maps[-1].info.height>=100
        spin_until(lambda:(out/'graph_reload/frozen_result_posthoc.json').exists())
        text=finish(p,'graph_reload',interrupt=True)
        assert 'Registering sensor' in text and 'Failed to read' not in text, text
        records['graph_reload']['loaded_saved_graph']=True

        for label, options, reason in [
            ('imu_dropout',['imu_dropout_at_s:=3.0'],'sensor_invalid_or_stale'),
            ('duplicate_map_authority',[],'tf_multiple_authorities:odom'),
            ('slam_process_failure',[],'Required runtime exited with code'),
        ]:
            p=launch(label,'slam_simulation.launch.py',['scenario:=balance','duration_s:=20.0',
                'output_dir:='+str(out/label),*options])
            spin_until(lambda:'odom' in children or p.poll() is not None)
            assert p.poll() is None, (out/(label+'.txt')).read_text()
            if label=='duplicate_map_authority':
                publisher=node.create_publisher(TFMessage,'/tf',10)
                bad=TransformStamped();bad.header.frame_id='map';bad.child_frame_id='odom';bad.transform.rotation.w=1.
                for _ in range(20):
                    publisher.publish(TFMessage(transforms=[bad]));rclpy.spin_once(node,timeout_sec=.02)
                node.destroy_publisher(publisher)
            elif label=='slam_process_failure':
                slam=[c for c in psutil.Process(p.pid).children(recursive=True) if c.name()=='async_slam_toolbox_node']
                assert len(slam)==1
                slam[0].kill()
            text=finish(p,label,failure=True)
            if label=='imu_dropout':
                assert reason in text or 'imu_packet_sample_gap_or_reversal' in text,text
            else:assert reason in text,text
            records[label]['expected_reason']=reason

        p=launch('missing_graph','slam.launch.py',['map_file_name:='+str(out/'absent')])
        text=finish(p,'missing_graph',failure=True)
        assert 'nonempty .posegraph and .data' in text,text
        records['passed']=True
    except BaseException as error:
        records['error']=repr(error)
        raise
    finally:
        for p in processes:
            if p.poll() is None:
                os.killpg(p.pid,signal.SIGINT)
                try:p.wait(timeout=12)
                except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=8)
        for h in handles:h.close()
        (out/'summary.json').write_text(json.dumps(records,indent=2)+'\n')
        node.destroy_node();rclpy.try_shutdown()
    print(json.dumps(records),flush=True)


if __name__=='__main__':main()
