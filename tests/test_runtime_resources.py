"""Owned PID-tree resource evidence, independent of ROS or robot algorithms."""
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import pytest

from tools.verification.runtime_resources import RuntimeResourceSampler, _stat


def test_owned_tree_threads_memory_and_reuse_identity(tmp_path):
    script = r'''
import subprocess,sys,threading,time
memory=bytearray(8*1024*1024)
children=[]
def fork_from_nonleader():
    child=subprocess.Popen([sys.executable,'-c','import time; data=bytearray(4*1024*1024); time.sleep(.8)'])
    children.append(child)
    child.wait()
def cpu_work():
    deadline=time.monotonic()+.5
    value=0
    while time.monotonic()<deadline:value+=1
threads=[threading.Thread(target=fork_from_nonleader,name='fixture-parent'),threading.Thread(target=cpu_work,name='fixture-cpu')]
for thread in threads:thread.start()
for thread in threads:thread.join()
time.sleep(.15)
'''
    child=subprocess.Popen([sys.executable,'-c',script])
    try:
        sampler=RuntimeResourceSampler(tmp_path/'resources',root=child,interval_s=.1,label='fixture')
        with sampler:
            child.wait(timeout=4.)
        report=json.loads((tmp_path/'resources/summary.json').read_text())
        assert len(report['processes'])==2  # root and grandchild from non-main thread
        assert report['max_live_threads']>=4
        assert report['sampled_peak_sum_process_rss_bytes']>12*1024*1024
        assert report['errors']=={}
        assert any(p.get('observed_cpu_delta_s',0)>.05 for p in report['processes'].values())
        assert report['monitor']['total_sample_thread_cpu_s']>=0
        assert all(':' in key for key in report['processes'])
        assert all('/' in key for key in report['threads'])
        samples=(tmp_path/'resources/samples.jsonl').read_text()
        assert 'cmdline' not in samples and 'argv' not in samples
        assert 'memory=bytearray' not in samples
    finally:
        if child.poll() is None:child.terminate();child.wait(timeout=2.)


def test_unrelated_root_and_evidence_overwrite_rejected(tmp_path):
    # Parent cannot be this process's directly-created child, even if same UID.
    fake=subprocess.Popen.__new__(subprocess.Popen)
    fake.pid=os.getppid()
    fake._child_created=False
    with pytest.raises(ValueError,match='owned_child'):
        RuntimeResourceSampler(tmp_path/'rejected',root=fake)
    sampler=RuntimeResourceSampler(tmp_path/'self',max_samples=1)
    sampler.sample();assert sampler.sample() is None
    report=sampler.close()
    assert report['capacity']['sample_capacity_exhausted'] is True
    assert report['samples']==1
    with pytest.raises(FileExistsError,match='preserve_resource_evidence'):
        RuntimeResourceSampler(tmp_path/'self')


def test_proc_stat_comm_can_contain_spaces_and_parenthesis(tmp_path):
    # Linux comm permits ')' and spaces. Use the last ')' to find counters.
    real=Path('/proc/self/stat').read_text();left,right=real.find('('),real.rfind(')')
    file=tmp_path/'stat';file.write_text(real[:left]+'(fixture ) with spaces)'+real[right+1:])
    parsed=_stat(file)
    assert parsed.pid==os.getpid() and parsed.comm=='fixture ) with spaces'
    assert parsed.start_ticks>0
