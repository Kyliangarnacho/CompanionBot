"""Low-frequency resource evidence for this diagnostic's own PID tree.

No global /proc scan and no argv/cmdline read. Roots must be this process or a
subprocess.Popen it directly created. Discovery follows only known processes'
per-thread children files. CPU counters have kernel tick resolution; sampling
can miss short-lived children/threads and the final counters of exited tasks.
Thread RSS is deliberately absent: address-space memory belongs to a process.

Example inside an existing diagnostic (not a new robot entry point)::

    child = subprocess.Popen(command, ...)
    with RuntimeResourceSampler(output / 'resources', root=child, label='nav2'):
        child.wait()

Use root=None to sample this diagnostic process and all descendants it starts.
The monitor never sends signals, waits on or reaps the monitored children.
"""
from __future__ import annotations

from collections import Counter
from dataclasses import dataclass
import heapq
import json
import math
import os
from pathlib import Path
import subprocess
import threading
import time


@dataclass(frozen=True)
class _Stat:
    pid: int
    comm: str
    state: str
    ppid: int
    start_ticks: int
    user_ticks: int
    system_ticks: int
    child_user_ticks: int
    child_system_ticks: int
    threads: int

    @property
    def identity(self):
        return f'{self.pid}:{self.start_ticks}'


def _stat(path):
    raw=Path(path).read_text()
    left,right=raw.find('('),raw.rfind(')')
    if left<0 or right<left: raise ValueError('malformed_proc_stat')
    fields=raw[right+2:].split()
    return _Stat(int(raw[:left].strip()),raw[left+1:right],fields[0],int(fields[1]),
        int(fields[19]),int(fields[11]),int(fields[12]),int(fields[13]),int(fields[14]),int(fields[17]))


def _status(path):
    result={}
    for line in Path(path).read_text().splitlines():
        name,sep,value=line.partition(':')
        if not sep:continue
        words=value.split()
        if name in ('VmRSS','VmHWM','VmSize','VmPeak') and len(words)==2 and words[1]=='kB':
            result[name]=int(words[0])*1024
        elif name in ('voluntary_ctxt_switches','nonvoluntary_ctxt_switches'):
            result[name]=int(words[0])
    return result


class RuntimeResourceSampler:
    """Bounded bookkeeping, streamed samples, one standard-library worker.

    add_child() accepts another Popen created by this Python process; no caller
    can accidentally ask this helper to enumerate an unrelated user's tree.
    The caller remains the sole lifecycle owner of all monitored children.
    """
    def __init__(self,output_dir,*,root=None,label='experiment',interval_s=.5,
                 max_samples=10000,max_processes=128,max_threads_per_process=256,
                 max_thread_identities=4096):
        if (not math.isfinite(interval_s) or not .1<=interval_s<=10.
                or any(isinstance(v,bool) or not isinstance(v,int) or v<1 for v in
                    (max_samples,max_processes,max_threads_per_process,max_thread_identities))):
            raise ValueError('invalid_resource_sampler_bounds')
        self.output_dir=Path(output_dir);self.output_dir.mkdir(parents=True,exist_ok=True)
        if (self.output_dir/'samples.jsonl').exists() or (self.output_dir/'summary.json').exists():
            raise FileExistsError('preserve_resource_evidence_choose_new_output_dir')
        self.interval_s,self.max_samples=interval_s,max_samples
        self.max_processes,self.max_threads_per_process=max_processes,max_threads_per_process
        self.max_thread_identities=max_thread_identities
        self.ticks_per_second=os.sysconf('SC_CLK_TCK')
        self.clock='host_perf_counter'
        self._roots={};self._processes={};self._threads={};self._errors=Counter()
        self._read_errors=Counter();self._root_labels={}
        self._lock=threading.RLock();self._stop=threading.Event();self._worker=None
        self._stream=(self.output_dir/'samples.jsonl').open('x')
        self._closed=False;self._started=None;self._ended=None;self.sample_count=0
        self._monitor_wall=0.;self._monitor_cpu=0.;self._monitor_wall_max=0.
        self._max_tree_rss=0;self._max_live_processes=0;self._max_live_threads=0
        self._sample_capacity_exhausted=False;self._monitor_native_tid=None
        self._fatal_error=None
        self.report=None
        try:
            if root is None:self._add_root(os.getpid(),str(label),allow_self=True)
            else:self.add_child(root,label=label)
        except BaseException:
            self._stream.close();raise

    def _add_root(self,pid,label,*,allow_self=False):
        value=_stat(f'/proc/{pid}/stat')
        if (pid==os.getpid() and not allow_self) or (pid!=os.getpid() and value.ppid!=os.getpid()):
            raise ValueError('resource_root_must_be_self_or_direct_owned_child')
        with self._lock:
            if self._closed:raise RuntimeError('resource_sampler_closed')
            self._roots[value.identity]=value.pid;self._root_labels[value.identity]=label
            if not self._remember_process(value,None,label,time.perf_counter()):
                raise RuntimeError('resource_root_capacity_exhausted')

    def add_child(self,process,*,label='owned-child'):
        if not isinstance(process,subprocess.Popen):
            raise TypeError('resource_root_requires_owned_subprocess_Popen')
        self._add_root(process.pid,str(label))

    def _error(self,kind,error):
        self._read_errors[f'{kind}:{type(error).__name__}']+=1

    def _remember_process(self,value,parent,label,now):
        if value.identity in self._processes:return True
        if len(self._processes)>=self.max_processes:
            self._errors['process_identity_capacity_rejections']+=1;return False
        self._processes[value.identity]=dict(pid=value.pid,start_ticks=value.start_ticks,
            comm=value.comm,discovered_from=parent,label=label,first_discovered_host_s=now,
            first=None,last=None,last_sample_host_s=None,terminal_observed=False,
            disappeared=False,samples=0,peak_rss_bytes=0,kernel_hwm_bytes=0,
            max_reported_threads=0,cpu_percent_interval_max=0.)
        try:self._processes[value.identity]['executable']=os.readlink(f'/proc/{value.pid}/exe')
        except OSError as error:self._error('owned_executable',error)
        return True

    def _discover(self,now):
        # Iterate only known owned identities; child discovery never scans /proc.
        pending=list(self._processes);seen=set()
        while pending:
            identity=pending.pop()
            if identity in seen:continue
            seen.add(identity);entry=self._processes[identity]
            if entry['terminal_observed'] or entry['disappeared']:continue
            pid=entry['pid']
            try:
                parent=_stat(f'/proc/{pid}/stat')
                if parent.identity!=identity:
                    entry['disappeared']=True;self._errors['pid_reuse_rejections']+=1;continue
                tasks=heapq.nsmallest(self.max_threads_per_process,
                    Path(f'/proc/{pid}/task').iterdir(),key=lambda p:int(p.name))
                if parent.threads>self.max_threads_per_process:
                    self._errors['thread_enumeration_capacity_exceeded']+=parent.threads-self.max_threads_per_process
                for task in tasks:
                    try:children=(task/'children').read_text().split()
                    except (OSError,ValueError) as error:self._error('children',error);continue
                    for child_pid in children:
                        try:child=_stat(f'/proc/{int(child_pid)}/stat')
                        except (OSError,ValueError) as error:self._error('child_stat',error);continue
                        if child.ppid!=pid:
                            self._errors['discovery_parent_changed_rejections']+=1;continue
                        if self._remember_process(child,identity,entry['label'],now):pending.append(child.identity)
            except (OSError,ValueError) as error:
                self._error('discovery',error)
                if isinstance(error,(FileNotFoundError,ProcessLookupError)):entry['disappeared']=True

    def _counter_row(self,value,status):
        return dict(pid=value.pid,identity=value.identity,comm=value.comm,state=value.state,
            start_ticks=value.start_ticks,user_cpu_s=value.user_ticks/self.ticks_per_second,
            system_cpu_s=value.system_ticks/self.ticks_per_second,
            cpu_s=(value.user_ticks+value.system_ticks)/self.ticks_per_second,
            voluntary_switches=status.get('voluntary_ctxt_switches'),
            involuntary_switches=status.get('nonvoluntary_ctxt_switches'))

    def _observe_entry(self,entry,row,now):
        previous=entry['last'];previous_time=entry['last_sample_host_s']
        if entry['first'] is None:
            entry['first']=dict(row);entry['first_sample_host_s']=now
        if previous is not None and now>previous_time:
            delta=row['cpu_s']-previous['cpu_s']
            if delta>=0:
                row['interval_cpu_percent_one_core']=100*delta/(now-previous_time)
                entry['cpu_percent_interval_max']=max(entry['cpu_percent_interval_max'],row['interval_cpu_percent_one_core'])
        entry['last']=dict(row);entry['last_sample_host_s']=now;entry['samples']+=1
        if row['state'] in ('Z','X','x'):entry['terminal_observed']=True

    def sample(self):
        """One nonblocking /proc sweep; optional manual use instead of start()."""
        before,before_cpu=time.perf_counter(),time.thread_time()
        with self._lock:
            if self._closed:raise RuntimeError('resource_sampler_closed')
            if self.sample_count>=self.max_samples:
                self._sample_capacity_exhausted=True;self._stop.set();return None
            if self._started is None:self._started=before
            self._discover(before)
            rows=[];live_threads=0
            for identity,entry in list(self._processes.items()):
                if entry['terminal_observed'] or entry['disappeared']:continue
                pid=entry['pid']
                try:
                    value=_stat(f'/proc/{pid}/stat')
                    if value.identity!=identity:
                        entry['disappeared']=True;self._errors['pid_reuse_rejections']+=1;continue
                    status=_status(f'/proc/{pid}/status')
                    row=self._counter_row(value,status)
                    row.update(rss_bytes=status.get('VmRSS',0),kernel_peak_rss_bytes=status.get('VmHWM',0),
                        virtual_bytes=status.get('VmSize',0),kernel_peak_virtual_bytes=status.get('VmPeak',0),
                        reported_threads=value.threads,ppid=value.ppid,
                        reaped_child_cpu_s=(value.child_user_ticks+value.child_system_ticks)/self.ticks_per_second)
                    self._observe_entry(entry,row,before)
                    entry['peak_rss_bytes']=max(entry['peak_rss_bytes'],row['rss_bytes'])
                    entry['kernel_hwm_bytes']=max(entry['kernel_hwm_bytes'],row['kernel_peak_rss_bytes'])
                    entry['max_reported_threads']=max(entry['max_reported_threads'],value.threads)
                    thread_rows=[]
                    tasks=heapq.nsmallest(self.max_threads_per_process,
                        Path(f'/proc/{pid}/task').iterdir(),key=lambda p:int(p.name))
                    seen_threads=set()
                    for task in tasks:
                        try:
                            thread=_stat(task/'stat');thread_status=_status(task/'status')
                            key=identity+'/'+thread.identity
                            if key not in self._threads:
                                if len(self._threads)>=self.max_thread_identities:
                                    self._errors['thread_identity_capacity_rejections']+=1;continue
                                self._threads[key]=dict(tid=thread.pid,start_ticks=thread.start_ticks,
                                    process_identity=identity,comm=thread.comm,first=None,last=None,
                                    last_sample_host_s=None,samples=0,terminal_observed=False,
                                    disappeared=False,cpu_percent_interval_max=0.)
                            item=self._counter_row(thread,thread_status);item['tid']=item.pop('pid')
                            self._threads[key]['missing_last_process_sample']=False
                            self._observe_entry(self._threads[key],item,before);thread_rows.append(item);seen_threads.add(key)
                        except (OSError,ValueError) as error:self._error('thread_sample',error)
                    for key,thread_entry in self._threads.items():
                        if thread_entry['process_identity']==identity and key not in seen_threads:
                            thread_entry['missing_last_process_sample']=True
                    row['threads']=thread_rows
                    row['live_thread_switch_sum']={field:sum(t[field] or 0 for t in thread_rows)
                        for field in ('voluntary_switches','involuntary_switches')}
                    live_threads+=len(thread_rows);rows.append(row)
                except (OSError,ValueError) as error:
                    self._error('process_sample',error)
                    if isinstance(error,(FileNotFoundError,ProcessLookupError)):entry['disappeared']=True
            tree_rss=sum(row['rss_bytes'] for row in rows)
            self._max_tree_rss=max(self._max_tree_rss,tree_rss)
            self._max_live_processes=max(self._max_live_processes,len(rows))
            self._max_live_threads=max(self._max_live_threads,live_threads)
            frame=dict(sample_index=self.sample_count,host_time_s=before,clock_domain=self.clock,
                process_count=len(rows),thread_count=live_threads,sum_process_rss_bytes=tree_rss,processes=rows)
            self._stream.write(json.dumps(frame,separators=(',',':'))+'\n');self._stream.flush()
            self.sample_count+=1;self._ended=before
            wall=time.perf_counter()-before;cpu=time.thread_time()-before_cpu
            self._monitor_wall+=wall;self._monitor_cpu+=cpu;self._monitor_wall_max=max(self._monitor_wall_max,wall)
            return frame

    def _run(self):
        self._monitor_native_tid=threading.get_native_id()
        while not self._stop.is_set():
            try:self.sample()
            except BaseException as error:
                with self._lock:
                    self._errors['monitor_fatal:'+type(error).__name__]+=1
                    self._fatal_error=repr(error)
                self._stop.set();break
            self._stop.wait(self.interval_s)

    def start(self):
        with self._lock:
            if self._closed:raise RuntimeError('resource_sampler_closed')
            if self._worker is not None:raise RuntimeError('resource_sampler_already_started')
            self._worker=threading.Thread(target=self._run,name='runtime-resource-sampler')
            self._worker.start()
        return self

    @staticmethod
    def _summary_entry(entry):
        result={k:v for k,v in entry.items() if k not in ('first','last')}
        first,last=entry.get('first'),entry.get('last')
        result['first_counters']=first;result['last_counters']=last
        if first is not None and last is not None:
            result['observed_cpu_delta_s']=last['cpu_s']-first['cpu_s']
            result['last_cpu_since_process_or_thread_start_s']=last['cpu_s']
            duration=entry['last_sample_host_s']-entry['first_sample_host_s']
            result['observed_window_s']=duration
            result['cpu_percent_one_core']=100*result['observed_cpu_delta_s']/duration if duration>0 else None
            for field in ('voluntary_switches','involuntary_switches'):
                result['observed_'+field+'_delta']=(last[field]-first[field]
                    if last[field] is not None and first[field] is not None else None)
        return result

    def close(self):
        with self._lock:
            if self._closed:return self.report
            self._stop.set()
        if self._worker is not None:
            self._worker.join(timeout=3.)
            if self._worker.is_alive():raise RuntimeError('resource_sampler_worker_failed_to_stop')
        # Capture the normal shutdown tail if the child still has /proc entries.
        if not self._sample_capacity_exhausted:self.sample()
        with self._lock:
            self._closed=True;self._stream.close()
            report=dict(schema_version=1,scope='WSL/Linux /proc sampled host resources; not target hardware performance',
                clock_domain=self.clock,interval_s=self.interval_s,samples=self.sample_count,
                first_sample_host_s=self._started,last_sample_host_s=self._ended,
                kernel_clock_ticks_per_second=self.ticks_per_second,
                root_identities=self._root_labels,max_live_processes=self._max_live_processes,
                max_live_threads=self._max_live_threads,sampled_peak_sum_process_rss_bytes=self._max_tree_rss,
                processes={key:self._summary_entry(value) for key,value in self._processes.items()},
                threads={key:self._summary_entry(value) for key,value in self._threads.items()},
                errors=dict(self._errors),proc_read_errors=dict(self._read_errors),
                fatal_error=self._fatal_error,
                capacity=dict(max_samples=self.max_samples,max_processes=self.max_processes,
                    max_threads_per_process=self.max_threads_per_process,max_thread_identities=self.max_thread_identities,
                    sample_capacity_exhausted=self._sample_capacity_exhausted),
                monitor=dict(native_tid=self._monitor_native_tid,total_sample_wall_s=self._monitor_wall,
                    total_sample_thread_cpu_s=self._monitor_cpu,max_sample_wall_s=self._monitor_wall_max),
                semantics=dict(cpu_percent='100% equals one logical core; counters quantized to kernel ticks',
                    process_context_switches='status fields belong to process leader only; live_thread_switch_sum is per sample, not whole-process lifetime total',
                    thread_context_switches='per observed TID/starttime; missed short-lived threads and exit tails are lower bounds',
                    memory='process RSS/HWM only; no per-thread RSS; sum of RSS can double-count shared pages',
                    high_water_memory='kernel VmHWM is since process start, not limited to the sampled window',
                    discovery='only known owned PID /task/*/children; short-lived/orphaned-before-discovery descendants can be missed',
                    privacy='no argv/cmdline/environment or unrelated process-tree scan',
                    profiler_overhead='monitor itself is included when sampling current diagnostic; monitor native TID and wall/thread CPU reported'),
                gil_wait='not measured by /proc; CPU use and context switches are not GIL-wait measurements',
                hardware_execution_ready=False)
            (self.output_dir/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
            self.report=report
        return report

    def __enter__(self):return self.start()
    def __exit__(self,exc_type,exc,tb):self.close()
