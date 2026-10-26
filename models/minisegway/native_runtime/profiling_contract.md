# 统一主机资源采样合同

`tools/verification/runtime_resources.py` 是有限诊断的辅助类，不是新的运行 / Demo 入口，也不持有运动权限。

```python
from runtime_resources import RuntimeResourceSampler

child = subprocess.Popen(command, ...)
with RuntimeResourceSampler(output / 'resources', root=child,
                            label='navigation-launch', interval_s=.5) as sampler:
    # Existing diagnostic remains the sole child lifecycle owner.
    child.wait()
report = sampler.report
```

省略 `root` 时采本诊断进程及其后来生成的后代。`add_child(Popen, label=...)` 可添加本 Python 进程直接创建的其他 root；传入任意 unrelated PID 不被接受。

只读已知 PID 的 `/proc/<pid>/stat`、`status`、`exe` 与 `task/<tid>/{stat,status,children}`。不扫描全 `/proc`，不采 argv / cmdline / environment，不发送信号，不 wait / reap child。每个身份包含 PID/TID + kernel start ticks，防止退出后的 PID 复用被归入原任务。非主线程创建的子进程也从该线程 children 文件发现。

原始 `samples.jsonl` 流式落盘；有限 summary 按 process/thread identity 聚合。默认 10,000 samples、128 process identities、每进程256线程、4,096 thread identities；任何容量边界或读失败都有计数，不能据不完整覆盖声称全系统资源通过。该工具仅提供观测，不更改机器人运行门禁。

- CPU：kernel utime/stime，`100%` 是一个 logical core。`SC_CLK_TCK` 单独记录；短窗口小于一个 tick 的 CPU 为量化值。逐身份保留第一次/最后一次实际观测和 delta；不把进程出生前或漏采的尾部伪称已观测。
- 内存：只报告 process RSS、kernel VmHWM / VmPeak 与整个采样窗口的 simultaneous RSS sum peak。线程共享地址空间，不重复报告每线程 RSS。VmHWM 覆盖 process lifetime，RSS sum 可能重复计共享页，不等于 PSS 或独占物理内存。
- Context switch：`/proc/<pid>/status` 的值是 leader thread 的计数；每个 TID 独立记录，不能把 leader 当 whole-process lifetime switch total。live-thread sum 与每身份 delta 都保留；短命线程 / exit tail 可能漏采，因此是实际采样覆盖的下界。
- Discovery：只追本诊断授权的 tree。采样间出生并退出、或发现前已经 orphan 的子任务可能无法发现；不会为了补齐而扫描其他用户进程。
- 开销：sampler 使用一个标准 `threading.Thread`，记录自己的 native TID、每次采样 wall / thread-CPU 总量与最大wall。采本诊断自身时该开销也计入主进程，不能称零成本 profiler。
- `/proc` CPU、context switch 都不是 GIL wait 指标。

成熟 py-spy 的官方 CLI 与实现已阅读：[CLI config](https://github.com/benfred/py-spy/blob/v0.4.1/src/config.rs)、[Python thread/GIL access](https://github.com/benfred/py-spy/blob/v0.4.1/src/python_spy.rs)、[官方 GIL 与权限说明](https://github.com/benfred/py-spy#how-does-gil-detection-work)。`--gil` 仅保留采样瞬间持有 GIL 的线程栈；会漏掉释放 GIL 的 native 执行，不等于每次 GIL 获取等待时间。blocking 采样会暂停目标，nonblocking 可出现栈不一致；性能证据必须标为独立 instrumentation run，不与 uninstrumented baseline 混比。尚未在此附件中宣称安装 / 采样成功；实际权限与软件进程试验结果后续由原始证据补全。

全部结果为 WSL/Linux host software 观测，不是 Pi5 / STM32 认证，`hardware_execution_ready=false`。

## 实际小夹具与权限结果

2026-10-24 同 `.venv/bin/python`（3.12.3、pip24.0）安装官方 `py-spy==0.4.1` manylinux wheel；安装报告 / wheel SHA256 位于 `results/session_20261023/profiling/py_spy_install.json`。它是诊断依赖，不成为正式机器人运行依赖。

资源 helper 的三个 pure fixtures 实测通过：非主线程生成 grandchild 的 tree discovery、CPU / RSS / thread 覆盖、无 argv 泄漏、拒绝 unrelated root / 覆盖旧证据、sample capacity、水位身份与带空格/右括号 kernel comm 解析。证据 `profiling/resources_fixture_attempt2/{pytest.txt,junit.xml}`。首次直接 pytest 被全局 ROS `launch_testing` 插件牵连收集其他 native tests、因未 source overlay 缺 shared library 而失败；保留 `resources_fixture/`，第二次只为该标准库 helper 禁用插件自动加载，未改测试预期。

自启动有限 Python / MuJoCo tiny fixture 的 `py-spy record --gil --format speedscope --rate100 --duration2`：默认 sandbox `EPERM / Failed to suspend process`，保留 stderr；仅对同一自启动夹具的一次授权沙箱外采样成功，186 samples / 0 sampling errors、2 thread profiles。未改变 `ptrace_scope`（读值1），未 attach 其他进程、未读取 locals。原始 speedscope 与摘要见 `profiling/py_spy_owned_child.json`、`py_spy_feasibility_summary.json`。

这个结果证明主机诊断工具可以采到 GIL-holder 栈；没有测得每次 GIL wait，也没有证明任何 CompanionBot 模块已经因此达到实时性能门禁。

## 已完成的有限原生返回诊断

`results/session_20261023/profiling/gil_return_probe/`保存原协议、独立诊断绑定及两组20s真实MuJoCo夹具。探针借用同一个正式NavigationRuntime实例、复用pybind11的GIL RAII；不增加生产状态机。normal/cancel-pending各7500tick parity与异常/容量验证通过后，仅执行无额外worker、固定4ms忙/6ms等待Python线程两个条件。`physics_comparison_attempt1.json`记录各10000step及完整物理/参考/权限逐值一致；返回→实际恢复GIL的wall区间P99分别0.271μs和4.029ms，后者110条超过2ms，全部与已记录busy区间相交。

该区间含恢复、OS调度及探针，不能称纯GIL等待或无插桩速度。thread CPU采样包围wall采样，两窗口不完全相同；sub-μs下CPU>wall不能作为时钟故障证据，也不做相减估计锁等待。两次unpaced物理夹具控制门禁通过、零underrun，只证明对应source-time执行；未认证2ms墙钟deadline、实际AI争用或Pi性能。原协议、源码/ELF、所有记录与两条件均保留，不据此继续微调生产实现。
