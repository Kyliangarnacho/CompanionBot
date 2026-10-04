"""Small local Tk interaction panel; widgets remain on the user's main thread.

The async Agent/Stage 7 runtime communicates through queues, not Tk calls.
Importing this module or testing the bridge never opens a window.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import queue
import threading


@dataclass
class DemoBridge:
    commands: queue.Queue = field(default_factory=lambda: queue.Queue(maxsize=64))
    updates: queue.Queue = field(default_factory=lambda: queue.Queue(maxsize=512))
    preview_line: str = "Agent SLEEP | camera starting | execution=fake"

    def submit(self, text: str):
        if text.strip():
            self._put_latest(self.commands, text.strip())

    def emit(self, kind: str, data):
        self._put_latest(self.updates, (kind, data))

    @staticmethod
    def _put_latest(target, value):
        while True:
            try:
                target.put_nowait(value)
                return
            except queue.Full:
                try:
                    target.get_nowait()
                except queue.Empty:
                    pass


def input_feedback(text: str, *, state: str, valid_voice=True,
                   new_turn=False, response=None, local=False, source="text", gate=None) -> dict:
    if not valid_voice:
        reasons = {"invalid_voice_format": "语音数据格式无效", "asr_not_final": "识别尚未完成",
                   "recognition_state_changed": "识别属于上一轮交互状态", "audio_stale_or_future": "音频事件过期或时间异常",
                   "playback_active": "播放中只接受带名字的打断口令，或使用按钮/文字", "echo_guard_active": "播报后的回声保护期",
                   "invalid_playback_control": "播放中只接受带名字的完整打断口令",
                   "unscored_behavior_requires_wake_prefix": "无置信度的语音行为请求须带唤醒词前缀",
                   "confidence_below_threshold": "识别置信度不足", "control_requires_wake_prefix": "语音机器人控制需要唤醒词前缀"}
        reason = reasons.get((gate or {}).get("reason"), "语音 confidence / 时间 / 播放保护未通过")
        if gate and gate.get("reason") == "confidence_below_threshold":
            reason += f"：{gate['score']:.2f} < {gate['threshold']:.2f}，请重说或使用文字"
        status = "REJECT"
    elif new_turn:
        status, reason = "ACCEPT", "已提交 Agent，正在生成"
    elif local:
        status, reason = "ACCEPT", "本地指令已接收；行为执行结果另见 Supervisor"
    elif response and response.startswith(("视觉请求未发送", "正在回答")):
        status, reason = "REJECT", response
    elif state == "SLEEP":
        status, reason = "IGNORED", "未唤醒：仅本地监听，不调用 Agent"
    else:
        status, reason = "ACCEPT", response or "本地输入已接收"
    return {"text": text, "status": status, "reason": reason, "source": source, "gate": gate or {}}


def format_metrics(payload: dict) -> str:
    values = payload["data"]
    def seconds(key):
        value = values.get(key)
        return f"{value:.2f}s" if isinstance(value, (float, int)) else "待确认"
    return (f"{'播报结束' if payload['phase'] == 'speech_complete' else '生成结束'} · "
            f"首文本 {seconds('first_text_s')} / 首语音提交 {seconds('first_speech_s')} / "
            f"完成 {seconds('interaction_wall_s' if payload['phase'] == 'speech_complete' else 'wall_s')} · "
            f"请求 {values.get('requests', 0)} / Tokens {values.get('input_tokens', 0)}+{values.get('output_tokens', 0)}")


def launch_panel(bridge: DemoBridge, run_worker):
    # Only the user's explicit demo launch constructs Tk. Codex headless checks
    # exercise the queues/state below without opening any desktop windows.
    import tkinter as tk
    from tkinter import ttk
    from tkinter.scrolledtext import ScrolledText

    root = tk.Tk()
    root.title("CompanionBot · Stage 8 交互")
    root.geometry("850x720")
    root.minsize(680, 560)
    frame = ttk.Frame(root, padding=12)
    frame.pack(fill="both", expand=True)
    frame.columnconfigure(0, weight=1)
    frame.rowconfigure(5, weight=1)
    state = tk.StringVar(value="正在打开 C920 · Agent SLEEP")
    device = tk.StringVar(value="麦克风等待相机就绪；默认按名称选择内置 Realtek")
    perception = tk.StringVar(value="感知：等待加载 Detector / Depth / ReID")
    asr = tk.StringVar(value="本地 ASR：等待音频；“你好小柒”允许同音名字变体")
    acceptance = tk.StringVar(value="说“你好小柒”唤醒；可输入文字。执行后端为 fake，实体运动未接通。")
    ttk.Label(frame, textvariable=state, font=("Microsoft YaHei UI", 13, "bold"), wraplength=800).grid(row=0, column=0, sticky="w")
    ttk.Label(frame, textvariable=device, wraplength=800).grid(row=1, column=0, sticky="w", pady=(5, 8))
    ttk.Label(frame, textvariable=perception, wraplength=800).grid(row=2, column=0, sticky="w")
    ttk.Label(frame, textvariable=asr, wraplength=800).grid(row=3, column=0, sticky="w")
    ttk.Label(frame, textvariable=acceptance, wraplength=800).grid(row=4, column=0, sticky="w", pady=6)
    output = ScrolledText(frame, wrap="word", height=1, font=("Microsoft YaHei UI", 11), state="disabled")
    output.grid(row=5, column=0, sticky="nsew")
    entry = ttk.Entry(frame, font=("Microsoft YaHei UI", 12))
    entry.grid(row=6, column=0, sticky="ew", pady=8)
    def submit(_event=None):
        text = entry.get().strip()
        if text:
            acceptance.set("已接收输入，等待本地门控确认…")
            bridge.submit(text)
            entry.delete(0, "end")
        return "break"
    entry.bind("<Return>", submit)
    buttons = ttk.Frame(frame)
    buttons.grid(row=7, column=0, sticky="ew")
    buttons.columnconfigure(tuple(range(8)), weight=1, uniform="actions")
    ttk.Button(buttons, text="发送", width=1, command=submit).grid(row=0, column=0, sticky="ew", padx=2)
    for column, (label, command) in enumerate((("文字唤醒", "你好小柒"), ("打断", "/interrupt"),
                           ("取消任务", "/cancel"), ("暂停", "/wait"),
                           ("停止", "/stop"), ("休眠", "/sleep"), ("状态", "/status")), start=1):
        ttk.Button(buttons, text=label, width=1, command=lambda c=command: bridge.submit(c)).grid(row=0, column=column, sticky="ew", padx=2)
    ttk.Label(frame, text="口头打断：“你好小柒，打断回答”（建议耳机，无 AEC）。按钮/文字仍可用；预览点击选择 Master，空格打断。",
              wraplength=800).grid(row=8, column=0, sticky="w", pady=8)
    closing, done = [False], [False]
    def close():
        closing[0] = True
        if done[0]:
            root.destroy()
        else:
            state.set("正在停止回答、语音和相机…")
            bridge.submit("/quit")
    root.protocol("WM_DELETE_WINDOW", close)
    root.bind("<Escape>", lambda _: bridge.submit("/interrupt"))
    def append(text):
        output.configure(state="normal")
        output.insert("end", text)
        if int(output.index("end-1c").split(".")[0]) > 1000:
            output.delete("1.0", "200.0")
        output.see("end")
        output.configure(state="disabled")
    def poll():
        for _ in range(300):
            try:
                kind, data = bridge.updates.get_nowait()
            except queue.Empty:
                break
            if kind == "state":
                state.set(data)
            elif kind in ("device", "level"):
                device.set(data)
                if kind == "device":
                    append("\n设备：" + data + "\n")
            elif kind == "input":
                origin = "语音" if data.get("source") == "voice" else "键盘/按钮"
                acceptance.set(origin + " · " + data["status"] + " · " + data["reason"])
                append("\n用户（" + origin + "）：" + data["text"] + "\n" + data["status"] + " · " + data["reason"] + "\n")
            elif kind == "asr":
                asr.set(data)
            elif kind == "perception":
                perception.set(data)
            elif kind == "stream":
                append(data)
            elif kind == "metrics":
                append("\n" + format_metrics(data) + "\n")
            elif kind in ("response", "feedback", "error"):
                append("\n" + str(data) + "\n")
            elif kind == "closed":
                done[0] = True
                state.set("已结束 · 相机/语音已停止")
                if closing[0]:
                    root.destroy()
                    return
        root.after(40, poll)
    def worker():
        try:
            run_worker()
        except BaseException as error:
            bridge.emit("error", "运行失败：" + type(error).__name__ + "；请查看阶段日志/终端。")
        finally:
            bridge.emit("closed", None)
    thread = threading.Thread(target=worker, name="companionbot-interaction")
    thread.start()
    entry.focus_set()
    root.after(40, poll)
    root.mainloop()
    bridge.submit("/quit")
    thread.join(timeout=5)
