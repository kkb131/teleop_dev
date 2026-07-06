"""tkinter GUI — 로컬(조종 PC) + 원격(robot PC agent) 컴포넌트 통합 관리.

    python3 -m launcher gui --config launcher/config/teleop_system.yaml --profile operator

profile:
    operator : operator group = 로컬 실행, robot group = 원격 agent 제어
    robot    : robot group 만 로컬 (robot PC 에서 단독 GUI 운용)
    local    : 전 그룹 로컬 (한 PC 개발/시뮬)

구조:
    - tkinter 는 thread-safe 하지 않으므로 모든 manager 호출 (특히 원격 HTTP)
      은 worker thread 가 수행하고, 결과는 queue 로 UI thread 에 전달.
    - 컴포넌트 행: ● 상태 / 이름 / 상태 텍스트 / Start / Stop.
      행 클릭 → 하단 로그 pane 이 해당 컴포넌트 tail (seq 커서 방식).
    - use_pty 컴포넌트 선택 시 키 전송 바 활성 (r/p/c/Space/x + 자유 입력).
    - Start All: robot group → operator group 순서 (수신부 먼저),
      Stop All: 역순 (sender 먼저 끊고 드라이버 정지).
"""

from __future__ import annotations

import queue
import threading
import tkinter as tk
from tkinter import ttk
from typing import Dict, List, Optional, Tuple

from launcher.agent_client import AgentUnreachable, RemoteManager
from launcher.config import LauncherConfig
from launcher.manager import LocalManager

_COLORS = {
    "running": "#2ecc71",   # green
    "done": "#3498db",      # blue (oneshot 완료)
    "stopped": "#95a5a6",   # gray
    "exited": "#e74c3c",    # red (비정상 종료)
    "unknown": "#f39c12",   # orange (원격 조회 실패)
}
_POLL_STATUS_MS = 1000
_POLL_LOG_MS = 500
_POLL_UIQ_MS = 100


class ManagerView:
    """manager (local 또는 remote) 1개 + 표시용 메타."""

    def __init__(self, title: str, manager, remote: bool, names: List[str],
                 pty_names: set):
        self.title = title
        self.manager = manager
        self.remote = remote
        self.names = names
        self.pty_names = pty_names
        self.reachable = True
        self.refreshing = False


def build_views(config: LauncherConfig, profile: str,
                agent_url: Optional[str]) -> List[ManagerView]:
    def names(group):
        return [c.name for c in config.by_group(group)]

    def ptys(group):
        return {c.name for c in config.by_group(group) if c.use_pty}

    if profile == "operator":
        url = agent_url or (
            f"http://{config.network.get('robot_pc_ip', '127.0.0.1')}"
            f":{config.agent.port}"
        )
        remote = RemoteManager(url, token=config.agent.token)
        local = LocalManager(config, groups=["operator"])
        return [
            ManagerView(f"Robot PC (원격: {url})", remote, True,
                        names("robot"), ptys("robot")),
            ManagerView("Operator PC (로컬)", local, False,
                        names("operator"), ptys("operator")),
        ]
    if profile == "robot":
        local = LocalManager(config, groups=["robot"])
        return [ManagerView("Robot PC (로컬)", local, False,
                            names("robot"), ptys("robot"))]
    # local: 전부 로컬
    local = LocalManager(config, groups=["robot", "operator"])
    return [
        ManagerView("Robot 구성요소 (로컬)", local, False,
                    names("robot"), ptys("robot")),
        ManagerView("Operator 구성요소 (로컬)", local, False,
                    names("operator"), ptys("operator")),
    ]


class LauncherGUI(tk.Tk):
    def __init__(self, config: LauncherConfig, views: List[ManagerView]):
        super().__init__()
        self.title("Teleop Launcher")
        self.geometry("980x720")
        self._config = config
        self._views = views
        self._rows: Dict[Tuple[int, str], dict] = {}
        self._selected: Optional[Tuple[int, str]] = None
        self._log_cursor: Dict[Tuple[int, str], int] = {}
        self._log_busy = False

        self._ui_q: "queue.Queue" = queue.Queue()
        self._task_q: "queue.Queue" = queue.Queue()
        self._worker = threading.Thread(target=self._task_loop, daemon=True)
        self._worker.start()

        self._build_widgets()
        self.after(_POLL_UIQ_MS, self._drain_ui_queue)
        self.after(200, self._poll_status)
        self.after(400, self._poll_log)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    # ── layout ──────────────────────────────────────────────────────────

    def _build_widgets(self):
        toolbar = ttk.Frame(self)
        toolbar.pack(fill="x", padx=8, pady=6)
        ttk.Button(toolbar, text="▶ Start All", command=self._start_all).pack(side="left")
        ttk.Button(toolbar, text="■ Stop All", command=self._stop_all).pack(side="left", padx=6)
        self._banner = ttk.Label(toolbar, text="", foreground="#e74c3c")
        self._banner.pack(side="left", padx=12)

        body = ttk.Frame(self)
        body.pack(fill="both", expand=True, padx=8)

        panels = ttk.Frame(body)
        panels.pack(fill="x")
        for vi, view in enumerate(self._views):
            frame = ttk.LabelFrame(panels, text=view.title)
            frame.pack(fill="x", pady=4)
            for name in view.names:
                self._build_row(frame, vi, name)

        # 로그 pane
        logframe = ttk.LabelFrame(body, text="log (컴포넌트 행을 클릭해 선택)")
        logframe.pack(fill="both", expand=True, pady=6)
        self._log_text = tk.Text(logframe, height=14, state="disabled",
                                 bg="#101418", fg="#d7dde4",
                                 font=("monospace", 9))
        scroll = ttk.Scrollbar(logframe, command=self._log_text.yview)
        self._log_text.configure(yscrollcommand=scroll.set)
        self._log_text.pack(side="left", fill="both", expand=True)
        scroll.pack(side="right", fill="y")

        # 키 전송 바 (pty 컴포넌트 선택 시 활성)
        keybar = ttk.Frame(body)
        keybar.pack(fill="x", pady=(0, 8))
        ttk.Label(keybar, text="키 전송:").pack(side="left")
        self._key_buttons = []
        for label, seq in (("r (sync)", "r"), ("p (pause)", "p"),
                           ("c (recal)", "c"), ("Space (E-Stop)", " "),
                           ("x (quit)", "x")):
            b = ttk.Button(keybar, text=label, state="disabled",
                           command=lambda s=seq: self._send_key(s))
            b.pack(side="left", padx=2)
            self._key_buttons.append(b)
        self._key_entry = ttk.Entry(keybar, width=12, state="disabled")
        self._key_entry.pack(side="left", padx=6)
        self._key_send = ttk.Button(keybar, text="Send", state="disabled",
                                    command=self._send_entry)
        self._key_send.pack(side="left")

    def _build_row(self, parent, vi: int, name: str):
        row = ttk.Frame(parent)
        row.pack(fill="x", padx=6, pady=2)
        dot = tk.Canvas(row, width=14, height=14, highlightthickness=0)
        oval = dot.create_oval(2, 2, 12, 12, fill=_COLORS["stopped"], outline="")
        dot.pack(side="left")
        lbl = ttk.Label(row, text=name, width=24, anchor="w")
        lbl.pack(side="left", padx=6)
        state = ttk.Label(row, text="stopped", width=28, anchor="w")
        state.pack(side="left")
        ttk.Button(row, text="Start",
                   command=lambda: self._component_action(vi, name, "start")
                   ).pack(side="right", padx=2)
        ttk.Button(row, text="Stop",
                   command=lambda: self._component_action(vi, name, "stop")
                   ).pack(side="right", padx=2)

        for w in (row, lbl, state):
            w.bind("<Button-1>", lambda e, k=(vi, name): self._select(k))
        self._rows[(vi, name)] = {"dot": dot, "oval": oval, "state": state,
                                  "label": lbl}

    # ── selection / key send ────────────────────────────────────────────

    def _select(self, key: Tuple[int, str]):
        if self._selected and self._selected in self._rows:
            self._rows[self._selected]["label"].configure(font=("TkDefaultFont", 9, "normal"))
        self._selected = key
        self._rows[key]["label"].configure(font=("TkDefaultFont", 9, "bold"))
        self._log_cursor.setdefault(key, 0)
        self._log_text.configure(state="normal")
        self._log_text.delete("1.0", "end")
        self._log_text.configure(state="disabled")
        self._log_cursor[key] = 0     # 처음부터 다시 tail

        vi, name = key
        is_pty = name in self._views[vi].pty_names
        state = "normal" if is_pty else "disabled"
        for b in self._key_buttons:
            b.configure(state=state)
        self._key_entry.configure(state=state)
        self._key_send.configure(state=state)

    def _send_key(self, seq: str):
        if not self._selected:
            return
        vi, name = self._selected
        view = self._views[vi]
        self._submit(lambda: view.manager.send_input(name, seq),
                     f"send {seq!r} → {name}")

    def _send_entry(self):
        text = self._key_entry.get()
        if text:
            self._key_entry.delete(0, "end")
            self._send_key(text)

    # ── actions (worker thread 로 위임) ─────────────────────────────────

    def _submit(self, fn, desc: str):
        self._task_q.put((fn, desc))

    def _component_action(self, vi: int, name: str, action: str):
        view = self._views[vi]
        fn = view.manager.start if action == "start" else view.manager.stop
        self._submit(lambda: fn(name), f"{action} {name}")

    def _start_all(self):
        def task():
            # robot (수신부/드라이버) 먼저 → operator (sender) 나중
            for view in self._views:
                view.manager.start_all(view.names)
        self._submit(task, "start all")

    def _stop_all(self):
        def task():
            for view in reversed(self._views):
                view.manager.stop_all(view.names)
        self._submit(task, "stop all")

    def _task_loop(self):
        while True:
            fn, desc = self._task_q.get()
            try:
                fn()
                # 폴링 태스크 (desc="") 는 배너를 건드리지 않음 —
                # agent_down 배너가 다음 poll 완료에 지워지는 것 방지
                if desc:
                    self._ui_q.put(("info", f"{desc}: 완료"))
            except AgentUnreachable as e:
                self._ui_q.put(("banner", f"agent 연결 실패 — {e}"))
            except Exception as e:
                self._ui_q.put(("banner", f"{desc} 오류: {e}"))

    # ── polling ─────────────────────────────────────────────────────────

    def _poll_status(self):
        for vi, view in enumerate(self._views):
            if view.refreshing:
                continue
            view.refreshing = True

            def refresh(vi=vi, view=view):
                try:
                    statuses = view.manager.status_all()
                    self._ui_q.put(("status", vi, statuses))
                    if view.remote:
                        self._ui_q.put(("banner", ""))
                except AgentUnreachable as e:
                    self._ui_q.put(("agent_down", vi, str(e)))
                except Exception as e:
                    self._ui_q.put(("banner", f"status 오류: {e}"))
                finally:
                    view.refreshing = False

            self._submit(refresh, "")
        self.after(_POLL_STATUS_MS, self._poll_status)

    def _poll_log(self):
        if self._selected and not self._log_busy:
            vi, name = self._selected
            view = self._views[vi]
            cursor = self._log_cursor.get((vi, name), 0)
            self._log_busy = True

            def fetch(vi=vi, name=name, view=view, cursor=cursor):
                try:
                    lines = view.manager.log_since(name, cursor, 300)
                    if lines:
                        self._ui_q.put(("log", vi, name, lines))
                except AgentUnreachable:
                    pass
                except Exception:
                    pass
                finally:
                    self._log_busy = False

            self._submit(fetch, "")
        self.after(_POLL_LOG_MS, self._poll_log)

    def _drain_ui_queue(self):
        try:
            while True:
                msg = self._ui_q.get_nowait()
                kind = msg[0]
                if kind == "status":
                    _, vi, statuses = msg
                    self._views[vi].reachable = True
                    for st in statuses:
                        self._apply_status(vi, st)
                elif kind == "agent_down":
                    _, vi, err = msg
                    self._views[vi].reachable = False
                    self._banner.configure(text=f"⚠ robot agent 연결 안 됨 ({err[:80]})")
                    for name in self._views[vi].names:
                        key = (vi, name)
                        if key in self._rows:
                            self._set_dot(key, "unknown")
                            self._rows[key]["state"].configure(text="(원격 조회 실패)")
                elif kind == "banner":
                    self._banner.configure(text=msg[1])
                elif kind == "info":
                    self._banner.configure(text="")
                elif kind == "log":
                    _, vi, name, lines = msg
                    if self._selected == (vi, name):
                        self._append_log(vi, name, lines)
        except queue.Empty:
            pass
        self.after(_POLL_UIQ_MS, self._drain_ui_queue)

    def _apply_status(self, vi: int, st: dict):
        key = (vi, st["name"])
        if key not in self._rows:
            return
        state = st.get("state", "stopped")
        self._set_dot(key, state)
        extra = ""
        if state == "running":
            extra = f"pid {st.get('pid')} · {st.get('uptime_s', 0)}s"
        elif st.get("returncode") is not None:
            extra = f"rc={st['returncode']}"
        self._rows[key]["state"].configure(text=f"{state} {extra}")

    def _set_dot(self, key, state: str):
        row = self._rows[key]
        row["dot"].itemconfigure(row["oval"], fill=_COLORS.get(state, "#95a5a6"))

    def _append_log(self, vi: int, name: str, lines):
        self._log_cursor[(vi, name)] = max(s for s, _ in lines)
        self._log_text.configure(state="normal")
        for _, line in lines:
            self._log_text.insert("end", line + "\n")
        # ring 유지 (너무 길어지면 앞부분 삭제)
        line_count = int(self._log_text.index("end-1c").split(".")[0])
        if line_count > 2000:
            self._log_text.delete("1.0", f"{line_count - 2000}.0")
        self._log_text.see("end")
        self._log_text.configure(state="disabled")

    # ── shutdown ────────────────────────────────────────────────────────

    def _on_close(self):
        # 로컬 컴포넌트만 정지 (원격은 agent 가 계속 관리 — 의도적으로 유지)
        for view in reversed(self._views):
            if not view.remote:
                try:
                    view.manager.stop_all(view.names)
                except Exception:
                    pass
        self.destroy()


def run_gui(config: LauncherConfig, profile: str = "operator",
            agent_url: Optional[str] = None) -> int:
    views = build_views(config, profile, agent_url)
    gui = LauncherGUI(config, views)
    gui.mainloop()
    return 0
