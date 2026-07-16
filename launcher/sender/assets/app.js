/* XR Teleop 조종 PC 대시보드 — framework-free, 1s 폴링.
 * 인증: ?token=... 최초 접속 → localStorage → X-Auth-Token 헤더. */

"use strict";

(function initToken() {
  const t = new URLSearchParams(location.search).get("token");
  if (t) localStorage.setItem("launcher_token", t);
})();

async function api(method, path, body) {
  const headers = { "Content-Type": "application/json" };
  const tok = localStorage.getItem("launcher_token");
  if (tok) headers["X-Auth-Token"] = tok;
  const resp = await fetch(path, {
    method, headers,
    body: body === undefined ? undefined : JSON.stringify(body),
  });
  const data = await resp.json().catch(() => ({}));
  if (!resp.ok) {
    const err = new Error(data.error || `${method} ${path} → ${resp.status}`);
    err.status = resp.status;
    err.data = data;
    throw err;
  }
  return data;
}

function banner(msg, kind) {
  const el = document.getElementById("banner");
  if (!el) return;
  el.textContent = msg || "";
  el.className = kind || "";
}

function pill(el, text) {
  el.className = `pill ${text}`;
  el.textContent = text;
}

// ── 로그 tail ────────────────────────────────────────────────────────────

const logState = { name: null, cursor: 0 };

function setLogTarget(name) {
  logState.name = name;
  logState.cursor = 0;
  const pre = document.getElementById("log");
  if (pre) pre.textContent = "";
}

async function pollLog() {
  if (!logState.name) return;
  try {
    const r = await api("GET",
      `/api/components/${logState.name}/log?since=${logState.cursor}&max=300`);
    if (r.lines.length) {
      const pre = document.getElementById("log");
      for (const [seq, line] of r.lines) {
        logState.cursor = Math.max(logState.cursor, seq);
        pre.textContent += line.replace(/\x1b\[[0-9;]*[A-Za-z]/g, "") + "\n";
      }
      const lines = pre.textContent.split("\n");
      if (lines.length > 1500) pre.textContent = lines.slice(-1500).join("\n");
      pre.scrollTop = pre.scrollHeight;
    }
  } catch (e) { /* 폴링 오류 무시 */ }
}

function ensureLogOption(name) {
  const sel = document.getElementById("log-select");
  if (!sel || [...sel.options].some(o => o.value === name)) return;
  const opt = document.createElement("option");
  opt.value = opt.textContent = name;
  sel.appendChild(opt);
}

// ── index 렌더 ───────────────────────────────────────────────────────────

function renderRunner(st) {
  const r = st.runner || {};
  const comp = r.component;
  const state = comp ? comp.state : "stopped";
  pill(document.getElementById("runner-pill"),
       state === "running" ? "running" : state);
  document.getElementById("runner-mode").textContent = r.mode || "";

  const rows = [];
  if (state === "running") {
    for (const [side, a] of Object.entries(r.arms || {})) {
      const label = side === "single" ? "팔" :
        (side === "right" ? "오른팔" : "왼팔");
      rows.push(`<b>${label}</b> <span class="pill ${a.state?.split(" ")[0] || "READY"}">` +
        `${a.spd || "?"}</span>` +
        (a.calib_count ? ` 캘리 ${a.calib_count}회` : ""));
    }
    for (const [side, h] of Object.entries(r.hands || {})) {
      const label = side === "single" ? "손" :
        (side === "right" ? "오른손" : "왼손");
      rows.push(`<b>${label}</b> <span class="pill ${h.tracking ? "TRACK" : "LOST"}">` +
        `${h.tracking ? "TRACK" : "LOST"}</span> skip=${h.skip}`);
    }
    const hs = r.handshake || {};
    rows.push(`<b>robot 응답</b> 수신 ${hs.received ?? 0} / 실패 ` +
      `${hs.failed ? `<span style="color:var(--red)">${hs.failed}</span>` : 0}`);
    if (r.last_warn)
      rows.push(`<b>경고</b> <span style="color:var(--orange)">${r.last_warn}</span>`);
    if (r.last_gesture) rows.push(`<b>제스처</b> ${r.last_gesture}`);
  }
  document.getElementById("runner-live").innerHTML = rows.join("<br>") || "—";

  const wsDot = document.getElementById("ws-dot");
  wsDot.className = "dot " + (r.headset_ws === "connected" ? "ok" :
    r.headset_ws === "disconnected" ? "bad" : "");
}

function renderAdb(st) {
  const a = st.adb || {};
  const p = document.getElementById("adb-pill");
  if (!a.installed) pill(p, "error"), p.textContent = "adb 없음";
  else if (a.connected && a.reverse_ok) pill(p, "running"), p.textContent = "연결됨";
  else if (a.connected) pill(p, "READY"), p.textContent = "reverse 필요";
  else pill(p, "stopped"), p.textContent = "미연결";

  const rows = [];
  if (!a.installed) {
    rows.push("adb 미설치 — xr_input_guide.md §2 설치 절차 참조");
  } else {
    const devs = (a.devices || []).map(d =>
      `${d.serial} <span class="pill ${d.state === "device" ? "running" : "error"}">${d.state}</span>`);
    rows.push(`<b>devices</b> ${devs.join(", ") || "없음"}`);
    const revs = (a.reverses || []).map(r => r.remote).join(", ");
    rows.push(`<b>reverse</b> ${revs || "없음"}` +
      (a.missing_reverses?.length
        ? ` <span style="color:var(--orange)">(누락: ${a.missing_reverses.join(", ")})</span>` : ""));
  }
  document.getElementById("adb-live").innerHTML = rows.join("<br>");
}

function renderCam(st) {
  const c = st.cam_viewer || {};
  const state = c.component ? c.component.state : "stopped";
  pill(document.getElementById("cam-pill"), state);
  const cams = c.cameras || [];
  document.getElementById("cam-stats").innerHTML = cams.length
    ? `<thead><tr><th>카메라</th><th>fps</th><th>KB/s</th><th>지연</th></tr></thead><tbody>` +
      cams.map(x => `<tr><td>${x.name}</td><td>${x.fps.toFixed(1)}</td>` +
        `<td>${x.kbps}</td><td>${x.lat_ms}ms</td></tr>`).join("") + "</tbody>"
    : "";
}

function renderSession(st) {
  const s = st.session || { state: "idle" };
  pill(document.getElementById("session-pill"), s.state);
  const cd = document.getElementById("countdown");
  cd.textContent = (s.state === "countdown" && s.countdown_remaining_s !== undefined)
    ? s.countdown_remaining_s.toFixed(0) : " ";
  document.getElementById("session-attempt").textContent =
    s.attempt ? `시도 ${s.attempt}${s.expected_arms ? ` — 확인: ${
      (s.confirmed_arms || []).join(", ") || "없음"} / 대상: ${
      s.expected_arms.join(", ")}` : ""}` : "";
  document.getElementById("session-msg").textContent = s.message || "";
  document.getElementById("session-history").innerHTML =
    (s.history || []).slice(-8).map(h => `<li>${h}</li>`).join("");
}

// ── index 초기화 ─────────────────────────────────────────────────────────

async function initIndex() {
  const modeSel = document.getElementById("mode-select");

  const selectedMode = () => modeSel.value;

  // 러너 시작 (409 → confirm 후 스왑)
  document.getElementById("runner-start").addEventListener("click", async () => {
    const mode = selectedMode();
    try {
      await api("POST", "/api/runner/start", { mode });
      banner(`러너 시작: ${mode}`, "ok");
    } catch (e) {
      if (e.status === 409 && e.data.running_mode) {
        if (!confirm(`${e.data.running_mode} 실행 중입니다. 정지하고 ${mode} 를 시작할까요?`))
          return;
        await api("POST", "/api/runner/stop");
        await new Promise(r => setTimeout(r, 1000));
        try {
          await api("POST", "/api/runner/start", { mode });
          banner(`러너 전환: ${mode}`, "ok");
        } catch (e2) { banner(`시작 실패: ${e2.message}`, "err"); }
      } else banner(`시작 실패: ${e.message}`, "err");
    }
  });
  document.getElementById("runner-stop").addEventListener("click", async () => {
    try {
      const r = await api("POST", "/api/runner/stop");
      banner(r.stopped ? `러너 정지: ${r.stopped}` : "실행 중인 러너 없음", "ok");
    } catch (e) { banner(`정지 실패: ${e.message}`, "err"); }
  });

  // 키패드
  document.querySelectorAll(".keypad button").forEach(btn =>
    btn.addEventListener("click", async () => {
      try {
        await api("POST", "/api/runner/key", { key: btn.dataset.key });
        banner(`키 전송: ${btn.dataset.key}`, "ok");
      } catch (e) { banner(`키 전송 실패: ${e.data?.error || e.message}`, "err"); }
    }));

  // 세션
  document.getElementById("session-start").addEventListener("click", async () => {
    try {
      await api("POST", "/api/session/start", { mode: selectedMode() });
      banner("조종 시작 시퀀스 진행 중 — 카운트다운 후 자동 캘리브레이션", "ok");
    } catch (e) { banner(`세션 시작 실패: ${e.data?.error || e.message}`, "err"); }
  });
  document.getElementById("session-cancel").addEventListener("click", () =>
    api("POST", "/api/session/cancel").catch(() => {}));

  // adb
  const adbOut = document.getElementById("adb-out");
  document.getElementById("adb-reverse").addEventListener("click", async () => {
    try {
      const r = await api("POST", "/api/adb/reverse");
      adbOut.style.display = "";
      adbOut.textContent = Object.entries(r.results || {})
        .map(([p, x]) => `$ ${x.cmd}\n${x.output || (x.ok ? "OK" : "실패")}`).join("\n");
      banner(r.ok ? "adb reverse 설정 완료" : "일부 reverse 실패 — 출력 확인",
             r.ok ? "ok" : "err");
    } catch (e) { banner(`adb 실패: ${e.data?.error || e.message}`, "err"); }
  });
  document.getElementById("adb-restart").addEventListener("click", async () => {
    try {
      const r = await api("POST", "/api/adb/restart");
      adbOut.style.display = "";
      adbOut.textContent = r.output || "";
      banner("adb 서버 재시작 완료 — 헤드셋에서 USB 디버깅 재승인 필요할 수 있음", "ok");
    } catch (e) { banner(`adb 실패: ${e.data?.error || e.message}`, "err"); }
  });

  // cam
  document.getElementById("cam-start").addEventListener("click", () =>
    api("POST", "/api/cam/start").then(() => banner("카메라 뷰어 시작", "ok"))
      .catch(e => banner(e.message, "err")));
  document.getElementById("cam-stop").addEventListener("click", () =>
    api("POST", "/api/cam/stop").catch(() => {}));

  // 로그
  document.getElementById("log-select").addEventListener("change", e =>
    setLogTarget(e.target.value));
  document.getElementById("log-clear").addEventListener("click", () =>
    document.getElementById("log").textContent = "");

  // 폴링
  let modesLoaded = false;
  const poll = async () => {
    try {
      const st = await api("GET", "/api/status");
      if (!modesLoaded) {
        modesLoaded = true;
        for (const r of st.runners || []) {
          const opt = document.createElement("option");
          opt.value = r.name;
          opt.textContent = r.label;
          modeSel.appendChild(opt);
        }
      }
      renderRunner(st);
      renderAdb(st);
      renderCam(st);
      renderSession(st);
      if (st.warnings?.length) banner(`⚠ ${st.warnings.join(" / ")}`, "warn");
      // 로그 select 채우기 (러너 컴포넌트 + cam)
      if (st.runner?.component) ensureLogOption(st.runner.component.name);
      if (st.cam_viewer?.component) ensureLogOption(st.cam_viewer.component.name);
      const sel = document.getElementById("log-select");
      if (!logState.name && sel.options.length) {
        sel.selectedIndex = 0;
        setLogTarget(sel.value);
      }
    } catch (e) { banner(`상태 조회 실패: ${e.message}`, "err"); }
  };
  await poll();
  setInterval(poll, 1000);
  setInterval(pollLog, 1000);
}

// ── 파라미터 페이지 (robot app.js 패턴 + floats3) ────────────────────────

function multiInputs(values, count, step) {
  const arr = Array.isArray(values) ? values : Array(count).fill("");
  return `<div class="multi">` + arr.map((x, i) =>
    `<input type="number" step="${step}" value="${x}" data-idx="${i}">`).join("") +
    `</div>`;
}

function buildParamForm(container, schema, values) {
  container.innerHTML = "";
  for (const f of schema) {
    const div = document.createElement("div");
    div.className = "field" + (f.danger ? " danger" : "");
    div.dataset.key = f.key;
    div.dataset.type = f.type;
    const unit = f.unit ? ` <span class="unit">[${f.unit}]</span>` : "";
    let inner = `<label>${f.label}${unit} <span style="opacity:.5">(${f.key})</span></label>`;
    const v = values[f.key];
    if (f.type === "float" || f.type === "int") {
      const step = f.type === "int" ? 1 : 0.01;
      inner += `<input type="number" step="${step}" value="${v ?? ""}"
        ${f.min !== undefined ? `min="${f.min}"` : ""} ${f.max !== undefined ? `max="${f.max}"` : ""}>`;
    } else if (f.type === "str") {
      inner += `<input type="text" value="${v ?? ""}">`;
    } else if (f.type === "bool") {
      inner += `<input type="checkbox" ${v ? "checked" : ""}>`;
    } else if (f.type === "enum") {
      inner += `<select>` + (f.choices || []).map(c =>
        `<option ${c === v ? "selected" : ""}>${c}</option>`).join("") + `</select>`;
    } else if (f.type === "range2") {
      inner += multiInputs(v, 2, 0.01);
    } else if (f.type === "floats3") {
      inner += multiInputs(v, 3, 0.5);
    } else if (f.type === "joints6") {
      inner += multiInputs(v, 6, 0.001);
    }
    if (f.help) inner += `<div class="help">${f.help}</div>`;
    inner += `<div class="err"></div>`;
    div.innerHTML = inner;
    container.appendChild(div);
  }
}

function collectParamForm(container) {
  const values = {};
  for (const div of container.querySelectorAll(".field")) {
    const key = div.dataset.key, type = div.dataset.type;
    if (type === "float" || type === "int") {
      const x = div.querySelector("input").value;
      if (x !== "") values[key] = type === "int" ? parseInt(x, 10) : parseFloat(x);
    } else if (type === "str") {
      values[key] = div.querySelector("input").value;
    } else if (type === "bool") {
      values[key] = div.querySelector("input").checked;
    } else if (type === "enum") {
      values[key] = div.querySelector("select").value;
    } else if (type === "range2" || type === "floats3" || type === "joints6") {
      values[key] = [...div.querySelectorAll("input")].map(i => parseFloat(i.value));
    }
  }
  return values;
}

function showParamErrors(container, errors) {
  container.querySelectorAll(".field .err").forEach(e => e.textContent = "");
  for (const [key, msg] of Object.entries(errors || {})) {
    const div = container.querySelector(`.field[data-key="${CSS.escape(key)}"]`);
    if (div) div.querySelector(".err").textContent = msg;
  }
}

async function initParams() {
  const form = document.getElementById("form");
  const p = await api("GET", "/api/params/xr-dual");
  buildParamForm(form, p.schema, p.values);
  if (!p.comments_preserved)
    banner("⚠ ruamel.yaml 미설치 — 저장 시 yaml 주석이 유실됩니다", "warn");

  document.getElementById("save").addEventListener("click", async () => {
    try {
      const r = await api("PUT", "/api/params/xr-dual",
                          { values: collectParamForm(form) });
      showParamErrors(form, {});
      banner(`저장됨 (${r.saved.length}개) — 러너 재시작 시 적용`, "ok");
      document.getElementById("save-note").textContent = "⟳ 러너 재시작 필요";
    } catch (e) {
      showParamErrors(form, e.data?.errors);
      banner("저장 실패 — 입력값을 확인하세요", "err");
    }
  });
}

// ── entry ────────────────────────────────────────────────────────────────

document.addEventListener("DOMContentLoaded", () => {
  const page = document.body.dataset.page;
  if (page === "index") initIndex();
  else if (page === "params") initParams();
});
