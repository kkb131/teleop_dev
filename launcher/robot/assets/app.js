/* Robot Launcher 대시보드 — framework-free, 1s 폴링.
 *
 * 인증: 서버에 token 이 설정된 경우 ?token=... 으로 최초 접속하면
 * localStorage 에 저장되고 이후 X-Auth-Token 헤더로 전송된다.
 */

"use strict";

// ── token / api ──────────────────────────────────────────────────────────

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

// ── banner ───────────────────────────────────────────────────────────────

function banner(msg, kind) {
  const el = document.getElementById("banner");
  if (!el) return;
  el.textContent = msg || "";
  el.className = kind || "";
}

// ── E-STOP (모든 페이지 공통) ────────────────────────────────────────────

function wireEstop() {
  const btn = document.getElementById("estop");
  if (!btn) return;
  btn.addEventListener("click", async () => {
    btn.disabled = true;
    banner("E-STOP 전송 중...", "warn");
    try {
      const r = await api("POST", "/api/estop", {});
      const parts = [];
      for (const [side, b] of Object.entries(r.burst_sent || {}))
        parts.push(`${side}: ${b.packets}발(port ${b.port})`);
      let msg = `E-STOP — ${parts.join(", ")}`;
      if (r.backstop_stopped?.length)
        msg += ` | 백스톱 정지: ${r.backstop_stopped.join(", ")}`;
      if (r.hands_stopping?.length)
        msg += ` | 손 정지: ${r.hands_stopping.join(", ")}`;
      if (r.warnings?.length) msg += `\n⚠ ${r.warnings.join(" / ")}`;
      banner(msg, "err");
    } catch (e) {
      banner(`E-STOP 실패: ${e.message}`, "err");
    } finally {
      btn.disabled = false;
    }
  });
}

// ── 상태 렌더 공통 ───────────────────────────────────────────────────────

function pill(state) {
  return `<span class="pill ${state}">${state}</span>`;
}

function hwDot(hw) {
  if (!hw) return "";
  const cls = hw.reachable ? "ok" : "bad";
  const tip = `${hw.host}:${hw.port} ${hw.reachable ? "도달 가능" : "도달 불가"}`;
  return `<span class="hwdot ${cls}" title="HW ${tip}"></span>`;
}

function armMetricsText(m) {
  if (!m || Object.keys(m).length === 0) return "(라이브 상태 없음)";
  const rows = [];
  if (m.ee_pos)
    rows.push(`EE  x=${m.ee_pos[0].toFixed(3)} y=${m.ee_pos[1].toFixed(3)} z=${m.ee_pos[2].toFixed(3)} m`);
  if (m.ee_rpy_deg)
    rows.push(`RPY ${m.ee_rpy_deg.map(v => v.toFixed(1)).join(" / ")} °`);
  if (m.safety !== undefined)
    rows.push(`Safety ${m.safety}${m.safety_msg ? " " + m.safety_msg : ""}`);
  if (m.estop_active !== undefined)
    rows.push(m.estop_active
      ? `<span class="bad">E-STOP ACTIVE (sender 'r' 또는 재시작으로 해제)</span>`
      : `E-Stop off`);
  if (m.admittance) rows.push(`Admit ${m.admittance}`);
  if (m.input_age_ms !== undefined) rows.push(`Input ${m.input_age_ms}ms ago`);
  return rows.join("\n");
}

function camMetricsText(m) {
  if (!m || !m.cameras || m.cameras.length === 0) return "(스트리밍 상태 없음)";
  const rows = m.cameras.map(c =>
    `${c.name}: ${c.fps.toFixed(1)}fps ${c.kbps}KB/s${c.fail ? ` fail=${c.fail}` : ""}`);
  if (m.stale) rows.push(`<span class="bad">⚠ stats 정지 (stale)</span>`);
  return rows.join("\n");
}

// ── 로그 tail ────────────────────────────────────────────────────────────

const logState = { name: null, cursor: 0 };

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
  } catch (e) { /* 폴링 오류는 조용히 */ }
}

function setLogTarget(name) {
  logState.name = name;
  logState.cursor = 0;
  const pre = document.getElementById("log");
  if (pre) pre.textContent = "";
}

// ── 파라미터 폼 (arm/cam 페이지 공용) ────────────────────────────────────

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
      const [a, b] = Array.isArray(v) ? v : ["", ""];
      inner += `<div class="multi"><input type="number" step="0.01" value="${a}" data-idx="0">
                <input type="number" step="0.01" value="${b}" data-idx="1"></div>`;
    } else if (f.type === "joints6") {
      const arr = Array.isArray(v) ? v : ["", "", "", "", "", ""];
      inner += `<div class="multi">` + arr.map((x, i) =>
        `<input type="number" step="0.001" value="${x}" data-idx="${i}">`).join("") + `</div>`;
    } else if (f.type === "cam_list") {
      inner += camListTable(Array.isArray(v) ? v : []);
    }
    if (f.help) inner += `<div class="help">${f.help}</div>`;
    inner += `<div class="err"></div>`;
    div.innerHTML = inner;
    container.appendChild(div);
  }
  // cam_list 행 추가/삭제
  container.querySelectorAll(".cam-add").forEach(btn =>
    btn.addEventListener("click", () => {
      const tbody = btn.closest(".field").querySelector("tbody");
      tbody.insertAdjacentHTML("beforeend", camRow({}));
      wireCamRowDel(btn.closest(".field"));
    }));
  wireCamRowDel(container);
}

function camRow(c) {
  return `<tr>
    <td><input class="wide" data-c="name" value="${c.name ?? ""}"></td>
    <td><input class="wide" data-c="serial" value="${c.serial ?? ""}" placeholder="자동"></td>
    <td><input type="number" data-c="width" value="${c.width ?? 640}"></td>
    <td><input type="number" data-c="height" value="${c.height ?? 480}"></td>
    <td><input type="number" data-c="fps" value="${c.fps ?? 30}"></td>
    <td><button type="button" class="cam-del">✕</button></td></tr>`;
}

function camListTable(cams) {
  return `<table class="cams">
    <thead><tr><th>name</th><th>serial</th><th>W</th><th>H</th><th>fps</th><th></th></tr></thead>
    <tbody>${cams.map(camRow).join("")}</tbody></table>
    <button type="button" class="cam-add" style="margin-top:6px">+ 카메라 추가</button>`;
}

function wireCamRowDel(scope) {
  scope.querySelectorAll(".cam-del").forEach(btn => {
    btn.onclick = () => btn.closest("tr").remove();
  });
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
    } else if (type === "range2" || type === "joints6") {
      values[key] = [...div.querySelectorAll("input")].map(i => parseFloat(i.value));
    } else if (type === "cam_list") {
      values[key] = [...div.querySelectorAll("tbody tr")].map(tr => {
        const g = c => tr.querySelector(`[data-c=${c}]`).value;
        return { name: g("name"), serial: g("serial"),
                 width: parseInt(g("width"), 10) || 0,
                 height: parseInt(g("height"), 10) || 0,
                 fps: parseInt(g("fps"), 10) || 0 };
      });
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

// 파라미터 패널 공용 초기화 (arm/cam 페이지)
async function setupParamPanel(target, partName) {
  const form = document.getElementById("form");
  const p = await api("GET", `/api/params/${target}`);
  document.getElementById("param-file").textContent = p.file;
  buildParamForm(form, p.schema, p.values);
  if (!p.comments_preserved)
    banner("⚠ ruamel.yaml 미설치 — 저장 시 yaml 주석이 유실됩니다", "warn");

  document.getElementById("save").addEventListener("click", async () => {
    const values = collectParamForm(form);
    try {
      const r = await api("PUT", `/api/params/${target}`, { values });
      showParamErrors(form, {});
      banner(`저장됨 (${r.saved.join(", ")}) — 재시작해야 적용됩니다`, "ok");
      document.getElementById("save-note").textContent = "⟳ 재시작 필요";
    } catch (e) {
      showParamErrors(form, e.data?.errors);
      banner(`저장 실패 — 입력값을 확인하세요`, "err");
    }
  });
  return p;
}

// 파츠 start/stop/restart 버튼 공용
function wirePartButtons(partName) {
  const act = (a, label) => async () => {
    banner(`${label} 중...`, "");
    try {
      await api("POST", `/api/parts/${partName}/${a}`);
      banner(`${label} 완료`, "ok");
      document.getElementById("save-note") &&
        (document.getElementById("save-note").textContent = "");
    } catch (e) { banner(`${label} 실패: ${e.message}`, "err"); }
  };
  document.getElementById("p-start")?.addEventListener("click", act("start", "시작"));
  document.getElementById("p-stop")?.addEventListener("click", act("stop", "정지"));
  document.getElementById("p-restart")?.addEventListener("click", act("restart", "재시작"));
}

async function moveHome(side, isRunning) {
  let stopReceiver = false;
  if (isRunning) {
    if (!confirm(`${side} 팔 수신부가 실행 중입니다.\n정지 후 초기 위치로 이동할까요?`))
      return;
    stopReceiver = true;
  }
  try {
    const r = await api("POST", `/api/arms/${side}/move_home`,
                        { stop_receiver: stopReceiver });
    banner(`초기 위치 이동 시작 (${r.component}) — 로그에서 진행 확인`, "ok");
    if (document.getElementById("log-select")) {
      // index: 로그 선택을 mover 로 전환
      ensureLogOption(r.component);
      document.getElementById("log-select").value = r.component;
      setLogTarget(r.component);
    }
  } catch (e) {
    banner(`초기 위치 이동 거부: ${e.data?.error || e.message}`, "err");
  }
}

// ── index 페이지 ─────────────────────────────────────────────────────────

function ensureLogOption(name) {
  const sel = document.getElementById("log-select");
  if (!sel || [...sel.options].some(o => o.value === name)) return;
  const opt = document.createElement("option");
  opt.value = opt.textContent = name;
  sel.appendChild(opt);
}

function renderIndex(st) {
  const cards = document.getElementById("cards");
  const oneshots = {};
  for (const o of st.oneshots || []) oneshots[o.name] = o;

  for (const [name, p] of Object.entries(st.parts)) {
    let card = document.getElementById(`card-${name}`);
    if (!card) {
      card = document.createElement("div");
      card.className = "card";
      card.id = `card-${name}`;
      card.innerHTML = `
        <h2><span class="hw"></span>${p.label}
            <span class="state"></span><span class="spacer"></span></h2>
        <div class="comps"></div>
        <div class="metrics"></div>
        <div class="btns">
          <button data-a="start" class="primary">시작</button>
          <button data-a="stop">정지</button>
          <button data-a="restart" class="warn">재시작</button>
          ${p.kind === "arm" ? `<button data-a="home">초기 위치 이동</button>
            <a href="/arm?side=${p.side}"><button>파라미터 →</button></a>` : ""}
          ${p.kind === "cam" ? `<a href="/cam"><button>파라미터/상태 →</button></a>` : ""}
        </div>`;
      cards.appendChild(card);
      card.querySelectorAll("button[data-a]").forEach(btn => {
        const a = btn.dataset.a;
        btn.addEventListener("click", async () => {
          if (a === "home")
            return moveHome(p.side, card.dataset.state === "running");
          banner(`${p.label} ${a}...`, "");
          try {
            await api("POST", `/api/parts/${name}/${a}`);
            banner(`${p.label} ${a} 완료`, "ok");
          } catch (e) { banner(`${p.label} ${a} 실패: ${e.message}`, "err"); }
        });
      });
    }
    card.dataset.state = p.state;
    card.querySelector(".state").outerHTML =
      `<span class="state">${pill(p.state)}</span>`;
    card.querySelector(".hw").outerHTML = `<span class="hw">${hwDot(p.hw)}</span>`;

    // 컴포넌트 sub-row (2개 이상인 파츠만 개별 버튼)
    const comps = card.querySelector(".comps");
    comps.innerHTML = p.components.map(c => `
      <div class="comp-row">
        <span class="name">${c.name}</span>
        <span class="st ${c.state}">${c.state}${c.pid ? ` (pid ${c.pid})` : ""}</span>
      </div>`).join("");

    // 메트릭
    let mtext = p.kind === "arm" ? armMetricsText(p.metrics)
      : p.kind === "cam" ? camMetricsText(p.metrics)
      : (p.state === "running" ? "드라이버+수신부 정상" : "");
    // move-home oneshot 상태 병합 (팔 카드)
    if (p.kind === "arm") {
      const o = oneshots[`move-home-${p.side}`];
      if (o) mtext += `\n[초기 위치 이동] ${o.state}` +
        (o.returncode !== null && o.returncode !== undefined ? ` rc=${o.returncode}` : "");
    }
    card.querySelector(".metrics").innerHTML = mtext || "&nbsp;";

    for (const c of p.components) ensureLogOption(c.name);
  }
  for (const name of Object.keys(oneshots)) ensureLogOption(name);
}

async function initIndex() {
  document.getElementById("start-all").addEventListener("click", async () => {
    banner("전체 시작 중... (드라이버 → 수신부 순서, 수 초 소요)", "");
    try {
      const r = await api("POST", "/api/start_all");
      banner(`전체 시작 완료: ${r.started.join(", ") || "(이미 실행 중)"}`, "ok");
    } catch (e) { banner(`전체 시작 실패: ${e.message}`, "err"); }
  });
  document.getElementById("stop-all").addEventListener("click", async () => {
    banner("전체 정지 중...", "");
    try {
      const r = await api("POST", "/api/stop_all");
      banner(`전체 정지 완료: ${r.stopped.join(", ") || "(이미 정지)"}`, "ok");
    } catch (e) { banner(`전체 정지 실패: ${e.message}`, "err"); }
  });
  document.getElementById("log-select").addEventListener("change", e =>
    setLogTarget(e.target.value));
  document.getElementById("log-clear").addEventListener("click", () =>
    document.getElementById("log").textContent = "");

  const poll = async () => {
    try {
      const st = await api("GET", "/api/status");
      renderIndex(st);
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

// ── arm 페이지 ───────────────────────────────────────────────────────────

async function initArm() {
  const side = new URLSearchParams(location.search).get("side") || "right";
  const partName = `arm-${side}`;
  document.getElementById("arm-title").textContent =
    side === "right" ? "오른팔 (arm-right)" : "왼팔 (arm-left)";

  wirePartButtons(partName);
  let running = false;
  document.getElementById("p-home").addEventListener("click", () =>
    moveHome(side, running));

  const p = await setupParamPanel(partName, partName);
  if (p.values["initial_pose.enabled"] === false)
    document.getElementById("left-warning").style.display = "";

  const poll = async () => {
    try {
      const st = await api("GET", "/api/status");
      const part = st.parts[partName];
      if (!part) return;
      running = part.state === "running";
      const m = part.metrics || {};
      document.getElementById("live").innerHTML =
        `<b>상태</b> ${pill(part.state)} ${hwDot(part.hw)}<br>` +
        armMetricsText(m).split("\n").map(l => l).join("<br>");
      if (!logState.name && part.components[0])
        setLogTarget(part.components[0].name);
    } catch (e) { /* 폴링 오류 무시 */ }
  };
  await poll();
  setInterval(poll, 1000);
  setInterval(pollLog, 1000);
}

// ── cam 페이지 ───────────────────────────────────────────────────────────

async function initCam() {
  wirePartButtons("cam");
  await setupParamPanel("cam", "cam");

  const poll = async () => {
    try {
      const st = await api("GET", "/api/status");
      const part = st.parts["cam"];
      if (!part) return;
      document.getElementById("live").innerHTML =
        `<b>상태</b> ${pill(part.state)}` +
        (part.metrics?.stale ? ` <span class="bad">stale</span>` : "");
      const cams = part.metrics?.cameras || [];
      document.getElementById("cam-stats").innerHTML =
        `<thead><tr><th>카메라</th><th>fps</th><th>KB/s</th><th>fail</th></tr></thead><tbody>` +
        cams.map(c => `<tr><td>${c.name}</td><td>${c.fps.toFixed(1)}</td>
          <td>${c.kbps}</td><td>${c.fail}</td></tr>`).join("") +
        (cams.length ? "" : `<tr><td colspan="4">(스트리밍 없음)</td></tr>`) +
        `</tbody>`;
      if (!logState.name && part.components[0])
        setLogTarget(part.components[0].name);
    } catch (e) { /* 폴링 오류 무시 */ }
  };
  await poll();
  setInterval(poll, 1000);
  setInterval(pollLog, 1000);
}

// ── entry ────────────────────────────────────────────────────────────────

document.addEventListener("DOMContentLoaded", () => {
  wireEstop();
  const page = document.body.dataset.page;
  if (page === "index") initIndex();
  else if (page === "arm") initArm();
  else if (page === "cam") initCam();
});
