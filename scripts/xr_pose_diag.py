#!/usr/bin/env python3
"""XR (Galaxy XR / Quest 3) → 조종 PC pose-only diagnostic.

BridgePoseStore 만 띄우고 30초 측정. UDP / 로봇 PC 없이 동작 검증.
xr_teleop Gate 2 (30Hz 안정 수신) 의 teleop_dev 이식.

사용 예:
  # smoke (1Hz 콘솔 로그, Ctrl+C 종료)
  python3 -m scripts.xr_pose_diag

  # 30 초 정량 측정 (mean_freq / lost_frames / wrist_jitter / recovery_latency)
  python3 -m scripts.xr_pose_diag --measure 30

  # 결과를 docs/vr_teleop/xr_spike/A3_diag_result.md 에 append
  python3 -m scripts.xr_pose_diag --measure 30 --report docs/vr_teleop/xr_spike/A3_diag_result.md

  # port 변경 (헤드셋 측 adb reverse tcp:8014 tcp:8014 같이 변경 필요)
  python3 -m scripts.xr_pose_diag --port 8014

Gate A 통과 기준:
  mean_freq_hz       ≥ 30 Hz
  lost_per_field     < 5 % of frames
  recovery_latency_s < 1.0 s (헤드셋 손 시야 밖 → 다시 진입)
  wrist_jitter_cm    ≤ 2 cm (정지 5초간 표준편차)
"""

from __future__ import annotations

import argparse
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Optional

import numpy as np

from sender.xr_common import BridgePoseStore


def _is_invalid(mat4: np.ndarray) -> bool:
    """SE(3) 4×4 가 채워졌는지 확인. zeros 면 invalid (BridgePoseStore 초기값).

    valid 한 wrist matrix 는 마지막 행이 [0,0,0,1].
    """
    if not np.isfinite(mat4).all():
        return True
    # 모두 0 (BridgePoseStore initial state) 또는 last row 가 [0,0,0,0] 이면 invalid
    if np.allclose(mat4, 0):
        return True
    if abs(mat4[3, 3] - 1.0) > 1e-6:
        return True
    return False


def _is_invalid_kp(kp_25: np.ndarray) -> bool:
    """25×3 keypoint 가 채워졌는지. wrist(0) 까지 모두 0 이면 invalid."""
    if not np.isfinite(kp_25).all():
        return True
    if np.allclose(kp_25, 0):
        return True
    return False


def _smoke(store: BridgePoseStore, hz: int = 1) -> None:
    """1Hz 콘솔 로그. Ctrl+C 종료."""
    dt = 1.0 / hz
    prev_stats = store.get_stats()
    prev_t = time.perf_counter()
    print("[xr_pose_diag] smoke 모드 (1Hz 로그). Ctrl+C 종료.")
    try:
        while True:
            time.sleep(dt)
            now = time.perf_counter()
            stats = store.get_stats()
            dmsg = stats["msg_count"] - prev_stats["msg_count"]
            dhead = stats["head_msg_count"] - prev_stats["head_msg_count"]
            dhand = stats["hand_msg_count"] - prev_stats["hand_msg_count"]
            dt_real = now - prev_t
            prev_stats, prev_t = stats, now

            head = store.head_pose
            rw = store.right_arm_pose
            rh = store.right_hand_positions

            head_ok = "OK" if not _is_invalid(head) else "—"
            rw_ok = "OK" if not _is_invalid(rw) else "—"
            rh_ok = "OK" if not _is_invalid_kp(rh) else "—"

            if dhand > 0 and not _is_invalid_kp(rh):
                wrist0 = rh[0]
                pos_str = f"  R.wrist=[{wrist0[0]:+.3f},{wrist0[1]:+.3f},{wrist0[2]:+.3f}]"
            else:
                pos_str = ""

            print(
                f"[xr_pose_diag] {dmsg/dt_real:6.1f} msg/s "
                f"(head {dhead/dt_real:5.1f}/s, hand {dhand/dt_real:5.1f}/s)  "
                f"head={head_ok} rArm={rw_ok} rHand={rh_ok}{pos_str}",
                flush=True,
            )
    except KeyboardInterrupt:
        print("\n[xr_pose_diag] stopped.")


def _measure(store: BridgePoseStore, duration: float, report: Optional[Path]) -> dict:
    """`duration` 초 측정. 결과 dict 반환. report 경로 주어지면 markdown append."""
    print(f"[xr_pose_diag] measuring for {duration:.0f} sec ...")
    print("[xr_pose_diag] 권장 사용 패턴:")
    print("              1) 처음 ~10초: 자연스러운 손/팔 움직임")
    print("              2) 한 번 손을 시야 밖으로 5초간 뺐다가 다시 들이밀기 (recovery 측정)")
    print("              3) 마지막 5초: 손 정지 (wrist jitter 측정)")
    print()

    # polling 200 Hz — sender 가 보통 보는 read 빈도
    poll_hz = 200.0
    poll_dt = 1.0 / poll_hz
    n_samples = int(duration * poll_hz)

    # field 별 stats
    fields = ["head", "left_arm", "right_arm", "left_hand", "right_hand"]
    lost = {f: 0 for f in fields}
    nan = {f: 0 for f in fields}

    # wrist position trail (jitter)
    rw_positions = []     # (n, 3) 우측 wrist position
    rw_valid = np.zeros(n_samples, dtype=bool)

    # ws msg rate
    prev_msg = store.get_stats()["msg_count"]
    prev_t = time.perf_counter()
    msg_per_sec = []

    t0 = time.perf_counter()
    for i in range(n_samples):
        # read all fields
        head = store.head_pose
        la = store.left_arm_pose
        ra = store.right_arm_pose
        lh = store.left_hand_positions
        rh = store.right_hand_positions

        for f, val in [("head", head), ("left_arm", la), ("right_arm", ra)]:
            if _is_invalid(val):
                lost[f] += 1
            elif not np.isfinite(val).all():
                nan[f] += 1
        for f, val in [("left_hand", lh), ("right_hand", rh)]:
            if _is_invalid_kp(val):
                lost[f] += 1
            elif not np.isfinite(val).all():
                nan[f] += 1

        # wrist position (right) for jitter
        if not _is_invalid(ra):
            rw_positions.append(ra[:3, 3].copy())
            rw_valid[i] = True
        else:
            rw_positions.append(np.array([np.nan, np.nan, np.nan]))

        # 1초 간격으로 msg rate 측정
        now = time.perf_counter()
        if now - prev_t >= 1.0:
            cur_msg = store.get_stats()["msg_count"]
            msg_per_sec.append((cur_msg - prev_msg) / (now - prev_t))
            prev_msg, prev_t = cur_msg, now

        # sleep
        elapsed = time.perf_counter() - t0
        target_t = (i + 1) * poll_dt
        if target_t > elapsed:
            time.sleep(target_t - elapsed)

    rw_positions = np.array(rw_positions)

    # recovery latency: rw_valid 가 False→True 전환 시 lost streak 의 길이 중 max
    recovery_streak = 0
    max_recovery = 0
    for v in rw_valid:
        if not v:
            recovery_streak += 1
        else:
            if recovery_streak > 0:
                max_recovery = max(max_recovery, recovery_streak)
            recovery_streak = 0
    recovery_latency_s = max_recovery * poll_dt

    # jitter: 마지막 5초 valid wrist pos 의 stdev
    last_5s_idx = max(0, n_samples - int(5 * poll_hz))
    last_pos = rw_positions[last_5s_idx:]
    last_pos = last_pos[~np.isnan(last_pos[:, 0])]
    if len(last_pos) >= 10:
        wrist_jitter_cm = float(np.linalg.norm(np.std(last_pos, axis=0)) * 100.0)
    else:
        wrist_jitter_cm = float("nan")

    mean_freq_hz = float(np.mean(msg_per_sec)) if msg_per_sec else 0.0

    result = {
        "duration_s": duration,
        "n_samples": n_samples,
        "mean_freq_hz": mean_freq_hz,
        "lost_per_field": lost,
        "nan_per_field": nan,
        "recovery_latency_s": recovery_latency_s,
        "wrist_jitter_cm": wrist_jitter_cm,
        "total_msgs": store.get_stats()["msg_count"],
        "timestamp": datetime.now().isoformat(timespec="seconds"),
    }

    # print
    print()
    print("─" * 60)
    print(f"[xr_pose_diag] measurement complete ({duration:.0f}s)")
    print(f"  mean_freq_hz        : {mean_freq_hz:.1f}  (target ≥ 30)")
    print(f"  lost_per_field      :")
    for f, c in lost.items():
        pct = 100.0 * c / n_samples
        flag = "✓" if pct < 5.0 else "✗"
        print(f"      {f:11s} {c:6d} / {n_samples} ({pct:5.1f}%) {flag}")
    print(f"  recovery_latency_s  : {recovery_latency_s:.3f}  (target < 1.0)")
    print(f"  wrist_jitter_cm     : {wrist_jitter_cm:.2f}  (target < 2.0, 정지 5초간)")
    print(f"  total ws msgs       : {result['total_msgs']}")
    print("─" * 60)

    if report:
        _append_report(report, result)
        print(f"[xr_pose_diag] report appended → {report}")

    return result


def _append_report(report_path: Path, result: dict) -> None:
    """markdown 표로 append."""
    report_path.parent.mkdir(parents=True, exist_ok=True)
    is_new = not report_path.exists()
    lost = result["lost_per_field"]
    n = result["n_samples"]
    with report_path.open("a") as f:
        if is_new:
            f.write("# xr_pose_diag 측정 결과\n\n")
        f.write(f"## {result['timestamp']}\n\n")
        f.write(f"| 항목 | 값 | 통과 기준 |\n")
        f.write(f"|---|---|---|\n")
        f.write(f"| mean_freq_hz | {result['mean_freq_hz']:.1f} | ≥ 30 |\n")
        f.write(f"| recovery_latency_s | {result['recovery_latency_s']:.3f} | < 1.0 |\n")
        f.write(f"| wrist_jitter_cm | {result['wrist_jitter_cm']:.2f} | < 2.0 |\n")
        for fld, c in lost.items():
            pct = 100.0 * c / n
            f.write(f"| lost_{fld} | {c}/{n} ({pct:.1f}%) | < 5% |\n")
        f.write(f"| total_msgs | {result['total_msgs']} | — |\n\n")


def main() -> int:
    parser = argparse.ArgumentParser(description="XR pose-only diagnostic for teleop_dev")
    parser.add_argument("--port", type=int, default=None,
                        help="ws bridge port (default 8013 or env XR_BRIDGE_PORT)")
    parser.add_argument("--measure", type=float, default=None, metavar="SEC",
                        help="정량 측정 모드 (지정한 초만큼 측정 후 결과 출력 + 종료)")
    parser.add_argument("--report", type=str, default=None,
                        help="--measure 결과를 markdown 으로 append 할 경로")
    parser.add_argument("--no-wait", action="store_true",
                        help="첫 ws msg 대기 안 함 (smoke 모드 즉시 시작)")
    args = parser.parse_args()

    store = BridgePoseStore(use_hand_tracking=True, port=args.port)
    print(f"[xr_pose_diag] ws server: http://localhost:{store.port}/")
    print(f"[xr_pose_diag] adb reverse tcp:{store.port} tcp:{store.port} 후 헤드셋 Chrome 접속")
    print(f"[xr_pose_diag] Enter VR (또는 AR) 클릭 + 손 들이밀기 시작")
    print()

    # 측정 모드면 첫 msg 도착 시점까지 대기 (max 60s) — 측정 시작점 정렬
    if args.measure is not None and not args.no_wait:
        print("[xr_pose_diag] 첫 ws 메시지 대기 (최대 60s) ...")
        wait_t0 = time.time()
        while time.time() - wait_t0 < 60.0:
            if store.get_stats()["msg_count"] > 0:
                print(f"[xr_pose_diag] 첫 msg 수신 — measurement 시작\n")
                break
            time.sleep(0.5)
        else:
            print("[xr_pose_diag] WARN: 60s 동안 msg 0 — 그래도 측정 진행")
            print("  → 헤드셋 Chrome 의 webxr_to_pose 페이지 'Enter VR/AR' 누른 후 손 들이미는지 확인")

    if args.measure is not None:
        report = Path(args.report) if args.report else None
        result = _measure(store, args.measure, report)
        # exit code: gate A 조건 모두 통과 시 0, 아니면 1
        gate_ok = (
            result["mean_freq_hz"] >= 30.0
            and result["recovery_latency_s"] < 1.0
            and all(100.0 * c / result["n_samples"] < 5.0
                    for c in result["lost_per_field"].values())
        )
        return 0 if gate_ok else 1

    _smoke(store)
    return 0


if __name__ == "__main__":
    sys.exit(main())
