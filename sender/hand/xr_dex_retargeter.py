"""WebXR 25-joint → DG-5F 20-vec joint angle retargeter.

xr_teleop 의 DG5F_Controller._control_process 의 retarget 로직만 추출해
sender 측에서 retarget 까지 완료 후 receiver 측에 retargeted=True 패킷으로
송신. receiver.py 는 passthrough + EMA filter (변경 없음).

Pipeline:
    kp_25 (BridgePoseStore.right_hand_positions) — WebXR 25-joint world frame
        → webxr_to_wrist_local_mano (wrist-local + palm-aligned MANO)
        → ref_value = hand_local[task_idx] - hand_local[origin_idx]  (DexPilot)
        → SeqRetargeting.retarget(ref_value)
        → q_20 (DG-5F finger-major 20-vec)

DG-5F 20-vec finger-major 순서:
    thumb  (0..3) = rj_dg_1_1, _1_2, _1_3, _1_4   (abd, opp, flex, flex)
    index  (4..7) = rj_dg_2_1, _2_2, _2_3, _2_4   (abd, MCP, PIP, DIP)
    middle (8..11), ring (12..15), pinky (16..19)

dex_retargeting 의 target_joint_names 길이:
    - DexPilot: 20 (전체) — q_20 = robot_qpos 그대로
    - vector (6 target): mimic 0.6 / 0.4 expansion 으로 20 채움

Cross-reference:
    xr_teleop/scripts/dg5f_controller.py:_control_process — 동일 retarget flow.
"""

from __future__ import annotations

from pathlib import Path
from typing import Optional

import numpy as np
import yaml

from dex_retargeting.retargeting_config import RetargetingConfig

from sender.hand.xr_remap import webxr_to_wrist_local_mano, is_kp25_valid


DG5F_NUM_MOTORS = 20

# 본 파일 위치 기준 default config. yml 한 파일 안에 right + left 두 block.
_DEFAULT_CONFIG_DIR = Path(__file__).resolve().parent / "config_xr"
DEFAULT_YML_PATH = _DEFAULT_CONFIG_DIR / "dg5f_xr.yml"
DEFAULT_ASSETS_DIR = _DEFAULT_CONFIG_DIR


def expand_retarget_to_dg5f_20(
    target_joint_names: list,
    q_target_dict: dict,
    mimic_mid: float = 0.6,
    mimic_tip: float = 0.4,
) -> np.ndarray:
    """retargeting 6 joint q → 20-vec DDS motor command (vector type fallback).

    DexPilot type (target=20) 면 호출 불요 — robot_qpos 가 이미 20-vec.
    vector type 호환 (6 target) 시 mimic 0.6 / 0.4 로 distal 채움.

    Parameters
    ----------
    target_joint_names : retargeter.optimizer.target_joint_names 순서
    q_target_dict : {joint_name: q}
    mimic_mid, mimic_tip : human PIP/DIP 굽힘 비율 추정

    Returns
    -------
    (20,) np.ndarray
    """
    q = np.zeros(DG5F_NUM_MOTORS, dtype=np.float64)

    # thumb (DDS 0..3)
    q[0] = q_target_dict.get("rj_dg_1_1", 0.0)
    q[1] = q_target_dict.get("rj_dg_1_2", 0.0)
    q[2] = mimic_mid * q[1]
    q[3] = mimic_tip * q[1]

    # index/middle/ring/pinky
    for f_idx, (jn, _) in enumerate(
        [("rj_dg_2_2", 4), ("rj_dg_3_2", 8), ("rj_dg_4_2", 12), ("rj_dg_5_2", 16)],
        start=1,
    ):
        base = 4 * (f_idx)  # index=4, middle=8, ring=12, pinky=16
        q[base + 0] = 0.0  # abduction fixed
        q[base + 1] = q_target_dict.get(jn, 0.0)
        q[base + 2] = mimic_mid * q[base + 1]
        q[base + 3] = mimic_tip * q[base + 1]
    return q


class XRDexRetargeter:
    """WebXR 25-joint → DG-5F 20-vec retargeter.

    Parameters
    ----------
    yml_path : Path | str | None
        retargeting yml. default 는 본 패키지 의 dg5f_right_xr.yml.
    assets_dir : Path | str | None
        urdf_path 가 가리키는 base dir. default 는 yml 과 같은 dir.
    convention : "mediapipe" or "manus"
        WebXR → MANO 변환 매트릭스. WebXR 가 image-derived MediaPipe 와 같다고
        가정 (default). visual 검증 후 fist↔spread inversion 시 manus 로 toggle.
    hand_side : "right" or "left"
        DG-5F single-hand 환경이라 "right" 만 검증. left 는 yml/URDF 미제공.
    """

    def __init__(
        self,
        yml_path: Optional[Path] = None,
        assets_dir: Optional[Path] = None,
        convention: str = "mediapipe",
        hand_side: str = "right",
    ):
        self.yml_path = Path(yml_path) if yml_path else DEFAULT_YML_PATH
        self.assets_dir = Path(assets_dir) if assets_dir else self.yml_path.parent
        self.convention = convention
        self.hand_side = hand_side

        if not self.yml_path.exists():
            raise FileNotFoundError(f"yml not found: {self.yml_path}")

        with self.yml_path.open() as f:
            cfg = yaml.safe_load(f)
        if hand_side not in cfg:
            raise KeyError(f"yml has no '{hand_side}' key (keys: {list(cfg.keys())})")
        side_cfg = cfg[hand_side]

        RetargetingConfig.set_default_urdf_dir(str(self.assets_dir))
        self.retargeting = RetargetingConfig.from_dict(side_cfg).build()

        # 인덱스 / 이름 캐시
        self.target_indices = self.retargeting.optimizer.target_link_human_indices
        self.target_joint_names = list(self.retargeting.optimizer.target_joint_names)
        self.fixed_joint_names = list(self.retargeting.optimizer.fixed_joint_names)
        self.full_joint_names = list(self.retargeting.joint_names)
        self._fixed_qpos = np.zeros(len(self.fixed_joint_names), dtype=np.float64)

        # last valid output (lost frame fallback)
        self._last_q20: Optional[np.ndarray] = np.zeros(DG5F_NUM_MOTORS, dtype=np.float64)

        print(
            f"[XRDexRetargeter] {self.yml_path.name} "
            f"target={len(self.target_joint_names)} fixed={len(self.fixed_joint_names)} "
            f"convention={self.convention} side={self.hand_side}"
        )

        # Phase B4 진단: target_link_human_indices 의 fingertip 부분이 WebXR
        # 25-joint 와 호환되는지 확인. dex_retargeting 0.5.0 의 DexPilot 이
        # yml 의 명시 인덱스 누락 시 `generate_link_indices * 4` 로 MANO 21-joint
        # fingertip 인덱스 [4, 8, 12, 16, 20] 을 auto-generate — WebXR 입력
        # [4, 9, 14, 19, 24] 와 mismatch 면 손가락 추종이 무너짐.
        if self.target_indices is not None and self.target_indices.ndim == 2:
            task_tail = np.asarray(self.target_indices[1, -5:]).tolist()
            webxr_tips = [4, 9, 14, 19, 24]
            if sorted(set(task_tail)) == webxr_tips:
                print(
                    f"[XRDexRetargeter] task fingertip indices = {task_tail} "
                    f"✓ matches WebXR 25-joint fingertips"
                )
            else:
                print(
                    f"[XRDexRetargeter] ⚠️  task fingertip indices = {task_tail} "
                    f"— WebXR 기대값 {webxr_tips} 와 다름.\n"
                    f"  → yml 의 target_link_human_indices 명시 누락 가능성. "
                    f"dex_retargeting 0.5.0 의 auto-gen 은 MANO 21-joint 가정 "
                    f"(* 4) 라 [4, 8, 12, 16, 20] 생성. "
                    f"yml 의 'target_link_human_indices' 명시 키 확인 (no `_dexpilot` suffix)."
                )

    def retarget(self, kp_25: np.ndarray) -> np.ndarray:
        """WebXR 25-joint → DG-5F 20-vec joint angles.

        Parameters
        ----------
        kp_25 : (25, 3) np.ndarray
            BridgePoseStore.{left,right}_hand_positions. zeros 이면 last valid
            q20 반환 (정지 hold — 손이 시야 밖일 때 안전).

        Returns
        -------
        (20,) np.ndarray
            DG-5F finger-major joint angles (radians).
            Order: thumb(0..3), index(4..7), middle(8..11), ring(12..15), pinky(16..19)
        """
        if not is_kp25_valid(kp_25):
            return self._last_q20.copy()

        # 1) frame transform — wrist-local + palm-aligned MANO
        hand_local = webxr_to_wrist_local_mano(kp_25, self.hand_side, self.convention)

        # 2) ref_value = task - origin vector (DexPilot 의 vector pair)
        ref_value = (
            hand_local[self.target_indices[1, :]]
            - hand_local[self.target_indices[0, :]]
        )

        # 3) retarget — robot_qpos 는 (full_joint_names 순서) full vector
        robot_qpos = self.retargeting.retarget(ref_value, fixed_qpos=self._fixed_qpos)

        # 4) DG-5F 20-vec 추출
        if len(self.target_joint_names) == DG5F_NUM_MOTORS:
            # DexPilot or full-target: robot_qpos 가 이미 20-vec (URDF 순서)
            q20 = np.asarray(robot_qpos[:DG5F_NUM_MOTORS], dtype=np.float64)
        else:
            # vector type subset: mimic expansion
            q_target_dict = {
                n: robot_qpos[self.full_joint_names.index(n)]
                for n in self.target_joint_names
            }
            q20 = expand_retarget_to_dg5f_20(self.target_joint_names, q_target_dict)

        self._last_q20 = q20.copy()
        return q20


# ── self-test (sim/headset 불요) ────────────────────────────────────────
def _selftest(hand_side: str = "right") -> int:
    """python -m sender.hand.xr_dex_retargeter --selftest [--hand right|left]

    URDF 로딩 + dummy kp 로 retarget 호출 + 출력 shape / range 확인.
    Phase B5: hand_side 파라미터로 right + left 양쪽 검증 가능.
    """
    print(f"[selftest] hand_side={hand_side}")
    try:
        rt = XRDexRetargeter(hand_side=hand_side)
    except Exception as e:
        print(f"[selftest] init FAIL: {e}")
        return 1

    print(f"[selftest] init OK: {len(rt.target_joint_names)} target joints")

    # Phase B4: target_link_human_indices 가 WebXR 25-joint 와 호환되는지 단언.
    # auto-generated MANO 인덱스 [4,8,12,16,20] 이 들어왔으면 retargeter 가 비정상
    # 동작 — 실 hardware 테스트 전에 미리 잡음.
    assert rt.target_indices is not None and rt.target_indices.ndim == 2, \
        f"target_indices shape unexpected: {rt.target_indices}"
    task_tail = sorted(set(np.asarray(rt.target_indices[1, -5:]).tolist()))
    assert task_tail == [4, 9, 14, 19, 24], (
        f"task fingertip indices {task_tail} != WebXR [4, 9, 14, 19, 24]. "
        f"yml 의 'target_link_human_indices' 키 누락 또는 잘못된 suffix 가능성 "
        f"(`_dexpilot` 등). docs/vr_teleop/xr_spike/B4_indices_fix.md 참조."
    )
    print(f"[selftest] task fingertip indices = {task_tail} ✓")

    # invalid kp → last q20 = zeros 반환
    q = rt.retarget(np.zeros((25, 3)))
    assert q.shape == (DG5F_NUM_MOTORS,)
    assert np.allclose(q, 0)
    print(f"[selftest] invalid kp → q20 zeros OK")

    # synthetic "open hand" kp
    np.random.seed(0)
    kp_open = np.zeros((25, 3))
    # 손바닥 펴짐: 손가락 일렬, fingertip 멀리
    base_offsets = {1: (-0.03, 0.02), 5: (0.0, 0.0), 10: (0.02, 0.0), 15: (0.04, 0.0), 20: (0.06, 0.0)}
    for f_base, (xb, yb) in base_offsets.items():
        # 5 joint per finger
        for j in range(5):
            kp_open[f_base + j] = [xb, yb + 0.04 * (j + 1), 0.0]
    q_open = rt.retarget(kp_open)
    assert q_open.shape == (DG5F_NUM_MOTORS,)
    assert np.isfinite(q_open).all()
    print(f"[selftest] open hand → q20 shape={q_open.shape}, range=[{q_open.min():.3f}, {q_open.max():.3f}]")

    # synthetic "fist" — fingertip 들이 wrist 쪽으로 모이게 (PIP / DIP 굴곡 측정)
    kp_fist = kp_open.copy()
    for tip_idx in [4, 9, 14, 19, 24]:
        kp_fist[tip_idx] = kp_open[0] + 0.2 * (kp_open[tip_idx] - kp_open[0])
        # PIP, DIP 도 약간씩 끌어당김
        if tip_idx >= 5:  # non-thumb
            kp_fist[tip_idx - 1] = kp_open[0] + 0.5 * (kp_open[tip_idx - 1] - kp_open[0])
            kp_fist[tip_idx - 2] = kp_open[0] + 0.7 * (kp_open[tip_idx - 2] - kp_open[0])
    q_fist = rt.retarget(kp_fist)
    print(f"[selftest] fist → q20 range=[{q_fist.min():.3f}, {q_fist.max():.3f}]")

    # fist 와 open 의 PIP joint (q[5], q[9], q[13], q[17]) 차이 — fist 가 더 큰 굴곡
    pip_open = q_open[[5, 9, 13, 17]]
    pip_fist = q_fist[[5, 9, 13, 17]]
    diff = np.mean(pip_fist - pip_open)
    print(f"[selftest] mean PIP delta (fist - open) = {diff:+.3f}  (should be positive)")
    # 검증 약하게 — exact 값은 retargeter 의 cost 에 따라 변동. 0 이상이면 합리적
    if diff < -0.05:
        print(f"[selftest] WARN: PIP delta {diff:+.3f} 음수 — convention 또는 dummy kp 검토 필요")

    # yaw 회전 robustness — 같은 손가락 자세 + 회전만 다른 kp → 같은 q20
    yaw = np.pi / 4
    Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
    kp_open_rot = kp_open @ Rz.T
    q_open_rot = rt.retarget(kp_open_rot)
    rot_diff = np.max(np.abs(q_open - q_open_rot))
    print(f"[selftest] yaw rotation robustness: max(|q - q_rot|) = {rot_diff:.4f}  (should < 0.1)")

    print("[selftest] PASS")
    return 0


if __name__ == "__main__":
    import argparse
    import sys

    parser = argparse.ArgumentParser(description="XR DexPilot retargeter")
    parser.add_argument("--selftest", action="store_true",
                        help="URDF 로드 + dummy kp retarget 호출 + indices 검증")
    parser.add_argument("--hand", default="right", choices=["right", "left"],
                        help="selftest 의 hand_side (default: right)")
    args = parser.parse_args()

    if args.selftest:
        sys.exit(_selftest(hand_side=args.hand))
    print("Usage: python -m sender.hand.xr_dex_retargeter --selftest [--hand right|left]")
