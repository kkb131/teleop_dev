#!/usr/bin/env python3
"""XRArmSender._user_pose 회귀 테스트 (하드웨어/BridgePoseStore 불필요).

commit c8ed119 리팩토링에서 호출부만 남고 메서드 정의가 누락돼
'r' calibrate 시 AttributeError 가 났던 버그의 재발 방지:
  1. _user_pose 메서드가 클래스에 존재
  2. hand_side="right" → store.right_arm_pose 반환
  3. hand_side="left"  → store.left_arm_pose 반환
  4. _read_input / _calibrate_now 가 참조하는 self 속성이 전부 정의됨
     (인스턴스 생성 없이 클래스 코드 객체의 self.X 참조를 정적 대조)

BridgePoseStore 는 aiohttp 서버를 띄우는 싱글턴이라 __init__ 을 우회
(__new__ + 필요한 속성만 주입)해 순수 로직만 검증한다.

Usage: python3 -m sender.arm.tests.test_xr_sender_user_pose
"""

import sys

import numpy as np

from sender.arm.xr_sender import XRArmSender


class _FakeStore:
    def __init__(self):
        self.right_arm_pose = np.eye(4)
        self.right_arm_pose[:3, 3] = [1.0, 2.0, 3.0]
        self.left_arm_pose = np.eye(4)
        self.left_arm_pose[:3, 3] = [-1.0, -2.0, -3.0]


def _bare_sender(hand_side: str) -> XRArmSender:
    s = XRArmSender.__new__(XRArmSender)  # __init__ 우회 (BridgePoseStore 회피)
    s.hand_side = hand_side
    s._store = _FakeStore()
    return s


def _class_self_attrs(cls) -> set:
    """클래스의 모든 메서드 바이트코드에서 self.X 로 접근하는 이름 수집."""
    import dis
    import inspect

    names = set()
    for _, fn in inspect.getmembers(cls, inspect.isfunction):
        try:
            instructions = list(dis.get_instructions(fn))
        except TypeError:
            continue
        for i, ins in enumerate(instructions):
            if ins.opname == "LOAD_FAST" and ins.argval == "self":
                nxt = instructions[i + 1] if i + 1 < len(instructions) else None
                if nxt and nxt.opname in ("LOAD_ATTR", "LOAD_METHOD"):
                    # py3.12+ LOAD_ATTR argval 이 tuple 인 경우 방어
                    val = nxt.argval
                    names.add(val if isinstance(val, str) else val[-1])
    return names


def main() -> int:
    passed = 0
    failed = 0

    # Test 1: 메서드 존재 (c8ed119 누락 재발 방지)
    print("[TEST] _user_pose 메서드 존재 ...", end=" ")
    if callable(getattr(XRArmSender, "_user_pose", None)):
        print("PASS")
        passed += 1
    else:
        print("FAIL — XRArmSender._user_pose 미정의")
        failed += 1
        return 1  # 이후 테스트 무의미

    # Test 2/3: hand_side 분기
    for side, expect in (("right", [1.0, 2.0, 3.0]), ("left", [-1.0, -2.0, -3.0])):
        print(f"[TEST] hand_side={side} → {side}_arm_pose ...", end=" ")
        pose = _bare_sender(side)._user_pose()
        if pose.shape == (4, 4) and np.allclose(pose[:3, 3], expect):
            print("PASS")
            passed += 1
        else:
            print(f"FAIL — got {pose[:3, 3] if pose is not None else pose}")
            failed += 1

    # Test 4: self.X 참조 전수 대조 — 정의 안 된 속성 잔존 여부.
    # __init__ 에서 할당하는 이름 + 클래스/베이스 정의 이름을 합집합으로 허용.
    print("[TEST] self.X 참조 중 미정의 속성 없음 ...", end=" ")
    import dis
    import inspect

    assigned = set()
    for klass in XRArmSender.__mro__:
        for _, fn in inspect.getmembers(klass, inspect.isfunction):
            try:
                instructions = list(dis.get_instructions(fn))
            except TypeError:
                continue
            for i, ins in enumerate(instructions):
                if ins.opname == "STORE_ATTR":
                    prev = instructions[i - 1] if i > 0 else None
                    # self.X = ... 패턴만 (직전 LOAD_FAST self 는 스택 구조상 보장이
                    # 어려우므로 STORE_ATTR 전부 허용 — 보수적 합집합)
                    assigned.add(ins.argval)
        assigned.update(n for n in vars(klass) if not n.startswith("__"))

    referenced = _class_self_attrs(XRArmSender)
    missing = sorted(n for n in referenced if n not in assigned)
    if not missing:
        print("PASS")
        passed += 1
    else:
        print(f"FAIL — 미정의 참조: {missing}")
        failed += 1

    print(f"\n{passed} passed, {failed} failed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
