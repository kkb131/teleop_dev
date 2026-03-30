"""Tesollo DG5F 핸드 전체 관절을 0도로 이동시키는 기본 스크립트.

Usage:
    python3 -m tesollo.tests.test_zero_position --ip 169.254.186.72
    python3 -m tesollo.tests.test_zero_position --ip 169.254.186.72 --motion-time 1000
"""

import argparse
import time

from teleop_dev.robot.hand.dg5f_client import DG5FClient


def main():
    parser = argparse.ArgumentParser(description="Tesollo DG5F 전체 관절 0도 이동")
    parser.add_argument("--ip", default="169.254.186.72", help="DG5F Modbus TCP IP")
    parser.add_argument("--slave-id", type=int, default=1, help="Modbus slave ID (default: 1)")
    parser.add_argument("--motion-time", type=int, default=500, help="모션 시간(ms)")
    args = parser.parse_args()

    with DG5FClient(ip=args.ip, slave_id=args.slave_id) as client:
        print(f"연결됨: {args.ip}")

        # 현재 위치 출력
        cur = client.get_positions()
        print(f"현재 위치(rad): {[f'{v:.3f}' for v in cur]}")

        # 모터 시작
        client.start()

        # 모션 시간 설정 + 0도 이동
        client.set_motion_times([args.motion_time] * 20)
        client.set_positions([0.0] * 20)
        print("0도 명령 전송 완료")

        # 이동 완료 대기
        time.sleep(args.motion_time / 1000 + 0.5)

        # 결과 확인
        final = client.get_positions()
        print(f"최종 위치(rad): {[f'{v:.3f}' for v in final]}")

        client.stop()
        print("완료")


if __name__ == "__main__":
    main()
