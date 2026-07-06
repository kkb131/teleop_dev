"""연결된 RealSense 장치 시리얼 나열.

사용:
    python3 -m robot.cam.list_cameras

출력된 시리얼을 robot/cam/config/default.yaml 의 cameras[].serial 또는
`python3 -m robot.cam.main --serials S1 S2` 에 사용. 장치 없으면 exit 1.
"""

import sys

from robot.cam.rs_color_camera import list_realsense_serials


def main() -> int:
    try:
        devices = list_realsense_serials()
    except ImportError:
        print("[list_cameras] pyrealsense2 미설치 — pip install pyrealsense2")
        return 1

    if not devices:
        print("[list_cameras] 연결된 RealSense 장치 없음")
        return 1

    print(f"[list_cameras] {len(devices)}개 장치:")
    for serial, name in devices:
        print(f"  serial={serial}  ({name})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
