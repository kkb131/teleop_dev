"""조종 PC 런처 — XR 러너 관리 + 웹 대시보드 (포트 9877).

    # 웹 대시보드 (헤드셋 브라우저에서도 접속 가능 — adb reverse 9877)
    python3 -m launcher.sender.web --config launcher/sender/config/sender.yaml

    # 터미널 브링업 (adb reverse 후 러너 실행)
    python3 -m launcher.sender.bringup --mode dual

기능: XR 러너(단일/듀얼) 시작·정지, adb 연결/reverse 상태·실행, 웹 키패드
(r/p/c/Space/x → pty), "조종 시작" 자동 캘리브레이션 시퀀스, xr_dual.yaml
파라미터 편집. launcher/robot 과 같은 self-contained 원칙 (teleop 코드는
subprocess 로만 실행).
"""
