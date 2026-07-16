"""로봇 PC 런처 — 브링업 CLI + 웹 대시보드.

teleop_dev 의 다른 모듈을 import 하지 않는 self-contained 패키지
(표준 라이브러리 + PyYAML [+ 선택: ruamel.yaml]). 구성요소는 yaml 에
데이터로 정의되고 subprocess 로 spawn 된다 — 기존 코드 무변경.

    # 로봇 PC — 웹 대시보드 데몬 (조종 PC 는 브라우저로 접속)
    python3 -m launcher.robot.web --config launcher/robot/config/robot.yaml

    # 로봇 PC — 터미널 브링업 (웹 없이)
    python3 -m launcher.robot.bringup --side both

코어 (config.py / process.py / manager.py) 는 프로세스 생명주기 엔진,
robot/ 은 로봇 PC 전용 CLI·웹·액션·상태 수집.
"""
