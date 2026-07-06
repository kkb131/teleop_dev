"""통합 실행기 (launcher) — teleop 시스템 구성요소 일괄 관리.

teleop_dev 의 다른 모듈을 import 하지 않는 self-contained 패키지
(표준 라이브러리 + PyYAML 만 사용). 구성요소는 yaml 에 데이터로 정의되고
subprocess 로 spawn 된다 — 기존 코드에 어떤 변경도 요구하지 않음.

    python3 -m launcher agent --config launcher/config/teleop_system.yaml
    python3 -m launcher gui   --config launcher/config/teleop_system.yaml --profile operator
"""
