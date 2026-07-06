"""통합 실행기 CLI.

    # robot PC — headless agent (robot group 컴포넌트 관리 + HTTP API)
    python3 -m launcher agent --config launcher/config/teleop_system.yaml

    # 조종 PC — GUI (operator group 로컬 관리 + robot group 원격 관리)
    python3 -m launcher gui --config launcher/config/teleop_system.yaml --profile operator

    # 한 PC 에서 전부 로컬 관리 (개발/시뮬)
    python3 -m launcher gui --config launcher/config/teleop_system.yaml --profile local
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from launcher.config import LauncherConfig

_DEFAULT_CONFIG = Path(__file__).parent / "config" / "teleop_system.yaml"


def main() -> int:
    parser = argparse.ArgumentParser(prog="launcher", description="teleop 통합 실행기")
    sub = parser.add_subparsers(dest="mode", required=True)

    p_agent = sub.add_parser("agent", help="headless agent 데몬 (robot PC)")
    p_agent.add_argument("--config", default=str(_DEFAULT_CONFIG))
    p_agent.add_argument("--host", default=None, help="bind host override")
    p_agent.add_argument("--port", type=int, default=None, help="bind port override")
    p_agent.add_argument("--groups", nargs="+", default=["robot"],
                         choices=["robot", "operator"],
                         help="agent 가 관리할 group (default: robot)")

    p_gui = sub.add_parser("gui", help="tkinter GUI")
    p_gui.add_argument("--config", default=str(_DEFAULT_CONFIG))
    p_gui.add_argument("--profile", default="operator",
                       choices=["operator", "robot", "local"],
                       help="operator: operator=로컬 + robot=원격(agent) / "
                            "robot: robot group 만 로컬 / local: 전부 로컬")
    p_gui.add_argument("--agent-url", default=None,
                       help="robot agent URL override (기본: http://<robot_pc_ip>:<agent_port>)")

    args = parser.parse_args()
    config = LauncherConfig.load(args.config)

    if args.mode == "agent":
        if args.host:
            config.agent.host = args.host
        if args.port:
            config.agent.port = args.port
        from launcher.agent import serve
        return serve(config, groups=args.groups)

    if args.mode == "gui":
        from launcher.gui import run_gui
        return run_gui(config, profile=args.profile, agent_url=args.agent_url)

    return 1


if __name__ == "__main__":
    sys.exit(main())
