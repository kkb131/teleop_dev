#!/usr/bin/env bash
# ============================================================
#  Manus + Tesollo 올인원 테스트 러너
#
#  Usage:
#    bash run_tests.sh                  # Phase 1만 (하드웨어 불필요)
#    bash run_tests.sh --phase 2       # Phase 2만 (Manus 글러브)
#    bash run_tests.sh --phase 3 --hand-ip 169.254.186.72
#    bash run_tests.sh --all           # 전체 Phase
#    bash run_tests.sh --all --hand-ip 169.254.186.72
# ============================================================

set -euo pipefail

# ── 색상 ────────────────────────────────────────────────
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[0;33m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m' # No Color

# ── 기본값 ──────────────────────────────────────────────
PHASE=""
RUN_ALL=false
HAND_IP="169.254.186.72"
HAND_PORT=502
SDK_PATH="manus/sdk/libManusSDK.so"
HAND_SIDE="right"

# SDK 파일 fallback: Integrated 버전 자동 탐색
if [[ ! -f "$SDK_PATH" ]] && [[ -f "manus/sdk/libManusSDK_Integrated.so" ]]; then
    SDK_PATH="manus/sdk/libManusSDK_Integrated.so"
    echo -e "${CYAN}[INFO] Using Integrated SDK: ${SDK_PATH}${NC}"
fi

# ── 인자 파싱 ───────────────────────────────────────────
while [[ $# -gt 0 ]]; do
    case $1 in
        --phase)    PHASE="$2"; shift 2;;
        --all)      RUN_ALL=true; shift;;
        --hand-ip)  HAND_IP="$2"; shift 2;;
        --hand-port) HAND_PORT="$2"; shift 2;;
        --sdk-path) SDK_PATH="$2"; shift 2;;
        --hand)     HAND_SIDE="$2"; shift 2;;
        -h|--help)
            echo "Usage: bash run_tests.sh [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --phase N       Run specific phase (1-4)"
            echo "  --all           Run all phases"
            echo "  --hand-ip IP    DG5F hand IP (default: 169.254.186.72)"
            echo "  --hand-port P   DG5F Modbus port (default: 502)"
            echo "  --sdk-path PATH Path to libManusSDK.so"
            echo "  --hand SIDE     Hand side: left|right (default: right)"
            echo ""
            echo "Phases:"
            echo "  1  Software only (no hardware needed)"
            echo "  2  Manus glove (SDK + USB dongle + gloves)"
            echo "  3  Tesollo DG5F (network connection to hand)"
            echo "  4  Integration (gloves + hand + 2 PCs)"
            exit 0;;
        *)
            echo "Unknown option: $1"
            echo "Run: bash run_tests.sh --help"
            exit 1;;
    esac
done

# 기본: Phase 1만
if [[ "$PHASE" == "" ]] && [[ "$RUN_ALL" == false ]]; then
    PHASE="1"
fi

# ── 디렉토리 ────────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# ── 로그 파일 ───────────────────────────────────────────
LOG_DIR="$SCRIPT_DIR/test_logs"
mkdir -p "$LOG_DIR"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
LOG_FILE="$LOG_DIR/test_results_${TIMESTAMP}.log"

# ── 결과 추적 ───────────────────────────────────────────
declare -a TEST_NAMES=()
declare -a TEST_RESULTS=()
TOTAL_PASS=0
TOTAL_FAIL=0
TOTAL_SKIP=0

run_test() {
    local name="$1"
    local cmd="$2"
    local desc="$3"

    echo -e "\n${CYAN}${BOLD}[$name]${NC} $desc"
    echo "  CMD: $cmd"
    echo "---[$name] $desc---" >> "$LOG_FILE"
    echo "CMD: $cmd" >> "$LOG_FILE"

    TEST_NAMES+=("$name")

    if eval "$cmd" >> "$LOG_FILE" 2>&1; then
        echo -e "  Result: ${GREEN}PASS${NC}"
        echo "Result: PASS" >> "$LOG_FILE"
        TEST_RESULTS+=("PASS")
        TOTAL_PASS=$((TOTAL_PASS + 1))
    else
        echo -e "  Result: ${RED}FAIL${NC}"
        echo "  (See log: $LOG_FILE)"
        echo "Result: FAIL" >> "$LOG_FILE"
        TEST_RESULTS+=("FAIL")
        TOTAL_FAIL=$((TOTAL_FAIL + 1))
    fi
    echo "" >> "$LOG_FILE"
}

skip_test() {
    local name="$1"
    local desc="$2"
    local reason="$3"

    echo -e "\n${YELLOW}[$name]${NC} $desc — ${YELLOW}SKIP${NC} ($reason)"
    TEST_NAMES+=("$name")
    TEST_RESULTS+=("SKIP")
    TOTAL_SKIP=$((TOTAL_SKIP + 1))
}

print_header() {
    local phase_num="$1"
    local phase_title="$2"
    local phase_hw="$3"

    echo ""
    echo -e "${BOLD}=========================================================${NC}"
    echo -e "${BOLD}  Phase $phase_num: $phase_title${NC}"
    echo -e "  $phase_hw"
    echo -e "${BOLD}=========================================================${NC}"
}

# ── Phase 1: 소프트웨어 검증 ────────────────────────────
run_phase_1() {
    print_header 1 "소프트웨어 검증" "하드웨어 불필요"

    run_test "M0" \
        "python3 -m manus.tests.test_step0_deps" \
        "시스템 의존성 체크 (gcc, libusb, Python, numpy, pynput)"

    run_test "T1" \
        "python3 -m tesollo.tests.test_retarget" \
        "Manus→DG5F 리타겟 로직 (10개 단위 테스트)"

    run_test "T2" \
        "python3 -m tesollo.tests.test_e2e" \
        "E2E mock 파이프라인 (UDP→리타겟→검증)"

    run_test "M4" \
        "python3 -m manus.tests.test_step4_udp" \
        "UDP 송수신 (mock 데이터)"
}

# ── Phase 2: Manus 글러브 ───────────────────────────────
run_phase_2() {
    print_header 2 "Manus 글러브" "SDK + USB 동글 + 글러브 필요"

    # SDK 파일 존재 확인
    if [[ ! -f "$SDK_PATH" ]]; then
        echo -e "\n  ${YELLOW}WARNING${NC}: SDK not found at $SDK_PATH"
        echo "  Download from: https://docs.manus-meta.com/2.4.0/Plugins/SDK/Linux/"
        echo "  Place libManusSDK.so in: manus/sdk/"
        skip_test "M1" "SDK 로드 + 동글 감지" "SDK file not found"
        skip_test "M2" "글러브 연결" "SDK required"
        skip_test "M3" "데이터 스트리밍" "SDK required"
        return
    fi

    run_test "M1" \
        "python3 -m manus.tests.test_step1_sdk --sdk-path $SDK_PATH" \
        "SDK 로드 + USB 동글 감지"

    # M1 결과 확인
    if [[ "${TEST_RESULTS[-1]}" == "FAIL" ]]; then
        echo -e "  ${YELLOW}NOTE${NC}: SDK 로드 또는 동글 감지 실패 — 글러브 테스트 스킵"
        skip_test "M2" "글러브 연결" "M1 failed"
        skip_test "M3" "데이터 스트리밍" "M1 failed"
        return
    fi

    echo ""
    echo -e "  ${CYAN}글러브 전원을 켜고 동글 근처에 위치시키세요.${NC}"
    echo -n "  준비되면 Enter... (s=skip) "
    read -r response
    if [[ "$response" == "s" || "$response" == "S" ]]; then
        skip_test "M2" "글러브 연결" "User skipped"
        skip_test "M3" "데이터 스트리밍" "User skipped"
        return
    fi

    run_test "M2" \
        "python3 -m manus.tests.test_step2_connection --hand $HAND_SIDE" \
        "글러브 연결 ($HAND_SIDE)"

    if [[ "${TEST_RESULTS[-1]}" == "FAIL" ]]; then
        skip_test "M3" "데이터 스트리밍" "M2 failed"
        return
    fi

    echo ""
    echo -e "  ${CYAN}글러브를 착용하고 손가락을 움직여 주세요.${NC}"
    echo -n "  준비되면 Enter... (s=skip) "
    read -r response
    if [[ "$response" == "s" || "$response" == "S" ]]; then
        skip_test "M3" "데이터 스트리밍" "User skipped"
        return
    fi

    run_test "M3" \
        "python3 -m manus.tests.test_step3_stream --duration 5 --hz 60 --hand $HAND_SIDE" \
        "5초 데이터 스트리밍 (60Hz)"
}

# ── Phase 3: Tesollo DG5F ───────────────────────────────
run_phase_3() {
    print_header 3 "Tesollo DG5F 핸드" "이더넷 연결 필요 (IP: $HAND_IP)"

    echo ""
    echo "  DG5F 핸드 연결 확인:"
    echo "    1. 이더넷 케이블로 PC와 DG5F 연결"
    echo "    2. PC IP를 같은 서브넷으로 설정 (예: 169.254.186.1/24)"
    echo "    3. ping $HAND_IP 으로 연결 확인"
    echo ""

    # 핑 테스트
    echo -e "  ${CYAN}Ping test...${NC}"
    if ping -c 1 -W 2 "$HAND_IP" > /dev/null 2>&1; then
        echo -e "  ${GREEN}PASS${NC}: $HAND_IP reachable"
    else
        echo -e "  ${RED}FAIL${NC}: $HAND_IP not reachable"
        echo -e "  ${YELLOW}NOTE${NC}: 네트워크 설정을 확인하세요."
        echo -n "  계속 진행? (y/n) "
        read -r response
        if [[ "$response" != "y" && "$response" != "Y" ]]; then
            skip_test "T3" "Modbus TCP 연결" "Hand not reachable"
            return
        fi
    fi

    run_test "T3" \
        "python3 -m tesollo.tests.test_modbus --ip $HAND_IP --port $HAND_PORT --hand $HAND_SIDE" \
        "Modbus TCP 연결 + 레지스터 읽기"
}

# ── Phase 4: 통합 테스트 ────────────────────────────────
run_phase_4() {
    print_header 4 "통합 테스트" "글러브 + 핸드 + 두 대의 PC"

    echo ""
    echo -e "  ${BOLD}통합 테스트는 수동으로 진행합니다:${NC}"
    echo ""
    echo "  ┌─────────────────────────────────────────────────────┐"
    echo "  │  Operator PC (Manus 글러브)                         │"
    echo "  │    conda activate tamp_sender                       │"
    echo "  │    cd ~/tamp_ws/src/tamp_dev                        │"
    echo "  │    python3 -m manus.manus_sender \\                  │"
    echo "  │        --target-ip <ROBOT_PC_IP> --hand $HAND_SIDE  │"
    echo "  └──────────────────────────┬──────────────────────────┘"
    echo "                             │ UDP:9872"
    echo "  ┌──────────────────────────▼──────────────────────────┐"
    echo "  │  Robot PC (AGX Orin)                                │"
    echo "  │    conda activate tamp_sender                       │"
    echo "  │    cd ~/tamp_ws/src/tamp_dev                        │"
    echo "  │    python3 -m tesollo.receiver \\                    │"
    echo "  │        --hand-ip $HAND_IP --hand $HAND_SIDE         │"
    echo "  └─────────────────────────────────────────────────────┘"
    echo ""
    echo "  Dry-run (DG5F 없이 리타겟 출력만 확인):"
    echo "    python3 -m tesollo.receiver --dry-run"
    echo ""
    echo "  비주얼라이저 (Operator PC에서):"
    echo "    python3 -m manus.hand_visualizer              # mock 데이터"
    echo "    python3 -m manus.hand_visualizer --sdk        # 실제 글러브"
    echo "    python3 -m manus.hand_visualizer --udp        # UDP 수신"
    echo ""
}

# ── 서머리 ──────────────────────────────────────────────
print_summary() {
    local total=$((TOTAL_PASS + TOTAL_FAIL + TOTAL_SKIP))

    echo ""
    echo -e "${BOLD}=========================================================${NC}"
    echo -e "${BOLD}  테스트 결과 요약${NC}"
    echo -e "${BOLD}=========================================================${NC}"
    echo ""

    # 개별 결과
    for i in "${!TEST_NAMES[@]}"; do
        local name="${TEST_NAMES[$i]}"
        local result="${TEST_RESULTS[$i]}"
        case $result in
            PASS) echo -e "  ${GREEN}[PASS]${NC} $name";;
            FAIL) echo -e "  ${RED}[FAIL]${NC} $name";;
            SKIP) echo -e "  ${YELLOW}[SKIP]${NC} $name";;
        esac
    done

    echo ""
    echo -e "  Total: $total | ${GREEN}Pass: $TOTAL_PASS${NC} | ${RED}Fail: $TOTAL_FAIL${NC} | ${YELLOW}Skip: $TOTAL_SKIP${NC}"

    if [[ $TOTAL_FAIL -eq 0 ]]; then
        echo -e "\n  ${GREEN}${BOLD}ALL TESTS PASSED!${NC}"
    else
        echo -e "\n  ${RED}${BOLD}$TOTAL_FAIL TEST(S) FAILED${NC}"
        echo "  상세 로그: $LOG_FILE"
    fi

    echo -e "${BOLD}=========================================================${NC}"
    echo "  Log saved: $LOG_FILE"
    echo ""
}

# ── 메인 ────────────────────────────────────────────────

echo -e "${BOLD}=========================================================${NC}"
echo -e "${BOLD}  Manus + Tesollo 올인원 테스트 러너${NC}"
echo -e "  $(date '+%Y-%m-%d %H:%M:%S')"
echo -e "  Log: $LOG_FILE"
echo -e "${BOLD}=========================================================${NC}"

echo "Date: $(date)" > "$LOG_FILE"
echo "Args: $*" >> "$LOG_FILE"
echo "" >> "$LOG_FILE"

if [[ "$RUN_ALL" == true ]]; then
    run_phase_1
    run_phase_2
    run_phase_3
    run_phase_4
elif [[ "$PHASE" == "1" ]]; then
    run_phase_1
elif [[ "$PHASE" == "2" ]]; then
    run_phase_2
elif [[ "$PHASE" == "3" ]]; then
    run_phase_3
elif [[ "$PHASE" == "4" ]]; then
    run_phase_4
else
    echo "Invalid phase: $PHASE (use 1-4 or --all)"
    exit 1
fi

print_summary

# 종료 코드
if [[ $TOTAL_FAIL -gt 0 ]]; then
    exit 1
fi
exit 0
