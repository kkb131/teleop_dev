# Manus SDK 설치

이 디렉토리에 Manus SDK for Linux 파일을 배치하세요.

## 모드 선택

Manus SDK Linux는 두 가지 모드를 제공합니다:

| 모드 | 라이브러리 | 설명 |
|------|-----------|------|
| **Integrated** | `libManusSDK_Integrated.so` | Linux PC에 USB 동글 직접 연결. gRPC/Protobuf 불필요 |
| **Remote** | `libManusSDK.so` | Windows PC의 MANUS Core에 네트워크 연결. gRPC + Protobuf 필요 |

> 우리 프로젝트는 **Integrated Mode**를 사용합니다 (직접 USB 동글 연결).

## 다운로드

1. Manus 개발자 포털에서 SDK 다운로드:
   - https://docs.manus-meta.com/3.1.0/Plugins/SDK/Linux/
   - "Download SDK" → Linux 버전 선택

2. 다운로드한 아카이브를 이 디렉토리에 압축 해제

## 시스템 의존성 설치

### Integrated Mode (필수)

```bash
sudo apt-get install -y build-essential libusb-1.0-0-dev zlib1g-dev \
    libudev-dev libncurses5-dev gdb pkg-config
```

### Remote Mode (추가 — Integrated Mode에서는 불필요)

gRPC v1.28.1 + Protobuf v3.11.2를 소스에서 빌드해야 합니다:

```bash
# 추가 패키지
sudo apt-get install -y libtool libzmq3-dev git

# gRPC 클론 (Protobuf 포함)
git clone -b v1.28.1 https://github.com/grpc/grpc /var/local/git/grpc
cd /var/local/git/grpc
git submodule update --init

# Protobuf 빌드
cd third_party/protobuf
./autogen.sh && ./configure --enable-shared
make -j$(nproc) && sudo make install

# gRPC 빌드
cd /var/local/git/grpc
make -j$(nproc) && sudo make install
sudo ldconfig
```

> Ubuntu 24.04에서는 `export CC=/usr/bin/gcc-11 CXX=/usr/bin/g++-11` 필요할 수 있음.

## 필요한 파일

압축 해제 후 다음 파일이 이 디렉토리에 있어야 합니다:

```
sdk/
├── libManusSDK_Integrated.so   # Integrated Mode 라이브러리
├── libManusSDK.so              # Remote Mode 라이브러리 (선택)
├── ManusSDK.h                  # C API 헤더
├── ManusSDKTypes.h             # 타입 정의 헤더
└── (기타 SDK 파일들)
```

## 확인

```bash
# 라이브러리 파일 확인
ls -la operator/hand/sdk/libManusSDK*.so

# 심볼 확인
nm -D operator/hand/sdk/libManusSDK_Integrated.so | grep -i "CoreSdk"

# 의존 라이브러리 확인
ldd operator/hand/sdk/libManusSDK_Integrated.so
```

## 주의사항

- SDK 파일은 `.gitignore`에 추가되어 있습니다 (라이선스 제약)
- 각 테스트 PC에서 별도로 다운로드/배치해야 합니다
- SDK 버전에 따라 `manus_reader.py`의 ctypes 매핑 업데이트가 필요할 수 있습니다
- SDK 버전 변경 시 `ManusSDKTypes.h`의 struct 레이아웃을 확인하세요
