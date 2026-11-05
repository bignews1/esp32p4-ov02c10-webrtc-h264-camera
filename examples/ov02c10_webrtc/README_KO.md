# OV02C10 WebRTC 공개 예제

ESP32-P4 Function EV Board와 OV02C10 카메라로 시작하는 별도 예제입니다.
비공개 홈패널 프로젝트 없이, 지정된 버전의 Espressif 공개 예제를 가져와
OV02C10 드라이버를 연결합니다. 최신 최적화 소스는 저장소의 `src/`에 보존합니다.

## 구성

- OV02C10 2레인 1920×1080/30fps 요청, H.264 WebRTC 영상
- 공식 예제의 오디오와 브라우저 접속 화면
- Wi-Fi 정보는 로컬 `sdkconfig`에만 저장
- HTTPS 키는 PC에서 새로 생성, Git 업로드 제외
- 고급 OWNER 인식·추적, 사용자 프로젝트의 QP/GOP·DMA 최적화는 기본 예제에
  이식하지 않았습니다. 먼저 기본 영상을 확인한 뒤 참고 소스로 통합합니다.

## 실행 순서

ESP-IDF **5.5.4 터미널**에서 공개 저장소를 별도 폴더로 받습니다.
현재 사용 중인 `ha_panel` 폴더에는 설치하지 않습니다.

```text
git clone https://github.com/bignews1/esp32p4-ov02c10-webrtc-h264-camera.git p4-camera-demo
cd p4-camera-demo/examples/ov02c10_webrtc
python -m pip install esp-bmgr-assist==0.8.4
python prepare.py
python build.py configure
python build.py menuconfig
```

`Public OV02C10 example` 메뉴에서 자신의 SSID와 비밀번호를 입력하고 저장합니다.
대상 칩 리비전은 이전 보드용 **1.x**이며, 이에 맞는 오디오 라이브러리를
고정했습니다. 3.x 보드는 별도 프로필과 호환 코덱이 필요합니다.
최소 리비전 값만 바꾸는 것으로 이식이 끝나지는 않습니다.

```text
python build.py build
python build.py verify
```

플래시할 시험 보드의 포트를 확인한 뒤 아래 명령을 실행합니다. `COM5`는 예시입니다.

```text
cd .work/esp-webrtc-solution/solutions/doorbell_local
idf.py -p COM5 flash monitor
```

시리얼 로그에 표시되는 `https://<보드 IP>/webrtc/test`로 같은 LAN의 브라우저에서
접속합니다. 자체 서명 인증서의 로컬 예외를 승인하고 연결합니다.
실제 화면의 시작/연결 버튼을 누른 뒤 영상부터 확인합니다.

`prepare.py`와 빌드 명령은 자동 플래시·clean·fullclean을 실행하지 않습니다.
준비를 다시 실행해도 기존 설정과 HTTPS 키를 보존합니다.
ESP32-C6에는 보드 및 호스트와 호환되는 ESP-Hosted 펌웨어가 필요합니다.

## 검증 상태

정확한 검사 결과는 [VALIDATION.md](VALIDATION.md)에 기록합니다.
ESP-IDF 5.5.4에서 새 폴더 전체 빌드와 리비전 100(1.0) 설정 검증을 통과했습니다.
컴파일 통과와 실제 카메라·오디오 작동은 서로 다른 검증입니다.
이 작업 환경에서는 실제 보드에 플래시하거나 카메라를 시험할 수 없습니다.

자세한 구성·요구사항·라이선스는 [영문 안내](README.md)를 참고하세요.
