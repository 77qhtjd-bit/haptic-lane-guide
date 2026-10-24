# 아두이노 업로드 가이드 (다른 컴퓨터용)

이 폴더의 `lane_actuator_ble` 스케치를 Arduino UNO R4 WiFi에 올리는 방법입니다. (1~2단계 공통 준비 → 아래 BLE 섹션)
이전 Wi-Fi/UDP 버전(`lane_actuator`)은 폐기되어 이 레포에는 넣지 않았습니다.
업로드가 끝나면 보드는 USB 전원만 있으면 동작합니다. 업로드한 컴퓨터는 더 이상 필요 없습니다.

## 1. Arduino IDE 2 설치와 보드 패키지
1. https://www.arduino.cc/en/software 에서 **Arduino IDE 2**를 설치합니다.
2. IDE에서 왼쪽 **Boards Manager**(보드 아이콘)를 열고 `UNO R4`를 검색합니다.
   **"Arduino UNO R4 Boards"**를 설치합니다.
3. 보드를 USB-C로 연결하고, 상단 보드 선택에서 **Arduino UNO R4 WiFi**와 해당 포트를 고릅니다.
   - Windows: 처음 연결하면 드라이버 설치 창이 뜰 수 있습니다. 설치를 허용하세요. 포트는 `COM3`처럼 보입니다.
   - Linux에서 포트가 안 보이거나 권한 오류가 나면 `sudo usermod -aG dialout $USER`를 실행한 뒤 로그아웃했다가 다시 로그인합니다.

## 2. 통신 모듈 펌웨어 업데이트
1. 시리얼 모니터를 포함해 보드를 쓰는 창을 모두 닫습니다.
2. **Tools → Firmware Updater**를 엽니다. 보드를 선택하고 **Check Updates**를 누릅니다.
   최신 펌웨어를 선택해 **Install**합니다.
3. ESP32-S3에 다른(커스텀) 펌웨어를 직접 올리지 마세요.
4. 스케치를 올린 뒤 시리얼 모니터에 `Please update the firmware`가 보이면 이 단계를 다시 합니다.

---

# (추가) 블루투스(BLE) 버전: `lane_actuator_ble` (팀 계획: 모터 5개)

**iPhone(Bluefy 브라우저) ↔ 아두이노를 BLE로 직접** 연결하는 버전입니다. **현재 사용하는 버전입니다.**

1. Arduino IDE → **Library Manager**(책 아이콘)에서 `ArduinoBLE`을 검색해 설치합니다. **코어에 포함되어 있지 않습니다.**
   `Arduino_LED_Matrix`는 코어에 포함되어 있습니다.
2. `lane_actuator_ble/lane_actuator_ble.ino`를 열고 Upload합니다.
3. 시리얼 모니터(9600)에 `Advertising as LaneActuator (5 motors)`가 보이면 정상입니다.
   `BLE.begin() failed`가 보이면 위 2번(펌웨어 업데이트)을 하세요.
4. 모터 핀 (왼쪽 → 오른쪽): **M1=D3 (크게 왼쪽), M2=D5 (작게 왼쪽), M3=D6 (직진 심장박동), M4=D9 (작게 오른쪽), M5=D10 (크게 오른쪽)**.
   모터는 MOSFET + 역전압 다이오드로 구동합니다(`laptop/README.md`의 Stage 4 배선도). 핀 번호와 펄스 길이 등은 스케치 맨 위 설정값에서 바꿀 수 있습니다.
5. 동작: 폰이 모터별 레벨(0~255)을 보내면 **아두이노가 펄스를 만듭니다.**
   0 = 꺼짐, 255 = 계속 켜짐, 1~254 = 60 ms 펄스, 주기는 레벨 1에서 1초 → 레벨 254에서 0.12초로 빨라짐.
   **300 ms 동안 명령이 없으면(또는 연결이 끊기면) 모든 모터가 꺼집니다.**
6. 매트릭스 화면:
   - 아래 줄 점선: 연결 대기 중(광고 중)
   - 위아래 줄 점선: 폰과 연결됨, 명령 없음
   - 세로 막대 5개(2, 4, 6, 8, 10번째 열 = M1~M5): 해당 모터가 켜져 있는 동안 켜짐. 막대 높이 = 레벨
7. 차선 판단까지 붙은 페이지: iPhone **Bluefy**에서 `web/spike-ground.html` → Start → Connect belt → Lock lane.
   전송 경로만 시험하려면 `runner-test.html`을 열고(GitHub Pages 주소 또는 노트북 `/web/runner-test.html`)
   ☑ Camera on, ☑ Decide + BLE → **Connect Arduino (BLE)** → `LaneActuator` → Start.
   지금은 판단 함수가 비어 있어서 **1초마다 M1 → M2 → M3 → M4 → M5 순서로** 돌아가며 진동합니다.
