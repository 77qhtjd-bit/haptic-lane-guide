# 아두이노 업로드 가이드 (다른 컴퓨터용)

이 폴더의 `lane_actuator` 스케치를 Arduino UNO R4 WiFi에 올리는 방법입니다.
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

## 3. Wi-Fi 정보 입력
0. (Windows) zip은 **반드시 압축을 푼 뒤** 작업하세요. zip 안에서 바로 열면 저장이 안 됩니다.
   폴더 이름 `lane_actuator`와 파일 이름 `lane_actuator.ino`는 바꾸지 마세요. IDE는 두 이름이 같아야 엽니다.
1. `lane_actuator/arduino_secrets.h.example`을 **같은 폴더에** `arduino_secrets.h`라는 이름으로 복사합니다.
   - Windows 탐색기는 확장자를 숨기기 때문에, 이름을 바꾸거나 메모장으로 저장하면 `arduino_secrets.h.txt`가 되기 쉽습니다.
     탐색기 **보기 → 파일 확장명**을 켜고 이름을 확인하세요.
   - 가장 쉬운 방법: IDE에서 스케치를 연 뒤 탭 옆 **⋯ → New Tab**으로 `arduino_secrets.h`를 만들고, example 내용을 붙여 넣습니다.
2. `SECRET_SSID`에 Wi-Fi 이름을, `SECRET_PASS`에 비밀번호를 따옴표 안에 입력합니다.
   - 노트북과 **같은 네트워크**여야 합니다. 권장하는 네트워크는 **iPhone 개인용 핫스팟**입니다.
   - iPhone 핫스팟 이름은 보통 설정 → 일반 → 정보 → 이름(예: `홍길동의 iPhone`)과 같습니다. 공백과 특수문자(영문 이름이면 ' 포함)까지 정확히 입력해야 합니다.
   - **설정 → 개인용 핫스팟 → "호환성 최대화"를 켜세요.** R4는 2.4 GHz만 지원하기 때문에, 이 옵션이 꺼져 있으면 접속하지 못합니다.
3. `arduino_secrets.h`는 비밀번호가 들어 있으니 공유하거나 git에 올리지 마세요.

## 4. 라이브러리
**추가로 설치할 라이브러리는 없습니다.** 스케치가 쓰는 라이브러리는 모두 1단계의 "Arduino UNO R4 Boards" 코어에 포함되어 있습니다.
- `WiFiS3.h` (Wi-Fi, UDP): 코어에 포함
- `Arduino_LED_Matrix.h` (내장 12x8 LED 매트릭스): 코어에 포함

## 5. 업로드와 IP 확인
1. IDE에서 `lane_actuator/lane_actuator.ino`를 엽니다. `arduino_secrets.h`가 같은 폴더에 있어야 합니다.
2. **Upload**(→ 버튼)를 누릅니다.
3. **Tools → Serial Monitor**를 열고 속도를 **9600 baud**로 맞춥니다. 다음과 같은 내용이 보입니다.
   ```
   WiFi firmware: 0.x.x (latest known: 0.x.x)
   Connecting to <SSID>
   Connected. IP: 172.20.10.5
   ```
   이 IP를 기록해 두세요. 노트북이 자동으로 찾지 못할 때 직접 입력하는 데 씁니다.
4. LED 매트릭스 화면의 의미:
   - 아래 줄 점선만 보임: 부팅 중 또는 Wi-Fi 접속 중
   - 숫자 + 아래 점선: Wi-Fi 접속됨, 아직 명령 없음("신호 없음"). 숫자는 **IP 마지막 자리**입니다(예: `5` → 172.20.10.5).
   - ◀ / ▶ 화살표 + 아래 막대: 왼쪽/오른쪽 명령. 막대 길이가 세기입니다.
   - 가운데 작은 네모: 정지(세기 0) 명령
5. 접속이 안 되면 시리얼 모니터에 `Connecting to ...`가 10초마다 반복됩니다. 이때는 SSID와 비밀번호, "호환성 최대화"를 확인하세요.

## 6. 업로드 후
- 보드는 **USB 전원만** 있으면 됩니다(보조배터리나 노트북 USB 포트 모두 가능). 전원을 켜면 저장된 Wi-Fi에 자동으로 접속합니다.
- 접속하면 2초마다 노트북에 자기 IP를 알립니다(UDP 브로드캐스트 `HELLO,<ip>`, 포트 4211).
  노트북의 `/control` 페이지에 IP가 자동으로 나타납니다.
- Wi-Fi 정보가 바뀌면 3단계와 5단계를 다시 하면 됩니다.

---

# (추가) 블루투스(BLE) 버전: `lane_actuator_ble` (팀 계획: 모터 5개)

Wi-Fi 대신 **iPhone(Bluefy 브라우저) ↔ 아두이노를 BLE로 직접** 연결하는 버전입니다. **현재 사용하는 버전입니다.**
Wi-Fi 정보(`arduino_secrets.h`)가 필요 없습니다. 이 스케치를 올리면 기존 스케치는 덮어써집니다.

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
7. iPhone **Bluefy**에서 `runner-test.html`을 열고(GitHub Pages 주소 또는 노트북 `/web/runner-test.html`)
   ☑ Camera on, ☑ Decide + BLE → **Connect Arduino (BLE)** → `LaneActuator` → Start.
   지금은 판단 함수가 비어 있어서 **1초마다 M1 → M2 → M3 → M4 → M5 순서로** 돌아가며 진동합니다.
