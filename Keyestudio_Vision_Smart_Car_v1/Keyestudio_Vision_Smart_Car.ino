/*
  keyestudio ESP32-CAM 비디오 스마트 카
  시리얼 모니터를 통해 동적 설정을 가능하게 하고, EEPROM을 이용한 영속적 저장을 지원하도록 개선됨.
  기능:
    - AP 모드에서 동적 SSID "ESP32-XXXX" 생성 (XXXX는 랜덤 4자리 숫자)
    - 시리얼 명령을 통해 설정 변경 및 자동 재부팅
    - EEPROM을 이용한 설정의 영속적 저장
*/

#include <WiFi.h>
#include <EEPROM.h>
#include "esp_wifi.h"
#include "esp_camera.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "SetMotor.h"
#include "app_server.h"
#include "driver/ledc.h"  // LED 컨트롤을 위한 헤더 파일 추가

// EEPROM 주소 정의
#define EEPROM_SIZE 512
#define ADDR_MODE          0   // 1 바이트: 1은 AP 모드, 0은 클라이언트 모드
#define ADDR_AP_PWD        1   // 32 바이트: AP 비밀번호
#define ADDR_AP_SSID       33  // 32 바이트: AP SSID
#define ADDR_WIFI_SSID     65  // 32 바이트: WiFi SSID
#define ADDR_WIFI_PWD      97  // 64 바이트: WiFi 비밀번호

// 기본 설정값
#define DEFAULT_MODE          1
#define DEFAULT_AP_PASSWORD   "88888888"
#define DEFAULT_WIFI_SSID     "YourWiFiSSID"
#define DEFAULT_WIFI_PASSWORD "YourWiFiPassword"

// AP 설정
int channel = 11;       // AP 모드 채널
int hidden = 0;         // AP 숨김 플래그
int maxconnection = 4;  // 최대 클라이언트 수

#define LED_PIN 12
// 여기서 핀 번호는 실제 연결된 LED 핀으로 수정하세요.

// 카메라 핀 정의 - 수정하지 마세요.
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

// 웹서버 / 제어 함수
void startCameraServer();

// 시리얼 명령 처리 변수
enum CommandState {
  IDLE,
  SET_PWD,
  SET_MODE_CMD,
  SET_SSID_CMD,
  SET_WIFI_PWD_CMD
};

String inputString = "";         // 시리얼 입력 버퍼
bool stringComplete = false;     // 전체 문자열 수신 플래그
CommandState currentState = IDLE;

// 설정 변수
uint8_t mode = DEFAULT_MODE; // 1은 AP 모드, 0은 클라이언트 모드
String apPassword = DEFAULT_AP_PASSWORD;
String apSSID = "";           // AP SSID
String wifiSSID = DEFAULT_WIFI_SSID;
String wifiPassword = DEFAULT_WIFI_PASSWORD;

// EEPROM 저장 필요 여부 플래그
bool needToSaveEEPROM = false;
bool rebootFlag = false;      // 재부팅 플래그

// 함수 프로토타입
void loadConfig();
void saveConfig();
void handleCommand(String command);
void promptNewPassword();
void promptNewMode();
void promptNewSSID();
void promptNewWiFiPassword();
void showConfig();
void resetToDefault();
void generateAPSSID();

void setup() {
  // 브라운아웃 디텍터 비활성화
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // 브라운아웃 방지

  // I2C 초기화 (SetMotor.h에 정의된 i2c_init() 사용 가정)
  i2c_init();  // I2C 초기화, SDA는 IO14, SCL은 IO13

  // 시리얼 초기화
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();
  Serial.println("ESP32-CAM 설정 시작");

  // EEPROM 초기화
  if (!EEPROM.begin(EEPROM_SIZE)) {
    Serial.println("EEPROM 초기화 실패");
    // 필요 시 EEPROM 초기화 실패 처리
  }

  // EEPROM에서 설정 불러오기
  loadConfig();

  // 카메라 설정
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  
  // 고사양으로 초기화하여 더 큰 버퍼 사전 할당
  if (psramFound()) {
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  // 카메라 초기화
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {  // ESP_OK
    Serial.printf("카메라 초기화 실패: 0x%x", err);
    return;
  }

  // 카메라 설정 조정
  sensor_t* s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_QVGA);
  s->set_vflip(s, 1);    // 수직 반전
  s->set_hmirror(s, 1);  // 수평 미러

  // LED 제어 초기화 - 새로운 API 사용
  ledcAttach(LED_PIN, 5000, 8);  // 핀 12에 5kHz 주파수와 8비트 해상도로 LEDC 연결

  // 모드에 따라 WiFi 설정
  if (mode == 0) {
    // 클라이언트 모드
    Serial.println("클라이언트 모드로 동작 중");
    Serial.println("WiFi에 연결 중...");
    WiFi.mode(WIFI_STA);
    WiFi.begin(wifiSSID.c_str(), wifiPassword.c_str());
    Serial.print("SSID: ");
    Serial.println(wifiSSID);
    Serial.print("비밀번호: ");
    Serial.println(wifiPassword);

    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    Serial.println();
    Serial.print("카메라 준비 완료! 'http://");
    Serial.print(WiFi.localIP());
    Serial.println("' 에 접속하세요");
  } else {
    // AP 모드
    Serial.println("접속 포인트(AP) 모드로 동작 중");
    Serial.println("접속 포인트 설정 중...");
    Serial.print("SSID: ");
    Serial.println(apSSID);
    Serial.print("비밀번호: ");
    Serial.println(apPassword);
    Serial.println("시리얼 명령을 통해 설정을 변경할 수 있습니다.");

    WiFi.mode(WIFI_AP);
    WiFi.softAP(apSSID.c_str(), apPassword.c_str(), channel, hidden, maxconnection);
    Serial.print("카메라 준비 완료! 'http://");
    Serial.print(WiFi.softAPIP());
    Serial.println("' 에 접속하세요");

    // 준비 완료 표시를 위한 LED 깜빡임
    for (int i = 0; i < 5; i++) {
      ledcWrite(12, 0);    // LED 끄기
      delay(200);
      ledcWrite(12, 255);  // LED 켜기
      delay(200);
    }
  }

  // 웹서버 시작
  startCameraServer();

  // 시리얼 입력 버퍼 초기화
  inputString.reserve(200);  // 메모리 예약하여 단편화 방지

  // 사용 가능한 명령어 안내
  Serial.println("\n사용 가능한 명령어:");
  Serial.println("  SET_PWD       - 새로운 AP 비밀번호 설정");
  Serial.println("  SET_MODE      - WiFi 모드 설정 (1은 AP, 0은 클라이언트)");
  Serial.println("  SET_SSID      - WiFi SSID 설정 (클라이언트 모드에서)");
  Serial.println("  SET_WIFI_PWD  - WiFi 비밀번호 설정 (클라이언트 모드에서)");
  Serial.println("  SHOW_CONFIG   - 현재 설정 표시");
  Serial.println("  RESET_DEFAULT - 기본 설정으로 재설정");
  Serial.println("  HELP          - 사용 가능한 명령어 목록 표시");
}

void loop() {
  // 시리얼 입력 처리
  if (stringComplete) {
    handleCommand(inputString);
    // 입력 버퍼 초기화
    inputString = "";
    stringComplete = false;
  }

  // EEPROM에 저장할 필요가 있을 경우 저장
  if (needToSaveEEPROM) {
    saveConfig();
    needToSaveEEPROM = false;
  }

  // 재부팅 플래그가 설정되었으면 재부팅
  if (rebootFlag) {
    rebootFlag = false;
    Serial.println("재부팅 중...");
    delay(1000); // 재부팅 전에 메시지가 전송될 시간을 줌
    ESP.restart();
  }

  // 기타 반복 작업을 여기에 추가할 수 있습니다
  delay(10);  // 루프 과부하 방지를 위한 짧은 지연
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      stringComplete = true;
    } else if (inChar != '\r') {  // 캐리지 리턴 무시
      inputString += inChar;
    }
  }
}

void loadConfig() {
  // 모드 읽기
  mode = EEPROM.read(ADDR_MODE);
  if (mode != 0 && mode != 1) {
    mode = DEFAULT_MODE;
    needToSaveEEPROM = true;
  }

  // AP 비밀번호 읽기
  char apPwdBuffer[33];
  EEPROM.get(ADDR_AP_PWD, apPwdBuffer);
  apPwdBuffer[32] = '\0'; // 널 종료 보장
  apPassword = String(apPwdBuffer);
  if (apPassword.length() == 0) {
    apPassword = DEFAULT_AP_PASSWORD;
    needToSaveEEPROM = true;
  }

  // AP SSID 읽기
  char apSsidBuffer[33];
  EEPROM.get(ADDR_AP_SSID, apSsidBuffer);
  apSsidBuffer[32] = '\0'; // 널 종료 보장
  apSSID = String(apSsidBuffer);
  if (apSSID.length() == 0) {
    generateAPSSID();
    needToSaveEEPROM = true;
  }

  // WiFi SSID 읽기
  char wifiSsidBuffer[33];
  EEPROM.get(ADDR_WIFI_SSID, wifiSsidBuffer);
  wifiSsidBuffer[32] = '\0'; // 널 종료 보장
  wifiSSID = String(wifiSsidBuffer);
  if (wifiSSID.length() == 0) {
    wifiSSID = DEFAULT_WIFI_SSID;
    needToSaveEEPROM = true;
  }

  // WiFi 비밀번호 읽기
  char wifiPwdBuffer[65];
  EEPROM.get(ADDR_WIFI_PWD, wifiPwdBuffer);
  wifiPwdBuffer[64] = '\0'; // 널 종료 보장
  wifiPassword = String(wifiPwdBuffer);
  if (wifiPassword.length() == 0) {
    wifiPassword = DEFAULT_WIFI_PASSWORD;
    needToSaveEEPROM = true;
  }

  if (needToSaveEEPROM) {
    saveConfig();
  }
}

void saveConfig() {
  // 모드 저장
  EEPROM.write(ADDR_MODE, mode);

  // AP 비밀번호 저장
  char apPwdBuffer[33];
  apPassword.toCharArray(apPwdBuffer, 33);
  EEPROM.put(ADDR_AP_PWD, apPwdBuffer);

  // AP SSID 저장
  char apSsidBuffer[33];
  apSSID.toCharArray(apSsidBuffer, 33);
  EEPROM.put(ADDR_AP_SSID, apSsidBuffer);

  // WiFi SSID 저장
  char wifiSsidBuffer[33];
  wifiSSID.toCharArray(wifiSsidBuffer, 33);
  EEPROM.put(ADDR_WIFI_SSID, wifiSsidBuffer);

  // WiFi 비밀번호 저장
  char wifiPwdBuffer[65];
  wifiPassword.toCharArray(wifiPwdBuffer, 65);
  EEPROM.put(ADDR_WIFI_PWD, wifiPwdBuffer);

  // 변경 사항 커밋
  EEPROM.commit();
  Serial.println("설정이 EEPROM에 저장되었습니다.");

  // 재부팅 플래그 설정
  rebootFlag = true;
}

void handleCommand(String command) {
  command.trim(); // 앞뒤 공백 제거
  String originalCommand = command; // 원본 명령어 저장
  command.toUpperCase(); // 일관성을 위해 대문자로 변환

  if (currentState == IDLE) {
    if (command.startsWith("SET_PWD")) {
      promptNewPassword();
    }
    else if (command.startsWith("SET_MODE")) {
      promptNewMode();
    }
    else if (command.startsWith("SET_SSID")) {
      if (mode == 0) {
        promptNewSSID();
      }
      else {
        Serial.println("오류: SET_SSID 명령은 클라이언트 모드에서만 사용 가능합니다.");
      }
    }
    else if (command.startsWith("SET_WIFI_PWD")) {
      if (mode == 0) {
        promptNewWiFiPassword();
      }
      else {
        Serial.println("오류: SET_WIFI_PWD 명령은 클라이언트 모드에서만 사용 가능합니다.");
      }
    }
    else if (command.startsWith("SHOW_CONFIG")) {
      showConfig();
    }
    else if (command.startsWith("RESET_DEFAULT")) {
      resetToDefault();
    }
    else if (command.startsWith("HELP")) {
      Serial.println("\n사용 가능한 명령어:");
      Serial.println("  SET_PWD       - 새로운 AP 비밀번호 설정");
      Serial.println("  SET_MODE      - WiFi 모드 설정 (1은 AP, 0은 클라이언트)");
      Serial.println("  SET_SSID      - WiFi SSID 설정 (클라이언트 모드에서)");
      Serial.println("  SET_WIFI_PWD  - WiFi 비밀번호 설정 (클라이언트 모드에서)");
      Serial.println("  SHOW_CONFIG   - 현재 설정 표시");
      Serial.println("  RESET_DEFAULT - 기본 설정으로 재설정");
      Serial.println("  HELP          - 사용 가능한 명령어 목록 표시");
    }
    else {
      Serial.println("알 수 없는 명령어입니다. 'HELP'를 입력하여 사용 가능한 명령어를 확인하세요.");
    }
  }
  else {
    // 현재 상태에 따라 멀티 스텝 명령어 처리
    switch (currentState) {
      case SET_PWD:
        apPassword = command;
        if (apPassword.length() < 8) {
          Serial.println("오류: AP 비밀번호는 최소 8자 이상이어야 합니다.");
        }
        else {
          needToSaveEEPROM = true;
          Serial.println("AP 비밀번호가 성공적으로 업데이트되었습니다.");
        }
        currentState = IDLE;
        break;

      case SET_MODE_CMD:
        if (command == "1") {
          mode = 1;
          needToSaveEEPROM = true;
          Serial.println("모드가 AP로 설정되었습니다.");
        }
        else if (command == "0") {
          mode = 0;
          needToSaveEEPROM = true;
          Serial.println("모드가 클라이언트로 설정되었습니다.");
        }
        else {
          Serial.println("유효하지 않은 모드입니다. '1'은 AP, '0'은 클라이언트 모드를 의미합니다.");
        }
        currentState = IDLE;
        break;

      case SET_SSID_CMD:
        wifiSSID = originalCommand; // 원본 명령어를 사용하여 대소문자 유지
        if (wifiSSID.length() == 0) {
          Serial.println("오류: SSID는 비어 있을 수 없습니다.");
        }
        else {
          needToSaveEEPROM = true;
          Serial.println("WiFi SSID가 성공적으로 업데이트되었습니다.");
        }
        currentState = IDLE;
        break;

      case SET_WIFI_PWD_CMD:
        wifiPassword = command;
        if (wifiPassword.length() < 8) {
          Serial.println("오류: WiFi 비밀번호는 최소 8자 이상이어야 합니다.");
        }
        else {
          needToSaveEEPROM = true;
          Serial.println("WiFi 비밀번호가 성공적으로 업데이트되었습니다.");
        }
        currentState = IDLE;
        break;

      default:
        currentState = IDLE;
        break;
    }
  }
}

void promptNewPassword() {
  Serial.println("새 AP 비밀번호를 입력하세요 (최소 8자):");
  currentState = SET_PWD;
}

void promptNewMode() {
  Serial.println("새 모드를 입력하세요 (1은 AP, 0은 클라이언트):");
  currentState = SET_MODE_CMD;
}

void promptNewSSID() {
  Serial.println("새 WiFi SSID를 입력하세요:");
  currentState = SET_SSID_CMD;
}

void promptNewWiFiPassword() {
  Serial.println("새 WiFi 비밀번호를 입력하세요 (최소 8자):");
  currentState = SET_WIFI_PWD_CMD;
}

void showConfig() {
  Serial.println("\n현재 설정:");
  Serial.print("모드: ");
  if (mode == 1) {
    Serial.println("AP");
    Serial.print("SSID: ");
    Serial.println(apSSID);
    Serial.print("AP 비밀번호: ");
    Serial.println(apPassword);
    Serial.print("AP IP: ");
    Serial.println(WiFi.softAPIP());
  }
  else {
    Serial.println("클라이언트");
    Serial.print("WiFi SSID: ");
    Serial.println(wifiSSID);
    Serial.print("WiFi 비밀번호: ");
    Serial.println(wifiPassword);
    Serial.print("연결된 IP: ");
    Serial.println(WiFi.localIP());
  }
  Serial.println();
}

void resetToDefault() {
  Serial.println("기본 설정으로 재설정 중...");
  mode = DEFAULT_MODE;
  apPassword = DEFAULT_AP_PASSWORD;
  wifiSSID = DEFAULT_WIFI_SSID;
  wifiPassword = DEFAULT_WIFI_PASSWORD;
  apSSID = ""; // AP SSID를 비워 재생성 필요
  needToSaveEEPROM = true;
  saveConfig();
  Serial.println("기본 설정으로 성공적으로 재설정되었습니다.");
  Serial.println("변경 사항을 적용하려면 장치를 다시 시작하세요.");
}

void generateAPSSID() {
  uint32_t rand_num = esp_random();
  int randomNumber = (rand_num % 9000) + 1000; // 1000-9999
  apSSID = "ESP32-" + String(randomNumber);
  Serial.print("새 AP SSID 생성: ");
  Serial.println(apSSID);
}
