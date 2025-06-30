// —————— 라이브러리 불러오기 ——————
#include <Wire.h>                 // I2C 통신용
#include <LiquidCrystal_I2C.h>    // I2C LCD 제어용
#include <SPI.h>                  // SPI 통신용 (RFID, SD)
#include <MFRC522.h>              // RFID RC522 라이브러리
#include <RtcDS1302.h>            // RTC DS1302 라이브러리
#include <SD.h>                   // SD 카드 라이브러리
#include <IRremote.h>             // IR 리모컨 라이브러리
#include <OneWire.h>              // 1-Wire 통신
#include <DallasTemperature.h>    // DS18B20 온도 센서
#include <toneAC.h>               // ToneAC2: Timer2 기반
#include <MPU6050.h>              // mpu6050 가속도 센서

// —————— 핀 정의 ——————

// 키패드 행과 열 핀 정의
const uint8_t rowPins[4] = {29, 28, 27, 26};   // R1~R4
const uint8_t colPins[4] = {30, 31, 32, 33};   // C1~C4

// LCD I2C 핀은 별도 설정 없음 (I2C 주소 기반)
#define LCD_ADDR    0x27
#define LCD_COLS    16
#define LCD_ROWS    2

// RTC 핀 정의
#define RTC_RST_PIN 24
#define RTC_CLK_PIN 22
#define RTC_DAT_PIN 23

// RFID 핀 정의
#define RST_PIN     5  // RST
#define SS_PIN      4  // SDA
// SPI 핀 공유: SCK 52, MOSI 51, MISO 50

// SD카드 CS 핀 정의
#define SD_CS_PIN   53  // SPI CS
// SPI 핀 공유: SCK 52, MOSI 51, MISO 50

// 조이스틱 핀 정의
#define VRX A0
#define VRY A1
#define SW  -1   // 미사용

// IR 장애물 센서 핀 정의
#define IR_SENSOR_PIN 10 

// 택트 스위치 핀 정의
#define WAKE_BUTTON_PIN 13

// RGB LED 핀 정의
#define RGB_PIN_R 7
#define RGB_PIN_G 8
#define RGB_PIN_B 9

// 로터리 엔코더 핀 정의
#define ROTARY_CLK 3
#define ROTARY_DT  2
#define ROTARY_SW  -1   // 미사용

// IR 리시버 핀 정의
#define IR_RECV_PIN 6

// 시프트 레지스터 핀 정의
#define DATA_PIN 36   // DS
#define LATCH_PIN 35  // STCP
#define CLOCK_PIN 34  // SHCP

// 부저 핀 정의
// ToneAC2 출력 A, B 는 11, 12로 사용

// 온도 센서 핀 정의
#define ONE_WIRE_BUS   A8      // DS18B20 (1-Wire)

// 조도 센서 핀 정의
#define LIGHT_PIN      A9

// 불꽃 센서 핀 정의
#define FLAME1_PIN     A15    // 불꽃 센서1
#define FLAME2_PIN     A10    // 불꽃 센서2

// 초음파 센서 핀 정의
#define TRIG1            44
#define ECHO1            45
#define TRIG2            41
#define ECHO2            40

// 충격 센서 핀 정의
#define SHOCK_PIN        38

// 사운드 감지 센서 핀 정의
#define SOUND_SENSOR_PIN A14

// PIR 센서 핀 정의
#define PIR1_PIN         25
#define PIR2_PIN         42

// —————— 전역 변수 ——————
// 조이스틱
int joyY;                              // 현재 Y축 아날로그 값
bool joyHolding = false;              // Y축을 계속 누르고 있는 중인지 여부
unsigned long joyHoldStart = 0;       // 누르기 시작한 시간
unsigned long lastJoyScroll = 0;      // 마지막 스크롤 타이밍 기록

const int joyFastThreshold = 1000;    // 1초 이상 유지 시 빠른 스크롤로 전환 (ms)
const int joyIntervalNormal = 500;    // 일반 속도 스크롤 간격 (ms)
const int joyIntervalFast = 150;       // 빠른 속도 스크롤 간격 (ms)

// SD카드 폴더 경로 설정
const char *SD_USERS_FOLDER       = "USERS";  // 사용자 정보 폴더
const char *SD_COMMUTE_LOG_FOLDER = "LOG1";   // 출퇴근 로그 폴더
const char *SD_HUMAN_LOG_FOLDER = "LOG2";    // 사람 감지 로그 폴더
const char *SD_FIRE_LOG_FOLDER = "LOG3";      // 화재 감지 로그 폴더
const char *SD_SETTING_FOLDER = "SET";         // 세팅 폴더

// 관리자 사번, 비밀번호, 주 야간모드 지정
String adminID;   // 관리자 사번
String adminPW;   // 관리자 비밀번호
bool isNightMode = false;        // 주간/야간 모드

// 절전 타이머
bool systemActive = false;           // 시스템 동작 상태
unsigned long activeStart = 0;       // 마지막 활성화 시각
const unsigned long activeTimeout = 10000;  // 10초 동작 지속 시간

// 밝기, 소리
int brightness;     // LED 밝기 (0–30)
int volumeLevel;    // 부저 소리 (0–30)

// 로터리 엔코더
int lastClkState;   // 상태 저장

// 로터리 엔코더 디바운싱
unsigned long lastEncoderDebounceTime = 0;
const unsigned long encoderDebounceDelay = 5;  // 5ms
volatile int encoderPos = 0;   // lastClkState: 이전 CLK 읽기값을 저장해 방향 판별 및 디바운싱에 사용
volatile uint8_t lastState;       // 이전 2비트 상태: (DT<<1)|CLK
volatile int8_t stepSum     = 0;   // 4스텝 누적용 버퍼

// 비차단용 전역 상태 변수
bool nbLedEnable  = false;
bool nbBuzzEnable = false;

unsigned long nbPrevLedMillis  = 0;
unsigned long nbPrevBuzzMillis = 0;

bool nbLedOnState    = false;   // LED 현재 ON/OFF 상태
int  nbLedPattern    = 0;       // 패턴 토글용: 0=파랑, 1=빨강
int  nbLedBlinkCount = 0;       // 깜빡임 횟수 카운트
int nbLedCycleCount = 0;        // on-phase 카운트용 (3번마다 색 전환)

bool nbBuzzOnState   = false;   // 부저 ON/OFF 상태 toggle
int  nbToneStep      = 0;       // tone1(0) / tone2(1) 선택

int  nbDutyOn, nbDutyOff;       // 감마 기반 ON/OFF 시간(ms)
int  nbLedInterval;             // LED가 다음 전환까지 기다릴 시간
int  nbBuzzInterval = 40;       // 부저가 다음 전환까지 기다릴 시간

// 리모컨으로 입력된 숫자를 문자열로 모을 버퍼
String irBuffer = "";
unsigned long lastIrMillis = 0;    // 마지막으로 IR 신호를 받은 시각
const unsigned long irTimeout = 3000; // 3초 동안 숫자 입력이 없으면 초기화

// 택트 스위치 디바운싱
const unsigned long WAKE_DEBOUNCE_DELAY = 50;  // 50ms debounce 기간
bool wakeLastRaw    = HIGH;    // 이전에 읽은 RAW 값 (풀업 모드이므로 초기값은 HIGH)
bool wakeLastStable = HIGH;    // 디바운싱 후 최종 안정 상태
unsigned long wakeLastChangeTime = 0;  // RAW 값이 바뀐 시점 기록

// 화재 임계값, 유지시간
const float        TEMP_THRESHOLD     = 40.0;      // °C
const int          LIGHT_THRESHOLD    = 50;        // 아날로그 값
const int          FLAME_THRESHOLD    = 200;       // 아날로그 값
const unsigned long HOLD_DURATION     = 5000;      // 5초 유지 (ms)
const unsigned long SENSOR_INTERVAL   = 1000;      // 1초 간격 (ms)

// 개별 센서 상태 플래그, 타이머
bool          tempFlag    = false;
unsigned long tempTime    = 0;
bool          lightFlag   = false;
unsigned long lightTime   = 0;
bool          flameFlag   = false;
unsigned long flameTime   = 0;

// 화재 감지 전체 상태, 타이머
bool          fireState   = false;
unsigned long fireTime    = 0;

// 야간모드 초기화
bool nightModeInitialized = false;

// AdminMenu 진입 상태
bool adminActive = false;                   // Admin? 메뉴 진입 상태
unsigned long adminStart = 0;               // Admin? 메뉴 진입 시각
const unsigned long ADMIN_TIMEOUT = 20000;  // 20초 제한
bool pinInputActive = false;                // PIN 입력 상태
String pinBuffer = "";                      // 누적된 PIN

// 인체 감지용 임계값·유지시간
const float   angleThreshold    = 20.0;      // 기울기 임계 (°)
const int     distanceThreshold = 50;        // 초음파 거리 임계 (cm)
const unsigned long HOLD_DURATION_HUMAN      = 5000; // 센서 5초 유지
const unsigned long sensorInterval = 1000;          // 1초 (인체 감지 주기)

// 인체 감지용 플래그·타이머
unsigned long lastSensorMillis = 0;

// 인체 감지 전체 상태, 타이머
bool humanAlertState = false;
unsigned long humanAlertTime = 0;

// 기울기, 충격, 사운드, pir, 초음파 감지 플래그·타이머
unsigned long tiltTime   = 0, shockTime   = 0, soundTime = 0;
unsigned long pir1Time   = 0, pir2Time    = 0;
unsigned long us1Time    = 0, us2Time     = 0;

bool tiltFlag   = false, shockFlag   = false, soundFlag = false;
bool pir1Flag   = false, pir2Flag    = false;
bool us1Flag    = false, us2Flag     = false;

// 0: 꺼짐, 1: 켜짐, 2: 디버그 모드
int fireDetectMode = 1;   // 화재 감지 모드
int humanDetectMode = 1;  // 인체 감지 모드

// 룩업 테이블: 인덱스 = (lastState<<2)|newState
const int8_t QUAD_TABLE[16] = {
   0, -1, +1,  0,
  +1,  0,  0, -1,
  -1,  0,  0, +1,
   0, +1, -1,  0
};

// —————— 객체 선언 ——————
LiquidCrystal_I2C lcd(LCD_ADDR, LCD_COLS, LCD_ROWS);        // LCD 객체
MFRC522 rfid(SS_PIN, RST_PIN);                              // RFID 객체
ThreeWire myWire(RTC_DAT_PIN, RTC_CLK_PIN, RTC_RST_PIN);    // RTC 통신 설정
RtcDS1302<ThreeWire> Rtc(myWire);                           // RTC 객체
IRrecv irrecv(IR_RECV_PIN);                                 // IR 리모컨 수신 객체
decode_results irCode;                                      // 디코딩된 IR 코드를 저장할 구조체
OneWire oneWire(ONE_WIRE_BUS);                              // 1-Wire
DallasTemperature sensors(&oneWire);                        // DS18B20
MPU6050 mpu;                                                // MPU6050 mpu

// 키패드 키 배열 정의
char keys[4][4] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};

// —————— 유틸리티 함수 ——————

// —————— 키패드, 조이스틱, 리모컨 ——————

// 키패드에서 눌린 키를 반환하는 함수
// 행과 열을 스캔하여 눌린 키를 감지하고 반환
char getKeypad() {
  while (true) {
    for (uint8_t r=0; r<4; r++) {
      for (uint8_t i=0; i<4; i++)
        digitalWrite(rowPins[i], i==r?LOW:HIGH);  // 해당 행만 LOW로 설정
      delayMicroseconds(50);
      for (uint8_t c=0; c<4; c++) {
        if (digitalRead(colPins[c])==LOW) {
          char k = keys[r][c];
          while (digitalRead(colPins[c])==LOW);  // 키가 떼질 때까지 대기
          delay(50);  // 짧은 대기 후 반환
          activeStart = millis();    // 키패드 입력 시 타이머 리셋
          return k;
        }
      }
    }
  }
}

// 키패드 + 조이스틱 통합 입력 처리 함수
// 키패드로부터 입력을 받아 반환하는 함수 (기존 키패드와 동일 동작)
char getNavKey() {
  char key = getKeypad();
  return key;
}

// 조이스틱 Y축 입력을 기반으로 페이지 인덱스를 스크롤하는 함수
// current: 현재 페이지 번호, total: 총 페이지 수, updated: 스크롤 발생 여부 플래그
int getNavScroll(int current, int total, bool &updated) {
  joyY = analogRead(VRY);               // Y축 값 읽기
  unsigned long now = millis();         // 현재 시간 (ms)
  int dir = 0;                           // 방향: 1=아래(다음), -1=위(이전)
  updated = false;

  // 조이스틱이 중립 구간을 벗어났을 때
  if (joyY < 400 || joyY > 600) {
    int interval = (now - joyHoldStart >= joyFastThreshold) ? joyIntervalFast : joyIntervalNormal;

    if (!joyHolding) {
      // 처음 누르기 시작했을 때
      joyHolding = true;
      joyHoldStart = now;
      lastJoyScroll = now;
      dir = (joyY < 400) ? 1 : -1;       // 위쪽: 다음, 아래쪽: 이전
      updated = true;
    } else if (now - lastJoyScroll >= interval) {
      // 일정 시간 경과 후 반복 스크롤
      dir = (joyY < 400) ? 1 : -1;
      updated = true;
      lastJoyScroll = now;
    }
  } else {
    // 조이스틱이 중립으로 돌아오면 상태 초기화
    joyHolding = false;
  }

  // 방향에 따라 인덱스 변경
  if (updated) {
    activeStart = millis();  // 스크롤 시 타이머 리셋
    if (dir == 1) return current % total + 1;                 // 다음 항목으로 이동
    else return (current == 1 ? total : current - 1);         // 이전 항목으로 이동
  }

  return current;  // 입력 변화 없음 → 현재 위치 유지
}

// 키패드 논블로킹 스캔: 눌린 키가 있으면 반환, 없으면 0 반환
char getNavKeyNB() {
  for (uint8_t r = 0; r < 4; r++) {
    // 각 행을 차례로 L로
    for (uint8_t i = 0; i < 4; i++)
      digitalWrite(rowPins[i], i == r ? LOW : HIGH);
    delayMicroseconds(50);
    for (uint8_t c = 0; c < 4; c++) {
      if (digitalRead(colPins[c]) == LOW) {
        // 눌린 키를 바로 리턴하되, 릴리즈 기다리기
        char k = keys[r][c];
        while (digitalRead(colPins[c]) == LOW);
        delay(50);
        activeStart = millis();    // 키패드 입력 시 타이머 리셋
        return k;
      }
    }
  }
  return 0;  // 아무 키도 안 눌림
}

// 사용자로부터 8자리 숫자 사번을 입력받는 함수
// '#' 입력 시 완료, 'C' 입력 시 취소, '*' 입력 시 백스페이스
String inputID(int idOrNewIdOrpwOrNewPW) {
  String num;
  lcd.clear();
  if (idOrNewIdOrpwOrNewPW == 1) lcd.print("Enter ID:");
  else if (idOrNewIdOrpwOrNewPW == 2) lcd.print("Enter NEW ID:");
  else if (idOrNewIdOrpwOrNewPW == 3) lcd.print("Enter PW:");
  else if (idOrNewIdOrpwOrNewPW == 4) lcd.print("Enter NEW PW:");
  lcd.setCursor(0,1);
  while (true) {
    char k = getKeypad();
    if (k=='#') break;                       // 입력 완료
    if (k=='C') return "";                   // 입력 취소
    if (k=='*' && num.length()>0) num.remove(num.length()-1);  // 한 자리 삭제
    else {
      // 숫자 추가 (ID는 최대 8자리, PW는 최대 4자리)
      if (idOrNewIdOrpwOrNewPW == 1 || idOrNewIdOrpwOrNewPW == 2) {
        if (k >= '0' && k <= '9' && num.length() < 8) {
          num += k;
        }
      } 
      else {
        if (k >= '0' && k <= '9' && num.length() < 4) {
          num += k;
        }
      }
    }
    lcd.setCursor(0,1);
    lcd.print("        ");      // 줄 초기화
    lcd.setCursor(0,1);
    lcd.print(num);             // 현재 입력된 번호 표시
  }
  return num;
}

// IR 코드값을 숫자 문자('0'~'9') 또는 확인('#')으로 변환
char mapIRToChar(unsigned long irRawValue) {
  switch (irRawValue) {
    case 0x16: return '0';   // '0' 버튼
    case 0xC: return '1';   // '1' 버튼
    case 0x18: return '2';   // '2' 버튼
    case 0x5E: return '3';   // '3' 버튼
    case 0x8: return '4';   // '4' 버튼
    case 0x1C: return '5';   // '5' 버튼
    case 0x5A: return '6';   // '6' 버튼
    case 0x42: return '7';   // '7' 버튼
    case 0x52: return '8';   // '8' 버튼
    case 0x4A: return '9';   // '9' 버튼
    case 0x43: return '#';   // Play/Pause (확인) 버튼

    default:
      // 해당하지 않는 IR 코드가 들어오면 '\0' 반환
      return '\0';
  }
}

// —————— SD 카드 ——————

// UID 문자열을 파일명으로 쓰기 위해 공백 제거하는 함수
String uidToName(const String &uid) {
  String s=uid; s.replace(" ","");
  return s;
}

// LCD의 두 번째 줄에 현재 시간을 HH:MM:SS 형식으로 표시하는 함수
void displayTime() {
  RtcDateTime now = Rtc.GetDateTime();       // RTC 시간 읽기
  char buf[9];
  snprintf_P(buf,sizeof(buf),PSTR("%02u:%02u:%02u"),
             now.Hour(),now.Minute(),now.Second());
  lcd.setCursor(0,1);
  lcd.print(buf);
}

// USERS 폴더 내 등록된 사용자 수를 반환하는 함수
int countUsersOnSD() {
  File dir = SD.open(SD_USERS_FOLDER);       // USERS 폴더 열기
  if (!dir) return 0;                        // 폴더 없으면 0 반환
  int cnt = 0;
  while (true) {
    File f = dir.openNextFile();             // 다음 파일 읽기
    if (!f) break;
    cnt++; f.close();                        // 파일 수 카운트
  }
  dir.close();
  return cnt;
}

// 지정한 index번째에 해당하는 사용자 파일 이름을 반환 (8.3 형식)
String getUserFileAt(int idx) {
  File dir = SD.open(SD_USERS_FOLDER);
  int cnt = 0;
  while (true) {
    File f = dir.openNextFile();
    if (!f) break;
    cnt++;
    if (cnt == idx) {
      String n = f.name();                   // 파일 이름 추출
      f.close(); dir.close();
      return n;
    }
    f.close();
  }
  dir.close();
  return "";
}

// index번째 사용자 파일에서 "ID:" 라인을 찾아 해당 사번을 반환
String getIDAt(int idx) {
  File dir = SD.open(SD_USERS_FOLDER);
  if (!dir) return "";

  int cnt = 0;
  while (true) {
    File f = dir.openNextFile();
    if (!f) break;
    cnt++;
    if (cnt == idx) {
      String line;
      while (f.available()) {
        line = f.readStringUntil('\n');
        if (line.startsWith("ID:")) {
          String id = line.substring(4);  // "ID:" 이후, 예: "22011949"
          id.trim();                       // 공백, \r 제거
          f.close();
          dir.close();
          return id;
        }
      }
      f.close();
      break;
    }
    f.close();
  }
  dir.close();
  return "";
}

// LOG 폴더 내 날짜별 로그 파일 수를 반환하는 함수
int countLogs(int type) {
  const char* folder;
  if (type == 1)       folder = SD_COMMUTE_LOG_FOLDER;  // "LOG1"
  else if (type == 2)  folder = SD_HUMAN_LOG_FOLDER;   // "LOG2"
  else if (type == 3)  folder = SD_FIRE_LOG_FOLDER;     // "LOG3"
  else                 folder = SD_COMMUTE_LOG_FOLDER;  // 기본: LOG1
  File dir = SD.open(folder);
  if (!dir) return 0;
  int cnt = 0;
  while (true) {
    File f = dir.openNextFile();
    if (!f) break;
    cnt++; f.close();
  }
  dir.close();
  return cnt;
}

// 주어진 날짜 로그 파일에서 항목(라인)의 수를 반환
int countLogEntries(const String &path) {
  File f = SD.open(path);
  if (!f) return 0;
  int cnt = 0;
  while (f.available()) {
    f.readStringUntil('\n');  // 한 줄씩 읽으며 카운트
    cnt++;
  }
  f.close();
  return cnt;
}

// 주어진 로그 파일에서 index번째 라인을 반환
String getLogLine(const String &path, int idx) {
  File f = SD.open(path);
  if (!f) return "";
  String line;
  int cnt = 0;
  while (f.available()) {
    line = f.readStringUntil('\n');
    cnt++;
    if (cnt == idx) break;
  }
  f.close();
  return line;
}

// 주어진 로그 파일에서 index번째 항목을 삭제
// 삭제 후 임시 버퍼에 저장하고 전체 파일을 덮어씀
void deleteLogEntry(const String &path, int delIdx) {
  File f = SD.open(path);
  if (!f) return;
  String buf;
  int cnt = 0;
  while (f.available()) {
    String line = f.readStringUntil('\n');
    cnt++;
    if (cnt != delIdx) buf += line + "\n";  // 삭제 대상 제외하고 누적
  }
  f.close();
  SD.remove(path);                          // 기존 파일 삭제
  File w = SD.open(path, FILE_WRITE);       // 새로 작성
  if (w) {
    w.print(buf);
    w.close();
  }
}

// UID와 ID를 기반으로 사용자 등록 파일을 SD 카드에 저장하는 함수
// 파일명은 UID 기반, 내용은 UID와 사번을 포함
void registerOnSD(const String &uid, const String &num) {
  if (!SD.exists(SD_USERS_FOLDER)) SD.mkdir(SD_USERS_FOLDER);  // 폴더 없으면 생성

  String fn = String(SD_USERS_FOLDER) + "/" + uidToName(uid) + ".txt";
  digitalWrite(SS_PIN, HIGH);       // RFID 모듈 비활성화
  digitalWrite(SD_CS_PIN, LOW);     // SD카드 모듈 활성화

  File f = SD.open(fn, FILE_WRITE); // 사용자 파일 열기
  if (f) {
    f.print("UID: "); f.println(uid);   // UID 저장
    f.print("ID: ");  f.println(num);   // ID 저장
    f.close();
    lcd.clear(); lcd.print("Registered");
  } else {
    lcd.clear(); lcd.print("SD Write Err");
  }

  digitalWrite(SD_CS_PIN, HIGH);    // SD카드 모듈 비활성화
  delay(1000);                      // 사용자 확인 시간
}

// 로그를 날짜별 파일에 시간, ID, UID를 함께 기록
void logEvent(int type, const String &id, const String &uid) {
  RtcDateTime now = Rtc.GetDateTime();

  const char* folder;
  if (type == 1)       folder = SD_COMMUTE_LOG_FOLDER;  // "LOG1"
  else if (type == 2)  folder = SD_HUMAN_LOG_FOLDER;   // "LOG2"
  else if (type == 3)  folder = SD_FIRE_LOG_FOLDER;     // "LOG3"
  else                 folder = SD_COMMUTE_LOG_FOLDER;  // 기본: LOG1
  
  // 날짜 기반 파일명 생성: LOG1/YYYYMMDD.TXT
  char fn[30];
    sprintf(fn, "%s/%04d%02d%02d.TXT", folder, now.Year(), now.Month(), now.Day());

  // 현재 시간 형식: [HH:MM:SS]
  char ts[12];
  sprintf(ts, "[%02d:%02d:%02d]", now.Hour(), now.Minute(), now.Second());
  digitalWrite(SS_PIN, HIGH);    // RFID 비활성화
  digitalWrite(SD_CS_PIN, LOW);  // SD카드 활성화

  // 기존 로그 읽기
  String oldContent = "";
  File oldFile = SD.open(fn);
  if (oldFile) {
    while (oldFile.available()) {
      oldContent += oldFile.readStringUntil('\n') + "\n";
    }
    oldFile.close();
  }

  // 파일 새로 쓰기 (최신 로그를 맨 위에)
  SD.remove(fn);  // 기존 파일 삭제
  File newFile = SD.open(fn, FILE_WRITE);
  if (newFile) {
    newFile.print(ts);          // 시간 기록
    newFile.print(" ID: ");     // ID 앞 태그
    newFile.print(id);          // 사용자 ID
    newFile.print(" UID: ");    // UID 앞 태그
    newFile.println(uid);       // UID 기록
    newFile.print(oldContent);  // 기존 로그 덧붙이기
    newFile.close();
  } else {
    Serial.println("WRITE ERROR");  // 실패 시 시리얼 출력
  }

  digitalWrite(SD_CS_PIN, HIGH);  // SD카드 비활성화
}

// 날짜 로그 파일을 최신순으로 가져오는 함수
String getEventFileAt_Desc(int type, int idx) {
  char prevFile[13] = "";  // 이전에 찾은 파일명을 저장 (초기에는 빈 문자열)

  const char* folder;
  if (type == 1)       folder = SD_COMMUTE_LOG_FOLDER;  // "LOG1"
  else if (type == 2)  folder = SD_HUMAN_LOG_FOLDER;   // "LOG2"
  else if (type == 3)  folder = SD_FIRE_LOG_FOLDER;     // "LOG3"
  else                 folder = SD_COMMUTE_LOG_FOLDER;  // 기본: LOG1

  // 원하는 인덱스만큼 반복 (최신부터 차례로 찾아감)
  for (int i = 0; i < idx; i++) {
    File dir = SD.open(folder);  // 로그 폴더 열기
    char best[13] = "";  // 이번 루프에서 찾을 가장 최신 파일명을 저장할 변수

    // 폴더 내 파일들을 순회
    while (true) {
      File f = dir.openNextFile();  // 다음 파일 열기
      if (!f) break;                // 없으면 반복 종료
      const char* name = f.name();  // 파일명 가져오기

      if (prevFile[0] == '\0') {
        // 첫 번째 루프인 경우 → 전체 중 가장 큰(최신) 파일을 찾음
        if (strcmp(name, best) > 0)
          strcpy(best, name);
      } else {
        // 두 번째 루프부터는 prevFile보다 작은 이름 중 가장 큰 것을 찾음
        if (strcmp(name, prevFile) < 0 && strcmp(name, best) > 0)
          strcpy(best, name);
      }

      f.close();  // 파일 닫기
    }

    dir.close();             // 디렉토리 닫기
    strcpy(prevFile, best);  // 이번에 찾은 best 파일명을 prevFile에 저장
  }

  return String(prevFile);  // idx번째 최신 파일명을 반환
}

// 설정 불러오는 함수
void loadSettings() {
  // 1) config.txt 파일 열기 시도
  File f = SD.open("SET/config.txt");
  if (!f) {
    // 파일이 없는 경우에도 기본값으로 초기화
    brightness  = 15;
    volumeLevel = 15;
    adminID     = "22011949";
    adminPW     = "1234";
    return;
  }

  // 2) 파일이 열렸으므로 먼저 밝기와 볼륨 파싱
  brightness  = f.parseInt();
  volumeLevel = f.parseInt();

  // 3) 남은 문자열(예: " 22011949 1234\n")을 읽어서 ID/PW로 분리
  String rest = f.readString();  // 예: " 22011949 1234\n"
  f.close();

  // 4) 앞뒤 공백 제거 (예: "22011949 1234")
  rest.trim();

  // 5) 공백 기준으로 토큰 분리
  int spaceIndex = rest.indexOf(' ');
  if (spaceIndex < 0) {
    // 공백이 없다면 포맷이 예상과 다름 → ID만 저장, PW는 빈 문자열
    adminID = rest;
    adminID.trim();
    adminPW = "";
  } else {
    adminID = rest.substring(0, spaceIndex);
    adminID.trim();
    adminPW = rest.substring(spaceIndex + 1);
    adminPW.trim();
  }
}

// 설정 저장하는 함수
void saveSettings() {
  // 1) 기존 파일 제거(있을 경우)
  SD.remove("SET/config.txt");

  // 2) config.txt 열기 (쓰기)
  File f = SD.open("SET/config.txt", FILE_WRITE);

  // 3) brightness, volume, adminID, adminPW를 공백으로 구분하여 기록
  //    예: "15 15 22011949 1234"
  f.print(brightness);
  f.print(" ");
  f.print(volumeLevel);
  f.print(" ");
  f.print(adminID);
  f.print(" ");
  f.print(adminPW);
  f.println();  // 줄바꿈 추가
  f.close();
}

// —————— LED, 부저, 로터리 엔코더, 택트 스위치 ——————

// ledOn==true/BuzzerOn==true 로 콜하면 파랑→빨강 2회씩 깜빡이고,
// 부저도 tone1/tone2 반복하며 울리기 시작
void triggerLedAndBuzzer(bool ledOn, bool buzzerOn) {
  nbLedEnable    = ledOn;
  nbBuzzEnable   = buzzerOn;
  nbPrevLedMillis  = millis();
  nbPrevBuzzMillis = millis();

  // LED: 깜빡임 시작 전 OFF 상태로 세팅하고, 다음 on-phase 패턴은 파랑으로
  nbLedOnState    = false;
  nbLedPattern    = 1;     // 첫 토글 시 !1 → 0(파랑)  
  nbLedBlinkCount = 0;     // (더 이상 사용하지 않음)

  // 부저: 무조건 OFF로 시작
  nbBuzzOnState = false;
  nbToneStep    = 0;

  // 밝기 기반 감마 보정 초기 계산
  float norm     = brightness / 30.0;
  float adj      = pow(norm, 2.0);
  nbDutyOn       = int(adj * 80.0 + 10.0);  // 10~90ms
  nbDutyOff      = 100 - nbDutyOn;
  nbLedInterval  = nbDutyOn;

  // 부저 인터벌 초기화
  nbBuzzInterval = 40;
}

// 매 프레임 호출할 업데이트 함수
void updateLedAndBuzzer() {
  unsigned long now = millis();
  const unsigned long BUZZER_ON_TIME  = 100;  // 부저 ON 지속(ms)
  const unsigned long BUZZER_OFF_TIME = 50;   // 부저 OFF 지속(ms)

  // === LED 3회씩 파랑 ↔ 빨강 깜빡임 상태 머신 ===
  if (nbLedEnable) {
    if (brightness > 0) {
      // [기존 깜빡임 로직 그대로 사용]
      if (now - nbPrevLedMillis >= nbLedInterval) {
        nbPrevLedMillis = now;
        if (nbLedOnState) {
          // on → off
          nbLedOnState = false;
          digitalWrite(LATCH_PIN, LOW);
          shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, 0);
          digitalWrite(LATCH_PIN, HIGH);
          nbLedInterval = nbDutyOff;
        } else {
          // off → on
          nbLedOnState = true;
          byte pattern = nbLedPattern ? 0b11110000 : 0b00001111;
          digitalWrite(LATCH_PIN, LOW);
          shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, pattern);
          digitalWrite(LATCH_PIN, HIGH);
          nbLedInterval = nbDutyOn;
          if (++nbLedCycleCount >= 3) {
            nbLedCycleCount = 0;
            nbLedPattern = 1 - nbLedPattern;
          }
        }
      }
    } else {
      // brightness == 0 일 때는 무조건 LED 꺼두기
      digitalWrite(LATCH_PIN, LOW);
      shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, 0);
      digitalWrite(LATCH_PIN, HIGH);
    }
  }

  // === 부저 상태 머신 ===
  if (nbBuzzEnable) {
    if (volumeLevel > 0) {
      if (now - nbPrevBuzzMillis >= nbBuzzInterval) {
        nbPrevBuzzMillis = now;
        if (nbBuzzOnState) {
          noToneAC();
          irrecv.enableIRIn();
          nbBuzzOnState  = false;
          nbToneStep     = 1 - nbToneStep;
          nbBuzzInterval = BUZZER_OFF_TIME;
        } else {
          int freq = map(volumeLevel,1,30,1600,1100) + (nbToneStep ? 150 : 0);
          toneAC(freq); 
          nbBuzzOnState  = true;
          nbBuzzInterval = BUZZER_ON_TIME;
        }
      }
    } else {
      // volumeLevel == 0 일 때 부저 완전 정지
      noToneAC();
      irrecv.enableIRIn();
    }
  }
}

// 로터리 엔코더 디바운스 함수
void onEncoderChange() {
  static unsigned long lastMicros = 0;
  unsigned long now = micros();
  if (now - lastMicros < 2000) return;      // 2 ms 디바운스
  lastMicros = now;

  uint8_t newState = (digitalRead(ROTARY_DT) << 1) | digitalRead(ROTARY_CLK);
  uint8_t idx      = (lastState << 2) | newState;
  int8_t  delta    = QUAD_TABLE[idx];       // +1, -1, 또는 0

  if (delta != 0) stepSum += delta;         // 유효 이동만 누적
  lastState = newState;

  /* 00(‘00’) 상태로 돌아왔을 때 4스텝 누적 값 확정
     → 한 디텐트 당 +4 또는 -4 가 모여야 ±1 카운트 */
  if (newState == 0) {
    if (stepSum >= 3)      encoderPos++;    // 시계방향
    else if (stepSum <= -3) encoderPos--;   // 반시계방향
    stepSum = 0;                           // 버퍼 초기화
  }
}

// 택트 스위치 디바운스 함수
bool readWakeButton() {
  bool raw = digitalRead(WAKE_BUTTON_PIN);  // 현재 RAW 읽기

  // RAW 값이 이전과 다르면, 변화를 감지한 시점을 기록하고 lastRaw 업데이트
  if (raw != wakeLastRaw) {
    wakeLastChangeTime = millis();
    wakeLastRaw = raw;
  }

  // 만약 RAW 값이 50ms 동안 동일하게 유지되었다면 안정 상태로 간주
  if (millis() - wakeLastChangeTime > WAKE_DEBOUNCE_DELAY) {
    if (wakeLastStable != raw) {
      wakeLastStable = raw;
    }
  }

  // 최종 안정 상태 반환 (LOW = 눌림, HIGH = 풀업 상태)
  return wakeLastStable;
}

// —————— 화재 감지 및 인체 감지 ——————

// 화재 감지
bool holdStateFire(bool cond, bool& flag, unsigned long& t0,
               const char* label, unsigned long duration = HOLD_DURATION, bool debug = false) {
  unsigned long now = millis();
  int sensorValue = 0;
  int threshold = 0;
  String valueStr;

  // (1) 센서 값과 임계값을 저장 (디버그용 출력에 사용)
  if (strstr(label, "온도")) {
    // 온도는 float이므로 별도 출력
    float tempC = sensors.getTempCByIndex(0);
    sensorValue = (int)tempC;
    threshold = (int)TEMP_THRESHOLD;
    valueStr = String(sensorValue);
  } else if (strstr(label, "조도")) {
    int lightVal = analogRead(LIGHT_PIN);
    sensorValue = lightVal;
    threshold = LIGHT_THRESHOLD;
    valueStr = String(sensorValue);
  } else if (strstr(label, "불꽃")) {
    int f1 = analogRead(FLAME1_PIN);
    int f2 = analogRead(FLAME2_PIN);
    threshold = FLAME_THRESHOLD;
    valueStr = "센서1=" + String(f1) + ", 센서2=" + String(f2);
    sensorValue = min(f1, f2);  // 조건 평가에 사용
  }

  // (2) 조건이 처음 참이 된 순간
  if (cond && !flag) {
    flag = true;
    t0 = now;
    if (debug) {
      // 유지 중(최초 감지)
      Serial.print(label);
      Serial.print(" 유지 중… ✅ 값: ");
      Serial.print(valueStr);
      Serial.print(" (Threshold: ");
      Serial.print(threshold);
      Serial.println(")");
    }
    return false;  // 아직 duration 경과 전
  }

  // (3) 이미 flag가 참이고 duration 경과 전
  if (flag) {
    unsigned long elapsed = now - t0;
    if (elapsed < duration) {
      if (debug) {
        Serial.print(label);
        Serial.print(" 유지 중… (");
        Serial.print(elapsed / 1000.0, 2);
        Serial.print("s) ✅ 값: ");
        Serial.print(valueStr);
        Serial.print(" (Threshold: ");
        Serial.print(threshold);
        Serial.println(")");
      }
      return false;
    } else {
      // (4) duration 경과하여 최종 감지 상태
      flag = false;
      if (debug) {
        Serial.print(label);
        Serial.print(" 5초 유지 완료! 🔥 ✅ 값: ");
        Serial.print(valueStr);
        Serial.print(" (Threshold: ");
        Serial.print(threshold);
        Serial.println(")");
      }
      return true;
    }
  }

  // (5) cond가 거짓이면 "없음" 출력
  if (debug) {
    Serial.print(label);
    Serial.print(" 없음 ❌ 값: ");
    Serial.print(valueStr);
    Serial.print(" (Threshold: ");
    Serial.print(threshold);
    Serial.println(")");
  }
  return false;
}

// 인체 감지
// duration: 0으로 설정하면 바로 한 번 감지됨으로 리턴
bool holdStateHuman(bool cond, bool& flag, unsigned long& t0,
                    const char* label, unsigned long duration = HOLD_DURATION_HUMAN, bool debug = false) {
  unsigned long now = millis();
  float elapsedS = (now - t0) / 1000.0;
  String valueStr;
  String thresholdStr;

  // (1) 센서 값과 임계값 문자열 생성 (디버그 출력용)
  if (strstr(label, "기울기")) {
    int16_t ax, ay, az;
    mpu.getAcceleration(&ax, &ay, &az);
    float pitch = atan2(ax/16384.0, sqrt(sq(ay/16384.0) + sq(az/16384.0))) * 180/PI;
    float roll  = atan2(ay/16384.0, sqrt(sq(ax/16384.0) + sq(az/16384.0))) * 180/PI;
    valueStr = "Pitch=" + String(pitch, 2) + "°, Roll=" + String(roll, 2) + "°";
    thresholdStr = String(angleThreshold) + "°";
  } else if (strstr(label, "초음파1")) {
    float d1 = getDistance(TRIG1, ECHO1);
    valueStr = String(d1, 2) + "cm";
    thresholdStr = String(distanceThreshold) + "cm";
  } else if (strstr(label, "초음파2")) {
    float d2 = getDistance(TRIG2, ECHO2);
    valueStr = String(d2, 2) + "cm";
    thresholdStr = String(distanceThreshold) + "cm";
  } else if (strstr(label, "충격")) {
    int shockVal = digitalRead(SHOCK_PIN);
    valueStr = String(shockVal);
    thresholdStr = "HIGH=감지";
  } else if (strstr(label, "소리")) {
    int soundVal = analogRead(SOUND_SENSOR_PIN);
    valueStr = String(soundVal);
    thresholdStr = "100";
  } else if (strstr(label, "PIR1")) {
    int pir1Val = digitalRead(PIR1_PIN);
    valueStr = String(pir1Val);
    thresholdStr = "HIGH=감지";
  } else if (strstr(label, "PIR2")) {
    int pir2Val = digitalRead(PIR2_PIN);
    valueStr = String(pir2Val);
    thresholdStr = "HIGH=감지";
  }

  // (2) 조건이 처음 참이 된 순간
  if (cond && !flag) {
    flag = true;
    t0 = now;
    if (debug) {
      Serial.print(label);
      Serial.print(" 유지 중… ✅ 값: ");
      Serial.print(valueStr);
      Serial.print(" (Threshold: ");
      Serial.print(thresholdStr);
      Serial.println(")");
    }
    return true;  // 최초 감지 시 true 반환
  }

  // (3) 이미 flag가 참이고 duration 경과 전
  if (flag) {
    if (now - t0 < duration) {
      if (debug) {
        Serial.print(label);
        Serial.print(" 유지 중… (");
        Serial.print(elapsedS, 2);
        Serial.print("s) ✅ 값: ");
        Serial.print(valueStr);
        Serial.print(" (Threshold: ");
        Serial.print(thresholdStr);
        Serial.println(")");
      }
      return true;
    } else {
      // (4) duration 경과하여 최종 감지 상태 해제
      flag = false;
      if (debug) {
        Serial.print(label);
        Serial.print(" 5초 유지 완료! 🔍 ✅ 값: ");
        Serial.print(valueStr);
        Serial.print(" (Threshold: ");
        Serial.print(thresholdStr);
        Serial.println(")");
      }
      return false;
    }
  }

  // (5) cond가 거짓이면 "없음" 출력
  if (debug) {
    Serial.print(label);
    Serial.print(" 없음 ❌ 값: ");
    Serial.print(valueStr);
    Serial.print(" (Threshold: ");
    Serial.print(thresholdStr);
    Serial.println(")");
  }
  return false;
}

// 초음파 거리(cm) 측정
float getDistance(int trig, int echo) {
  digitalWrite(trig, LOW);  delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(20);
  digitalWrite(trig, LOW);
  long d = pulseIn(echo, HIGH, 30000);
  return d ? d * 0.034 / 2 : -1;
}

// 택트 스위치, IR 논블로킹으로 동시 처리 
void processAdmin() {
  unsigned long now = millis();

  // (A) 화재 알림 15초 경과 시 종료
  if (fireState && now - fireTime >= 15000UL) {
    // LED·부저 OFF
    digitalWrite(LATCH_PIN, LOW);
    shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, 0x00);
    digitalWrite(LATCH_PIN, HIGH);
    noToneAC();
    irrecv.enableIRIn();
    fireState = false;
  }

  // (B) 인체 알림 15초 경과 시 종료
  if (humanAlertState && now - humanAlertTime >= 15000UL) {
    digitalWrite(LATCH_PIN, LOW);
    shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, 0x00);
    digitalWrite(LATCH_PIN, HIGH);
    noToneAC();
    irrecv.enableIRIn();
    humanAlertState = false;
  }
  
  // 1) PIN 입력 논블로킹 처리
  if (pinInputActive) {
    // 1-A) 논블로킹 키패드 입력
    char k = getNavKeyNB();
    if (k >= '0' && k <= '9') {
      pinBuffer += k;
      lcd.print(k);
    }
    else if (k == '*') {
      // 백스페이스: 마지막 문자 삭제
      if (pinBuffer.length() > 0) {
        pinBuffer.remove(pinBuffer.length() - 1);
        // 화면 갱신
        lcd.setCursor(0,1);
        lcd.print("    ");
        lcd.setCursor(0,1);
        lcd.print(pinBuffer);
      }
    }
    else if (k == '#') {
      // PIN 입력 완료
      pinInputActive = false;
      if (pinBuffer == adminPW) {
        // 주간 모드 전환 전 LED·부저 즉시 끄기
        digitalWrite(LATCH_PIN, LOW);
        shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, 0x00);
        digitalWrite(LATCH_PIN, HIGH);
        noToneAC();
        irrecv.enableIRIn();
        fireState = false;
        humanAlertState = false;

        // 주간 모드 전환
        isNightMode = false;
        nightModeInitialized = false;
        lcd.clear();
        lcd.print("Day Mode");
        delay(1000);

        // RFID 모듈 재활성화 준비
        digitalWrite(RST_PIN, HIGH);
        delay(50);
        rfid.PCD_Init();
        rfid.PCD_AntennaOn();
        systemActive = true;
        activeStart = millis();

        lcd.clear();
        lcd.print("Ready");
        return;
      } else {
        // 잘못된 비밀번호
        lcd.clear();
        lcd.print("Wrong PW");
        delay(1000);
        // 재진입 준비
        pinBuffer = "";
        pinInputActive = true;
        lcd.clear();
        lcd.print("Enter PW:");
        lcd.setCursor(0,1);
      }
    }

    // 1-B) 논블로킹 IR 입력
    if (IrReceiver.decode()) {
      IrReceiver.resume();
      char c = mapIRToChar(IrReceiver.decodedIRData.command);
      if (c >= '0' && c <= '9') {
        pinBuffer += c;
        lcd.print(c);
      }
      else if (c == '#') {
        // PIN 입력 완료
        pinInputActive = false;
        if (pinBuffer == adminPW) {
          // 주간 모드 전환 전 LED·부저 즉시 끄기
          digitalWrite(LATCH_PIN, LOW);
          shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, 0x00);
          digitalWrite(LATCH_PIN, HIGH);
          noToneAC();
          irrecv.enableIRIn();
          fireState = false;
          humanAlertState = false;

          // 주간 모드 전환
          isNightMode = false;
          nightModeInitialized = false;
          lcd.clear();
          lcd.print("Day Mode");
          delay(1000);

          // RFID 모듈 재활성화 준비
          digitalWrite(RST_PIN, HIGH);
          delay(50);
          rfid.PCD_Init();
          rfid.PCD_AntennaOn();
          systemActive = true;
          activeStart = millis();

          lcd.clear();
          lcd.print("Ready");
          return;
        } else {
          // 잘못된 비밀번호
          lcd.clear();
          lcd.print("Wrong PW");
          delay(1000);
          // 재진입 준비
          pinBuffer = "";
          pinInputActive = true;
          lcd.clear();
          lcd.print("Enter PW:");
          lcd.setCursor(0,1);
        }
      }
    }
    return;  // PIN 모드일 때는 다른 입력 루틴 실행 안 함
  }

  // 2) 택트 스위치 눌림 감지
  if (readWakeButton() == LOW && !adminActive) {
    // Admin? 화면 표시 (최초 진입 시 한 번만)
    adminActive = true;
    adminStart  = millis();
    lcd.backlight();
    lcd.clear();
    lcd.print("Admin?");
    // RFID 모듈 활성화
    digitalWrite(RST_PIN, HIGH);
    delay(50);
    rfid.PCD_Init();
    rfid.PCD_AntennaOn();
  }

  // 3) Admin? 메뉴 논블로킹 처리
  if (adminActive) {
    // — (3-A) RFID 인증만 처리 —
    if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
      // UID 문자열 생성
      String tag;
      for (byte i = 0; i < rfid.uid.size; i++) {
        if (rfid.uid.uidByte[i] < 0x10) tag += "0";
        tag += String(rfid.uid.uidByte[i], HEX);
        if (i < rfid.uid.size - 1) tag += " ";
      }
      tag.toUpperCase();
      String userFile = String(SD_USERS_FOLDER) + "/" + uidToName(tag) + ".txt";

      if (SD.exists(userFile)) {
        // 파일에서 “ID:” 라인을 찾아 사용자 ID 읽기
        File f = SD.open(userFile);
        String line, userID;
        while (f.available()) {
          line = f.readStringUntil('\n');
          if (line.startsWith("ID:")) {
            userID = line.substring(4);
            break;
          }
        }
        f.close();
        userID.trim();

        if (userID == adminID) {
          // 관리자 카드 → PIN 입력 단계 진입
          pinInputActive = true;
          pinBuffer = "";
          lcd.clear();
          lcd.print("Enter PW:");
          lcd.setCursor(0, 1);
        } else {
          // 관리자 카드가 아님
          lcd.clear();
          lcd.print("No Access");
          delay(1000);
          lcd.clear();
          lcd.print("Admin?");
        }

        rfid.PICC_HaltA();
        rfid.PCD_StopCrypto1();
      } else {
        // 등록되지 않은 카드
        lcd.clear();
        lcd.print("Unknown");
        delay(1000);
        lcd.clear();
        lcd.print("Admin?");
      }
    }

    // — (3-B) 20초 타이머 관리 및 백라이트 제어 —
    if (millis() - adminStart < ADMIN_TIMEOUT) {
      // 진입 후 20초 이내 → 백라이트 유지
      lcd.backlight();
    } else {
      // 타임아웃 도달 → 종료 처리
      digitalWrite(LATCH_PIN, LOW);
      shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, 0x00);
      digitalWrite(LATCH_PIN, HIGH);
      noToneAC();
      adminActive = false;
      pinInputActive = false;
      lcd.clear();
      lcd.noBacklight();
      rfid.PCD_AntennaOff();
      digitalWrite(RST_PIN, LOW);
    }

    // 메뉴 진입 중에도 LED·부저 경보 갱신
    updateLedAndBuzzer();
    return;  // Admin? 모드일 때는 IR PIN 입력 파트로 넘어가지 않음
  }

  // 4) IR 리모컨 입력 → 직접 PIN 입력 (언제든 가능)
  if (IrReceiver.decode()) {
    // 반복 신호 무시
    if (IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT) {
      IrReceiver.resume();
    } else {
      unsigned long rawCommand = IrReceiver.decodedIRData.command;
      char c = mapIRToChar(rawCommand);
      IrReceiver.resume();

      if (c >= '0' && c <= '9') {
        // 숫자 입력 시 irBuffer에 누적, LCD에 표시
        irBuffer += c;
        lastIrMillis = millis();
        lcd.backlight();
        lcd.clear();
        lcd.print("Enter PW:");
        lcd.setCursor(0, 1);
        lcd.print(irBuffer);
      }
      else if (c == '#') {
        // ‘#’ 입력 시 PIN 확인
        if (irBuffer == adminPW) {
          // 주간 모드 전환 전 LED·부저 즉시 끄기
          digitalWrite(LATCH_PIN, LOW);
          shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, 0x00);
          digitalWrite(LATCH_PIN, HIGH);
          noToneAC();
          irrecv.enableIRIn();
          fireState = false;
          humanAlertState = false;

          // 주간 모드 전환
          isNightMode = false;
          nightModeInitialized = false;
          irBuffer = "";
          lcd.clear();
          lcd.print("Day Mode");
          delay(1000);

          digitalWrite(RST_PIN, HIGH);
          delay(50);
          rfid.PCD_Init();
          rfid.PCD_AntennaOn();
          systemActive = true;
          activeStart = millis();
          lcd.clear();
          lcd.print("Ready");
          return;
        } else {
          // 잘못된 PIN
          irBuffer = "";
          lcd.clear();
          lcd.print("Wrong PW");
          delay(1000);
          lcd.clear();
          lcd.noBacklight();
        }
      }
    }
  }

  // 5) IR 입력 버퍼 타임아웃 (3초)
  if (irBuffer.length() > 0 && (millis() - lastIrMillis) >= irTimeout) {
    irBuffer = "";
    lcd.clear();
    lcd.noBacklight();
  }
}

// —————— 관리자 메뉴 & 서브메뉴 ——————

// 관리자 메뉴 1번 – 등록된 사용자 목록을 출력하고 개별/전체 삭제 기능 제공
void menuUsers() {
  int total = countUsersOnSD();  // 전체 등록 사용자 수
  if (!total) {
    lcd.clear(); lcd.print("No Users");
    delay(1500);
    return;
  }

  int idx = 1;        // 현재 사용자 인덱스
  int prevIdx = -1;   // 이전 인덱스 기억
  while (true) {
    // 1) 인덱스 변경 감지 시에만 화면 갱신
    if (idx != prevIdx) {
      lcd.clear();
      lcd.print(String(idx) + "/" + String(total));
      lcd.setCursor(0, 1);
      lcd.print(getIDAt(idx));
      prevIdx = idx;
    }

    // 2) 조이스틱으로 스크롤 처리
    bool changed = false;
    idx = getNavScroll(idx, total, changed);

    // 3) 조이스틱이 움직이지 않았을 때만 논블로킹 키패드 입력 처리
    char k = 0;
    if (!changed) {
      k = getNavKeyNB();  // 논블로킹: 키가 있으면 반환, 없으면 0
    }

    if      (k == 'C') break;  // 뒤로 가기
    else if (k == 'A') idx = (idx == 1 ? total : idx - 1);
    else if (k == 'B') idx = idx % total + 1;

    // — 현재 사용자 삭제 ('*') —
    else if (k == '*') {
      lcd.clear(); lcd.print("Delete it?");
      lcd.setCursor(0,1); lcd.print("1:Yes 2:No");
      char confirm = getNavKey();  // 블로킹: 1/2 대기
      if (confirm == '1') {
        SD.remove(String(SD_USERS_FOLDER) + "/" + getUserFileAt(idx));
        total = countUsersOnSD();
        if (!total) {
          lcd.clear(); lcd.print("No Users"); delay(1000);
          return;
        }
        idx = constrain(idx, 1, total);
        prevIdx = -1;  // 화면 갱신 강제
      } else if (confirm == '2') {
        // 삭제 취소: 목록 화면으로 복귀
        prevIdx = -1;
      }
    }

    // — 전체 사용자 삭제 ('D') —
    else if (k == 'D') {
      lcd.clear(); lcd.print("Reset all?");
      lcd.setCursor(0,1); lcd.print("1:Yes 2:No");
      char confirm = getNavKey();  // 블로킹: 1/2 대기
      if (confirm == '1') {
        File dir = SD.open(SD_USERS_FOLDER);
        while (true) {
          File f = dir.openNextFile();
          if (!f) break;
          SD.remove(String(SD_USERS_FOLDER) + "/" + String(f.name()));
          f.close();
        }
        dir.close();
        lcd.clear(); lcd.print("No Users"); delay(1000);
        return;
      } else if (confirm == '2') {
        // 초기화 취소: 목록 화면으로 복귀
        prevIdx = -1;
      }
    }

    delay(20);  // 민감도 조절
  }
}

// 관리자 메뉴 2번 – 날짜별 출퇴근 로그 조회 및 삭제 기능 제공
void menuCommute() {
  // 출퇴근 관리 로그는 type == 1
  const char* folder = SD_COMMUTE_LOG_FOLDER;

  int totalF = countLogs(1);  // 날짜 로그 파일 개수 확인
  if (!totalF) {
    lcd.clear(); lcd.print("No Logs");
    delay(1500);
    return;
  }

  int fIdx = 1;        // 날짜 인덱스
  int prevFIdx = -1;   // 이전 날짜 인덱스 기억

  while (true) {
    // 1) 날짜 목록 표시
    if (fIdx != prevFIdx) {
      lcd.clear();
      lcd.print(String(fIdx) + "/" + String(totalF));
      lcd.setCursor(0,1);
      String fn = getEventFileAt_Desc(1, fIdx);  // 최신순 파일명
      String date = fn.substring(0,4) + "/" + fn.substring(4,6) + "/" + fn.substring(6,8);
      lcd.print(date);
      prevFIdx = fIdx;
    }

    // 2) 조이스틱 스크롤 처리
    bool changed = false;
    fIdx = getNavScroll(fIdx, totalF, changed);

    // 3) 조이스틱 안 움직일 때만 논블로킹 키패드 처리
    if (!changed) {
      char k = getNavKeyNB();
      if      (k == 'C') return;
      else if (k == 'A') fIdx = (fIdx == 1 ? totalF : fIdx - 1);
      else if (k == 'B') fIdx = fIdx % totalF + 1;

      // — 현재 날짜 로그 삭제 ('*') —
      else if (k == '*') {
        lcd.clear(); lcd.print("Delete?");
        lcd.setCursor(0,1); lcd.print("1:Yes 2:No");
        char confirm = getNavKey();
        if (confirm == '1') {
          String fn = getEventFileAt_Desc(1, fIdx);
          SD.remove(String(folder) + "/" + fn);
          totalF = countLogs(1);
          if (!totalF) {
            lcd.clear(); lcd.print("No Logs"); delay(1000);
            return;
          }
          fIdx = constrain(fIdx, 1, totalF);
          prevFIdx = -1;
        } else if (confirm == '2') {
          // 삭제 취소: 날짜 목록 화면으로 복귀
          prevFIdx = -1;
        }
      }

      // — 전체 로그 초기화 ('D') —
      else if (k == 'D') {
        lcd.clear(); lcd.print("Reset all?");
        lcd.setCursor(0,1); lcd.print("1:Yes 2:No");
        char confirm = getNavKey();
        if (confirm == '1') {
          File dir = SD.open(folder);
          while (true) {
            File f = dir.openNextFile();
            if (!f) break;
            SD.remove(String(folder) + "/" + String(f.name()));
            f.close();
          }
          dir.close();
          lcd.clear(); lcd.print("No Logs"); delay(1000);
          return;
        } else if (confirm == '2') {
          // 초기화 취소: 날짜 목록 화면으로 복귀
          prevFIdx = -1;
        }
      }

      // — 로그 항목 탐색 ('#') —
      else if (k == '#') {
        String fn = getEventFileAt_Desc(1, fIdx);
        String path = String(folder) + "/" + fn;
        int totalE = countLogEntries(path);
        if (!totalE) {
          lcd.clear(); lcd.print("No Entries"); delay(1000);
          prevFIdx = -1;
          continue;
        }
        int eIdx = 1, prevEIdx = -1;
        while (true) {
          if (eIdx != prevEIdx) {
            lcd.clear();
            String line = getLogLine(path, eIdx);
            String time = line.substring(1,9);
            int p = line.indexOf("ID: ");
            String id = (p>=0? line.substring(p+4,p+12):"");
            lcd.print(String(eIdx)+"/"+String(totalE)+" " + time);
            lcd.setCursor(0,1); lcd.print("ID:"+id);
            prevEIdx = eIdx;
          }
          bool subChanged = false;
          eIdx = getNavScroll(eIdx, totalE, subChanged);
          if (!subChanged) {
            char kk = getNavKeyNB();
            if (kk=='C') break;
            else if (kk=='A') eIdx = (eIdx==1? totalE: eIdx-1);
            else if (kk=='B') eIdx = eIdx % totalE + 1;
            else if (kk=='*') {
              lcd.clear(); lcd.print("Delete it?");
              lcd.setCursor(0,1); lcd.print("1:Yes 2:No");
              char c2 = getNavKey();
              if (c2=='1') {
                deleteLogEntry(path, eIdx);
                totalE = countLogEntries(path);
                if (!totalE) break;
                eIdx = constrain(eIdx,1,totalE);
                prevEIdx = -1;
              } else if (c2=='2') {
                prevEIdx = -1;
              }
            }
            else if (kk=='D') {
              lcd.clear(); lcd.print("Reset all?");
              lcd.setCursor(0,1); lcd.print("1:Yes 2:No");
              char c2 = getNavKey();
              if (c2=='1') {
                SD.remove(path);
                break;
              } else if (c2=='2') {
                prevEIdx = -1;
              }
            }
          }
          delay(20);
        }
        totalF = countLogs(1);
        if (!totalF) return;
        fIdx = constrain(fIdx, 1, totalF);
        prevFIdx = -1;
      }
    }
    delay(20);
  }
}

// 관리자 메뉴 3번 – 날짜별 인체 감지 로그 조회 및 삭제 기능 제공
void menuHUMAN() {
  // 인체 감지 로그는 type == 2
  const char* folder = SD_HUMAN_LOG_FOLDER;

  int totalF = countLogs(2);  // 인체 감지 로그 파일 개수 확인
  if (!totalF) {
    lcd.clear(); lcd.print("No Logs");
    delay(1500);
    return;
  }

  int fIdx = 1;        // 날짜(또는 이벤트) 인덱스
  int prevFIdx = -1;   // 이전 인덱스 기억

  while (true) {
    // 1) 목록 표시
    if (fIdx != prevFIdx) {
      lcd.clear();
      lcd.print(String(fIdx) + "/" + String(totalF));
      lcd.setCursor(0,1);
      String fn = getEventFileAt_Desc(2, fIdx);  // 최신순 파일명
      String date = fn.substring(0,4) + "/" + fn.substring(4,6) + "/" + fn.substring(6,8);
      lcd.print(date);
      prevFIdx = fIdx;
    }

    // 2) 조이스틱 스크롤 처리
    bool changed = false;
    fIdx = getNavScroll(fIdx, totalF, changed);

    // 3) 조이스틱 안 움직일 때만 논블로킹 키패드 처리
    if (!changed) {
      char k = getNavKeyNB();
      if      (k == 'C') return;
      else if (k == 'A') fIdx = (fIdx == 1 ? totalF : fIdx - 1);
      else if (k == 'B') fIdx = fIdx % totalF + 1;

      // — 현재 이벤트 로그 삭제 ('*') —
      else if (k == '*') {
        lcd.clear(); lcd.print("Delete?");
        lcd.setCursor(0,1); lcd.print("1:Yes 2:No");
        char confirm = getNavKey();
        if (confirm == '1') {
          String fn = getEventFileAt_Desc(2, fIdx);
          SD.remove(String(folder) + "/" + fn);
          totalF = countLogs(2);
          if (!totalF) {
            lcd.clear(); lcd.print("No Logs"); delay(1000);
            return;
          }
          fIdx = constrain(fIdx, 1, totalF);
          prevFIdx = -1;
        } else if (confirm == '2') {
          // 삭제 취소 → 목록 화면으로 복귀
          prevFIdx = -1;
        }
      }

      // — 전체 로그 초기화 ('D') —
      else if (k == 'D') {
        lcd.clear(); lcd.print("Reset all?");
        lcd.setCursor(0,1); lcd.print("1:Yes 2:No");
        char confirm = getNavKey();
        if (confirm == '1') {
          File dir = SD.open(folder);
          while (true) {
            File f = dir.openNextFile();
            if (!f) break;
            SD.remove(String(folder) + "/" + String(f.name()));
            f.close();
          }
          dir.close();
          lcd.clear(); lcd.print("No Logs"); delay(1000);
          return;
        } else if (confirm == '2') {
          // 초기화 취소 → 목록 화면으로 복귀
          prevFIdx = -1;
        }
      }

      // — 로그 항목 탐색 ('#') —
      else if (k == '#') {
        String fn = getEventFileAt_Desc(2, fIdx);
        String path = String(folder) + "/" + fn;
        int totalE = countLogEntries(path);
        if (!totalE) {
          lcd.clear(); lcd.print("No Entries"); delay(1000);
          prevFIdx = -1;
          continue;
        }
        int eIdx = 1, prevEIdx = -1;
        while (true) {
          if (eIdx != prevEIdx) {
            lcd.clear();
            String line = getLogLine(path, eIdx);
            String time = line.substring(1,9);
            int p = line.indexOf("ID: ");
            String id = (p>=0? line.substring(p+4,p+9):"");
            lcd.print(String(eIdx)+"/"+String(totalE)+" " + time);
            lcd.setCursor(0,1); lcd.print(id);
            prevEIdx = eIdx;
          }
          bool subChanged = false;
          eIdx = getNavScroll(eIdx, totalE, subChanged);
          if (!subChanged) {
            char kk = getNavKeyNB();
            if (kk=='C') break;
            else if (kk=='A') eIdx = (eIdx==1? totalE: eIdx-1);
            else if (kk=='B') eIdx = eIdx % totalE + 1;
            else if (kk=='*') {
              lcd.clear(); lcd.print("Delete it?");
              lcd.setCursor(0,1); lcd.print("1:Yes 2:No");
              char c2 = getNavKey();
              if (c2=='1') {
                deleteLogEntry(path, eIdx);
                totalE = countLogEntries(path);
                if (!totalE) break;
                eIdx = constrain(eIdx,1,totalE);
                prevEIdx = -1;
              } else if (c2=='2') {
                prevEIdx = -1;
              }
            }
            else if (kk=='D') {
              lcd.clear(); lcd.print("Reset all?");
              lcd.setCursor(0,1); lcd.print("1:Yes 2:No");
              char c2 = getNavKey();
              if (c2=='1') {
                SD.remove(path);
                break;
              } else if (c2=='2') {
                prevEIdx = -1;
              }
            }
          }
          delay(20);
        }
        totalF = countLogs(2);
        if (!totalF) return;
        fIdx = constrain(fIdx, 1, totalF);
        prevFIdx = -1;
      }
    }
    delay(20);
  }
}

// 관리자 메뉴 4번 – 날짜별 화재 감지 로그 조회 및 삭제 기능 제공
void menuFire() {
  const char* folder = SD_FIRE_LOG_FOLDER;
  int totalF = countLogs(3);
  if (!totalF) {
    lcd.clear(); lcd.print("No Logs");
    delay(1500);
    return;
  }

  int fIdx = 1, prevFIdx = -1;
  while (true) {
    // 1) 목록 표시 (날짜)
    if (fIdx != prevFIdx) {
      lcd.clear();
      lcd.print(String(fIdx) + "/" + String(totalF));
      lcd.setCursor(0,1);
      String fn   = getEventFileAt_Desc(3, fIdx);           // 최신순 파일명
      String date = fn.substring(0,4) + "/"                  // "YYYY/MM/DD"
                    + fn.substring(4,6) + "/"
                    + fn.substring(6,8);
      lcd.print(date);                                      // e.g. "2025/06/01"
      prevFIdx = fIdx;
    }

    // 조이스틱 스크롤 처리
    bool changed = false;
    fIdx = getNavScroll(fIdx, totalF, changed);

    // 삭제 등 메뉴 진입
    char k = (!changed ? getNavKeyNB() : 0);
    if (k == 'C') return;
    else if (k == 'A') fIdx = (fIdx == 1 ? totalF : fIdx - 1);
    else if (k == 'B') fIdx = fIdx % totalF + 1;

    // — 로그 항목 탐색 ('#') —
    else if (k == '#') {
      String fn = getEventFileAt_Desc(3, fIdx);
      String path = String(folder) + "/" + fn;
      int totalE = countLogEntries(path);
      if (!totalE) {
        lcd.clear(); lcd.print("No Entries"); delay(1000);
        prevFIdx = -1;
        continue;
      }
      int eIdx = 1, prevEIdx = -1;
      while (true) {
        if (eIdx != prevEIdx) {
          lcd.clear();
          String line = getLogLine(path, eIdx);
          String time = line.substring(1,9);
          int p = line.indexOf("ID: ");
          String id = (p>=0? line.substring(p+4,p+8):"");
          lcd.print(String(eIdx)+"/"+String(totalE)+" " + time);
          lcd.setCursor(0,1); lcd.print(id);
          prevEIdx = eIdx;
        }
        bool subChanged = false;
        eIdx = getNavScroll(eIdx, totalE, subChanged);
        if (!subChanged) {
          char kk = getNavKeyNB();
          if (kk=='C') break;
          else if (kk=='A') eIdx = (eIdx==1? totalE: eIdx-1);
          else if (kk=='B') eIdx = eIdx % totalE + 1;
          else if (kk=='*') {
            lcd.clear(); lcd.print("Delete it?");
            lcd.setCursor(0,1); lcd.print("1:Yes 2:No");
            char c2 = getNavKey();
            if (c2=='1') {
              deleteLogEntry(path, eIdx);
              totalE = countLogEntries(path);
              if (!totalE) break;
              eIdx = constrain(eIdx,1,totalE);
              prevEIdx = -1;
            } else if (c2=='2') {
              prevEIdx = -1;
            }
          }
          else if (kk=='D') {
            lcd.clear(); lcd.print("Reset all?");
            lcd.setCursor(0,1); lcd.print("1:Yes 2:No");
            char c2 = getNavKey();
            if (c2=='1') {
              SD.remove(path);
              break;
            } else if (c2=='2') {
              prevEIdx = -1;
            }
          }
        }
        delay(20);
      }
      totalF = countLogs(3);
        if (!totalF) return;
        fIdx = constrain(fIdx, 1, totalF);
        prevFIdx = -1;
    }
    delay(20);
  }
}

// 관리자 메뉴 5번 – 밝기, 볼륨, 관리자 ID PW 수정, 야간모드 전환 기능 제공
bool menuSetting() {
  int page = 1, prevPage = -1;
  lcd.clear();
  lcd.setCursor(0,0);  lcd.print("...");
  loadSettings();
  while (true) {
    // 1) 화면 갱신
    if (page != prevPage) {
      lcd.clear();
      lcd.setCursor(0,0);  lcd.print(String(page) + "/4");
      lcd.setCursor(0,1);
      if      (page == 1) lcd.print("1:Bright 2:Vol");
      else if (page == 2) lcd.print("3:AdminID 4:PW");
      else if (page == 3) lcd.print("5:fMode 6: hMode");
      else                lcd.print("7:Night");
      prevPage = page;
    }

    // 2) 조이스틱 스크롤
    bool changed = false;
    page = getNavScroll(page, 4, changed);

    // 3) 조이스틱이 움직이지 않을 때만 키패드 처리
    if (!changed) {
      char k = getNavKeyNB();
      if (k == 'C') {
        saveSettings();  // 저장 후 나가기
        return false;
      }
      else if (k == 'A') page = (page == 1 ? 4 : page - 1);  // 이전 페이지
      else if (k == 'B') page = (page == 4 ? 1 : page + 1);  // 다음 페이지
      else if (k == '1') {
        // Bright 메뉴 진입 → LED만 깜빡임 시작
        encoderPos = 0;              // ISR 누적값 초기화
        int lastShownB = brightness;
        lcd.clear();
        lcd.setCursor(0,0); lcd.print("Bright:");
        lcd.setCursor(8,0);
        if      (brightness == 0)  lcd.print("OFF");
        else if (brightness == 30) lcd.print("MAX");
        else                       lcd.print(brightness);

        triggerLedAndBuzzer(true, false);

        while (true) {
          updateLedAndBuzzer();  // 비차단으로 LED 상태 유지

          // ISR에서 누적된 회전량을 기반으로 밝기 조절 (방향 반전 적용)
          noInterrupts();
          int delta = -encoderPos;   // ← 방향 반전
          encoderPos = 0;
          interrupts();
          if (delta != 0) {
            brightness = constrain(brightness + delta, 0, 30);
            lcd.setCursor(8,0);
            lcd.print("   ");
            lcd.setCursor(8,0);
            if      (brightness == 0)  lcd.print("OFF");
            else if (brightness == 30) lcd.print("MAX");
            else                       lcd.print(brightness);

            // 감마 보정 재계산
            float norm = float(brightness) / 30.0;
            float adj  = pow(norm, 2.0);
            nbDutyOn    = int(adj * 80.0 + 10.0);
            nbDutyOff   = 100 - nbDutyOn;
            if (nbLedOnState) nbLedInterval = nbDutyOn;
          }

          // C키로 메뉴 탈출…
          if (getNavKeyNB() == 'C') {
            nbLedEnable = false;
            nbBuzzEnable = false;
            digitalWrite(LATCH_PIN, LOW);
            shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, 0);
            digitalWrite(LATCH_PIN, HIGH);
            prevPage = -1;
            break;
          }
          delay(20);
        }
      }
      else if (k == '2') {
        // Vol 메뉴 진입 → 부저만 울림 시작
        encoderPos = 0;              // ISR 누적값 초기화
        int lastShownV = volumeLevel;
        lcd.clear();
        lcd.setCursor(0,0); lcd.print("Vol: ");
        lcd.setCursor(5,0);
        if      (volumeLevel == 0)  lcd.print("MUTE");
        else if (volumeLevel == 30) lcd.print("MAX");
        else                        lcd.print(volumeLevel);

        triggerLedAndBuzzer(false, true);

        while (true) {
          updateLedAndBuzzer();  // 비차단으로 부저 상태 유지

          // ISR에서 누적된 회전량을 기반으로 볼륨 조절 (방향 반전 적용)
          noInterrupts();
          int delta = -encoderPos;   // ← 방향 반전
          encoderPos = 0;
          interrupts();
          if (delta != 0) {
            volumeLevel = constrain(volumeLevel + delta, 0, 30);
            lcd.setCursor(5,0);
            lcd.print("    ");
            lcd.setCursor(5,0);
            if      (volumeLevel == 0)  lcd.print("MUTE");
            else if (volumeLevel == 30) lcd.print("MAX");
            else                        lcd.print(volumeLevel);
          }

          // C키로 메뉴 탈출
          if (getNavKeyNB() == 'C') {
            nbLedEnable = false;
            nbBuzzEnable = false;
            noToneAC();
            irrecv.enableIRIn();
            prevPage = -1;
            break;
          }
          delay(20);
        }
      }
      else if (k == '3'){
        lcd.clear();
        String newID = inputID(2);
        if (newID == "") {
          // 'C'를 눌러 취소했을 때
          lcd.clear();
          lcd.print("Cancelled");
          delay(1000);
        }
        else if (newID.length() == 8) {
          adminID = newID;
          saveSettings();
          lcd.clear(); lcd.print("ID Updated");
          delay(1000);
        }
        else {
          lcd.clear(); lcd.print("Invalid Input");
          delay(1000);
        }
        prevPage = -1;  // 메뉴 화면 갱신
      }
      else if (k == '4'){
        lcd.clear();
        // inputID(4) => "Enter NEW PW:" 프롬프트
        String newPW = inputID(4);
        if (newPW == "") {
          // 'C'를 눌러 취소했을 때
          lcd.clear();
          lcd.print("Cancelled");
          delay(1000);
        }
        else if (newPW.length() == 4) {
          adminPW = newPW;
          saveSettings();
          lcd.clear(); lcd.print("PW Updated");
          delay(1000);
        }
        else {
          lcd.clear(); lcd.print("Invalid Input");
          delay(1000);
        }
        prevPage = -1;  // 메뉴 화면 갱신
      }
      else if (k == '5') {
        // 1) 사용자에게 Fire Mode 옵션 보여주기
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Fire Mode");
        lcd.setCursor(0, 1);
        lcd.print("1:ON 2:OFF 3:DBG");

        // 블로킹으로 숫자 입력 대기
        char choice = getNavKey();  

        // 'C'를 누르면 뒤로 가기
        if (choice == 'C') {
          prevPage = -1;
          continue;
        }

        // 2) 선택이 유효한 경우에만 "Change Mode?" 묻기
        if (choice == '1' || choice == '2' || choice == '3') {
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("Change Mode?");
          lcd.setCursor(0, 1);
          lcd.print("1:Yes 2:No");

          // 블로킹으로 Yes/No 입력 대기
          char confirm = getNavKey();

          // 'C'를 누르거나 2:No 면 아무 변경 없이 뒤로 가기
          if (confirm == '1') {
            if (choice == '1') {
              fireDetectMode = 1;  // 켜짐
            } else if (choice == '2') {
              fireDetectMode = 0;  // 꺼짐
            } else { 
              fireDetectMode = 2;  // 디버그 모드
            }
          }
        }
        // FIRE Mode 화면 표시 부분
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("FIRE Mode:");

        // fireDetectMode 값에 따라 ON/OFF/DBG 출력
        lcd.setCursor(0, 1);
        switch (fireDetectMode) {
          case 0:
            lcd.print("OFF");
            break;
          case 1:
            lcd.print("ON");
            break;
          case 2:
            lcd.print("DBG");
            break;
          default:
            lcd.print("ERR");  // 예외 처리
            break;
        }
        delay(1000);
        prevPage = -1;
      }
      else if (k == '6') {
        // 1) 사용자에게 HUMAN Mode 옵션 보여주기
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("HUMAN Mode");
        lcd.setCursor(0, 1);
        lcd.print("1:ON 2:OFF 3:DBG");

        // 블로킹으로 숫자 입력 대기
        char choice = getNavKey();  

        // 'C'를 누르면 뒤로 가기
        if (choice == 'C') {
          prevPage = -1;
          continue;
        }

        // 2) 선택이 유효한 경우에만 "Change Mode?" 묻기
        if (choice == '1' || choice == '2' || choice == '3') {
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("Change Mode?");
          lcd.setCursor(0, 1);
          lcd.print("1:Yes 2:No");

          // 블로킹으로 Yes/No 입력 대기
          char confirm = getNavKey();

          // 'C'를 누르거나 2:No 면 아무 변경 없이 뒤로 가기
          if (confirm == '1') {
            if (choice == '1') {
              humanDetectMode = 1;  // 켜짐
            } else if (choice == '2') {
              humanDetectMode = 0;  // 꺼짐
            } else { 
              humanDetectMode = 2;  // 디버그 모드
            }
          }
        }
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("HUMAN Mode:");

        // humanDetectMode 값에 따라 ON/OFF/DBG 출력
        lcd.setCursor(0, 1);
        switch (humanDetectMode) {
          case 0:
            lcd.print("OFF");
            break;
          case 1:
            lcd.print("ON");
            break;
          case 2:
            lcd.print("DBG");
            break;
          default:
            lcd.print("ERR");  // 예외 처리
            break;
        }
        delay(1000);
        prevPage = -1;
      }
      else if (k == '7'){
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Change Mode?");
        lcd.setCursor(0, 1);
        lcd.print("1: Yes  2: No");
        char confirm = getNavKey();  // 블로킹 대기
        if (confirm == '1') {
          isNightMode = true; // 야간 모드로 전환
          nightModeInitialized = false;

          return true; // 관리자 모드에서 빠져나감
        } else {
          lcd.clear();
          lcd.print("Cancelled");
        }
        delay(1000);
        prevPage = -1;
      }
    }
    delay(20);
  }
}

// RFID 태그가 관리자 사번과 일치할 경우 실행되는 관리자 모드 메뉴
// 1: 사용자 조회 및 삭제, 2: 출퇴근 로그 관리, 3/4: 향후 확장용 (사람 감지/화재 감지), 5: 세팅
void adminMenu() {
  int page = 1;
  int prevPage = -1;

  while (true) {
    // 화면 갱신
    if (page != prevPage) {
      lcd.clear();
      lcd.setCursor(0,0);  lcd.print(String(page) + "/3");
      lcd.setCursor(0,1);
      if      (page == 1) lcd.print("1:Users 2:Commut");
      else if (page == 2) lcd.print("3:HUMAN 4:Fire");
      else                lcd.print("5:Setting");
      prevPage = page;
    }

    // 1) 조이스틱 탐색
    bool changed = false;
    page = getNavScroll(page, 3, changed);

    // 2) 조이스틱이 움직이지 않을 때만 키패드 처리
    if (!changed) {
      char k = getNavKeyNB();
      if (k == 'C') {
        lcd.clear(); lcd.print("Ready"); delay(500);
        return;
      }
      else if (k == 'A') page = (page == 1 ? 3 : page - 1);  // 이전 페이지
      else if (k == 'B') page = (page == 3 ? 1 : page + 1);  // 다음 페이지
      else if (k == '1') {
        menuUsers();
        prevPage = -1;  // 복귀 시 화면 갱신
      }
      else if (k == '2') {
        menuCommute();
        prevPage = -1;
      }
      else if (k == '3') {
        menuHUMAN();
        prevPage = -1;
      }
      else if (k == '4') {
        menuFire();
        prevPage = -1;
      }
      else if (k == '5') {
        if (menuSetting()) return;
        prevPage = -1;
      }
    }
  }
}

// 야간 모드 전용 처리 로직:
//  1) LCD와 RFID를 끄고,
//  2) 택트 스위치로 관리자 카드를 스캔 → PW 확인 → 주간 모드로 전환
//  3) IR 리모컨으로 직접 PW 입력 → 주간 모드로 전환
void handleNightMode() {
  // 20초 동안 “Admin?” 상태를 유지하기 위한 타이머
  unsigned long lcdKeepUntil = 0;
  // 센서 판정용 타이머
  static unsigned long lastSensorMillis = 0;

  // (1) 처음 진입 시: LCD 끄고, RFID 전원/안테나 끄기
  if (!nightModeInitialized) {
    lcd.clear();
    lcd.setCursor(0, 0);    lcd.print("Night Mode");
    lcd.setCursor(0, 1);    lcd.print("Standby ...");
    delay(5000);

    lcd.clear();
    lcd.noBacklight();
    // RFID 모듈 비활성화
    rfid.PCD_AntennaOff();
    digitalWrite(RST_PIN, LOW);

    // — 비차단 깜빡임/부저 패턴 해제
    nbLedEnable  = false;
    nbBuzzEnable = false;

    // 즉시 하드웨어 OFF
    digitalWrite(LATCH_PIN, LOW);
    shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, 0x00);
    digitalWrite(LATCH_PIN, HIGH);
    noToneAC();

    // 센서·타이머 초기화
    lastSensorMillis = millis();
    adminStart       = millis();
    fireTime         = millis();
    humanAlertTime   = millis();

    // 화재 상태 해제
    fireState = false;
    // 인체 감지 상태 해제
    humanAlertState = false;

    // 관리자 메뉴/핀 입력 상태 해제
    adminActive     = false;
    pinInputActive  = false;
    irBuffer        = "";
  }

  while (isNightMode) {
    unsigned long now = millis();

    // ─── (A) 화재 알림 상태 처리 ───
    if (fireState) {
      if (now - fireTime < 15000UL) {
        // 15초 동안 LED·부저만 업데이트
        updateLedAndBuzzer();
        // 택트 스위치&IR 입력 처리 함수 호출
        processAdmin();
        continue;
      } else {
        // 15초 경과 → 화재 알림 종료
        digitalWrite(LATCH_PIN, LOW);
        shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, 0x00);
        digitalWrite(LATCH_PIN, HIGH);
        noToneAC();
        irrecv.enableIRIn();
        fireState = false;
      }
    }

    // ─── (B) 인체 알림 상태 처리 ───
    if (humanAlertState) {
      if (now - humanAlertTime < 15000UL) {
        // 15초 동안 LED·부저만 업데이트 (인체 감지)
        updateLedAndBuzzer();
        // 택트 스위치&IR 입력 처리 함수 호출
        processAdmin();
        continue;
      } else {
        // 15초 경과 → 인체 알림 종료
        digitalWrite(LATCH_PIN, LOW);
        shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, 0x00);
        digitalWrite(LATCH_PIN, HIGH);
        noToneAC();
        irrecv.enableIRIn();
        humanAlertState = false;
      }
    }

    // ─── (C) 1초마다 센서 판정 (화재/인체 알림이 아닐 때만 실행) ───
    if (!fireState && !humanAlertState && (now - lastSensorMillis >= SENSOR_INTERVAL)) {
      lastSensorMillis = now;

      // 우선 인체 센서 모두 체크하여 출력
      int humanCount = 0;
      if (humanDetectMode != 0) {
        // 1) 기울기 (MPU6050)
        int16_t ax, ay, az;
        mpu.getAcceleration(&ax, &ay, &az);
        float pitch = atan2(ax/16384.0, sqrt(sq(ay/16384.0) + sq(az/16384.0))) * 180/PI;
        float roll  = atan2(ay/16384.0, sqrt(sq(ax/16384.0) + sq(az/16384.0))) * 180/PI;
        if (holdStateHuman(
              abs(pitch) > angleThreshold || abs(roll) > angleThreshold,
              tiltFlag, tiltTime, "🙂 기울기",
              HOLD_DURATION_HUMAN,
              (humanDetectMode == 2)
            )) {
          humanCount++;
        }
        
        // 2) 초음파1
        float d1 = getDistance(TRIG1, ECHO1);
        if (holdStateHuman(
              d1 > 0 && d1 <= distanceThreshold,
              us1Flag, us1Time, "🙂 초음파1",
              HOLD_DURATION_HUMAN,
              (humanDetectMode == 2)
            )) {
          humanCount++;
        }
        
        // 3) 초음파2
        float d2 = getDistance(TRIG2, ECHO2);
        if (holdStateHuman(
              d2 > 0 && d2 <= distanceThreshold,
              us2Flag, us2Time, "🙂 초음파2",
              HOLD_DURATION_HUMAN,
              (humanDetectMode == 2)
            )) {
          humanCount++;
        }

        // 4) 충격 센서
        if (holdStateHuman(
              digitalRead(SHOCK_PIN) == HIGH,
              shockFlag, shockTime, "🙂 충격",
              HOLD_DURATION_HUMAN,
              (humanDetectMode == 2)
            )) {
          humanCount++;
        }

        // 5) 소리 센서
        if (holdStateHuman(
              analogRead(SOUND_SENSOR_PIN) > 100,
              soundFlag, soundTime, "🙂 소리",
              HOLD_DURATION_HUMAN,
              (humanDetectMode == 2)
            )) {
          humanCount++;
        }

        // 6) PIR 센서1
        if (holdStateHuman(
              digitalRead(PIR1_PIN) == HIGH,
              pir1Flag, pir1Time, "🙂 PIR1",
              HOLD_DURATION_HUMAN,
              (humanDetectMode == 2)
            )) {
          humanCount++;
        }
        // 7) PIR 센서2
        if (holdStateHuman(
              digitalRead(PIR2_PIN) == HIGH,
              pir2Flag, pir2Time, "🙂 PIR2",
              HOLD_DURATION_HUMAN,
              (humanDetectMode == 2)
            )) {
          humanCount++;
        }

        if (humanDetectMode == 2) {
          Serial.print("DEBUG 인체 감지용 센서 수 총합: ");
          Serial.println(humanCount);
        }
      }

      // 인체 출력 끝난 뒤 구분선
      if (humanDetectMode == 2) {
        Serial.println("=======================");
      }

      // 그다음 화재 센서 모두 체크하여 출력
      bool tempAlert = false, lightAlert = false, flameAlert = false;
      if (fireDetectMode != 0) {
        // 1) DS18B20 온도 판정
        sensors.requestTemperatures();
        float tempC = sensors.getTempCByIndex(0);
        if (tempC == DEVICE_DISCONNECTED_C) tempC = -100;
        bool tempCond  = (tempC >= TEMP_THRESHOLD);
        tempAlert = holdStateFire(
          tempCond, tempFlag, tempTime,
          "🔥 화재(온도)", HOLD_DURATION,
          (fireDetectMode == 2)
        );

        // 2) 조도 판정
        int lightVal = analogRead(LIGHT_PIN);
        bool lightCond  = (lightVal <= LIGHT_THRESHOLD);
        lightAlert = holdStateFire(
          lightCond, lightFlag, lightTime,
          "🔥 화재(조도)", HOLD_DURATION,
          (fireDetectMode == 2)
        );

        // 3) 불꽃 판정
        int flame1 = analogRead(FLAME1_PIN);
        int flame2 = analogRead(FLAME2_PIN);
        bool flameCond  = (flame1 <= FLAME_THRESHOLD || flame2 <= FLAME_THRESHOLD);
        flameAlert = holdStateFire(
          flameCond, flameFlag, flameTime,
          "🔥 화재(불꽃)", HOLD_DURATION,
          (fireDetectMode == 2)
        );
      }

      // 인체와 화재 구분선 이후 출력 끝나면 빈 줄 추가
      if (fireDetectMode == 2) {
        Serial.println();
      }

      // ==== 화재 상태 진입 ====
      if ((tempAlert || lightAlert || flameAlert) && !fireState) {
        fireState = true;
        fireTime  = now;
        triggerLedAndBuzzer(true, true);  // 화재 경보
        logEvent(3, "FIRE", "");          // SD 카드 기록
        continue;
      }

      // ==== 인체 상태 진입 ====
      int otherCount = humanCount - (tiltFlag ? 1 : 0);
      if ((tiltFlag && humanCount > 0) || (otherCount >= 2)) {
        humanAlertState = true;
        humanAlertTime = now;
        triggerLedAndBuzzer(true, true);
        logEvent(2, "HUMAN", "");
        continue;  // 즉시 인체 알림 모드로 진입
      }
    }

    // ─── (D) 택트 스위치 & IR 리모컨 입력 논블로킹 처리 ───
    processAdmin();

    delay(20);
  }

  // 주간 모드로 복귀할 때
  nightModeInitialized = false;
}


// —————— setup & loop ——————

// 시스템 시작 시 초기 설정을 수행하는 함수
void setup() {
  Serial.begin(9600);         // 시리얼 디버깅용
  Wire.begin();               // I2C 시작
  lcd.init(); lcd.backlight(); // LCD 초기화 및 백라이트 켜기
  lcd.clear(); lcd.print("System Ready");

  // SD 카드 설정
  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);      // 기본적으로 비활성화
  if (!SD.begin(SD_CS_PIN)) {
    lcd.clear(); lcd.print("SD init FAIL");
    while (true);                     // SD 초기화 실패 시 멈춤
  }
  loadSettings();             // 기본 설정값 가져오기

  // SD 폴더들이 없으면 생성
  if (!SD.exists(SD_USERS_FOLDER)) SD.mkdir(SD_USERS_FOLDER);
  if (!SD.exists(SD_COMMUTE_LOG_FOLDER)) SD.mkdir(SD_COMMUTE_LOG_FOLDER);
  if (!SD.exists(SD_HUMAN_LOG_FOLDER))  SD.mkdir(SD_HUMAN_LOG_FOLDER);
  if (!SD.exists(SD_FIRE_LOG_FOLDER))    SD.mkdir(SD_FIRE_LOG_FOLDER);
  if (!SD.exists(SD_SETTING_FOLDER))    SD.mkdir(SD_SETTING_FOLDER);

  // RTC 설정
  Rtc.Begin();
  RtcDateTime comp(__DATE__, __TIME__);
  if (!Rtc.IsDateTimeValid())     Rtc.SetDateTime(comp);          // 시간 초기화
  if (Rtc.GetIsWriteProtected())  Rtc.SetIsWriteProtected(false); // 쓰기 보호 해제
  if (!Rtc.GetIsRunning())        Rtc.SetIsRunning(true);         // RTC 시작

  // RFID 설정
  pinMode(RST_PIN, OUTPUT);        // RST 핀 출력 모드
  digitalWrite(RST_PIN, HIGH);     // 시작하자마자 활성화
  SPI.begin();
  rfid.PCD_Init();
  rfid.PCD_AntennaOn();

  // 키패드 핀 설정
  for (int i = 0; i < 4; i++) {
    pinMode(rowPins[i], OUTPUT); digitalWrite(rowPins[i], HIGH);
    pinMode(colPins[i], INPUT_PULLUP);
  }

  // 조이스틱 설정
  pinMode(VRX, INPUT);
  pinMode(VRY, INPUT);

  // IR 센서 설정
  pinMode(IR_SENSOR_PIN, INPUT);

  // IR 리시버 초기화
  IrReceiver.begin(IR_RECV_PIN, ENABLE_LED_FEEDBACK);  // IRrecv 시작

  // 택트 스위치 설정
  pinMode(WAKE_BUTTON_PIN, INPUT_PULLUP);  // 내부 풀업 활성화

  // 로터리 엔코더 설정
  pinMode(ROTARY_CLK, INPUT_PULLUP);
  pinMode(ROTARY_DT,  INPUT_PULLUP);
  lastState = (digitalRead(ROTARY_DT)<<1) | digitalRead(ROTARY_CLK);  // 초기 상태 저장
  attachInterrupt(digitalPinToInterrupt(ROTARY_CLK), onEncoderChange, CHANGE);  // 두 핀 모두 상태 변화 인터럽트 등록
  attachInterrupt(digitalPinToInterrupt(ROTARY_DT),  onEncoderChange, CHANGE);

  // 시프트 레지스터 설정
  pinMode(DATA_PIN, OUTPUT);
  pinMode(LATCH_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);

  // LED 출력 클리어
  digitalWrite(LATCH_PIN, LOW);
  shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, 0);
  digitalWrite(LATCH_PIN, HIGH);

  // DS18B20 센서 초기화
  sensors.begin();

  // MPU6050 초기화
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 연결 실패!");
    while (1);
  }

  // 초음파 센서
  pinMode(TRIG1, OUTPUT);
  pinMode(ECHO1, INPUT);
  digitalWrite(TRIG1, LOW);
  pinMode(TRIG2, OUTPUT);
  pinMode(ECHO2, INPUT);
  digitalWrite(TRIG2, LOW);

  // 충격 센서
  pinMode(SHOCK_PIN, INPUT);

  // 사운드 센서
  pinMode(SOUND_SENSOR_PIN, INPUT);

  // PIR 센서
  pinMode(PIR1_PIN, INPUT);
  pinMode(PIR2_PIN, INPUT);

  // 활동 상태로 두고 타이머 초기화
  systemActive = true;
  activeStart = millis();

  
  delay(1000);
  lcd.clear(); lcd.print("Ready");    // 시스템 대기 상태 표시
}

// 시스템 메인 루프 – RFID 태그 감지 및 행동 수행
void loop() {
  unsigned long now = millis();

  // 1) 야간 모드 진입 여부 판단
  if (isNightMode) {
    handleNightMode();
    return;  // handleNightMode() 내부에서 주간 모드로 전환 시 RFID를 켜고 systemActive=true로 설정했으므로,
            // 바로 이 아래부터 정상적으로 평소 루프를 타게 됩니다.
  }

  // 2) 평상시 절전 타이머 재활성화 조건 (IR 센서 감지 or 택트 스위치)
  if (digitalRead(IR_SENSOR_PIN) == LOW || readWakeButton() == LOW) {
    if (!systemActive) {
      systemActive = true;
      activeStart = now;
      // RFID ON
      digitalWrite(RST_PIN, HIGH);
      delay(50);
      rfid.PCD_Init();
      rfid.PCD_AntennaOn();
      // LCD ON
      lcd.backlight();
      lcd.clear();
      lcd.print("Ready");
    }
  }

  // 3) 시스템 비활성화 상태라면 탈출
  if (!systemActive) return;

  // 4) 절전 타이머 초과 시 자동 비활성화
  if (now - activeStart >= activeTimeout) {
    systemActive = false;
    // LCD OFF
    lcd.clear();
    lcd.noBacklight();
    // RFID OFF
    rfid.PCD_AntennaOff();
    digitalWrite(RST_PIN, LOW);
    return;
  }

  // 5) RFID 태그 검사
  if (!rfid.PICC_IsNewCardPresent() || !rfid.PICC_ReadCardSerial()) {
    getNavKeyNB(); // 키패드 논블로킹으로 타이머 리셋용
    return;
  }

  // 6) UID 문자열 생성
  String tag;
  for (byte i = 0; i < rfid.uid.size; i++) {
    if (rfid.uid.uidByte[i] < 0x10) tag += "0";
    tag += String(rfid.uid.uidByte[i], HEX);
    if (i < rfid.uid.size - 1) tag += " ";
  }
  tag.toUpperCase();

  String userFile = String(SD_USERS_FOLDER) + "/" + uidToName(tag) + ".txt";

  lcd.clear();
  if (SD.exists(userFile)) {
    // 등록된 사용자 (ID 읽기)
    File f = SD.open(userFile);
    String line, userID;
    while (f.available()) {
      line = f.readStringUntil('\n');
      if (line.startsWith("ID:")) {
        userID = line.substring(4);
        break;
      }
    }
    f.close();
    userID.trim();

    if (userID == adminID) {
      // 관리자 모드 진입
      lcd.clear();
      lcd.print("Enter PW:");
      String pw = inputID(3);
      pw.trim();

      if (pw != adminPW) {
        lcd.clear();
        lcd.print("Wrong PW");
        delay(1500);
        lcd.clear();
        lcd.print("Ready");
        activeStart = millis();
        rfid.PICC_HaltA();
        rfid.PCD_StopCrypto1();
        return;
      }
      adminMenu();
    } else {
      // 일반 사용자 출퇴근 로그 기록
      lcd.setCursor(0, 0);
      lcd.print("ID:" + userID);
      displayTime();
      logEvent(1, userID, tag);
      delay(2000);
      activeStart = millis();
    }
  } else {
    // 등록되지 않은 사용자 => 등록 절차
    while (true) {
      String num = inputID(1);
      if (num == "") {
        lcd.clear();
        lcd.print("Cancelled");
        delay(1500);
        break;
      }
      num.trim();
      if (num.length() == 8) {
        registerOnSD(tag, num);
        lcd.clear();
        lcd.print("Reged:" + num);
        delay(2000);
        activeStart = millis();
        break;
      }
      lcd.clear();
      lcd.print("Retry (8 digits)");
      delay(1500);
      lcd.clear();
      lcd.print("Enter ID:");
    }
  }

  // 7) 태그 종료 및 대기 상태 복귀
  lcd.clear();
  lcd.print("Ready");
  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();
}
