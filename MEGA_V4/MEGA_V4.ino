// —————— 라이브러리 불러오기 ——————
#include <Wire.h>                // I2C 통신용
#include <LiquidCrystal_I2C.h>   // I2C LCD 제어용
#include <SPI.h>                 // SPI 통신용 (RFID, SD)
#include <MFRC522.h>             // RFID RC522 라이브러리
#include <RtcDS1302.h>           // RTC DS1302 라이브러리
#include <SD.h>                  // SD 카드 라이브러리

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
#define RST_PIN     11  // RST
#define SS_PIN      10  // SDA
// SCK, MOSI, MISO는 SPI 핀 (52, 51, 50)

// SD카드 CS 핀 정의
#define SD_CS_PIN   53  // SPI CS
// SPI 핀 공유: SCK 52, MOSI 51, MISO 50

// 조이스틱 핀 정의
#define VRX A0
#define VRY A1
#define SW  -1   // 미사용

// IR 장애물 센서 핀 정의
#define IR_SENSOR_PIN 12 

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

// 시프트 레지스터 핀
#define DATA_PIN 36   // DS
#define LATCH_PIN 35  // STCP
#define CLOCK_PIN 34  // SHCP

// 부저 핀
#define BUZZER_PIN 37

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
// [추가] 추후 LOG2, LOG3도 여기에 확장 가능

// 관리자 사번 지정
const String adminCode = "22011949";

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

// 룩업 테이블: 인덱스 = (lastState<<2)|newState
const int8_t QUAD_TABLE[16] = {
   0, -1, +1,  0,
  +1,  0,  0, -1,
  -1,  0,  0, +1,
   0, +1, -1,  0
};

// —————— 객체 선언 ——————
LiquidCrystal_I2C lcd(LCD_ADDR, LCD_COLS, LCD_ROWS);       // LCD 객체
MFRC522 rfid(SS_PIN, RST_PIN);                             // RFID 객체
ThreeWire myWire(RTC_DAT_PIN, RTC_CLK_PIN, RTC_RST_PIN);   // RTC 통신 설정
RtcDS1302<ThreeWire> Rtc(myWire);                          // RTC 객체

// 키패드 키 배열 정의
char keys[4][4] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};

// —————— 유틸리티 함수 ——————

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
String inputID() {
  String num;
  lcd.clear(); lcd.print("Enter ID:");
  lcd.setCursor(0,1);
  while (true) {
    char k = getKeypad();
    if (k=='#') break;                       // 입력 완료
    if (k=='C') return "";                   // 입력 취소
    if (k=='*' && num.length()>0) num.remove(num.length()-1);  // 한 자리 삭제
    else if (k>='0' && k<='9' && num.length()<8) num += k;     // 숫자 추가
    lcd.setCursor(0,1);
    lcd.print("        ");      // 줄 초기화
    lcd.setCursor(0,1);
    lcd.print(num);             // 현재 입력된 번호 표시
  }
  return num;
}

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
          String id = line.substring(4);  // "ID: " 다음 문자열 추출
          id.trim();                      // 공백 제거
          f.close(); dir.close();
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

// LOG1 폴더 내 날짜별 로그 파일 수를 반환하는 함수
int countCommuteLogs() {
  File dir = SD.open(SD_COMMUTE_LOG_FOLDER);
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

// 지정된 index번째 날짜 로그 파일명을 반환 (예: 20240525.TXT)
String getCommuteFileAt(int idx) {
  File dir = SD.open(SD_COMMUTE_LOG_FOLDER);
  int cnt = 0;
  while (true) {
    File f = dir.openNextFile();
    if (!f) break;
    cnt++;
    if (cnt == idx) {
      String n = f.name();
      f.close(); dir.close();
      return n;
    }
    f.close();
  }
  dir.close();
  return "";
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

// 출퇴근 로그를 날짜별 파일에 시간, ID, UID를 함께 기록
void logCommute(const String &id, const String &uid) {
  RtcDateTime now = Rtc.GetDateTime();

  // 날짜 기반 파일명 생성: LOG1/YYYYMMDD.TXT
  char fn[25];
  sprintf(fn, "LOG1/%04d%02d%02d.TXT",
          now.Year(), now.Month(), now.Day());

  // 현재 시간 형식: [HH:MM:SS]
  char ts[12];
  sprintf(ts, "[%02d:%02d:%02d]",
          now.Hour(), now.Minute(), now.Second());

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
String getCommuteFileAt_Desc(int idx) {
  char prevFile[13] = "";  // 이전에 찾은 파일명을 저장 (초기에는 빈 문자열)

  // 원하는 인덱스만큼 반복 (최신부터 차례로 찾아감)
  for (int i = 0; i < idx; i++) {
    File dir = SD.open(SD_COMMUTE_LOG_FOLDER);  // 로그 폴더 열기
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
  File f = SD.open("SET/config.txt");
  if (f) {
    brightness = f.parseInt();
    volumeLevel = f.parseInt();
    f.close();
  }
}

// 설정 저장하는 함수
void saveSettings() {
  // 기존 파일 삭제 → 새로 생성하며 쓰기
  SD.remove("SET/config.txt");
  File f = SD.open("SET/config.txt", FILE_WRITE);
  if (f) {
    f.print(brightness);
    f.print(" ");
    f.print(volumeLevel);
    f.close();
  }
}

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
          noTone(BUZZER_PIN);
          nbBuzzOnState  = false;
          nbToneStep     = 1 - nbToneStep;
          nbBuzzInterval = BUZZER_OFF_TIME;
        } else {
          int baseFreq = map(volumeLevel, 1, 30, 100, 2000);
          tone(BUZZER_PIN, baseFreq + (nbToneStep ? 250 : 0));
          nbBuzzOnState  = true;
          nbBuzzInterval = BUZZER_ON_TIME;
        }
      }
    } else {
      // volumeLevel == 0 일 때 부저 완전 정지
      noTone(BUZZER_PIN);
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
  int totalF = countCommuteLogs();  // 날짜 로그 파일 개수 확인
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
      String fn = getCommuteFileAt_Desc(fIdx);  // 최신순 파일명
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
          String fn = getCommuteFileAt_Desc(fIdx);
          SD.remove(String(SD_COMMUTE_LOG_FOLDER) + "/" + fn);
          totalF = countCommuteLogs();
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
          File dir = SD.open(SD_COMMUTE_LOG_FOLDER);
          while (true) {
            File f = dir.openNextFile();
            if (!f) break;
            SD.remove(String(SD_COMMUTE_LOG_FOLDER) + "/" + String(f.name()));
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
        String fn = getCommuteFileAt_Desc(fIdx);
        String path = String(SD_COMMUTE_LOG_FOLDER) + "/" + fn;
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
        totalF = countCommuteLogs();
        if (!totalF) return;
        fIdx = constrain(fIdx, 1, totalF);
        prevFIdx = -1;
      }
    }
    delay(20);
  }
}

// 관리자 메뉴 5번 – 밝기 및 볼륨 조절 기능 제공
void menuSetting() {
  int sel = 1, prevSel = -1;
  lcd.clear();
  lcd.setCursor(0,0);  lcd.print("...");
  loadSettings();
  while (true) {
    // 1) 화면 갱신
    if (sel != prevSel) {
      lcd.clear();
      lcd.setCursor(0,0);  lcd.print("1/1");
      lcd.setCursor(0,1);  lcd.print("1:Bright 2:Vol");
      prevSel = sel;
    }

    // 2) 조이스틱 스크롤
    bool changed = false;
    sel = getNavScroll(sel, 2, changed);

    // 3) 조이스틱이 움직이지 않을 때만 키패드 처리
    if (!changed) {
      char k = getNavKeyNB();
      if (k == 'C') {
        saveSettings();  // 저장 후 나가기
        break;
      }
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
            prevSel = -1;
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
            noTone(BUZZER_PIN);
            prevSel = -1;
            break;
          }
          delay(20);
        }
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
      else if (page == 2) lcd.print("3:People 4:Fire");
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
        // menuPeople();
        prevPage = -1;
      }
      else if (k == '4') {
        // menuFire();
        prevPage = -1;
      }
      else if (k == '5') {
        menuSetting();
        prevPage = -1;
      }
    }
  }
}

// —————— setup & loop ——————

// 시스템 시작 시 초기 설정을 수행하는 함수
void setup() {
  Serial.begin(9600);        // 시리얼 디버깅용
  loadSettings();
  Wire.begin();              // I2C 시작
  lcd.init(); lcd.backlight(); // LCD 초기화 및 백라이트 켜기

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

  // SD 카드 설정
  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);      // 기본적으로 비활성화
  if (!SD.begin(SD_CS_PIN)) {
    lcd.clear(); lcd.print("SD init FAIL");
    while (true);                     // SD 초기화 실패 시 멈춤
  }
  
  // IR 센서 설정
  pinMode(IR_SENSOR_PIN, INPUT);

  // 택트 스위치 설정
  pinMode(WAKE_BUTTON_PIN, INPUT_PULLUP);  // 내부 풀업 활성화

  lcd.clear(); lcd.print("System Ready");
  delay(1000);
  lcd.clear(); lcd.print("Ready");    // 시스템 대기 상태 표시

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

  // 부저 설정
  pinMode(BUZZER_PIN, OUTPUT);

  // 활동 상태로 두고 타이머 초기화
  systemActive = true;
  activeStart = millis();
}

// 시스템 메인 루프 – RFID 태그 감지 및 행동 수행
void loop() {
  unsigned long now = millis();
  // IR 센서가 장애물을 감지하거나 택트 스위치가 눌릴 때 시스템을 활성화함
  if (digitalRead(IR_SENSOR_PIN) == LOW || digitalRead(WAKE_BUTTON_PIN) == LOW) { 
    if (!systemActive) {
      systemActive = true;
      activeStart = now;
      // RFID 모듈 ON
      digitalWrite(RST_PIN, HIGH);
      delay(50);
      rfid.PCD_Init();
      rfid.PCD_AntennaOn();
      // LCD ON
      lcd.backlight(); lcd.clear(); lcd.print("Ready");
    }
  }

  // 시스템이 비활성화 상태이면 RFID, LCD, 조이스틱은 작동하지 않음
  if (!systemActive) return;

  // 시스템 활성화 유지 시간 초과 시 자동 비활성화
  if (now - activeStart >= activeTimeout) {
    systemActive = false;
    // LCD OFF
    lcd.clear();
    lcd.noBacklight();

    // RFID 모듈 OFF
    rfid.PCD_AntennaOff();
    digitalWrite(RST_PIN, LOW);
    return;
  }

  // RFID 태그가 감지되지 않으면 나머지 루프 무시
  if (!rfid.PICC_IsNewCardPresent() || !rfid.PICC_ReadCardSerial()) {
    getNavKeyNB();  // 키패드 논블로킹 감지: 입력이 있으면 타이머 리셋됨
    return;
  }

  // UID 문자열 생성
  String tag;
  for (byte i = 0; i < rfid.uid.size; i++) {
    if (rfid.uid.uidByte[i] < 0x10) tag += "0";
    tag += String(rfid.uid.uidByte[i], HEX);
    if (i < rfid.uid.size - 1) tag += " ";
  }
  tag.toUpperCase();  // 대문자로 변환

  // UID 기반 사용자 파일 경로
  String userFile = String(SD_USERS_FOLDER) + "/" + uidToName(tag) + ".txt";

  lcd.clear();
  if (SD.exists(userFile)) {
    // 등록된 사용자 → 사번 읽기
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

    if (userID == adminCode) {
      adminMenu();                        // 관리자 메뉴 진입
      activeStart = millis();             // 관리자 메뉴 종료 후 타이머 리셋
    } else {
      lcd.setCursor(0, 0); lcd.print("ID:" + userID);  // 사용자 사번 출력
      displayTime();                                  // 시간 표시
      logCommute(userID, tag);                        // 출퇴근 로그 기록
      delay(2000);
      activeStart = millis();                         // 사용자 활동 후 타이머 리셋
    }
  } else {
    // 등록되지 않은 사용자 → 등록 절차
    while (true) {
      String num = inputID();      // 사번 입력 받기
      if (num == "") {
        lcd.clear(); lcd.print("Cancelled");
        delay(1500);
        break;
      }
      if (num.length() == 8) {
        registerOnSD(tag, num);    // 사용자 등록
        lcd.clear(); lcd.print("Reged:" + num);
        delay(2000);
        activeStart = millis();    // 등록 후 타이머 리셋
        break;
      }
      lcd.clear(); lcd.print("Retry (8 digits)");
      delay(1500);
      lcd.clear(); lcd.print("Enter ID:");
    }
  }

  // 태그 종료 처리 및 대기 상태 복귀
  lcd.clear(); lcd.print("Ready");
  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();
}