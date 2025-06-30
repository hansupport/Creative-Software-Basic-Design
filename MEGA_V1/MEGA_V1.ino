#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SPI.h>
#include <MFRC522.h>
#include <RtcDS1302.h>
#include <SD.h>

// —————— 핀 정의 ——————
// RFID (SPI / SS, RST)
#define SS_PIN      15    // RFID SDA/SS → D15
#define RST_PIN     14    // RFID RST    → D14

// 키패드 (4×4 매트릭스)
const uint8_t rowPins[4] = {5, 4, 3, 2};  // R1→D5, R2→D4, R3→D3, R4→D2
const uint8_t colPins[4] = {6, 7, 8, 9};  // C1→D6, C2→D7, C3→D8, C4→D9

// LCD (I2C)
#define LCD_ADDR    0x27
#define LCD_COLS    16
#define LCD_ROWS    2

// RTC (DS1302 ThreeWire)
#define RTC_RST_PIN 10    // RST → D10
#define RTC_CLK_PIN 11    // CLK → D11
#define RTC_DAT_PIN 12    // DAT → D12

// SD 카드 (SPI / CS)
#define SD_CS_PIN   53    // D53
const char *SD_USERS_FOLDER = "USERS";
const char *SD_COMMUTE_LOG_FOLDER = "LOG1";
const char *SD_DETCET_PEOPLE_LOG_FOLDER = "LOG2";
const char *SD_DETCET_FIRE_LOG_FOLDER = "LOG3";

// 관리자 사번
const String adminCode = "22011949";

// 장치 객체
LiquidCrystal_I2C lcd(LCD_ADDR, LCD_COLS, LCD_ROWS);
MFRC522       rfid(SS_PIN, RST_PIN);
ThreeWire     myWire(RTC_CLK_PIN, RTC_DAT_PIN, RTC_RST_PIN);
RtcDS1302<ThreeWire> Rtc(myWire);

// ↑↓ 커스텀 문자
byte arrowUp[8]   = {0b00100,0b01110,0b10101,0b00100,0b00100,0b00100,0b00100,0};
byte arrowDown[8] = {0b00100,0b00100,0b00100,0b00100,0b10101,0b01110,0b00100,0};

// 키패드 레이아웃
char keys[4][4] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};

// —————— 유틸리티 함수 ——————

/// 키패드에서 키 하나 반환
char getKeypad() {
  while (true) {
    for (uint8_t r = 0; r < 4; r++) {
      for (uint8_t i = 0; i < 4; i++)
        digitalWrite(rowPins[i], i == r ? LOW : HIGH);
      delayMicroseconds(50);
      for (uint8_t c = 0; c < 4; c++) {
        if (digitalRead(colPins[c]) == LOW) {
          char k = keys[r][c];
          while (digitalRead(colPins[c]) == LOW);
          delay(50);
          return k;
        }
      }
    }
  }
}

/// 8자리 숫자 입력 (# 완료, C 취소, * 백스페이스)
String inputID() {
  String num;
  lcd.clear();
  lcd.print("Enter ID:");
  lcd.setCursor(0, 1);
  while (true) {
    char k = getKeypad();
    if (k == '#') break;
    if (k == 'C') return "";
    if (k == '*' && num.length() > 0) num.remove(num.length() - 1);
    else if (k >= '0' && k <= '9' && num.length() < 8) num += k;
    lcd.setCursor(0, 1);
    lcd.print("        ");
    lcd.setCursor(0, 1);
    lcd.print(num);
  }
  return num;
}

/// SD_USERS_FOLDER 내부 파일 개수 반환
int countUsersOnSD() {
  File dir = SD.open(SD_USERS_FOLDER);
  if (!dir) return 0;
  int cnt = 0;
  while (true) {
    File f = dir.openNextFile();
    if (!f) break;
    cnt++;
    f.close();
  }
  dir.close();
  return cnt;
}

/// 인덱스(1~n)번째 파일의 사번을 반환 (없으면 empty)
String getIDAt(int index) {
  File dir = SD.open(SD_USERS_FOLDER);
  if (!dir) return "";
  int cnt = 0;
  while (true) {
    File f = dir.openNextFile();
    if (!f) break;
    cnt++;
    if (cnt == index) {
      // 파일 내용에서 “ID: xxxx” 부분 읽기
      String line;
      while (f.available()) {
        line = f.readStringUntil('\n');
        if (line.startsWith("ID:")) {
          String id = line.substring(4);  // "ID: " 이후 문자열
          id.trim();                      // 앞뒤 공백/개행 제거
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

/// 태그 UID → 파일 이름용 문자열 (공백 제거)
String uidToName(const String &uid) {
  String s = uid;
  s.replace(" ", "");
  return s;
}

// —————— 주요 기능 ——————

/// 시간 표시 (HH:MM:SS)
void displayTime() {
  RtcDateTime now = Rtc.GetDateTime();
  char buf[9];
  snprintf_P(buf, sizeof(buf), PSTR("%02u:%02u:%02u"),
            now.Hour(), now.Minute(), now.Second());
  lcd.setCursor(0, 1);
  lcd.print(buf);
}

/// SD에 신규 등록: UID 태그 파일 → 내용 “ID: <num>”
void registerOnSD(const String &uid, const String &num) {
  if (!SD.exists(SD_USERS_FOLDER)) SD.mkdir(SD_USERS_FOLDER);
  String filename = String(SD_USERS_FOLDER) + "/" + uidToName(uid) + ".txt";

  digitalWrite(SS_PIN, HIGH);      // RFID 비활성화
  digitalWrite(SD_CS_PIN, LOW);    // SD카드 활성화

  File f = SD.open(filename, FILE_WRITE);

  if (f) {
    f.print("UID: "); f.println(uid);
    f.print("ID:  "); f.println(num);
    f.close();
    lcd.clear(); lcd.print("Registered");
  } else {
    lcd.clear(); lcd.print("SD Write Err");
  }

  digitalWrite(SD_CS_PIN, HIGH);   // SD카드 비활성화

  delay(1000);
}

/// 날짜와 시간 남기는 로그
void logCommute(const String& id, const String& uid) {
  RtcDateTime now = Rtc.GetDateTime();

  // 날짜 기반 파일 이름
  char filename[25];
  sprintf(filename, "LOG1/%04d%02d%02d.TXT", now.Year(), now.Month(), now.Day());

  // 시간 문자열
  char timeStr[12];
  sprintf(timeStr, "[%02d:%02d:%02d]", now.Hour(), now.Minute(), now.Second());

  digitalWrite(SS_PIN, HIGH);      // RFID 비활성화
  digitalWrite(SD_CS_PIN, LOW);    // SD카드 활성화

  File f = SD.open(filename, FILE_WRITE);
  if (f) {
    f.print(timeStr);
    f.print(" ID: ");
    f.print(id);
    f.print(" UID: ");
    f.println(uid);
    f.close();
  } else {
    Serial.println("❌ 로그 쓰기 실패");
  }

  digitalWrite(SD_CS_PIN, HIGH);   // SD카드 비활성화
}


/// 관리자 메뉴: 1=조회, 2=삭제
void adminMenu() {
  int total = countUsersOnSD();
  lcd.clear(); lcd.print("1:View 2:Delete");
  char m;
  do { m = getKeypad(); } while (m!='1' && m!='2');
  if (m == '1') {
    int idx = 1;
    lcd.clear();
    lcd.createChar(0, arrowUp);
    lcd.createChar(1, arrowDown);
    while (true) {
      lcd.setCursor(0,0);
      lcd.print(String(idx) + "/" + String(total));
      lcd.setCursor(9,0);
      lcd.write(byte(0));
      lcd.print(" ");
      lcd.write(byte(1));
      lcd.setCursor(0,1);
      lcd.print("             ");
      lcd.setCursor(0,1);
      lcd.print(getIDAt(idx));
      char k = getKeypad();
      if (k=='A')      idx = idx % total + 1;
      else if (k=='B') idx = idx==1 ? total : idx-1;
      else if (k=='C') break;
      delay(200);
    }
  } else {
    // 삭제: 사번 입력 받아 파일 삭제
    String del = inputID();
    if (del.length()==8) {
      File dir = SD.open(SD_USERS_FOLDER);
      bool found=false;
      while (true) {
        File f = dir.openNextFile();
        if (!f) break;
        // 파일명 → UID 추출
        String fn = f.name(); f.close();
        String path = String(SD_USERS_FOLDER) + "/" + fn;
        File r = SD.open(path);
        String line;
        while (r.available()) {
          line = r.readStringUntil('\n');
          if (line.startsWith("ID:") && line.substring(4)==del) {
            r.close();
            SD.remove(path);
            lcd.clear(); lcd.print("Deleted");
            found=true;
            break;
          }
        }
        if (found) break;
        r.close();
      }
      dir.close();
      if (!found) {
        lcd.clear(); lcd.print("Not found");
      }
      delay(1000);
    }
  }
  lcd.clear(); lcd.print("Ready");
}

// —————— setup / loop ——————

void setup() {
  Serial.begin(9600);

  // LCD (I2C 기본 핀 D20=SDA, D21=SCL)
  Wire.begin();
  lcd.init();
  lcd.backlight();

  // RTC 초기화
  Rtc.Begin();
  RtcDateTime comp(__DATE__,__TIME__);
  if (!Rtc.IsDateTimeValid())     Rtc.SetDateTime(comp);
  if (Rtc.GetIsWriteProtected())  Rtc.SetIsWriteProtected(false);
  if (!Rtc.GetIsRunning())        Rtc.SetIsRunning(true);

  // RFID 초기화
  SPI.begin();
  rfid.PCD_Init();
  rfid.PCD_AntennaOn();

  // 키패드 설정
  for (int i=0; i<4; i++){
    pinMode(rowPins[i], OUTPUT);
    digitalWrite(rowPins[i], HIGH);
    pinMode(colPins[i], INPUT_PULLUP);
  }
  
  // 🔧 SD 카드 CS 핀 안정화 처리
  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);  // 초기 비활성화

  // SD 초기화
  if (!SD.begin(SD_CS_PIN)){
    lcd.clear(); lcd.print("SD init FAIL");
    while(true);
  }

  lcd.clear(); lcd.print("System Ready");
  delay(1000);
  lcd.clear(); lcd.print("Ready");
}

void loop() {
  // 카드 감지
  if (!rfid.PICC_IsNewCardPresent() || !rfid.PICC_ReadCardSerial()) return;

  // UID 문자열 생성 (tag에 저장됨)
  String tag;
  for (byte i=0; i<rfid.uid.size; i++) {
    if (rfid.uid.uidByte[i] < 0x10) tag += "0";
    tag += String(rfid.uid.uidByte[i], HEX);
    if (i < rfid.uid.size - 1) tag += " ";
  }
  tag.toUpperCase();

  // 해당 UID 파일 경로 확인
  String filename = String(SD_USERS_FOLDER) + "/" + uidToName(tag) + ".txt";

  lcd.clear();
  if (SD.exists(filename)) {
    // 파일에서 등록된 사번 읽기
    File f = SD.open(filename);
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

    // ✅ 관리자 또는 일반 사용자 구분
    if (userID == adminCode) {
      adminMenu();
    } else {
      lcd.setCursor(0, 0);
      lcd.print("ID:" + userID);
      displayTime();

      // ✅ 여기가 새로 추가된 부분!
      logCommute(userID, tag);  // 출퇴근 로그 기록

      delay(2000);
    }
  } else {
    // 신규 등록
    while (true) {
      String num = inputID();
      if (num == "") {
        lcd.clear(); lcd.print("Cancelled");
        delay(1500);
        break;
      }
      if (num.length() == 8) {
        registerOnSD(tag, num);
        lcd.clear(); lcd.print("Reged:" + num);
        delay(2000);
        break;
      }
      lcd.clear(); lcd.print("Retry (8 digits)");
      delay(1500);
      lcd.clear(); lcd.print("Enter ID:");
    }
  }

  // 마무리
  lcd.clear();
  lcd.print("Ready");
  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();
}

