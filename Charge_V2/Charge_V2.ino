// ===== 스마트 충전 시스템 통합 코드 =====

// 외부 라이브러리 불러오기
#include <DHT.h>                // DHT 온습도 센서 제어용
#include <SoftwareSerial.h>     // 블루투스 통신용 소프트웨어 시리얼

// ===== 핀 설정 =====
#define DHTPIN        11    // 실내 DHT11 온습도 센서 데이터 핀
#define DHTTYPE       DHT11 // DHT 센서 타입 지정
#define OUTDOOR_PIN   A4    // 실외 DHT11 온습도 센서 데이터 핀
#define LDR_PIN       A0    // 조도 센서 (LDR) 입력 핀
#define VOLTAGE_PIN   A1    // 전압 센서 입력 핀
#define WATER_PIN     A2    // 수위 센서 입력 핀
#define OBSTACLE_PIN  12    // 인체 감지 센서 (PIR 또는 IR) 입력 핀
#define REED_PIN      9     // 창문 리드 스위치 입력 핀
#define BUZZER_PIN    10    // 부저 출력 핀
#define POT_PIN       A3    // 가변저항 입력 핀 (부저 톤 조절용)

// 블루투스 (HC-05) 통신 핀 정의
#define BT_RX         3     // HC-05 TX → Arduino RX
#define BT_TX         4     // HC-05 RX ← Arduino TX
SoftwareSerial btSerial(BT_RX, BT_TX);  // 소프트웨어 시리얼 객체 생성

// ===== 임계치 설정 =====
#define WATER_THRESHOLD 250    // 수위 감지 임계값
#define LDR_THRESHOLD   975    // 조도 감지 임계값 (야간 판단 기준)

// 7세그먼트 시프트레지스터 핀 설정
const int dataPin  = 7;  // 시프트레지스터 데이터 핀
const int clockPin = 5;  // 시프트레지스터 클럭 핀
const int latchPin = 6;  // 시프트레지스터 래치 핀

// DHT 객체 생성 (실내/실외 센서)
DHT indoorDHT(DHTPIN, DHTTYPE);
DHT outdoorDHT(OUTDOOR_PIN, DHTTYPE);

// 7세그 숫자 패턴 (공통 애노드 기준)
// 각 비트: dp,g,f,e,d,c,b,a 순서 (dp 제외)
byte digits[11] = {
  0b01110111, // 0: ABCDEF segments on
  0b00010100, // 1: BC segments on
  0b10110011, // 2: ABGED segments on
  0b10110110, // 3: ABCGD segments on
  0b11010100, // 4: FBCG segments on
  0b11100110, // 5: AFCDG segments on
  0b11100111, // 6: AFEDCG segments on
  0b00110100, // 7: ABC segments on
  0b11110111, // 8: all segments on
  0b11110110, // 9: ABCDFG segments on
  0b00000000  // off: 모든 세그먼트 off
};

// ===== 플래그 변수 =====
bool lastReedState = false;     // 이전 리드 스위치 상태 저장
bool tempAlerted   = false;     // 온도 알림 중복 방지 플래그
bool humLowAlerted = false;     // 습도 낮음 알림 중복 방지 플래그
bool humHighAlerted= false;     // 습도 높음 알림 중복 방패 플래그
bool waterAlerted  = false;     // 수위 알림 중복 방지 플래그
bool nightAlerted  = false;     // 야간 경고 중복 방지 플래그

// ===== 보조 함수 =====

// 부저와 7세그 표시 동시에 수행
void buzzToneWithDisplay(int f, int count, int digit) {
  for (int i = 0; i < count; i++) {
    showDigit(digit);         // 7세그에 숫자 표시
    tone(BUZZER_PIN, f);      // 부저 울림 시작
    delay(300);               // 0.3초 유지
    noTone(BUZZER_PIN);       // 부저 정지
    delay(100);               // 0.1초 대기
  }
}

// 부저만 울리는 함수 (count회 반복)
void buzzTone(int f, int c) {
  for (int i = 0; i < c; i++) {
    tone(BUZZER_PIN, f);      // 부저 울림 시작
    delay(300);
    noTone(BUZZER_PIN);       // 부저 정지
    delay(100);
  }
}

// 지정된 숫자 n을 7세그에 출력
void showDigit(int n) {
  digitalWrite(latchPin, LOW);                  // 래치 해제
  shiftOut(dataPin, clockPin, MSBFIRST, digits[n]);
  digitalWrite(latchPin, HIGH);                 // 래치 설정
}

// 7세그를 모두 끄는 함수
void showBlank() {
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin, clockPin, MSBFIRST, digits[10]);
  digitalWrite(latchPin, HIGH);
}

// 블루투스로 문자 전송
void sendTX(char c) {
  btSerial.write(c);
}

// ===== Arduino 초기화 =====
void setup() {
  Serial.begin(9600);       // 시리얼 모니터 통신 시작
  indoorDHT.begin();        // 실내 DHT 센서 초기화
  outdoorDHT.begin();       // 실외 DHT 센서 초기화
  btSerial.begin(9600);     // 블루투스 시리얼 시작

  // 센서 및 출력 핀 모드 설정
  pinMode(LDR_PIN, INPUT);
  pinMode(VOLTAGE_PIN, INPUT);
  pinMode(WATER_PIN, INPUT);
  pinMode(OBSTACLE_PIN, INPUT);
  pinMode(REED_PIN, INPUT_PULLUP);
  pinMode(POT_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(dataPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(latchPin, OUTPUT);
}

// ===== 메인 제어 루프 =====
void loop() {
  // 1) 센서값 읽기
  int ldrValue = analogRead(LDR_PIN);                 // 조도
  bool reedOpen = (digitalRead(REED_PIN) == HIGH);     // 창문 열림 감지
  float outdoorTemp = outdoorDHT.readTemperature();   // 실외 온도
  float indoorTemp  = indoorDHT.readTemperature();    // 실내 온도
  float hum         = indoorDHT.readHumidity();       // 실내 습도
  int waterVal      = analogRead(WATER_PIN);          // 수위 값
  bool personDetected = (digitalRead(OBSTACLE_PIN) == LOW); // 사람 감지
  float voltage     = analogRead(VOLTAGE_PIN) * (5.0/1023.0) * 5.0; // 배터리 전압

  // 2) 야간 경고: 어두울 때 창문 열림 감지
  if (ldrValue > LDR_THRESHOLD) {
    if (reedOpen && !nightAlerted) {
      sendTX('0');                // '0' 전송
      nightAlerted = true;        // 플래그 설정
    }
    if (!reedOpen) nightAlerted = false;
    showBlank();                  // 7세그 끔
    delay(1000);
    return;                       // 루프 종료
  }

  // 3) 부저 주파수 계산 (50ms마다 갱신)
  static int stablePot = 0;
  static unsigned long lastPot = 0;
  if (millis() - lastPot > 50) {
    stablePot = analogRead(POT_PIN);
    lastPot = millis();
  }
  int freq = map(stablePot, 0, 1023, 500, 3000);

  // 4) 사람 감지 시 7세그 + 부저 제어
  if (personDetected) {
    int vDigit = constrain(int((voltage - 3.0)/0.1222), 0, 9);
    showDigit(vDigit);             // 전압 레벨 숫자로 표시
    if (vDigit >= 7) buzzToneWithDisplay(freq, 3, vDigit);
    else             noTone(BUZZER_PIN);
  } else {
    showBlank();                   // 감지 없으면 꺼짐
  }

  // 5) 블루투스 메시지 초기화
  char btMsg = 'X';
  String action = "";

  // 사람 감지 시 추가 전송 없이 대기
  if (personDetected) {
    delay(1000);
    return;
  }

  // 6) 온도 기준 메시지 결정
  if (outdoorTemp <= 20.0) {
    if (indoorTemp >= 21.0 && !tempAlerted) {
      buzzTone(freq, 3);
      btMsg = '1'; action = "히터 끄세요!"; tempAlerted = true;
    }
    if (indoorTemp < 20.5) tempAlerted = false;
  } else {
    if (indoorTemp <= 25.0 && !tempAlerted) {
      btMsg = '2'; action = "에어컨 끄세요!"; tempAlerted = true;
    }
    if (indoorTemp > 25.5) tempAlerted = false;
  }

  // 7) 습도 기준 메시지 결정
  if (hum <= 30.0 && !humLowAlerted) {
    btMsg = '3'; action = "습도 낮음"; humLowAlerted = true;
  }
  if (hum > 31.0) humLowAlerted = false;
  if (hum >= 70.0 && !humHighAlerted) {
    btMsg = '4'; action = "습도 높음"; humHighAlerted = true;
  }
  if (hum < 69.0) humHighAlerted = false;

  // 8) 수위 + 창문 상태 메시지 결정
  if (reedOpen && waterVal >= WATER_THRESHOLD && !waterAlerted) {
    btMsg = '5'; action = "물 감지"; waterAlerted = true;
  }
  if (!reedOpen || waterVal < 230) {
    waterAlerted = false;
  }

  // 9) 블루투스 전송
  if (btMsg != 'X') {
    sendTX(btMsg);
  }

  delay(200);  // 0.2초 대기 후 다음 루프
}
