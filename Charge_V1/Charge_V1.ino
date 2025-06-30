// 스마트 환기 시스템 통합 코드 (Bluetooth 소프트웨어 시리얼 핀 수정)
#include <DHT.h>
#include <SoftwareSerial.h>  // SoftwareSerial 헤더 추가

// ===== 핀 설정 =====
#define DHTPIN        11    // 실내 온습도 DHT11
#define DHTTYPE       DHT11
#define OUTDOOR_PIN   A4    // 실외 온습도 DHT11
#define LDR_PIN       A0    // 조도 센서
#define VOLTAGE_PIN   A1    // 전압 센서
#define WATER_PIN     A2    // 수위 센서
#define OBSTACLE_PIN  12    // 사람 인식 센서
#define REED_PIN      9     // 창문 리드 스위치
#define BUZZER_PIN    10    // 부저
#define POT_PIN       A3    // 가변저항 (부저 톤)

// Bluetooth SoftwareSerial 핀
#define BT_RX         3     // HC-05 TX → Arduino RX
#define BT_TX         4     // HC-05 RX ← Arduino TX
SoftwareSerial btSerial(BT_RX, BT_TX);

// ===== 임계치 =====
#define WATER_THRESHOLD 250
#define LDR_THRESHOLD   975

// 7세그먼트 시프트레지스터 핀
const int dataPin  = 7;
const int clockPin = 5;
const int latchPin = 6;

DHT indoorDHT(DHTPIN, DHTTYPE);
DHT outdoorDHT(OUTDOOR_PIN, DHTTYPE);

byte digits[11] = {
  0b01110111, // 0
  0b00010100, // 1
  0b10110011, // 2
  0b10110110, // 3
  0b11010100, // 4
  0b11100110, // 5
  0b11100111, // 6
  0b00110100, // 7
  0b11110111, // 8
  0b11110110, // 9
  0b00000000  // off
};

// 플래그 변수
bool lastReedState = false;
bool tempAlerted   = false;
bool humLowAlerted = false;
bool humHighAlerted= false;
bool waterAlerted  = false;
bool nightAlerted  = false;

void setup() {
  Serial.begin(9600);
  indoorDHT.begin();
  outdoorDHT.begin();
  btSerial.begin(9600);                   // Bluetooth 통신 시작

  pinMode(LDR_PIN, INPUT);
  pinMode(VOLTAGE_PIN, INPUT);
  pinMode(WATER_PIN, INPUT);
  pinMode(OBSTACLE_PIN, INPUT);
  pinMode(REED_PIN, INPUT);
  pinMode(POT_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(dataPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(latchPin, OUTPUT);
}

void loop() {
  // 1. 센서 읽기
  int ldrValue        = analogRead(LDR_PIN);
  bool reedOpen       = (digitalRead(REED_PIN) == LOW);
  float outdoorTemp   = outdoorDHT.readTemperature();
  float indoorTemp    = indoorDHT.readTemperature();
  float hum           = indoorDHT.readHumidity();
  int waterVal        = analogRead(WATER_PIN);
  bool personDetected = (digitalRead(OBSTACLE_PIN) == LOW);
  float voltage       = analogRead(VOLTAGE_PIN) * (5.0/1023.0) * 5.0;

  // 2. 야간 모드: 조도 낮으면 창문 열림 시 경고 '0'
  if (ldrValue > LDR_THRESHOLD) {
    if (reedOpen && !nightAlerted) {
      sendTX('0');
      Serial.println("야간 경고: 창문 열림 -> TX=0");
      nightAlerted = true;
    }
    if (!reedOpen) nightAlerted = false;
    showBlank();
    delay(1000);
    return;
  }

  // 3. 부저 톤 계산 (가변저항 안정화)
  static int stablePot = 0;
  static unsigned long lastPot = 0;
  if (millis() - lastPot > 50) {
    stablePot = analogRead(POT_PIN);
    lastPot = millis();
  }
  int freq = map(stablePot, 0, 1023, 500, 3000);

  // 4. 7세그 표시 (사람 감지 시)
  if (personDetected) {
    int vDigit = constrain(int((voltage - 3.0) / 0.13), 0, 9);
    // 항상 7세그에 숫자 표시
    showDigit(vDigit);

    // vDigit이 7 이상일 때만 부저 울림
    if (vDigit >= 7) {
      buzzToneWithDisplay(freq, 3, vDigit);
    } else {
      // 7 미만일 땐 부저 끔
      noTone(BUZZER_PIN);
    }
  } else {
    showBlank();
  }

  // 5. 상태 출력
  Serial.print("외부 온도: "); Serial.println(outdoorTemp);
  Serial.print("창문: ");      Serial.println(reedOpen ? "닫힘" : "열림");
  Serial.print("조도: ");      Serial.println(ldrValue);
  Serial.print("실내 온도: "); Serial.println(indoorTemp);
  Serial.print("습도: ");      Serial.println(hum);
  Serial.print("수위: ");      Serial.println(waterVal);
  Serial.print("전압: ");      Serial.println(voltage);

  char btMsg = 'X';  String action = "";

  // 6. 사람 모드
  if (personDetected) {
    Serial.println("사람모드, TX 전송없음");
    delay(1000);
    return;
  }

  // 7. 환기 조건 (실내 온도, 습도, 수위)
  // 7.1 실내 온도 경고: 겨울(outdoorTemp<=20) 21℃ 이상->'1', 여름->25℃ 이하->'2'
  if (outdoorTemp <= 20.0) {
    bool heatOff = (indoorTemp >= 21.0);
    if (heatOff && !tempAlerted) {
      buzzTone(freq, 3);
      btMsg = '1'; action = "히터 끄세요!"; tempAlerted = true;
    }
    if (indoorTemp < 20.5) tempAlerted = false;
  } else {
    bool acOff = (indoorTemp <= 25.0);
    if (acOff && !tempAlerted) {
      btMsg = '2'; action = "에어컨 끄세요!"; tempAlerted = true;
    }
    if (indoorTemp > 25.5) tempAlerted = false;
  }

  // 7.2 습도 경고: 낮음<=30->'3', 높음>=70->'4'
  if (hum <= 30.0 && !humLowAlerted) {
    btMsg = '3'; action = "습도 낮음"; humLowAlerted = true;
  }
  if (hum > 31.0) humLowAlerted = false;
  if (hum >= 70.0 && !humHighAlerted) {
    btMsg = '4'; action = "습도 높음"; humHighAlerted = true;
  }
  if (hum < 69.0) humHighAlerted = false;

  // 7.3 수위+창문열림 경고->'5'
  bool waterBad = (waterVal >= WATER_THRESHOLD && !reedOpen);
  if (waterBad && !waterAlerted) {
    btMsg = '5'; action = "물 감지"; waterAlerted = true;
  }
  if (waterVal < 230) waterAlerted = false;

  // 8. Bluetooth 전송
  if (btMsg != 'X') {
    Serial.print("TX 전송: "); Serial.println(btMsg);
    sendTX(btMsg);
  }
  Serial.print("결과: "); Serial.println(action);
  Serial.println("------------------");

  delay(1000);
}

// ===== 보조 함수 =====
void buzzToneWithDisplay(int f, int count, int digit) {
  for (int i = 0; i < count; i++) {
    // 1) 7세그에 숫자 출력
    showDigit(digit);

    // 2) 부저 켜기
    tone(BUZZER_PIN, f);
    delay(300);

    // 3) 부저 끄기
    noTone(BUZZER_PIN);
    delay(100);
  }
}
void buzzTone(int f, int c) {
  for (int i = 0; i < c; i++) {
    tone(BUZZER_PIN, f);
    delay(300);
    noTone(BUZZER_PIN);
    delay(100);
  }
}
void showDigit(int n){ digitalWrite(latchPin,LOW); shiftOut(dataPin,clockPin,MSBFIRST,digits[n]); digitalWrite(latchPin,HIGH); }
void showBlank(){ digitalWrite(latchPin,LOW); shiftOut(dataPin,clockPin,MSBFIRST,digits[10]); digitalWrite(latchPin,HIGH); }
void sendTX(char c){ btSerial.write(c); }
