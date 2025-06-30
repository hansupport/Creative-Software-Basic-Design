#include <Servo.h>
#include <SoftwareSerial.h>

// ───── 소프트웨어 시리얼 정의 ─────
// A4 → RX, A5 → TX
SoftwareSerial swSer(A4, A5);

// ───── 서보 관련 정의 ─────
Servo myServo;
const int servoPin = A3;
int angle = 90;       // 초기값을 90도로 설정
int step = 15;        // 각도 변화량
unsigned long lastServoMillis = 0;
const unsigned long SERVO_INTERVAL = 500; // 0.5초 간격

// ───── 라인트레이싱용 핀 정의 ─────
// 왼쪽 앞 모터 (L298N_LEFT_FRONT)
//   IN3 → LF_IN1 (D12), IN4 → LF_IN2 (D13), ENB → LF_ENA (D11)
const int LF_IN1 = 12;
const int LF_IN2 = 13;
const int LF_ENA = 11;

// 왼쪽 뒤 모터 (L298N_LEFT_REAR)
//   IN1 → LR_IN1 (D8), IN2 → LR_IN2 (D9), ENA → LR_ENA (D10)
const int LR_IN1 = 8;
const int LR_IN2 = 9;
const int LR_ENA = 3;

// 오른쪽 앞 모터 (L298N_RIGHT_FRONT)
//   IN1 → RF_IN1 (D7), IN2 → RF_IN2 (D3), ENA → RF_ENA (D5)
const int RF_IN1 = 7;
const int RF_IN2 = 10;
const int RF_ENA = 5;

// 오른쪽 뒤 모터 (L298N_RIGHT_REAR)
//   IN3 → RR_IN1 (D4), IN4 → RR_IN2 (D2), ENB → RR_ENA (D6)
const int RR_IN1 = 4;
const int RR_IN2 = 2;
const int RR_ENA = 6;

// ───── 트래킹 센서 핀 정의 ─────
const int TRACKING_L = A0;
const int TRACKING_C = A1;
const int TRACKING_R = A2;

// ───── 임계값 정의 ─────
const int THRESHOLD = 500;  // >THRESHOLD ⇒ 검정, ≤THRESHOLD ⇒ 흰색

// ───── 전역 변수 ─────
// 라인트레이싱 모드 켜기/끄기 (기본값: 끄기)
bool lineTracingEnabled = false;
unsigned long traceStartTime = 0;
const unsigned long TRACE_DURATION = 10000; // 10초 간 주행

int movementMode = 0;         // 0: 정지, 1: 전진, 2: 좌회전, 3: 우회전, 4: 후진
int prevMovementMode = -1;    // 이전 모드 (초기값: -1)
int forwardSpeedControl = 60; // 전진 속도 비율 (0~100%)
int turnSpeedControl    = 80; // 회전 시 전진 바퀴 속도 비율 (0~100%)
int reverseSpeedControl = 50; // 회전 시 후진 바퀴 속도 비율 (0~100%)
int modeDelay = 10;           // 모드 전환 시 딜레이(ms)

const int motorSpeed = 240;   // 모터 기준 PWM (0~255)

bool isSearching = false;
unsigned long whiteStartTime = 0;

// ───── 디버그 출력 인터벌 ─────
const unsigned long PRINT_INTERVAL = 20;
unsigned long lastPrintTime = 0;

void setup() {
  // 소프트웨어 시리얼 초기화 (보드레이트 9600)
  swSer.begin(9600);
  // 하드웨어 시리얼도 열어두면, PC 연결 시 디버그 확인 가능
  Serial.begin(9600);

  // 서보 초기화
  myServo.attach(servoPin);
  myServo.write(angle); // 초기 각도 90도로 설정

  // 모터 제어 핀 모두 OUTPUT
  pinMode(LF_IN1, OUTPUT);
  pinMode(LF_IN2, OUTPUT);
  pinMode(LR_IN1, OUTPUT);
  pinMode(LR_IN2, OUTPUT);
  pinMode(RF_IN1, OUTPUT);
  pinMode(RF_IN2, OUTPUT);
  pinMode(RR_IN1, OUTPUT);
  pinMode(RR_IN2, OUTPUT);

  pinMode(LF_ENA, OUTPUT);
  pinMode(LR_ENA, OUTPUT);
  pinMode(RF_ENA, OUTPUT);
  pinMode(RR_ENA, OUTPUT);

  // 트래킹 센서 입력
  pinMode(TRACKING_L, INPUT);
  pinMode(TRACKING_C, INPUT);
  pinMode(TRACKING_R, INPUT);
}

void loop() {
  // A. 모드와 상관없이 swSer로 들어오는 문자를 먼저 읽기
  if (swSer.available()) {
    char cmd = swSer.read();
    if (cmd == '0') {
      // 시스템 정지
      lineTracingEnabled = false;
      stopMotors();
      angle = 90;
      myServo.write(angle);
      Serial.println("System stopped via '0'");
    }
    else if (cmd == '1') {
      // 라인트레이싱 모드 ON
      lineTracingEnabled = true;
      traceStartTime = millis();
      prevMovementMode = -1;
      angle = 90;
      myServo.write(angle);
      Serial.println("Line Tracing Mode ON");
    }
  }

  if (lineTracingEnabled) {
    // (1) 30초가 지나면 모드 끄기 + 서보 초기위치(90도)로 복귀
    if (millis() - traceStartTime >= TRACE_DURATION) {
      lineTracingEnabled = false;
      stopMotors();
      movementMode = 0;
      prevMovementMode = -1;

      // 서보 90도로 복귀
      angle = 90;
      myServo.write(angle);
      Serial.println("Line Tracing Mode OFF (30s 경과)");
    }
    else {
      // (2) 서보 모터 1초 간격으로 회전
      if (millis() - lastServoMillis >= SERVO_INTERVAL) {
        lastServoMillis = millis();

        myServo.write(angle);
        Serial.print("서보 각도: ");
        Serial.println(angle);

        angle += step;
        if (angle >= 180 || angle <= 0) {
          step = -step;
        }
      }

      // (3) 트래킹 센서 값 읽어서 흑/백 판별
      int leftValue   = analogRead(TRACKING_L);
      int centerValue = analogRead(TRACKING_C);
      int rightValue  = analogRead(TRACKING_R);

      bool Lb = (leftValue   > THRESHOLD);
      bool Cb = (centerValue > THRESHOLD);
      bool Rb = (rightValue  > THRESHOLD);

      // (4) 흰색(WWW) 탐색 로직
      if (!Lb && !Cb && !Rb) {
        if (!isSearching) {
          isSearching = true;
          whiteStartTime = millis();
          movementMode = 1;
        } else {
          if (millis() - whiteStartTime < 1000) {
            movementMode = 1;
          } else {
            movementMode = 0;
          }
        }
      }
      else {
        isSearching = false;
        // (5) 센서 조합에 따른 모드 결정
        if (!Lb &&  Cb && !Rb) {
          movementMode = 1;       // WBW: 전진
        }
        else if (!Lb &&  Cb &&  Rb) {
          movementMode = 3;       // WBB: 우회전
        }
        else if ( Lb &&  Cb && !Rb) {
          movementMode = 2;       // BBW: 좌회전
        }
        else if (!Lb && !Cb &&  Rb) {
          movementMode = 3;       // WWB: 우회전
        }
        else if ( Lb && !Cb && !Rb) {
          movementMode = 2;       // BWW: 좌회전
        }
        else if ( Lb &&  Cb &&  Rb) {
          movementMode = 1;       // BBB: 전진
        }
        else {
          movementMode = 0;       // 기타: 정지
        }
      }

      // (6) 모드 전환이 있으면 멈추고 짧은 딜레이
      if (movementMode != prevMovementMode) {
        stopMotors();
        delay(modeDelay);
      }

      // (7) 결정된 모드 실행
      switch (movementMode) {
        case 0:
          stopMotors();
          break;
        case 1:
          moveForward();
          break;
        case 2:
          turnLeft();
          break;
        case 3:
          turnRight();
          break;
        case 4:
          moveBackward();
          break;
      }
      prevMovementMode = movementMode;

      delay(10);
    }
  }
  else {
    // 라인트레이싱 모드 꺼짐 상태: 모터 정지, 서보 90도 고정
    stopMotors();
    myServo.write(90);
  }
}

// ───── 모터 제어 함수들 ─────
void moveForward() {
  int s = motorSpeed * forwardSpeedControl / 100;
  digitalWrite(LF_IN1, LOW);  digitalWrite(LF_IN2, HIGH); analogWrite(LF_ENA, s);
  digitalWrite(LR_IN1, HIGH); digitalWrite(LR_IN2, LOW);  analogWrite(LR_ENA, s);
  digitalWrite(RF_IN1, LOW);  digitalWrite(RF_IN2, HIGH); analogWrite(RF_ENA, s);
  digitalWrite(RR_IN1, HIGH); digitalWrite(RR_IN2, LOW);  analogWrite(RR_ENA, s);
}

void moveBackward() {
  int s = motorSpeed * forwardSpeedControl / 100;
  digitalWrite(LF_IN1, HIGH); digitalWrite(LF_IN2, LOW);  analogWrite(LF_ENA, s);
  digitalWrite(LR_IN1, LOW);  digitalWrite(LR_IN2, HIGH); analogWrite(LR_ENA, s);
  digitalWrite(RF_IN1, HIGH); digitalWrite(RF_IN2, LOW);  analogWrite(RF_ENA, s);
  digitalWrite(RR_IN1, LOW);  digitalWrite(RR_IN2, HIGH); analogWrite(RR_ENA, s);
}

void turnLeft() {
  int fwd = motorSpeed * turnSpeedControl / 100;
  int rev = motorSpeed * reverseSpeedControl / 100;
  // 왼쪽 바퀴 후진
  digitalWrite(LF_IN1, HIGH); digitalWrite(LF_IN2, LOW);  analogWrite(LF_ENA, rev);
  digitalWrite(LR_IN1, LOW);  digitalWrite(LR_IN2, HIGH); analogWrite(LR_ENA, rev);
  // 오른쪽 바퀴 전진
  digitalWrite(RF_IN1, LOW);  digitalWrite(RF_IN2, HIGH); analogWrite(RF_ENA, fwd);
  digitalWrite(RR_IN1, HIGH); digitalWrite(RR_IN2, LOW);  analogWrite(RR_ENA, fwd);
}

void turnRight() {
  int fwd = motorSpeed * turnSpeedControl / 100;
  int rev = motorSpeed * reverseSpeedControl / 100;
  // 왼쪽 바퀴 전진
  digitalWrite(LF_IN1, LOW);  digitalWrite(LF_IN2, HIGH); analogWrite(LF_ENA, fwd);
  digitalWrite(LR_IN1, HIGH); digitalWrite(LR_IN2, LOW);  analogWrite(LR_ENA, fwd);
  // 오른쪽 바퀴 후진
  digitalWrite(RF_IN1, HIGH); digitalWrite(RF_IN2, LOW);  analogWrite(RF_ENA, rev);
  digitalWrite(RR_IN1, LOW);  digitalWrite(RR_IN2, HIGH); analogWrite(RR_ENA, rev);
}

void stopMotors() {
  digitalWrite(LF_IN1, LOW); digitalWrite(LF_IN2, LOW); analogWrite(LF_ENA, 0);
  digitalWrite(LR_IN1, LOW); digitalWrite(LR_IN2, LOW); analogWrite(LR_ENA, 0);
  digitalWrite(RF_IN1, LOW); digitalWrite(RF_IN2, LOW); analogWrite(RF_ENA, 0);
  digitalWrite(RR_IN1, LOW); digitalWrite(RR_IN2, LOW); analogWrite(RR_ENA, 0);
}
