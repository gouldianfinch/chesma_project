// Компиляция: ESP32 Core 3.0.2
// Библиотеки: Preferences, FastLED, GyverButton

#include <Arduino.h>
#include <Preferences.h>
#include <FastLED.h>
#include <GyverButton.h>

// ================= PINS =================
#define AIN1 27
#define AIN2 26
#define PWMA 25
#define BIN1 32
#define BIN2 33
#define PWMB 23
#define HEATER 18
#define SOLENOID 19
#define START_LIMIT 4
#define END_LIMIT 5
#define HALL 15
#define HALL_ENCODER_A 21
#define HALL_ENCODER_B 22
#define LED_PIN 2
#define NUM_LEDS 1
#define STBY 14

// ============ PARAMETERS ============
const float DIST_MM_BETWEEN_LIMITS = 75.0;
volatile long encoderCount = 0;
volatile long hallDelta = 0;
bool portOpen = false;
long portPulses = 3;
uint8_t motor1Speed = 150;
uint8_t motor2Speed = 180;
long pos1 = 0, pos2 = 0, pos3 = 0;
unsigned long t_preheat = 3000;
unsigned long t_sol = 500;
unsigned long t_led = 3500;
uint8_t rgb1[3] = {250, 20, 0};
uint8_t rgb2[3] = {255, 90, 0};
// Jerk
int jerk_count1 = 5;
int jerk_count2 = 5;
int jerk_on_ms1 = 400;
int jerk_on_ms2 = 200;
int jerk_off_ms1 = 500;
int jerk_off_ms2 = 100;
uint8_t jerk_speed1 = 90;
uint8_t jerk_speed2 = 180;
// Ramp
int ramp_low = 50;
int ramp_high = 200;
int ramp_step = 5;

Preferences prefs;
const char* PREF_NAMESPACE = "gun_project";
const char* PREF_KEY_PPM = "pulsesPerMM";
const char* PREF_KEY_POS1 = "pos1";
const char* PREF_KEY_POS2 = "pos2";
const char* PREF_KEY_POS3 = "pos3";
const char* PREF_KEY_PORT_OPEN = "portOpen";
const char* PREF_KEY_PORT_PULSES = "portPulses";
const char* PREF_KEY_HALL_DELTA = "hallDelta";
const char* PREF_KEY_SPEED1 = "speed1";
const char* PREF_KEY_SPEED2 = "speed2";
const char* PREF_KEY_T_PREHEAT = "t_preheat";
const char* PREF_KEY_T_SOL = "t_sol";
const char* PREF_KEY_T_LED = "t_led";
const char* PREF_KEY_RGB1 = "rgb1";
const char* PREF_KEY_RGB2 = "rgb2";
const char* PREF_KEY_JERK_COUNT1 = "jerk_count1";
const char* PREF_KEY_JERK_COUNT2 = "jerk_count2";
const char* PREF_KEY_JERK_ON1 = "jerk_on1";
const char* PREF_KEY_JERK_ON2 = "jerk_on2";
const char* PREF_KEY_JERK_OFF1 = "jerk_off1";
const char* PREF_KEY_JERK_OFF2 = "jerk_off2";
const char* PREF_KEY_JERK_SPEED1 = "jerk_speed1";
const char* PREF_KEY_JERK_SPEED2 = "jerk_speed2";
const char* PREF_KEY_RAMP_LOW = "ramp_low";
const char* PREF_KEY_RAMP_HIGH = "ramp_high";
const char* PREF_KEY_RAMP_STEP = "ramp_step";
float pulsesPerMM = 0.0;

CRGB leds[NUM_LEDS];
GButton startBtn(START_LIMIT);
GButton endBtn(END_LIMIT);

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
volatile bool stopRequested = false;

// Состояния
enum FireState { IDLE, OPENING_PORT, MOVING_TO_POS2, PREHEATING, FIRING, CLOSING_PORT };
FireState fireState = IDLE;
unsigned long fireStartTime = 0;
bool solenoidActive = false;
bool motor1Moving = false;
int jerkCurrentCount = 0;
long jerkTargetPos = 0;

// Переменные для неблокирующего управления мотором 1
long motor1TargetPos = 0;
uint8_t motor1CurrentSpeed = 0;
int8_t motor1Direction = 0;
bool motor1UseRamp = false;
unsigned long motor1RampLastUpdate = 0;

// forward declarations
void setMotor1(int8_t dir, uint8_t speed);
void setMotor2(int8_t dir, uint8_t speed);
void stopAll();
void doFullCalibration();
void doHoming();
void handleSerialCommand(String cmd);
void moveMotor1ByMM(float mm, uint8_t speed);
void moveMotor1ToPos(long target, uint8_t speed, bool useRamp);
void updateMotor1();
void jerkMotor1ToPos(long target, uint8_t speed);
void moveMotor2ByHalls(long targetDelta, uint8_t speed, bool useJerk);
void openPort();
void closePort();
void fire();
void shoot();
void updateFire();
void saveParams();
void loadParams();
void printInfo();

// =================== INTERRUPTS ===================
void IRAM_ATTR encoderA_isr() {
  bool a = digitalRead(HALL_ENCODER_A);
  bool b = digitalRead(HALL_ENCODER_B);
  if (a == b) encoderCount++;
  else encoderCount--;
}

void IRAM_ATTR hall_isr() {
  portENTER_CRITICAL_ISR(&mux);
  hallDelta++;
  portEXIT_CRITICAL_ISR(&mux);
}

// =================== SETUP ===================
void setup() {
  Serial.begin(115200);
  delay(50);
  Serial.println("=== ESP32 motion controller start ===");

  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT); pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT); digitalWrite(STBY, HIGH);

  pinMode(HEATER, OUTPUT); digitalWrite(HEATER, LOW);
  pinMode(SOLENOID, OUTPUT); digitalWrite(SOLENOID, LOW);

  pinMode(HALL, INPUT_PULLUP);
  pinMode(START_LIMIT, INPUT_PULLUP);
  pinMode(END_LIMIT, INPUT_PULLUP);

  FastLED.addLeds<NEOPIXEL, LED_PIN>(leds, NUM_LEDS);
  leds[0] = CRGB::Black; FastLED.show();

  pinMode(HALL_ENCODER_A, INPUT_PULLUP);
  pinMode(HALL_ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HALL_ENCODER_A), encoderA_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(HALL), hall_isr, RISING);

  startBtn.tick(); endBtn.tick();

  prefs.begin(PREF_NAMESPACE, false);
  loadParams();

  Serial.println("Homing to START_LIMIT...");
  doHoming();

  if (portOpen || hallDelta > 0) {
    Serial.println("Port is open. Closing...");
    closePort();
  }

  Serial.println("Type HELP for command list.");
  printInfo();
}

// =================== LOOP ====================
void loop() {
  startBtn.tick();
  endBtn.tick();

  updateMotor1();  // Неблокирующее обновление мотора 1
  updateFire();

  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) handleSerialCommand(line);
  }
}

// =================== MOTORS ===================
void setMotor1(int8_t dir, uint8_t speed) {
  if (speed == 0) {
    digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW); analogWrite(PWMA, 0); return;
  }
  digitalWrite(AIN1, dir > 0 ? HIGH : LOW);
  digitalWrite(AIN2, dir > 0 ? LOW : HIGH);
  analogWrite(PWMA, speed);
}

void setMotor2(int8_t dir, uint8_t speed) {
  if (speed == 0) {
    digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW); analogWrite(PWMB, 0); return;
  }
  digitalWrite(BIN1, dir > 0 ? HIGH : LOW);
  digitalWrite(BIN2, dir > 0 ? LOW : HIGH);
  analogWrite(PWMB, speed);
}

void stopAll() {
  portENTER_CRITICAL(&mux);
  stopRequested = true;
  portEXIT_CRITICAL(&mux);
  setMotor1(0, 0); setMotor2(0, 0);
  digitalWrite(HEATER, LOW); digitalWrite(SOLENOID, LOW);
  leds[0] = CRGB::Black; FastLED.show();
  fireState = IDLE;
  solenoidActive = false;
  motor1Moving = false;
  jerkCurrentCount = 0;
  Serial.println("[STOP] All stopped.");
}

// =================== CALIBRATION & HOMING ===================
void doFullCalibration() {
  portENTER_CRITICAL(&mux);
  stopRequested = false;
  portEXIT_CRITICAL(&mux);

  leds[0] = CRGB::Blue; FastLED.show();

  if (digitalRead(END_LIMIT) == LOW) {
    Serial.println("Calib: END_LIMIT pressed, moving backward...");
    setMotor1(-1, 80);
    unsigned long tstart = millis();
    while (digitalRead(END_LIMIT) == LOW) {
      if (millis() - tstart > 5000) {
        Serial.println("Calib ERROR: timeout moving away from END_LIMIT");
        stopAll();
        return;
      }
      delay(2);
    }
    stopAll();
  }

  Serial.println("Calib: homing to START_LIMIT...");
  setMotor1(-1, 80);
  unsigned long tstart = millis();
  while (digitalRead(START_LIMIT) == HIGH) {
    if (millis() - tstart > 15000) {
      Serial.println("Calib ERROR: timeout homing START_LIMIT");
      stopAll();
      return;
    }
    delay(2);
  }
  stopAll();

  portENTER_CRITICAL(&mux);
  encoderCount = 0;
  portEXIT_CRITICAL(&mux);
  Serial.println("Calib: START_LIMIT reached, encoder cleared.");

  setMotor1(1, 90); delay(200); setMotor1(0, 0); delay(100);

  Serial.println("Calib: moving to END_LIMIT...");
  long startCount;
  portENTER_CRITICAL(&mux);
  startCount = encoderCount;
  portEXIT_CRITICAL(&mux);

  setMotor1(1, 120);
  tstart = millis();
  while (digitalRead(END_LIMIT) == HIGH) {
    if (millis() - tstart > 20000) {
      Serial.println("Calib ERROR: timeout to END_LIMIT");
      stopAll();
      return;
    }
    delay(2);
  }
  stopAll();

  long endCount;
  portENTER_CRITICAL(&mux);
  endCount = encoderCount;
  portEXIT_CRITICAL(&mux);

  long pulsesBetween = endCount - startCount;
  if (pulsesBetween <= 0) {
    Serial.println("Calib ERROR: invalid pulse count.");
    stopAll();
    return;
  }

  pulsesPerMM = (float)pulsesBetween / DIST_MM_BETWEEN_LIMITS;
  Serial.printf("Calib done. pulsesBetween=%ld -> pulsesPerMM=%.3f\n", pulsesBetween, pulsesPerMM);

  Serial.println("Calib: returning to START_LIMIT...");
  setMotor1(-1, 90);
  tstart = millis();
  while (digitalRead(START_LIMIT) == HIGH) {
    if (millis() - tstart > 15000) {
      Serial.println("Calib ERROR: timeout returning to START_LIMIT");
      stopAll();
      return;
    }
    delay(2);
  }
  stopAll();

  portENTER_CRITICAL(&mux);
  encoderCount = 0;
  portEXIT_CRITICAL(&mux);
  Serial.println("Calib: returned to START_LIMIT.");

  saveParams();
  Serial.println("Calibration saved.");

  for (int i = 0; i < 5; i++) {
    leds[0] = CRGB::Green; FastLED.show(); delay(200);
    leds[0] = CRGB::Black; FastLED.show(); delay(200);
  }
}

void doHoming() {
  portENTER_CRITICAL(&mux);
  stopRequested = false;
  portEXIT_CRITICAL(&mux);

  leds[0] = CRGB::Blue; FastLED.show();

  if (digitalRead(START_LIMIT) == LOW) {
    Serial.println("Homing: already at START_LIMIT.");
    portENTER_CRITICAL(&mux);
    encoderCount = 0;
    portEXIT_CRITICAL(&mux);
    Serial.println("Homing: encoder cleared.");
    leds[0] = CRGB::Black; FastLED.show();
    return;
  }

  if (digitalRead(END_LIMIT) == LOW) {
    Serial.println("Homing: END_LIMIT pressed, moving backward...");
    setMotor1(-1, 80);
    unsigned long tstart = millis();
    while (digitalRead(END_LIMIT) == LOW) {
      if (millis() - tstart > 5000) {
        Serial.println("Homing ERROR: timeout moving away from END_LIMIT");
        stopAll();
        return;
      }
      delay(2);
    }
    stopAll();
  }

  Serial.println("Homing to START_LIMIT...");
  setMotor1(-1, 80);
  unsigned long tstart = millis();
  while (digitalRead(START_LIMIT) == HIGH) {
    if (millis() - tstart > 15000) {
      Serial.println("Homing ERROR: timeout reaching START_LIMIT");
      stopAll();
      return;
    }
    delay(2);
  }
  stopAll();

  portENTER_CRITICAL(&mux);
  encoderCount = 0;
  portEXIT_CRITICAL(&mux);
  Serial.println("Homing: START_LIMIT reached, encoder cleared.");
  leds[0] = CRGB::Black; FastLED.show();
}

// =================== MOVEMENT ===================
void moveMotor1ByMM(float mm, uint8_t speed) {
  if (!portOpen) { Serial.println("ERROR: Port closed."); return; }
  if (pulsesPerMM <= 0.0) { Serial.println("ERROR: Run CAL."); return; }
  portENTER_CRITICAL(&mux);
  long current = encoderCount;
  portEXIT_CRITICAL(&mux);
  long target = (long)round(current + mm * pulsesPerMM);
  moveMotor1ToPos(target, speed, true);
}

void moveMotor1ToPos(long target, uint8_t speed, bool useRamp) {
  if (!portOpen) { Serial.println("ERROR: Port closed."); return; }
  if (pulsesPerMM <= 0.0) { Serial.println("ERROR: Run CAL."); return; }
  
  portENTER_CRITICAL(&mux);
  stopRequested = false;
  long current = encoderCount;
  portEXIT_CRITICAL(&mux);

  Serial.printf("Moving to pos: current=%ld -> target=%ld\n", current, target);
  
  if (target == current) { 
    motor1Moving = false; 
    return; 
  }

  motor1TargetPos = target;
  motor1Direction = (target > current) ? 1 : -1;
  motor1UseRamp = useRamp;
  motor1CurrentSpeed = useRamp ? ramp_low : speed;
  motor1RampLastUpdate = millis();
  motor1Moving = true;
  
  setMotor1(motor1Direction, motor1CurrentSpeed);
}

void updateMotor1() {
  if (!motor1Moving) return;
  
  portENTER_CRITICAL(&mux);
  long current = encoderCount;
  bool stopReq = stopRequested;
  portEXIT_CRITICAL(&mux);
  
  // Проверка на stop request
  if (stopReq) {
    setMotor1(0, 0);
    motor1Moving = false;
    return;
  }
  
  // Плавное увеличение скорости (ramp)
  if (motor1UseRamp && millis() - motor1RampLastUpdate >= 10) {
    if (motor1CurrentSpeed < ramp_high) {
      motor1CurrentSpeed = min(motor1CurrentSpeed + ramp_step, ramp_high);
      setMotor1(motor1Direction, motor1CurrentSpeed);
    }
    motor1RampLastUpdate = millis();
  }
  
  // Проверка концевиков
  if (motor1Direction > 0 && digitalRead(END_LIMIT) == LOW) {
    setMotor1(0, 0);
    motor1Moving = false;
    Serial.println("END_LIMIT reached");
    return;
  }
  
  if (motor1Direction < 0 && digitalRead(START_LIMIT) == LOW) {
    setMotor1(0, 0);
    motor1Moving = false;
    Serial.println("START_LIMIT reached");
    return;
  }
  
  // Проверка достижения целевой позиции
  if ((motor1Direction > 0 && current >= motor1TargetPos) || 
      (motor1Direction < 0 && current <= motor1TargetPos)) {
    setMotor1(0, 0);
    motor1Moving = false;
    Serial.println("Target position reached");
    return;
  }
}

void jerkMotor1ToPos(long target, uint8_t speed) {
  if (!portOpen) { Serial.println("ERROR: Port closed."); return; }
  portENTER_CRITICAL(&mux);
  long current = encoderCount;
  portEXIT_CRITICAL(&mux);

  jerkTargetPos = target;
  jerkCurrentCount = 0;
  Serial.printf("JERK1 to pos: current=%ld -> target=%ld, count=%d, on_ms=%d, off_ms=%d, speed=%d\n",
                current, target, jerk_count1, jerk_on_ms1, jerk_off_ms1, speed);
}

void moveMotor2ByHalls(long targetDelta, uint8_t speed, bool useJerk) {
  if (targetDelta == 0) return;
  portENTER_CRITICAL(&mux);
  stopRequested = false;
  hallDelta = 0;
  portEXIT_CRITICAL(&mux);

  long absTarget = abs(targetDelta);
  int8_t dir = (targetDelta > 0) ? 1 : -1;
  Serial.printf("MOVE2: delta=%ld, dir=%d, jerk=%d\n", targetDelta, dir, useJerk);

  if (!useJerk) {
    setMotor2(dir, speed);
    unsigned long t0 = millis();
    while (true) {
      portENTER_CRITICAL(&mux);
      long now = hallDelta;
      bool stopReq = stopRequested;
      portEXIT_CRITICAL(&mux);

      if (stopReq || now >= absTarget) {
        setMotor2(0, 0);
        Serial.println(now >= absTarget ? "MOVE2 done." : "MOVE2 aborted.");
        portENTER_CRITICAL(&mux);
        portOpen = (dir > 0);
        portEXIT_CRITICAL(&mux);
        saveParams();
        return;
      }
      if (millis() - t0 > 10000) { setMotor2(0, 0); return; }
      delay(1);
    }
  } else {
    long now = 0;
    while (now < absTarget) {
      portENTER_CRITICAL(&mux);
      now = hallDelta;
      bool stopReq = stopRequested;
      portEXIT_CRITICAL(&mux);

      if (stopReq || now >= absTarget) {
        setMotor2(0, 0);
        Serial.println(now >= absTarget ? "MOVE2 jerk done." : "MOVE2 jerk aborted.");
        portENTER_CRITICAL(&mux);
        portOpen = (dir > 0);
        portEXIT_CRITICAL(&mux);
        saveParams();
        return;
      }

      setMotor2(dir, speed);
      Serial.printf("MOVE2: jerk ON (target=%ld, now=%ld)\n", absTarget, now);
      delay(jerk_on_ms2);
      setMotor2(0, 0);
      Serial.printf("MOVE2: jerk OFF\n");
      delay(jerk_off_ms2);
    }
  }
}

void openPort() {
  if (portOpen) return;
  Serial.printf("Opening port: %ld pulses\n", portPulses);
  moveMotor2ByHalls(portPulses, jerk_speed2, true);
}

void closePort() {
  if (!portOpen) return;
  Serial.printf("Closing port: %ld pulses\n", portPulses);
  moveMotor2ByHalls(-portPulses, motor2Speed, false);
}

void shoot() {
  if (fireState != IDLE) { Serial.println("ERROR: Sequence in progress."); return; }
  if (pos1 == pos2 || pos2 == pos3 || pos1 == pos3) { Serial.println("ERROR: Invalid positions."); return; }

  Serial.println("Starting SHOOT sequence...");
  fireState = OPENING_PORT;
  fireStartTime = millis();
  jerkCurrentCount = 0;
  motor1Moving = false;
  portENTER_CRITICAL(&mux);
  long current = encoderCount;
  portEXIT_CRITICAL(&mux);
  jerkTargetPos = pos2;
  openPort();
}

void fire() {
  if (fireState != IDLE) { Serial.println("ERROR: Sequence in progress."); return; }
  if (!portOpen) { Serial.println("ERROR: Port must be open for FIRE."); return; }

  Serial.println("Starting FIRE sequence (port already open)...");
  fireState = PREHEATING;
  fireStartTime = millis();
  digitalWrite(HEATER, HIGH);
}

// =================== updateFire() ===================
void updateFire() {
  if (fireState == IDLE) return;
  unsigned long now = millis();
  unsigned long elapsed = now - fireStartTime;

  // === OPENING_PORT ===
  if (fireState == OPENING_PORT && portOpen) {
    Serial.println("Port opened, moving to pos2...");
    fireState = MOVING_TO_POS2;
    fireStartTime = now;
    jerkCurrentCount = 0;
    jerkMotor1ToPos(pos2, jerk_speed1);
  }

  if (fireState == MOVING_TO_POS2) {
    portENTER_CRITICAL(&mux);
    long current = encoderCount;
    bool stopReq = stopRequested;
    portEXIT_CRITICAL(&mux);

    if (stopReq || digitalRead(END_LIMIT) == LOW) {
      setMotor1(0, 0);
      fireState = IDLE;
      return;
    }

    if (jerkCurrentCount >= jerk_count1 || current >= jerkTargetPos) {
      setMotor1(0, 0);
      Serial.println("MOVING_TO_POS2 done.");
      fireState = PREHEATING;
      fireStartTime = now;
      digitalWrite(HEATER, HIGH);
      return;
    }

    unsigned long cycleStart = jerkCurrentCount * (jerk_on_ms1 + jerk_off_ms1);
    unsigned long cycleEnd = cycleStart + jerk_on_ms1;

    if (elapsed >= cycleStart && elapsed < cycleEnd) {
      setMotor1(1, jerk_speed1);
      if (elapsed < cycleStart + 10) {
        Serial.printf("MOVING_TO_POS2: jerk %d ON\n", jerkCurrentCount + 1);
      }
    } else if (elapsed >= cycleEnd) {
      setMotor1(0, 0);
      jerkCurrentCount++;
    } else {
      setMotor1(0, 0);
    }
  }

  if (fireState == PREHEATING && elapsed >= t_preheat) {
    Serial.println("PREHEATING done, starting FIRING phase");
    fireState = FIRING;
    fireStartTime = millis();  // СБРАСЫВАЕМ ТАЙМЕР!
    digitalWrite(SOLENOID, HIGH);
    solenoidActive = true;
    Serial.println("Solenoid ON, starting LED gradient and motor movement");
    moveMotor1ToPos(pos3, motor1Speed, true);  // Теперь неблокирующее!
  }

  if (fireState == FIRING) {
    unsigned long firingElapsed = millis() - fireStartTime;

    // LED: плавный градиент - обновляется КАЖДЫЙ цикл loop()
    if (firingElapsed < t_led) {
      float t = (float)firingElapsed / (float)t_led;
      uint8_t r = rgb1[0] + (int)((rgb2[0] - rgb1[0]) * t);
      uint8_t g = rgb1[1] + (int)((rgb2[1] - rgb1[1]) * t);
      uint8_t b = rgb1[2] + (int)((rgb2[2] - rgb1[2]) * t);
      leds[0] = CRGB(r, g, b);
      FastLED.show();
    } else {
      // Выключаем LED один раз
      if (leds[0] != CRGB::Black) {
        leds[0] = CRGB::Black;
        FastLED.show();
        Serial.println("LED OFF");
      }
    }

    // Выключаем соленоид и нагреватель по таймеру
    if (solenoidActive && firingElapsed >= t_sol) {
      digitalWrite(SOLENOID, LOW);
      digitalWrite(HEATER, LOW);
      solenoidActive = false;
      Serial.println("Solenoid and Heater OFF");
    }

    // Проверяем завершение ВСЕЙ фазы FIRING
    if (!motor1Moving && !solenoidActive && firingElapsed >= t_led) {
      Serial.println("FIRING phase complete, closing port...");
      fireState = CLOSING_PORT;
      fireStartTime = millis();
      closePort();
    }
  }

  if (fireState == CLOSING_PORT && !portOpen) {
    Serial.println("Port closed. SHOOT sequence completed.");
    fireState = IDLE;
  }
}

// =================== PREFERENCES, INFO, COMMANDS ===================
void saveParams() {
  prefs.putFloat(PREF_KEY_PPM, pulsesPerMM);
  prefs.putLong(PREF_KEY_POS1, pos1);
  prefs.putLong(PREF_KEY_POS2, pos2);
  prefs.putLong(PREF_KEY_POS3, pos3);
  prefs.putBool(PREF_KEY_PORT_OPEN, portOpen);
  prefs.putLong(PREF_KEY_PORT_PULSES, portPulses);
  prefs.putLong(PREF_KEY_HALL_DELTA, hallDelta);
  prefs.putUChar(PREF_KEY_SPEED1, motor1Speed);
  prefs.putUChar(PREF_KEY_SPEED2, motor2Speed);
  prefs.putULong(PREF_KEY_T_PREHEAT, t_preheat);
  prefs.putULong(PREF_KEY_T_SOL, t_sol);
  prefs.putULong(PREF_KEY_T_LED, t_led);
  prefs.putBytes(PREF_KEY_RGB1, rgb1, 3);
  prefs.putBytes(PREF_KEY_RGB2, rgb2, 3);
  prefs.putInt(PREF_KEY_JERK_COUNT1, jerk_count1);
  prefs.putInt(PREF_KEY_JERK_COUNT2, jerk_count2);
  prefs.putInt(PREF_KEY_JERK_ON1, jerk_on_ms1);
  prefs.putInt(PREF_KEY_JERK_ON2, jerk_on_ms2);
  prefs.putInt(PREF_KEY_JERK_OFF1, jerk_off_ms1);
  prefs.putInt(PREF_KEY_JERK_OFF2, jerk_off_ms2);
  prefs.putUChar(PREF_KEY_JERK_SPEED1, jerk_speed1);
  prefs.putUChar(PREF_KEY_JERK_SPEED2, jerk_speed2);
  prefs.putInt(PREF_KEY_RAMP_LOW, ramp_low);
  prefs.putInt(PREF_KEY_RAMP_HIGH, ramp_high);
  prefs.putInt(PREF_KEY_RAMP_STEP, ramp_step);
  Serial.println("Params saved.");
}

void loadParams() {
  if (prefs.isKey(PREF_KEY_PPM)) {
    pulsesPerMM = prefs.getFloat(PREF_KEY_PPM, 0.0);
    Serial.printf("Loaded pulsesPerMM = %.3f\n", pulsesPerMM);
  } else {
    pulsesPerMM = 0.0;
    Serial.println("No calibration data found. Run CAL.");
  }
  pos1 = prefs.getLong(PREF_KEY_POS1, 0);
  pos2 = prefs.getLong(PREF_KEY_POS2, 0);
  pos3 = prefs.getLong(PREF_KEY_POS3, 0);
  portOpen = prefs.getBool(PREF_KEY_PORT_OPEN, false);
  portPulses = prefs.getLong(PREF_KEY_PORT_PULSES, 3);
  hallDelta = prefs.getLong(PREF_KEY_HALL_DELTA, 0);
  motor1Speed = prefs.getUChar(PREF_KEY_SPEED1, 150);
  motor2Speed = prefs.getUChar(PREF_KEY_SPEED2, 180);
  t_preheat = prefs.getULong(PREF_KEY_T_PREHEAT, 3000);
  t_sol = prefs.getULong(PREF_KEY_T_SOL, 500);
  t_led = prefs.getULong(PREF_KEY_T_LED, 3500);
  jerk_count1 = prefs.getInt(PREF_KEY_JERK_COUNT1, 5);
  jerk_count2 = prefs.getInt(PREF_KEY_JERK_COUNT2, 5);
  jerk_on_ms1 = prefs.getInt(PREF_KEY_JERK_ON1, 400);
  jerk_on_ms2 = prefs.getInt(PREF_KEY_JERK_ON2, 200);
  jerk_off_ms1 = prefs.getInt(PREF_KEY_JERK_OFF1, 500);
  jerk_off_ms2 = prefs.getInt(PREF_KEY_JERK_OFF2, 100);
  jerk_speed1 = prefs.getUChar(PREF_KEY_JERK_SPEED1, 90);
  jerk_speed2 = prefs.getUChar(PREF_KEY_JERK_SPEED2, 180);
  ramp_low = prefs.getInt(PREF_KEY_RAMP_LOW, 50);
  ramp_high = prefs.getInt(PREF_KEY_RAMP_HIGH, 200);
  ramp_step = prefs.getInt(PREF_KEY_RAMP_STEP, 5);
  prefs.getBytes(PREF_KEY_RGB1, rgb1, 3);
  prefs.getBytes(PREF_KEY_RGB2, rgb2, 3);
}

void printInfo() {
  portENTER_CRITICAL(&mux);
  long ec = encoderCount;
  long hd = hallDelta;
  portEXIT_CRITICAL(&mux);

  Serial.println("=== STATUS ===");
  Serial.printf(" encoderCount = %ld\n", ec);
  Serial.printf(" hallDelta = %ld\n", hd);
  Serial.printf(" pulsesPerMM = %.3f\n", pulsesPerMM);
  Serial.printf(" pos1 = %ld, pos2 = %ld, pos3 = %ld\n", pos1, pos2, pos3);
  Serial.printf(" portOpen = %s\n", portOpen ? "true" : "false");
  Serial.printf(" portPulses = %ld\n", portPulses);
  Serial.printf(" motor1Speed = %d\n", motor1Speed);
  Serial.printf(" motor2Speed = %d\n", motor2Speed);
  Serial.printf(" t_preheat = %lu ms\n", t_preheat);
  Serial.printf(" t_sol = %lu ms\n", t_sol);
  Serial.printf(" t_led = %lu ms\n", t_led);
  Serial.printf(" rgb1 = (%d, %d, %d)\n", rgb1[0], rgb1[1], rgb1[2]);
  Serial.printf(" rgb2 = (%d, %d, %d)\n", rgb2[0], rgb2[1], rgb2[2]);
  Serial.printf(" jerk_count1 = %d, jerk_count2 = %d\n", jerk_count1, jerk_count2);
  Serial.printf(" jerk_on_ms1 = %d ms, jerk_on_ms2 = %d ms\n", jerk_on_ms1, jerk_on_ms2);
  Serial.printf(" jerk_off_ms1 = %d ms, jerk_off_ms2 = %d ms\n", jerk_off_ms1, jerk_off_ms2);
  Serial.printf(" jerk_speed1 = %d, jerk_speed2 = %d\n", jerk_speed1, jerk_speed2);
  Serial.printf(" ramp_low = %d, ramp_high = %d, ramp_step = %d\n", ramp_low, ramp_high, ramp_step);
  Serial.printf(" START_LIMIT (rear, pin %d) = %s\n", START_LIMIT, digitalRead(START_LIMIT) == LOW ? "PRESSED" : "OPEN");
  Serial.printf(" END_LIMIT (front, pin %d) = %s\n", END_LIMIT, digitalRead(END_LIMIT) == LOW ? "PRESSED" : "OPEN");
  Serial.println("===============");
}

// =================== SERIAL COMMANDS ===================
void handleSerialCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;
  String up = cmd;
  up.toUpperCase();

  if (up == "HELP") {
    Serial.println("Commands:");
    Serial.println(" INFO                 - show status");
    Serial.println(" CAL                  - run full calibration");
    Serial.println(" HOME                 - home to START_LIMIT");
    Serial.println(" MOVE1 <mm>           - move motor1 by mm (+ forward, - backward)");
    Serial.println(" MOVE2 <count>        - move motor2 by hall pulses (+ forward, - backward)");
    Serial.println(" JERK1                - jerk motor1 to pos2");
    Serial.println(" JERK2                - jerk motor2");
    Serial.println(" OPEN_PORT            - open port (motor2)");
    Serial.println(" CLOSE_PORT           - close port (motor2)");
    Serial.println(" SET_PORT_PULSES <n>  - set port pulses (default 3)");
    Serial.println(" SET_POS1             - save current motor1 position as pos1");
    Serial.println(" SET_POS2             - save current motor1 position as pos2");
    Serial.println(" SET_POS3             - save current motor1 position as pos3");
    Serial.println(" GOTO_POS1            - move motor1 to pos1");
    Serial.println(" GOTO_POS2            - move motor1 to pos2");
    Serial.println(" GOTO_POS3            - move motor1 to pos3");
    Serial.println(" SET_SPEED1 <value>   - set motor1 speed (0-255)");
    Serial.println(" SET_SPEED2 <value>   - set motor2 speed (0-255)");
    Serial.println(" SET_JERK_COUNT1 <n>  - set jerk count for motor1");
    Serial.println(" SET_JERK_COUNT2 <n>  - set jerk count for motor2");
    Serial.println(" SET_JERK_ON1 <ms>    - set jerk ON time for motor1");
    Serial.println(" SET_JERK_ON2 <ms>    - set jerk ON time for motor2");
    Serial.println(" SET_JERK_OFF1 <ms>   - set jerk OFF time for motor1");
    Serial.println(" SET_JERK_OFF2 <ms>   - set jerk OFF time for motor2");
    Serial.println(" SET_JERK_SPEED1 <value> - set jerk speed for motor1 (0-255)");
    Serial.println(" SET_JERK_SPEED2 <value> - set jerk speed for motor2 (0-255)");
    Serial.println(" SET_RAMP <low> <high> <step> - set ramp parameters for motor1");
    Serial.println(" FIRE                 - execute fire sequence");
    Serial.println(" SHOOT                - execute full shoot sequence");
    Serial.println(" SET_T_PREHEAT <ms>   - set preheat time (ms)");
    Serial.println(" SET_T_SOL <ms>       - set solenoid time (ms)");
    Serial.println(" SET_T_LED <ms>       - set LED time (ms)");
    Serial.println(" SET_RGB1 <r> <g> <b> - set start RGB color (0-255)");
    Serial.println(" SET_RGB2 <r> <g> <b> - set end RGB color (0-255)");
    Serial.println(" STOP                 - stop motion and fire sequence");
    Serial.println(" SET_PPM <value>      - set pulsesPerMM manually");
    Serial.println(" GET_PPM              - print pulsesPerMM");
    Serial.println(" SAVE                 - save params");
    Serial.println(" LOAD                 - load params");
    return;
  }

  if (up == "INFO") { printInfo(); return; }
  if (up == "CAL") { doFullCalibration(); return; }
  if (up == "HOME") { doHoming(); return; }
  if (up == "OPEN_PORT") { openPort(); return; }
  if (up == "CLOSE_PORT") { closePort(); return; }
  if (up == "FIRE") { fire(); return; }
  if (up == "SHOOT") { shoot(); return; }
  if (up == "JERK1") { jerkMotor1ToPos(pos2, jerk_speed1); return; }
  if (up == "JERK2") { moveMotor2ByHalls(portPulses, jerk_speed2, true); return; }
  if (up.startsWith("SET_PORT_PULSES")) {
    int sp = cmd.indexOf(' ');
    if (sp == -1) { Serial.println("Usage: SET_PORT_PULSES <n>"); return; }
    long n = cmd.substring(sp + 1).toInt();
    if (n <= 0) { Serial.println("Invalid port pulses."); return; }
    portPulses = n;
    saveParams();
    Serial.printf("portPulses set to %ld\n", portPulses);
    return;
  }
  if (up == "SET_POS1") {
    portENTER_CRITICAL(&mux);
    pos1 = encoderCount;
    portEXIT_CRITICAL(&mux);
    saveParams();
    Serial.printf("pos1 set to %ld\n", pos1);
    return;
  }
  if (up == "SET_POS2") {
    portENTER_CRITICAL(&mux);
    pos2 = encoderCount;
    portEXIT_CRITICAL(&mux);
    saveParams();
    Serial.printf("pos2 set to %ld\n", pos2);
    return;
  }
  if (up == "SET_POS3") {
    portENTER_CRITICAL(&mux);
    pos3 = encoderCount;
    portEXIT_CRITICAL(&mux);
    saveParams();
    Serial.printf("pos3 set to %ld\n", pos3);
    return;
  }
  if (up == "GOTO_POS1") {
    moveMotor1ToPos(pos1, motor1Speed, true);
    return;
  }
  if (up == "GOTO_POS2") {
    moveMotor1ToPos(pos2, motor1Speed, true);
    return;
  }
  if (up == "GOTO_POS3") {
    moveMotor1ToPos(pos3, motor1Speed, true);
    return;
  }
  if (up.startsWith("SET_SPEED1")) {
    int sp = cmd.indexOf(' ');
    if (sp == -1) { Serial.println("Usage: SET_SPEED1 <value>"); return; }
    int v = cmd.substring(sp + 1).toInt();
    if (v < 0 || v > 255) { Serial.println("Invalid speed (0-255)."); return; }
    motor1Speed = v;
    saveParams();
    Serial.printf("motor1Speed set to %d\n", motor1Speed);
    return;
  }
  if (up.startsWith("SET_SPEED2")) {
    int sp = cmd.indexOf(' ');
    if (sp == -1) { Serial.println("Usage: SET_SPEED2 <value>"); return; }
    int v = cmd.substring(sp + 1).toInt();
    if (v < 0 || v > 255) { Serial.println("Invalid speed (0-255)."); return; }
    motor2Speed = v;
    saveParams();
    Serial.printf("motor2Speed set to %d\n", motor2Speed);
    return;
  }
  if (up.startsWith("SET_JERK_COUNT1")) {
    int sp = cmd.indexOf(' ');
    if (sp == -1) { Serial.println("Usage: SET_JERK_COUNT1 <n>"); return; }
    int v = cmd.substring(sp + 1).toInt();
    if (v <= 0) { Serial.println("Invalid jerk count (>0)."); return; }
    jerk_count1 = v;
    saveParams();
    Serial.printf("jerk_count1 set to %d\n", jerk_count1);
    return;
  }
  if (up.startsWith("SET_JERK_COUNT2")) {
    int sp = cmd.indexOf(' ');
    if (sp == -1) { Serial.println("Usage: SET_JERK_COUNT2 <n>"); return; }
    int v = cmd.substring(sp + 1).toInt();
    if (v <= 0) { Serial.println("Invalid jerk count (>0)."); return; }
    jerk_count2 = v;
    saveParams();
    Serial.printf("jerk_count2 set to %d\n", jerk_count2);
    return;
  }
  if (up.startsWith("SET_JERK_ON1")) {
    int sp = cmd.indexOf(' ');
    if (sp == -1) { Serial.println("Usage: SET_JERK_ON1 <ms>"); return; }
    int v = cmd.substring(sp + 1).toInt();
    if (v <= 0) { Serial.println("Invalid jerk ON time (>0)."); return; }
    jerk_on_ms1 = v;
    saveParams();
    Serial.printf("jerk_on_ms1 set to %d ms\n", jerk_on_ms1);
    return;
  }
  if (up.startsWith("SET_JERK_ON2")) {
    int sp = cmd.indexOf(' ');
    if (sp == -1) { Serial.println("Usage: SET_JERK_ON2 <ms>"); return; }
    int v = cmd.substring(sp + 1).toInt();
    if (v <= 0) { Serial.println("Invalid jerk ON time (>0)."); return; }
    jerk_on_ms2 = v;
    saveParams();
    Serial.printf("jerk_on_ms2 set to %d ms\n", jerk_on_ms2);
    return;
  }
  if (up.startsWith("SET_JERK_OFF1")) {
    int sp = cmd.indexOf(' ');
    if (sp == -1) { Serial.println("Usage: SET_JERK_OFF1 <ms>"); return; }
    int v = cmd.substring(sp + 1).toInt();
    if (v < 0) { Serial.println("Invalid jerk OFF time (>=0)."); return; }
    jerk_off_ms1 = v;
    saveParams();
    Serial.printf("jerk_off_ms1 set to %d ms\n", jerk_off_ms1);
    return;
  }
  if (up.startsWith("SET_JERK_OFF2")) {
    int sp = cmd.indexOf(' ');
    if (sp == -1) { Serial.println("Usage: SET_JERK_OFF2 <ms>"); return; }
    int v = cmd.substring(sp + 1).toInt();
    if (v < 0) { Serial.println("Invalid jerk OFF time (>=0)."); return; }
    jerk_off_ms2 = v;
    saveParams();
    Serial.printf("jerk_off_ms2 set to %d ms\n", jerk_off_ms2);
    return;
  }
  if (up.startsWith("SET_JERK_SPEED1")) {
    int sp = cmd.indexOf(' ');
    if (sp == -1) { Serial.println("Usage: SET_JERK_SPEED1 <value>"); return; }
    int v = cmd.substring(sp + 1).toInt();
    if (v <= 0 || v > 255) { Serial.println("Invalid jerk speed (1-255)."); return; }
    jerk_speed1 = v;
    saveParams();
    Serial.printf("jerk_speed1 set to %d\n", jerk_speed1);
    return;
  }
  if (up.startsWith("SET_JERK_SPEED2")) {
    int sp = cmd.indexOf(' ');
    if (sp == -1) { Serial.println("Usage: SET_JERK_SPEED2 <value>"); return; }
    int v = cmd.substring(sp + 1).toInt();
    if (v <= 0 || v > 255) { Serial.println("Invalid jerk speed (1-255)."); return; }
    jerk_speed2 = v;
    saveParams();
    Serial.printf("jerk_speed2 set to %d\n", jerk_speed2);
    return;
  }
  if (up.startsWith("SET_RAMP")) {
    int sp1 = cmd.indexOf(' ');
    if (sp1 == -1) { Serial.println("Usage: SET_RAMP <low> <high> <step>"); return; }
    int sp2 = cmd.indexOf(' ', sp1 + 1);
    if (sp2 == -1) { Serial.println("Usage: SET_RAMP <low> <high> <step>"); return; }
    int sp3 = cmd.indexOf(' ', sp2 + 1);
    if (sp3 == -1) { Serial.println("Usage: SET_RAMP <low> <high> <step>"); return; }
    int low = cmd.substring(sp1 + 1, sp2).toInt();
    int high = cmd.substring(sp2 + 1, sp3).toInt();
    int step = cmd.substring(sp3 + 1).toInt();
    if (low <= 0 || high <= 0 || step <= 0 || low > high) {
      Serial.println("Invalid ramp parameters: low>0, high>0, step>0, low<=high");
      return;
    }
    ramp_low = low;
    ramp_high = high;
    ramp_step = step;
    saveParams();
    Serial.printf("ramp_low=%d, ramp_high=%d, ramp_step=%d\n", ramp_low, ramp_high, ramp_step);
    return;
  }
  if (up.startsWith("SET_T_PREHEAT")) {
    int sp = cmd.indexOf(' ');
    if (sp == -1) { Serial.println("Usage: SET_T_PREHEAT <ms>"); return; }
    unsigned long v = cmd.substring(sp + 1).toInt();
    if (v <= 0) { Serial.println("Invalid time (must be > 0)."); return; }
    t_preheat = v;
    saveParams();
    Serial.printf("t_preheat set to %lu ms\n", t_preheat);
    return;
  }
  if (up.startsWith("SET_T_SOL")) {
    int sp = cmd.indexOf(' ');
    if (sp == -1) { Serial.println("Usage: SET_T_SOL <ms>"); return; }
    unsigned long v = cmd.substring(sp + 1).toInt();
    if (v <= 0) { Serial.println("Invalid time (must be > 0)."); return; }
    t_sol = v;
    saveParams();
    Serial.printf("t_sol set to %lu ms\n", t_sol);
    return;
  }
  if (up.startsWith("SET_T_LED")) {
    int sp = cmd.indexOf(' ');
    if (sp == -1) { Serial.println("Usage: SET_T_LED <ms>"); return; }
    unsigned long v = cmd.substring(sp + 1).toInt();
    if (v <= 0) { Serial.println("Invalid time (must be > 0)."); return; }
    t_led = v;
    saveParams();
    Serial.printf("t_led set to %lu ms\n", t_led);
    return;
  }
  if (up.startsWith("SET_RGB1")) {
    int sp1 = cmd.indexOf(' ');
    if (sp1 == -1) { Serial.println("Usage: SET_RGB1 <r> <g> <b>"); return; }
    int sp2 = cmd.indexOf(' ', sp1 + 1);
    if (sp2 == -1) { Serial.println("Usage: SET_RGB1 <r> <g> <b>"); return; }
    int sp3 = cmd.indexOf(' ', sp2 + 1);
    if (sp3 == -1) { Serial.println("Usage: SET_RGB1 <r> <g> <b>"); return; }
    int r = cmd.substring(sp1 + 1, sp2).toInt();
    int g = cmd.substring(sp2 + 1, sp3).toInt();
    int b = cmd.substring(sp3 + 1).toInt();
    if (r < 0 || r > 255 || g < 0 || g > 255 || b < 0 || b > 255) {
      Serial.println("Invalid RGB values (0-255).");
      return;
    }
    rgb1[0] = r; rgb1[1] = g; rgb1[2] = b;
    saveParams();
    Serial.printf("rgb1 set to (%d, %d, %d)\n", rgb1[0], rgb1[1], rgb1[2]);
    return;
  }
  if (up.startsWith("SET_RGB2")) {
    int sp1 = cmd.indexOf(' ');
    if (sp1 == -1) { Serial.println("Usage: SET_RGB2 <r> <g> <b>"); return; }
    int sp2 = cmd.indexOf(' ', sp1 + 1);
    if (sp2 == -1) { Serial.println("Usage: SET_RGB2 <r> <g> <b>"); return; }
    int sp3 = cmd.indexOf(' ', sp2 + 1);
    if (sp3 == -1) { Serial.println("Usage: SET_RGB2 <r> <g> <b>"); return; }
    int r = cmd.substring(sp1 + 1, sp2).toInt();
    int g = cmd.substring(sp2 + 1, sp3).toInt();
    int b = cmd.substring(sp3 + 1).toInt();
    if (r < 0 || r > 255 || g < 0 || g > 255 || b < 0 || b > 255) {
      Serial.println("Invalid RGB values (0-255).");
      return;
    }
    rgb2[0] = r; rgb2[1] = g; rgb2[2] = b;
    saveParams();
    Serial.printf("rgb2 set to (%d, %d, %d)\n", rgb2[0], rgb2[1], rgb2[2]);
    return;
  }
  if (up.startsWith("MOVE1")) {
    int sp = cmd.indexOf(' ');
    if (sp == -1) { Serial.println("Usage: MOVE1 <mm>"); return; }
    float mm = cmd.substring(sp + 1).toFloat();
    Serial.printf("Command: MOVE1 %.3f mm\n", mm);
    moveMotor1ByMM(mm, motor1Speed);
    return;
  }
  if (up.startsWith("MOVE2")) {
    int sp = cmd.indexOf(' ');
    if (sp == -1) { Serial.println("Usage: MOVE2 <count>"); return; }
    long cnt = cmd.substring(sp + 1).toInt();
    Serial.printf("Command: MOVE2 %ld\n", cnt);
    moveMotor2ByHalls(cnt, motor2Speed, false);
    return;
  }
  if (up.startsWith("SET_PPM")) {
    int sp = cmd.indexOf(' ');
    if (sp == -1) { Serial.println("Usage: SET_PPM <value>"); return; }
    float v = cmd.substring(sp + 1).toFloat();
    if (v <= 0) { Serial.println("Invalid PPM."); return; }
    pulsesPerMM = v;
    saveParams();
    Serial.printf("pulsesPerMM set to %.3f\n", pulsesPerMM);
    return;
  }
  if (up == "GET_PPM") { Serial.printf("pulsesPerMM = %.3f\n", pulsesPerMM); return; }
  if (up == "SAVE") { saveParams(); return; }
  if (up == "LOAD") { loadParams(); return; }
  if (up == "STOP") { stopAll(); return; }

  Serial.println("Unknown command. Type HELP.");
}