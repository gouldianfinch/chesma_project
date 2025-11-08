/*
 * WhyNot SLAVE_GUN Controller
 * Version: 3.1 (FULLY FIXED + CANDLE SPEED)
 * 
 * FIXES:
 * - Fire delay now works correctly
 * - portPulses tracking with exact hall sensor count
 * - Positions in millimeters with auto-homing
 */

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Preferences.h>
#include <FastLED.h>
#include <GyverButton.h>

const String MY_NAME = "gun_l_1";
const String MY_TYPE = "gun";
const char* WIFI_SSID = "WhyNot";
const char* WIFI_PASS = "whynotmadeit";
uint8_t broadcastMac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

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

const float DIST_MM_BETWEEN_LIMITS = 75.0;
volatile long encoderCount = 0;
volatile long hallDelta = 0;
bool portOpen = false;
long portPulses = 3;
uint8_t motor1Speed = 150;
uint8_t motor2Speed = 180;

float pos1_mm = 20.0;
float pos2_mm = 60.0;
float pos3_mm = 10.0;

unsigned long t_preheat = 3000;
unsigned long t_sol = 500;
unsigned long t_led = 3500;
unsigned long t_delay_before_fire = 0;

uint8_t rgb1[3] = {250, 20, 0};
uint8_t rgb2[3] = {255, 90, 0};

int jerk_count1 = 5;
int jerk_count2 = 5;
int jerk_on_ms1 = 400;
int jerk_on_ms2 = 200;
int jerk_off_ms1 = 500;
int jerk_off_ms2 = 100;
uint8_t jerk_speed1 = 90;
uint8_t jerk_speed2 = 180;

int ramp_low = 50;
int ramp_high = 200;
int ramp_step = 5;

unsigned long shot_count = 0;

Preferences prefs;
float pulsesPerMM = 0.0;

CRGB leds[NUM_LEDS];
GButton startBtn(START_LIMIT);
GButton endBtn(END_LIMIT);

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
volatile bool stopRequested = false;

enum FireState { IDLE, OPENING_PORT, MOVING_TO_POS2, DELAYING_BEFORE_FIRE, PREHEATING, FIRING, CLOSING_PORT };
FireState fireState = IDLE;
unsigned long fireStartTime = 0;
unsigned long fireDelayStartTime = 0;  // Separate timer for delay!
unsigned long firePhaseStartTime = 0;  // Separate timer for each phase!
bool solenoidActive = false;
bool motor1Moving = false;
int jerkCurrentCount = 0;
long jerkTargetPos = 0;

long motor1TargetPos = 0;
uint8_t motor1CurrentSpeed = 0;
int8_t motor1Direction = 0;
bool motor1UseRamp = false;
unsigned long motor1RampLastUpdate = 0;

void setMotor1(int8_t dir, uint8_t speed);
void setMotor2(int8_t dir, uint8_t speed);
void stopAll();
void doFullCalibration();
void doHoming();
void moveMotor1ToPos(long targetPulses, uint8_t speed, bool useRamp);
void updateMotor1();
void jerkMotor1ToPos(long target, uint8_t speed);
void moveMotor2ByHalls(long targetDelta, uint8_t speed, bool useJerk);
void openPort();
void closePort();
void shoot();
void fire();
void updateFire();
void saveParams();
void loadParams();
void printInfo();
void handleCommand(String cmd);
void send(String msg);

long mmToPulses(float mm) { return (long)(mm * pulsesPerMM); }

void IRAM_ATTR encoderA_isr() {
  bool a = digitalRead(HALL_ENCODER_A);
  bool b = digitalRead(HALL_ENCODER_B);
  if (a == b) encoderCount++; else encoderCount--;
}

void IRAM_ATTR hall_isr() {
  portENTER_CRITICAL_ISR(&mux);
  hallDelta++;
  portEXIT_CRITICAL_ISR(&mux);
}

void OnDataRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  char buf[256];
  memcpy(buf, data, len);
  buf[len] = 0;
  String msg = String(buf);
  int comma = msg.indexOf(',');
  if (comma == -1) return;
  String target = msg.substring(0, comma);
  String cmd = msg.substring(comma + 1);
  if (target != MY_NAME && target != "all") return;
  Serial.println("[RX] " + cmd);
  handleCommand(cmd);
}

void send(String msg) { esp_now_send(broadcastMac, (uint8_t*)msg.c_str(), msg.length()); }

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== " + MY_NAME + " (GUN v3.1) ===");

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

  prefs.begin("gun", false);
  loadParams();

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts++ < 20) delay(500);
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("[WiFi] OK: %s\n", WiFi.localIP().toString().c_str());
  }

  if (esp_now_init() == ESP_OK) {
    esp_now_register_recv_cb(OnDataRecv);
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, broadcastMac, 6);
    peer.ifidx = WIFI_IF_STA;
    esp_now_add_peer(&peer);
    Serial.println("[ESP-NOW] OK");
  }

  printInfo();
  
  if (pulsesPerMM > 0) {
    Serial.println("[AUTO-HOME] Starting...");
    doHoming();
  }
  
  Serial.println("=== READY ===\n");
}

void loop() {
  static unsigned long lastHB = 0;
  startBtn.tick();
  endBtn.tick();
  if (startBtn.isPress() || endBtn.isPress()) stopAll();
  if (millis() - lastHB > 10000) {
    send("HEARTBEAT," + MY_NAME + "," + MY_TYPE);
    lastHB = millis();
  }
  updateMotor1();
  updateFire();
  delay(1);
}

void setMotor1(int8_t dir, uint8_t speed) {
  if (dir == 1) { digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW); }
  else if (dir == -1) { digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH); }
  else { digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW); }
  analogWrite(PWMA, speed);
}

void setMotor2(int8_t dir, uint8_t speed) {
  if (dir == 1) { digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW); }
  else if (dir == -1) { digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH); }
  else { digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW); }
  analogWrite(PWMB, speed);
}

void stopAll() {
  setMotor1(0, 0);
  setMotor2(0, 0);
  digitalWrite(HEATER, LOW);
  digitalWrite(SOLENOID, LOW);
  leds[0] = CRGB::Black;
  FastLED.show();
  motor1Moving = false;
  fireState = IDLE;
  portENTER_CRITICAL(&mux);
  stopRequested = false;
  portEXIT_CRITICAL(&mux);
  Serial.println("STOP");
  send("STATUS," + MY_NAME + ",STOPPED");
}

void doFullCalibration() {
  Serial.println("=== CALIBRATION ===");
  stopAll();
  delay(500);

  setMotor1(1, 100);
  unsigned long t0 = millis();
  while (digitalRead(END_LIMIT) == HIGH && millis() - t0 < 15000) delay(10);
  setMotor1(0, 0);
  delay(500);

  portENTER_CRITICAL(&mux);
  encoderCount = 0;
  portEXIT_CRITICAL(&mux);

  setMotor1(-1, 100);
  t0 = millis();
  while (digitalRead(START_LIMIT) == HIGH && millis() - t0 < 15000) delay(10);
  setMotor1(0, 0);
  delay(500);

  long totalPulses;
  portENTER_CRITICAL(&mux);
  totalPulses = abs(encoderCount);
  portEXIT_CRITICAL(&mux);

  pulsesPerMM = (float)totalPulses / DIST_MM_BETWEEN_LIMITS;
  Serial.printf("Pulses: %ld, PPM: %.2f\n", totalPulses, pulsesPerMM);

  saveParams();
  doHoming();
  send("STATUS," + MY_NAME + ",CALIBRATED");
}

void doHoming() {
  Serial.printf("HOMING to %.1fmm\n", pos1_mm);
  moveMotor1ToPos(mmToPulses(pos1_mm), motor1Speed, true);
}

void moveMotor1ToPos(long targetPulses, uint8_t speed, bool useRamp) {
  motor1TargetPos = targetPulses;
  motor1UseRamp = useRamp;
  motor1Moving = true;
  
  long current;
  portENTER_CRITICAL(&mux);
  current = encoderCount;
  portEXIT_CRITICAL(&mux);
  
  motor1Direction = (targetPulses > current) ? 1 : -1;
  motor1CurrentSpeed = useRamp ? ramp_low : speed;
  motor1RampLastUpdate = millis();
  
  setMotor1(motor1Direction, motor1CurrentSpeed);
}

void updateMotor1() {
  if (!motor1Moving) return;

  long current;
  portENTER_CRITICAL(&mux);
  current = encoderCount;
  portEXIT_CRITICAL(&mux);

  if (motor1Direction > 0 && digitalRead(END_LIMIT) == LOW) {
    setMotor1(0, 0);
    motor1Moving = false;
    return;
  }
  
  if (motor1Direction < 0 && digitalRead(START_LIMIT) == LOW) {
    setMotor1(0, 0);
    motor1Moving = false;
    return;
  }
  
  if ((motor1Direction > 0 && current >= motor1TargetPos) || 
      (motor1Direction < 0 && current <= motor1TargetPos)) {
    setMotor1(0, 0);
    motor1Moving = false;
    return;
  }

  if (motor1UseRamp && millis() - motor1RampLastUpdate > 50) {
    long distance = abs(motor1TargetPos - current);
    uint16_t temp = motor1CurrentSpeed + ramp_step;
    if (temp > ramp_high) temp = ramp_high;
    motor1CurrentSpeed = temp;
    if (distance < 200) {
      temp = (motor1CurrentSpeed > ramp_step) ? motor1CurrentSpeed - ramp_step : ramp_low;
      if (temp < ramp_low) temp = ramp_low;
      motor1CurrentSpeed = temp;
    }
    setMotor1(motor1Direction, motor1CurrentSpeed);
    motor1RampLastUpdate = millis();
  }
}

void jerkMotor1ToPos(long target, uint8_t speed) {
  if (!portOpen) return;
  portENTER_CRITICAL(&mux);
  long current = encoderCount;
  portEXIT_CRITICAL(&mux);
  jerkTargetPos = target;
  jerkCurrentCount = 0;
}

void moveMotor2ByHalls(long targetDelta, uint8_t speed, bool useJerk) {
  if (targetDelta == 0) return;
  
  portENTER_CRITICAL(&mux);
  stopRequested = false;
  hallDelta = 0;
  portEXIT_CRITICAL(&mux);

  long absTarget = abs(targetDelta);
  int8_t dir = (targetDelta > 0) ? 1 : -1;
  Serial.printf("MOVE2: target=%ld halls, dir=%d, jerk=%d\n", absTarget, dir, useJerk);

  if (!useJerk) {
    // Smooth movement without jerk
    setMotor2(dir, speed);
    unsigned long t0 = millis();
    while (true) {
      portENTER_CRITICAL(&mux);
      long now = hallDelta;
      bool stopReq = stopRequested;
      portEXIT_CRITICAL(&mux);

      Serial.printf("MOVE2: current=%ld/%ld\n", now, absTarget);

      if (stopReq || now >= absTarget) {
        setMotor2(0, 0);
        Serial.printf("MOVE2 done: %ld halls\n", now);
        portENTER_CRITICAL(&mux);
        portOpen = (dir > 0);
        portEXIT_CRITICAL(&mux);
        saveParams();
        return;
      }
      if (millis() - t0 > 10000) {
        setMotor2(0, 0);
        Serial.println("MOVE2 timeout!");
        return;
      }
      delay(10);
    }
  } else {
    // Jerk movement
    Serial.printf("MOVE2 JERK: count=%d, on=%dms, off=%dms\n", jerk_count2, jerk_on_ms2, jerk_off_ms2);
    for (int i = 0; i < jerk_count2; i++) {
      portENTER_CRITICAL(&mux);
      long now = hallDelta;
      portEXIT_CRITICAL(&mux);
      
      if (now >= absTarget) break;
      
      setMotor2(dir, jerk_speed2);
      Serial.printf("JERK %d ON: halls=%ld/%ld\n", i+1, now, absTarget);
      delay(jerk_on_ms2);
      setMotor2(0, 0);
      delay(jerk_off_ms2);
    }
    
    // Continue smooth to target
    setMotor2(dir, speed);
    unsigned long t0 = millis();
    while (true) {
      portENTER_CRITICAL(&mux);
      long now = hallDelta;
      portEXIT_CRITICAL(&mux);

      if (now >= absTarget || millis() - t0 > 5000) {
        setMotor2(0, 0);
        Serial.printf("MOVE2 jerk done: %ld halls\n", now);
        portENTER_CRITICAL(&mux);
        portOpen = (dir > 0);
        portEXIT_CRITICAL(&mux);
        saveParams();
        return;
      }
      delay(10);
    }
  }
}

void openPort() {
  if (portOpen) return;
  Serial.printf("Opening port: %ld halls\n", portPulses);
  moveMotor2ByHalls(portPulses, jerk_speed2, true);
}

void closePort() {
  if (!portOpen) return;
  Serial.printf("Closing port: %ld halls\n", portPulses);
  moveMotor2ByHalls(-portPulses, motor2Speed, false);
}

void shoot() {
  if (fireState != IDLE) return;
  Serial.println("SHOOT START");
  fireState = OPENING_PORT;
  fireStartTime = millis();
  jerkCurrentCount = 0;
  motor1Moving = false;
  jerkTargetPos = mmToPulses(pos2_mm);
  openPort();
  send("STATUS," + MY_NAME + ",SHOOTING");
}

void fire() {
  if (fireState != IDLE || !portOpen) return;
  Serial.println("FIRE START");
  
  if (t_delay_before_fire > 0) {
    fireState = DELAYING_BEFORE_FIRE;
    fireDelayStartTime = millis();
    Serial.printf("Delaying %lu ms before fire\n", t_delay_before_fire);
  } else {
    fireState = PREHEATING;
    firePhaseStartTime = millis();
    digitalWrite(HEATER, HIGH);
    Serial.println("Preheating...");
  }
  send("STATUS," + MY_NAME + ",FIRING");
}

void updateFire() {
  if (fireState == IDLE) return;

  if (fireState == OPENING_PORT && portOpen) {
    Serial.println("Port opened, moving to pos2");
    fireState = MOVING_TO_POS2;
    firePhaseStartTime = millis();
    jerkCurrentCount = 0;
    jerkMotor1ToPos(jerkTargetPos, jerk_speed1);
  }

  if (fireState == MOVING_TO_POS2) {
    unsigned long elapsed = millis() - firePhaseStartTime;
    
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
      Serial.println("Pos2 reached");
      
      if (t_delay_before_fire > 0) {
        fireState = DELAYING_BEFORE_FIRE;
        fireDelayStartTime = millis();
        Serial.printf("Delaying %lu ms\n", t_delay_before_fire);
      } else {
        fireState = PREHEATING;
        firePhaseStartTime = millis();
        digitalWrite(HEATER, HIGH);
        Serial.println("Preheating...");
      }
      return;
    }

    unsigned long cycleStart = jerkCurrentCount * (jerk_on_ms1 + jerk_off_ms1);
    unsigned long cycleEnd = cycleStart + jerk_on_ms1;

    if (elapsed >= cycleStart && elapsed < cycleEnd) {
      setMotor1(1, jerk_speed1);
    } else if (elapsed >= cycleEnd) {
      setMotor1(0, 0);
      jerkCurrentCount++;
    } else {
      setMotor1(0, 0);
    }
  }

  // FIXED: Separate timer for delay!
  if (fireState == DELAYING_BEFORE_FIRE) {
    if (millis() - fireDelayStartTime >= t_delay_before_fire) {
      fireState = PREHEATING;
      firePhaseStartTime = millis();  // NEW timer!
      digitalWrite(HEATER, HIGH);
      Serial.println("Delay complete, preheating...");
    }
    return;  // Don't process other states during delay!
  }

  if (fireState == PREHEATING) {
    unsigned long preheatElapsed = millis() - firePhaseStartTime;
    if (preheatElapsed >= t_preheat) {
      Serial.println("Preheat done, FIRING!");
      fireState = FIRING;
      firePhaseStartTime = millis();  // NEW timer for firing!
      digitalWrite(SOLENOID, HIGH);
      solenoidActive = true;
      moveMotor1ToPos(mmToPulses(pos3_mm), motor1Speed, true);
    }
    return;
  }

  if (fireState == FIRING) {
    unsigned long firingElapsed = millis() - firePhaseStartTime;

    // LED gradient
    if (firingElapsed < t_led) {
      float t = (float)firingElapsed / t_led;
      uint8_t r = rgb1[0] + (rgb2[0] - rgb1[0]) * t;
      uint8_t g = rgb1[1] + (rgb2[1] - rgb1[1]) * t;
      uint8_t b = rgb1[2] + (rgb2[2] - rgb1[2]) * t;
      leds[0] = CRGB(r, g, b);
      FastLED.show();
    } else if (leds[0] != CRGB::Black) {
      leds[0] = CRGB::Black;
      FastLED.show();
    }

    if (solenoidActive && firingElapsed >= t_sol) {
      digitalWrite(SOLENOID, LOW);
      digitalWrite(HEATER, LOW);
      solenoidActive = false;
      Serial.println("Solenoid OFF");
    }

    if (!motor1Moving && !solenoidActive && firingElapsed >= t_led) {
      Serial.println("Firing complete, closing");
      fireState = CLOSING_PORT;
      shot_count++;
      prefs.putULong("shot_count", shot_count);
      Serial.printf("Shots: %lu\n", shot_count);
      closePort();
    }
  }

  if (fireState == CLOSING_PORT && !portOpen) {
    Serial.println("SHOOT COMPLETE");
    fireState = IDLE;
    send("INFO," + MY_NAME + ",shot_count=" + String(shot_count));
  }
}

void saveParams() {
  prefs.putFloat("ppm", pulsesPerMM);
  prefs.putFloat("pos1", pos1_mm);
  prefs.putFloat("pos2", pos2_mm);
  prefs.putFloat("pos3", pos3_mm);
  prefs.putBool("portOpen", portOpen);
  prefs.putLong("portPulses", portPulses);
  prefs.putUChar("speed1", motor1Speed);
  prefs.putUChar("speed2", motor2Speed);
  prefs.putULong("t_preheat", t_preheat);
  prefs.putULong("t_sol", t_sol);
  prefs.putULong("t_led", t_led);
  prefs.putULong("t_delay", t_delay_before_fire);
  prefs.putBytes("rgb1", rgb1, 3);
  prefs.putBytes("rgb2", rgb2, 3);
  prefs.putInt("jc1", jerk_count1);
  prefs.putInt("jc2", jerk_count2);
  prefs.putInt("jo1", jerk_on_ms1);
  prefs.putInt("jo2", jerk_on_ms2);
  prefs.putInt("jf1", jerk_off_ms1);
  prefs.putInt("jf2", jerk_off_ms2);
  prefs.putUChar("js1", jerk_speed1);
  prefs.putUChar("js2", jerk_speed2);
  prefs.putInt("rl", ramp_low);
  prefs.putInt("rh", ramp_high);
  prefs.putInt("rs", ramp_step);
}

void loadParams() {
  pulsesPerMM = prefs.getFloat("ppm", 0.0);
  pos1_mm = prefs.getFloat("pos1", 20.0);
  pos2_mm = prefs.getFloat("pos2", 60.0);
  pos3_mm = prefs.getFloat("pos3", 10.0);
  portOpen = prefs.getBool("portOpen", false);
  portPulses = prefs.getLong("portPulses", 3);
  motor1Speed = prefs.getUChar("speed1", 150);
  motor2Speed = prefs.getUChar("speed2", 180);
  t_preheat = prefs.getULong("t_preheat", 3000);
  t_sol = prefs.getULong("t_sol", 500);
  t_led = prefs.getULong("t_led", 3500);
  t_delay_before_fire = prefs.getULong("t_delay", 0);
  jerk_count1 = prefs.getInt("jc1", 5);
  jerk_count2 = prefs.getInt("jc2", 5);
  jerk_on_ms1 = prefs.getInt("jo1", 400);
  jerk_on_ms2 = prefs.getInt("jo2", 200);
  jerk_off_ms1 = prefs.getInt("jf1", 500);
  jerk_off_ms2 = prefs.getInt("jf2", 100);
  jerk_speed1 = prefs.getUChar("js1", 90);
  jerk_speed2 = prefs.getUChar("js2", 180);
  ramp_low = prefs.getInt("rl", 50);
  ramp_high = prefs.getInt("rh", 200);
  ramp_step = prefs.getInt("rs", 5);
  shot_count = prefs.getULong("shot_count", 0);
  prefs.getBytes("rgb1", rgb1, 3);
  prefs.getBytes("rgb2", rgb2, 3);
}

void printInfo() {
  Serial.println("=== INFO ===");
  Serial.printf("PPM: %.2f\n", pulsesPerMM);
  Serial.printf("Pos: %.1f/%.1f/%.1fmm\n", pos1_mm, pos2_mm, pos3_mm);
  Serial.printf("Port: %ld halls\n", portPulses);
  Serial.printf("Shots: %lu\n", shot_count);
}

void handleCommand(String cmd) {
  cmd.trim();
  String up = cmd;
  up.toUpperCase();

  if (up == "SHOOT") shoot();
  else if (up == "FIRE") fire();
  else if (up == "STOP") stopAll();
  else if (up == "CAL") doFullCalibration();
  else if (up == "HOME") doHoming();
  else if (up == "OPEN_PORT") openPort();
  else if (up == "CLOSE_PORT") closePort();
  else if (up == "INFO") {
    printInfo();
    send("INFO," + MY_NAME + ",shot_count=" + String(shot_count));
  }
  else if (up == "RESET_SHOT_COUNT") {
    shot_count = 0;
    prefs.putULong("shot_count", 0);
  }
  else if (up == "PING") send("REGISTER," + MY_NAME + "," + MY_TYPE);
  else if (up.startsWith("SET_DELAY,")) { t_delay_before_fire = cmd.substring(10).toInt(); saveParams(); }
  else if (up.startsWith("SET_POS1,")) { pos1_mm = cmd.substring(9).toFloat(); saveParams(); }
  else if (up.startsWith("SET_POS2,")) { pos2_mm = cmd.substring(9).toFloat(); saveParams(); }
  else if (up.startsWith("SET_POS3,")) { pos3_mm = cmd.substring(9).toFloat(); saveParams(); }
  else if (up.startsWith("SET_PORT_PULSES,")) { portPulses = cmd.substring(16).toInt(); saveParams(); }
  else if (up.startsWith("SET_SPEED1,")) { motor1Speed = cmd.substring(11).toInt(); saveParams(); }
  else if (up.startsWith("SET_SPEED2,")) { motor2Speed = cmd.substring(11).toInt(); saveParams(); }
  else if (up.startsWith("SET_T_PREHEAT,")) { t_preheat = cmd.substring(14).toInt(); saveParams(); }
  else if (up.startsWith("SET_T_SOL,")) { t_sol = cmd.substring(10).toInt(); saveParams(); }
  else if (up.startsWith("SET_T_LED,")) { t_led = cmd.substring(10).toInt(); saveParams(); }
  else if (up.startsWith("SET_RGB1,")) {
    int c1 = cmd.indexOf(','), c2 = cmd.indexOf(',', c1 + 1), c3 = cmd.indexOf(',', c2 + 1);
    if (c1 != -1 && c2 != -1 && c3 != -1) {
      rgb1[0] = cmd.substring(c1 + 1, c2).toInt();
      rgb1[1] = cmd.substring(c2 + 1, c3).toInt();
      rgb1[2] = cmd.substring(c3 + 1).toInt();
      saveParams();
    }
  }
  else if (up.startsWith("SET_RGB2,")) {
    int c1 = cmd.indexOf(','), c2 = cmd.indexOf(',', c1 + 1), c3 = cmd.indexOf(',', c2 + 1);
    if (c1 != -1 && c2 != -1 && c3 != -1) {
      rgb2[0] = cmd.substring(c1 + 1, c2).toInt();
      rgb2[1] = cmd.substring(c2 + 1, c3).toInt();
      rgb2[2] = cmd.substring(c3 + 1).toInt();
      saveParams();
    }
  }
  else if (up.startsWith("SET_JERK_COUNT1,")) { jerk_count1 = cmd.substring(16).toInt(); saveParams(); }
  else if (up.startsWith("SET_JERK_COUNT2,")) { jerk_count2 = cmd.substring(16).toInt(); saveParams(); }
  else if (up.startsWith("SET_JERK_ON_MS1,")) { jerk_on_ms1 = cmd.substring(16).toInt(); saveParams(); }
  else if (up.startsWith("SET_JERK_ON_MS2,")) { jerk_on_ms2 = cmd.substring(16).toInt(); saveParams(); }
  else if (up.startsWith("SET_JERK_OFF_MS1,")) { jerk_off_ms1 = cmd.substring(17).toInt(); saveParams(); }
  else if (up.startsWith("SET_JERK_OFF_MS2,")) { jerk_off_ms2 = cmd.substring(17).toInt(); saveParams(); }
  else if (up.startsWith("SET_JERK_SPEED1,")) { jerk_speed1 = cmd.substring(16).toInt(); saveParams(); }
  else if (up.startsWith("SET_JERK_SPEED2,")) { jerk_speed2 = cmd.substring(16).toInt(); saveParams(); }
  else if (up.startsWith("SET_RAMP_LOW,")) { ramp_low = cmd.substring(13).toInt(); saveParams(); }
  else if (up.startsWith("SET_RAMP_HIGH,")) { ramp_high = cmd.substring(14).toInt(); saveParams(); }
  else if (up.startsWith("SET_RAMP_STEP,")) { ramp_step = cmd.substring(14).toInt(); saveParams(); }
}
