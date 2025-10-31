/*
 * WhyNot SLAVE_LIGHT Controller
 * Version: 2.0 (Production Ready)
 * 
 * Based on: slave_demo_v1_1.ino (working version)
 * Added:
 * - 5 independent LED channels
 * - Candle flicker effect
 * - Smooth fade on/off
 * - RGB color control
 * - EEPROM parameter storage
 * - Comprehensive command set
 */

#include <WiFi.h>
#include <esp_now.h>
#include <FastLED.h>
#include <Preferences.h>

// =================== DEVICE CONFIGURATION ===================
const String MY_NAME = "led_1";  // ← CHANGE FOR EACH DEVICE: led_1, led_2, led_3...
const String MY_TYPE = "light";

const char* WIFI_SSID = "WhyNot";
const char* WIFI_PASS = "whynotmadeit";

uint8_t broadcastMac[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

// =================== LED CONFIGURATION ===================
#define NUM_CHANNELS 5
#define MAX_LEDS_PER_CHANNEL 60

// Pin assignments for each channel
const uint8_t LED_PINS[NUM_CHANNELS] = { 2, 4, 5, 18, 19 };
const uint8_t LEDS_PER_CHANNEL[NUM_CHANNELS] = { 30, 30, 30, 30, 30 };

// LED arrays
CRGB leds_ch0[MAX_LEDS_PER_CHANNEL];
CRGB leds_ch1[MAX_LEDS_PER_CHANNEL];
CRGB leds_ch2[MAX_LEDS_PER_CHANNEL];
CRGB leds_ch3[MAX_LEDS_PER_CHANNEL];
CRGB leds_ch4[MAX_LEDS_PER_CHANNEL];

// =================== CHANNEL PARAMETERS ===================
struct ChannelParams {
  bool enabled = true;
  bool on = false;
  uint8_t r = 255;
  uint8_t g = 100;
  uint8_t b = 0;
  uint8_t brightness = 255;
  uint8_t speed = 50;
  uint8_t mode = 0;  // 0=candle, 1=solid, 2=fade

  // Animation state
  float currentBrightness = 0;
  float targetBrightness = 0;
  unsigned long lastUpdate = 0;
  uint8_t fadeDirection = 1;
  bool transitioning = false;
};

ChannelParams channels[NUM_CHANNELS];
Preferences prefs;

// =================== FORWARD DECLARATIONS ===================
void handleCommand(String cmd);
void updateChannel(uint8_t ch);
void updateAllChannels();
void setChannelColor(uint8_t ch, uint8_t r, uint8_t g, uint8_t b);
void turnChannelOn(uint8_t ch);
void turnChannelOff(uint8_t ch);
void applyCandleEffect(uint8_t ch);
void applySolidColor(uint8_t ch);
void applyFadeEffect(uint8_t ch);
void saveParams();
void loadParams();
void send(String msg);

// =================== ESP-NOW CALLBACKS ===================
void OnDataRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  char buf[256];
  memcpy(buf, data, len);
  buf[len] = 0;
  String msg = String(buf);

  // Parse: "target,COMMAND" or "all,COMMAND"
  int comma = msg.indexOf(',');
  if (comma == -1) return;

  String target = msg.substring(0, comma);
  String cmd = msg.substring(comma + 1);

  // Check if message is for this device
  if (target != MY_NAME && target != "all") return;

  Serial.println("[RX] " + cmd);
  handleCommand(cmd);
}

// =================== SEND FUNCTION ===================
void send(String msg) {
  esp_now_send(broadcastMac, (uint8_t*)msg.c_str(), msg.length());
}

// =================== LED CONTROL ===================
CRGB* getLEDArray(uint8_t ch) {
  switch (ch) {
    case 0: return leds_ch0;
    case 1: return leds_ch1;
    case 2: return leds_ch2;
    case 3: return leds_ch3;
    case 4: return leds_ch4;
    default: return nullptr;
  }
}

void setChannelColor(uint8_t ch, uint8_t r, uint8_t g, uint8_t b) {
  if (ch >= NUM_CHANNELS) return;
  channels[ch].r = r;
  channels[ch].g = g;
  channels[ch].b = b;
  // Если включено — обновим targetBrightness
  if (channels[ch].on) {
    channels[ch].targetBrightness = channels[ch].brightness;
  }
  saveParams();
  Serial.printf("[CH%d] Color: RGB(%d,%d,%d)\n", ch, r, g, b);
}

void turnChannelOn(uint8_t ch) {
  if (ch >= NUM_CHANNELS) return;
  channels[ch].on = true;
  channels[ch].targetBrightness = channels[ch].brightness;
  saveParams();
  Serial.printf("[CH%d] ON (fade in)\n", ch);
}

void turnChannelOff(uint8_t ch) {
  if (ch >= NUM_CHANNELS) return;
  channels[ch].on = false;
  channels[ch].targetBrightness = 0;
  saveParams();
  Serial.printf("[CH%d] OFF (fade out)\n", ch);
}

void updateBrightness(uint8_t ch) {
  ChannelParams& p = channels[ch];
  if (p.currentBrightness == p.targetBrightness) return;

  float step = 2.0 + (p.speed / 25.0);  // Чем выше speed — тем быстрее
  if (p.currentBrightness < p.targetBrightness) {
    p.currentBrightness += step;
    if (p.currentBrightness > p.targetBrightness)
      p.currentBrightness = p.targetBrightness;
  } else {
    p.currentBrightness -= step;
    if (p.currentBrightness < p.targetBrightness)
      p.currentBrightness = p.targetBrightness;
  }
}

void applyCandleEffect(uint8_t ch) {
  if (ch >= NUM_CHANNELS || !channels[ch].on) return;

  ChannelParams& p = channels[ch];
  CRGB* leds = getLEDArray(ch);
  if (leds == nullptr) return;

  uint8_t numLeds = LEDS_PER_CHANNEL[ch];
  uint8_t baseBrightness = p.brightness;

  for (int i = 0; i < numLeds; i++) {
    int flicker = random(-30, 30);
    int ledBrightness = constrain(baseBrightness + flicker, 50, 255);

    leds[i].r = (p.r * ledBrightness) / 255;
    leds[i].g = (p.g * ledBrightness) / 255;
    leds[i].b = (p.b * ledBrightness) / 255;
  }

  FastLED.show();
}

void applySolidColor(uint8_t ch) {
  if (ch >= NUM_CHANNELS || !channels[ch].on) return;

  ChannelParams& p = channels[ch];
  CRGB* leds = getLEDArray(ch);
  if (leds == nullptr) return;

  uint8_t numLeds = LEDS_PER_CHANNEL[ch];
  CRGB color = CRGB(
    (p.r * p.brightness) / 255,
    (p.g * p.brightness) / 255,
    (p.b * p.brightness) / 255);

  fill_solid(leds, numLeds, color);
  FastLED.show();
}

void applyFadeEffect(uint8_t ch) {
  if (ch >= NUM_CHANNELS) return;

  ChannelParams& p = channels[ch];
  CRGB* leds = getLEDArray(ch);
  if (leds == nullptr) return;

  unsigned long now = millis();
  if (now - p.lastUpdate > 20) {
    if (p.transitioning) {
      if (p.fadeDirection == 1) {
        p.currentBrightness += 5;
        if (p.currentBrightness >= p.brightness) {
          p.currentBrightness = p.brightness;
          p.transitioning = false;
        }
      } else {
        p.currentBrightness -= 5;
        if (p.currentBrightness <= 0) {
          p.currentBrightness = 0;
          p.transitioning = false;
        }
      }
    }

    uint8_t bright = (uint8_t)p.currentBrightness;
    CRGB color = CRGB(
      (p.r * bright) / 255,
      (p.g * bright) / 255,
      (p.b * bright) / 255);

    fill_solid(leds, LEDS_PER_CHANNEL[ch], color);
    FastLED.show();

    p.lastUpdate = now;
  }
}

void updateChannel(uint8_t ch) {
  if (ch >= NUM_CHANNELS || !channels[ch].enabled) return;

  ChannelParams& p = channels[ch];
  CRGB* leds = getLEDArray(ch);
  if (!leds) return;

  uint8_t numLeds = LEDS_PER_CHANNEL[ch];

  // === 1. Плавное изменение яркости ===
  unsigned long now = millis();
  if (now - p.lastUpdate > 15) {
    updateBrightness(ch);
    p.lastUpdate = now;
  }

  uint8_t bright = (uint8_t)constrain(p.currentBrightness, 0, 255);

  // === 2. Если выключено и яркость 0 — гасим ===
  if (bright == 0) {
    fill_solid(leds, numLeds, CRGB::Black);
    FastLED.show();
    return;
  }

  // === 3. Применяем эффект ===
  switch (p.mode) {
    case 0:
      {  // Candle — с мерцанием
        for (int i = 0; i < numLeds; i++) {
          int base = bright * 0.8;
          int flicker = random(-bright / 3, bright / 3);
          int warmth = random(0, 30);  // слегка желтит
          leds[i].r = constrain((p.r * (base + flicker)) / 255 + warmth, 0, 255);
          leds[i].g = (p.g * (base + flicker)) / 255;
          leds[i].b = max(0, (p.b * (base + flicker)) / 255 - warmth);
        }
        FastLED.show();
        break;
      }
    case 1:
      {  // Solid
        CRGB color = CRGB((p.r * bright) / 255, (p.g * bright) / 255, (p.b * bright) / 255);
        fill_solid(leds, numLeds, color);
        FastLED.show();
        break;
      }
    case 2:
      {  // Fade (пульсация)
        static uint8_t fadeDir = 1;
        static float fadeBright = bright;
        if (now - p.lastUpdate > (150 - p.speed)) {
          if (fadeDir) {
            fadeBright += 3;
            if (fadeBright >= bright) {
              fadeBright = bright;
              fadeDir = 0;
            }
          } else {
            fadeBright -= 3;
            if (fadeBright <= bright * 0.3) {
              fadeBright = bright * 0.3;
              fadeDir = 1;
            }
          }
          uint8_t fb = (uint8_t)fadeBright;
          CRGB color = CRGB((p.r * fb) / 255, (p.g * fb) / 255, (p.b * fb) / 255);
          fill_solid(leds, numLeds, color);
          FastLED.show();
        }
        break;
      }
  }
}

void updateAllChannels() {
  for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
    updateChannel(i);
  }
}

// =================== PARAMETERS ===================
void saveParams() {
  prefs.begin("light_params", false);
  for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
    String prefix = "ch" + String(i) + "_";
    prefs.putBool((prefix + "enabled").c_str(), channels[i].enabled);
    prefs.putBool((prefix + "on").c_str(), channels[i].on);
    prefs.putUChar((prefix + "r").c_str(), channels[i].r);
    prefs.putUChar((prefix + "g").c_str(), channels[i].g);
    prefs.putUChar((prefix + "b").c_str(), channels[i].b);
    prefs.putUChar((prefix + "bright").c_str(), channels[i].brightness);
    prefs.putUChar((prefix + "speed").c_str(), channels[i].speed);
    prefs.putUChar((prefix + "mode").c_str(), channels[i].mode);
  }
  prefs.end();
}

void loadParams() {
  prefs.begin("light_params", true);
  for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
    String prefix = "ch" + String(i) + "_";
    channels[i].enabled = prefs.getBool((prefix + "enabled").c_str(), true);
    channels[i].on = prefs.getBool((prefix + "on").c_str(), false);
    channels[i].r = prefs.getUChar((prefix + "r").c_str(), 255);
    channels[i].g = prefs.getUChar((prefix + "g").c_str(), 100);
    channels[i].b = prefs.getUChar((prefix + "b").c_str(), 0);
    channels[i].brightness = prefs.getUChar((prefix + "bright").c_str(), 255);
    channels[i].speed = prefs.getUChar((prefix + "speed").c_str(), 50);
    channels[i].mode = prefs.getUChar((prefix + "mode").c_str(), 0);
    channels[i].targetBrightness = channels[i].on ? channels[i].brightness : 0;
    channels[i].currentBrightness = channels[i].targetBrightness;
  }
  prefs.end();
}

// =================== COMMAND HANDLER ===================
void handleCommand(String cmd) {
  cmd.trim();
  String up = cmd;
  up.toUpperCase();

  // Parse channel specification: COMMAND,CH,0
  uint8_t targetChannel = 0;
  bool allChannels = true;

  int firstComma = up.indexOf(',');
  if (firstComma != -1) {
    String rest = up.substring(firstComma + 1);
    if (rest.startsWith("CH,")) {
      int channelStart = firstComma + 4;
      int secondComma = up.indexOf(',', channelStart);
      String chNum = up.substring(channelStart, secondComma != -1 ? secondComma : up.length());
      targetChannel = chNum.toInt();
      allChannels = false;

      // Remove channel spec from command
      if (secondComma != -1) {
        cmd = cmd.substring(0, firstComma) + cmd.substring(secondComma);
        up = cmd;
        up.toUpperCase();
      } else {
        cmd = cmd.substring(0, firstComma);
        up = cmd;
        up.toUpperCase();
      }
    }
  }

  // === INDIVIDUAL CHANNEL COMMANDS (LIGHT_ON0, LIGHT_OFF1, etc.) ===
  if (up.startsWith("LIGHT_ON") && up.length() > 8) {
    // Extract channel number: LIGHT_ON0 -> 0
    uint8_t ch = up.substring(8).toInt();
    if (ch < NUM_CHANNELS) {
      turnChannelOn(ch);
      Serial.printf("[CMD] Channel %d ON\n", ch);
    }
  } else if (up.startsWith("LIGHT_OFF") && up.length() > 9) {
    // Extract channel number: LIGHT_OFF1 -> 1
    uint8_t ch = up.substring(9).toInt();
    if (ch < NUM_CHANNELS) {
      turnChannelOff(ch);
      Serial.printf("[CMD] Channel %d OFF\n", ch);
    }
  } else if (up.startsWith("LIGHT_BRIGHTNESS") && up.indexOf(',') != -1) {
    // LIGHT_BRIGHTNESS0,255 or LIGHT_BRIGHTNESS,CH,2,180
    int chStart = 16; // After "LIGHT_BRIGHTNESS"
    int comma = up.indexOf(',', chStart);
    if (comma != -1) {
      String chStr = up.substring(chStart, comma);
      uint8_t ch = chStr.toInt();
      String valStr = cmd.substring(comma + 1);
      uint8_t brightness = valStr.toInt();
      
      if (ch < NUM_CHANNELS) {
        channels[ch].brightness = brightness;
        if (channels[ch].on) {
          channels[ch].targetBrightness = brightness;
        }
        saveParams();
        Serial.printf("[CMD] Channel %d brightness: %d\n", ch, brightness);
      }
    }
  } else if (up.startsWith("LIGHT_SPEED") && up.indexOf(',') != -1) {
    // LIGHT_SPEED0,75 or LIGHT_SPEED,CH,3,25
    int chStart = 11; // After "LIGHT_SPEED"
    int comma = up.indexOf(',', chStart);
    if (comma != -1) {
      String chStr = up.substring(chStart, comma);
      uint8_t ch = chStr.toInt();
      String valStr = cmd.substring(comma + 1);
      uint8_t speed = valStr.toInt();
      
      if (ch < NUM_CHANNELS) {
        channels[ch].speed = speed;
        saveParams();
        Serial.printf("[CMD] Channel %d speed: %d\n", ch, speed);
      }
    }
  } 
  // === LIGHT COMMANDS ===
  else if (up == "LIGHT_ON") {
    if (allChannels) {
      for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
        if (channels[i].enabled) turnChannelOn(i);
      }
    } else {
      turnChannelOn(targetChannel);
    }
    send("STATUS," + MY_NAME + ",ON");
  } else if (up == "LIGHT_OFF") {
    if (allChannels) {
      for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
        if (channels[i].enabled) turnChannelOff(i);
      }
    } else {
      turnChannelOff(targetChannel);
    }
    send("STATUS," + MY_NAME + ",OFF");
  } else if (up.startsWith("LIGHT_COLOR,")) {
    int c1 = up.indexOf(',');
    int c2 = up.indexOf(',', c1 + 1);
    int c3 = up.indexOf(',', c2 + 1);

    if (c1 != -1 && c2 != -1 && c3 != -1) {
      uint8_t r = cmd.substring(c1 + 1, c2).toInt();
      uint8_t g = cmd.substring(c2 + 1, c3).toInt();
      uint8_t b = cmd.substring(c3 + 1).toInt();

      if (allChannels) {
        for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
          if (channels[i].enabled) setChannelColor(i, r, g, b);
        }
      } else {
        setChannelColor(targetChannel, r, g, b);
      }
    }
  } else if (up.startsWith("LIGHT_MODE,")) {
    int comma = up.indexOf(',');
    uint8_t mode = cmd.substring(comma + 1).toInt();

    if (allChannels) {
      for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
        if (channels[i].enabled) channels[i].mode = mode;
      }
    } else {
      channels[targetChannel].mode = mode;
    }
    saveParams();
  } else if (up == "INFO") {
    Serial.println("\n=== DEVICE INFO ===");
    Serial.println("Name: " + MY_NAME);
    Serial.println("Type: " + MY_TYPE + " (5 channels)");
    for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
      Serial.printf("CH%d: %s, RGB(%d,%d,%d), Bright=%d, Speed=%d, Mode=%d\n",
                    i, channels[i].on ? "ON" : "OFF",
                    channels[i].r, channels[i].g, channels[i].b,
                    channels[i].brightness, channels[i].speed,
                    channels[i].mode);
    }
    Serial.println("==================\n");
  } else if (up == "PING") {
    // Respond to discovery ping
    send("REGISTER," + MY_NAME + "," + MY_TYPE);
    Serial.println("[PING] Responded to discovery");
  } else {
    Serial.println("[CMD] Unknown: " + cmd);
  }
}

// =================== SETUP ===================
void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("\n=== " + MY_NAME + " (LIGHT v2.0) ===");

  // Initialize FastLED
  FastLED.addLeds<WS2812B, 2, GRB>(leds_ch0, LEDS_PER_CHANNEL[0]);
  FastLED.addLeds<WS2812B, 4, GRB>(leds_ch1, LEDS_PER_CHANNEL[1]);
  FastLED.addLeds<WS2812B, 5, GRB>(leds_ch2, LEDS_PER_CHANNEL[2]);
  FastLED.addLeds<WS2812B, 18, GRB>(leds_ch3, LEDS_PER_CHANNEL[3]);
  FastLED.addLeds<WS2812B, 19, GRB>(leds_ch4, LEDS_PER_CHANNEL[4]);
  FastLED.setBrightness(255);
  FastLED.clear();
  FastLED.show();

  // Load parameters
  loadParams();
  Serial.println("[EEPROM] Parameters loaded");

  // WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n[WiFi] Connected: " + WiFi.localIP().toString());
  Serial.printf("[WiFi] Channel: %d\n", WiFi.channel());

  // ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP-NOW] Init FAILED!");
    while (1)
      ;
  }
  esp_now_register_recv_cb(OnDataRecv);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, broadcastMac, 6);
  peer.channel = WiFi.channel();
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = false;

  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("[ESP-NOW] Peer FAILED!");
    while (1)
      ;
  }

  // Register with master
  send("REGISTER," + MY_NAME + "," + MY_TYPE);
  Serial.println("[ESP-NOW] Registered: " + MY_NAME);

  Serial.println("\n=== READY! ===\n");
}

// =================== LOOP ===================
void loop() {
  // Update all LED channels
  updateAllChannels();

  // Send heartbeat every 10 seconds
  static uint32_t lastHeartbeat = 0;
  if (millis() - lastHeartbeat > 10000) {
    send("HEARTBEAT," + MY_NAME);
    lastHeartbeat = millis();
  }

  delay(10);
}
