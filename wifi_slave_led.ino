#include <FastLED.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Preferences.h>

// LED setup
#define NUM_LEDS 1  // Or 30 for strip
#define DATA_PIN 2

CRGB leds[NUM_LEDS];

// Unique slaveID
const String slaveID = "slave1";  // Change for each slave, e.g., "slave2"

// WiFi credentials
const char* ssid = "WhyNot";
const char* password = "whynotmadeit";

// UDP
const int udpPort = 12345;
WiFiUDP udp;
IPAddress broadcastIP(255, 255, 255, 255);

// Animation vars
bool shooting = false;
unsigned long startTime = 0;
unsigned long shotDuration = 800;
CRGB startColor = CRGB(255, 165, 0);
CRGB endColor = CRGB(255, 255, 0);

// Preferences
Preferences prefs;

// Heartbeat interval
const unsigned long heartbeatInterval = 10000; // 10 seconds
unsigned long lastHeartbeat = 0;

void sendParams(String type) {
  udp.beginPacket(broadcastIP, udpPort);
  String msg = type + "," + slaveID + "," + String(shotDuration) + "," +
               String(startColor.r) + "," + String(startColor.g) + "," + String(startColor.b) + "," +
               String(endColor.r) + "," + String(endColor.g) + "," + String(endColor.b);
  udp.print(msg);
  udp.endPacket();
  Serial.println("Sent " + type + " with current params");
}

void setup() {
  Serial.begin(115200);
  
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
  Serial.print("Slave IP: ");
  Serial.println(WiFi.localIP());

  prefs.begin("shotParams", false);
  shotDuration = prefs.getULong("duration", 800);
  startColor.r = prefs.getUChar("startR", 255);
  startColor.g = prefs.getUChar("startG", 165);
  startColor.b = prefs.getUChar("startB", 0);
  endColor.r = prefs.getUChar("endR", 255);
  endColor.g = prefs.getUChar("endG", 255);
  endColor.b = prefs.getUChar("endB", 0);
  prefs.end();
  Serial.println("Loaded parameters from Preferences");

  FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(255);

  udp.begin(udpPort);
  Serial.println("UDP listener started");

  sendParams("register");
}

void loop() {
  // Send heartbeat
  if (millis() - lastHeartbeat >= heartbeatInterval) {
    sendParams("heartbeat");
    lastHeartbeat = millis();
  }

  // Check UDP
  int packetSize = udp.parsePacket();
  if (packetSize) {
    char packetBuffer[255];
    int len = udp.read(packetBuffer, 255);
    if (len > 0) packetBuffer[len] = 0;
    
    String msg = String(packetBuffer);
    bool isShoot = msg.startsWith("shoot,");
    bool isUpdate = msg.startsWith("update,");
    if (isShoot || isUpdate) {
      int offset = isShoot ? 6 : 7;
      int comma1 = msg.indexOf(',', offset);
      int comma2 = msg.indexOf(',', comma1 + 1);
      int comma3 = msg.indexOf(',', comma2 + 1);
      int comma4 = msg.indexOf(',', comma3 + 1);
      int comma5 = msg.indexOf(',', comma4 + 1);
      int comma6 = msg.indexOf(',', comma5 + 1);
      int comma7 = msg.indexOf(',', comma6 + 1);
      
      shotDuration = msg.substring(offset, comma1).toInt();
      startColor.r = msg.substring(comma1 + 1, comma2).toInt();
      startColor.g = msg.substring(comma2 + 1, comma3).toInt();
      startColor.b = msg.substring(comma3 + 1, comma4).toInt();
      endColor.r = msg.substring(comma4 + 1, comma5).toInt();
      endColor.g = msg.substring(comma5 + 1, comma6).toInt();
      endColor.b = msg.substring(comma6 + 1, comma7).toInt();
      
      prefs.begin("shotParams", false);
      prefs.putULong("duration", shotDuration);
      prefs.putUChar("startR", startColor.r);
      prefs.putUChar("startG", startColor.g);
      prefs.putUChar("startB", startColor.b);
      prefs.putUChar("endR", endColor.r);
      prefs.putUChar("endG", endColor.g);
      prefs.putUChar("endB", endColor.b);
      prefs.end();
      Serial.println("Parameters updated and saved to Preferences");
      
      sendParams("confirm");
      
      if (isShoot) {
        shooting = true;
        startTime = millis();
        Serial.println("Shoot triggered");
      }
    }
  }

  if (shooting) {
    unsigned long currentTime = millis();
    unsigned long elapsed = currentTime - startTime;

    if (elapsed < shotDuration) {
      float factor = (float)elapsed / shotDuration;
      uint8_t r = startColor.r + factor * (endColor.r - startColor.r);
      uint8_t g = startColor.g + factor * (endColor.g - startColor.g);
      uint8_t b = startColor.b + factor * (endColor.b - startColor.b);

      fill_solid(leds, NUM_LEDS, CRGB(r, g, b));
      FastLED.show();
    } else {
      fill_solid(leds, NUM_LEDS, CRGB::Black);
      FastLED.show();
      shooting = false;
      Serial.println("Shot animation complete");
    }
  }
}