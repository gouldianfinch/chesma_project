/*
 * WhyNot MASTER Controller
 * Version: 2.1 (Enhanced Gun Support)
 * 
 * Added support for all gun parameters:
 * - All timing parameters (t_preheat, t_sol, t_led, t_delay)
 * - Position settings (pos1, pos2, pos3)
 * - Motor speeds (speed1, speed2)
 * - RGB colors (rgb1, rgb2)
 * - Jerk parameters (count, on/off times, speeds for M1 and M2)
 * - Ramp parameters (low, high, step)
 * - Port pulses
 */

#include <WiFi.h>
#include <esp_now.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <LittleFS.h>
#include <map>

// =================== CONFIGURATION ===================
const char* WIFI_SSID = "WhyNot";
const char* WIFI_PASS = "whynotmadeit";

uint8_t broadcastMac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// =================== STRUCTURES ===================
AsyncWebServer server(80);

struct Device {
  String name;
  String type;  // "gun", "light", "mast"
  uint8_t mac[6];
  unsigned long lastSeen;
  bool online;
  
  // Gun parameters
  unsigned long shot_count = 0;
  unsigned long t_delay = 0;
  unsigned long t_preheat = 3000;
  unsigned long t_sol = 500;
  unsigned long t_led = 3500;
  long pos1 = 0;
  long pos2 = 1000;
  long pos3 = 2000;
  uint8_t speed1 = 150;
  uint8_t speed2 = 180;
  
  // Light parameters
  bool light_on = false;
  uint8_t r = 255, g = 100, b = 0;
};

std::map<String, Device> devices;

// =================== ESP-NOW CALLBACKS ===================
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  char buf[256];
  memcpy(buf, data, len);
  buf[len] = 0;
  String msg = String(buf);

  Serial.printf("[RX] %s | MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
    msg.c_str(), info->src_addr[0], info->src_addr[1], info->src_addr[2],
    info->src_addr[3], info->src_addr[4], info->src_addr[5]);

  // Parse REGISTER or HEARTBEAT
  if (msg.startsWith("REGISTER,") || msg.startsWith("HEARTBEAT,")) {
    int c1 = msg.indexOf(',');
    int c2 = msg.indexOf(',', c1 + 1);
    String name = msg.substring(c1 + 1, c2 != -1 ? c2 : msg.length());
    String type = c2 != -1 ? msg.substring(c2 + 1) : "unknown";

    if (devices.find(name) == devices.end()) {
      Device d;
      d.name = name;
      d.type = type;
      memcpy(d.mac, info->src_addr, 6);
      d.lastSeen = millis();
      d.online = true;
      devices[name] = d;
      Serial.println("[NEW] " + name + " (" + type + ")");
    } else {
      devices[name].lastSeen = millis();
      devices[name].online = true;
    }
  }
  
  // Parse INFO response from gun
  if (msg.startsWith("INFO,")) {
    int c1 = msg.indexOf(',');
    int c2 = msg.indexOf(',', c1 + 1);
    String name = msg.substring(c1 + 1, c2);
    
    // Parse shot_count=123
    if (msg.indexOf("shot_count=") != -1) {
      int start = msg.indexOf("shot_count=") + 11;
      int end = msg.indexOf(',', start);
      if (end == -1) end = msg.length();
      devices[name].shot_count = msg.substring(start, end).toInt();
    }
    
    // Parse t_delay=500
    if (msg.indexOf("t_delay=") != -1) {
      int start = msg.indexOf("t_delay=") + 8;
      int end = msg.indexOf(',', start);
      if (end == -1) end = msg.length();
      devices[name].t_delay = msg.substring(start, end).toInt();
    }
  }
  
  // Parse STATUS response from light
  if (msg.startsWith("STATUS,")) {
    int c1 = msg.indexOf(',');
    int c2 = msg.indexOf(',', c1 + 1);
    String name = msg.substring(c1 + 1, c2);
    String status = msg.substring(c2 + 1);
    
    if (status == "ON") {
      devices[name].light_on = true;
    } else if (status == "OFF") {
      devices[name].light_on = false;
    }
  }
}

void OnDataSent(const uint8_t *mac, esp_now_send_status_t status) {
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "[TX] OK" : "[TX] FAIL");
}

// =================== COMMAND FUNCTIONS ===================
void sendCommand(String to, String cmd) {
  String msg = to + "," + cmd;
  esp_err_t res = esp_now_send(broadcastMac, (uint8_t*)msg.c_str(), msg.length());
  Serial.println(res == ESP_OK ? "[SEND] " + msg : "[SEND] FAILED");
}

String getDevicesJSON() {
  String json = "[";
  bool first = true;
  
  for (auto &p : devices) {
    // Skip offline devices
    if (!p.second.online) continue;
    
    if (!first) json += ",";
    
    Device &d = p.second;
    json += "{";
    json += "\"name\":\"" + d.name + "\",";
    json += "\"type\":\"" + d.type + "\",";
    json += "\"online\":" + String(d.online ? "true" : "false");
    
    // Add type-specific data
    if (d.type == "gun") {
      json += ",\"shot_count\":" + String(d.shot_count);
      json += ",\"t_delay\":" + String(d.t_delay);
      json += ",\"t_preheat\":" + String(d.t_preheat);
      json += ",\"t_sol\":" + String(d.t_sol);
      json += ",\"t_led\":" + String(d.t_led);
      json += ",\"pos1\":" + String(d.pos1);
      json += ",\"pos2\":" + String(d.pos2);
      json += ",\"pos3\":" + String(d.pos3);
      json += ",\"speed1\":" + String(d.speed1);
      json += ",\"speed2\":" + String(d.speed2);
    } else if (d.type == "light") {
      json += ",\"on\":" + String(d.light_on ? "true" : "false");
      json += ",\"r\":" + String(d.r);
      json += ",\"g\":" + String(d.g);
      json += ",\"b\":" + String(d.b);
    }
    
    json += "}";
    first = false;
  }
  
  json += "]";
  return json;
}

// =================== SETUP ===================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== WhyNot MASTER Controller v2.1 ===");

  // WiFi (STA mode)
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n[WiFi] Connected: http://" + WiFi.localIP().toString());
  Serial.printf("[WiFi] Channel: %d\n", WiFi.channel());

  // LittleFS
  if (!LittleFS.begin()) {
    Serial.println("[LittleFS] Formatting...");
    LittleFS.format();
    LittleFS.begin();
  }
  Serial.println("[LittleFS] Mounted");

  // ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP-NOW] Init FAILED!");
    while(1);
  }
  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);

  // Add broadcast peer
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, broadcastMac, 6);
  peer.channel = WiFi.channel();
  peer.encrypt = false;
  peer.ifidx = WIFI_IF_STA;
  
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("[ESP-NOW] Broadcast peer FAILED!");
    while(1);
  }
  Serial.println("[ESP-NOW] Broadcast peer added");

  // =================== WEB SERVER ===================
  
  // Main page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *r) {
    if (LittleFS.exists("/index.html")) {
      r->send(LittleFS, "/index.html", "text/html");
    } else {
      r->send(404, "text/plain", "Upload index.html via LittleFS");
    }
  });

  // Get devices list
  server.on("/devices", HTTP_GET, [](AsyncWebServerRequest *r) {
    r->send(200, "application/json", getDevicesJSON());
  });

  // Send command (unified endpoint with support for all parameters)
  server.on("/cmd", HTTP_POST, [](AsyncWebServerRequest *r) {
    if (!r->hasParam("to", true) || !r->hasParam("cmd", true)) {
      r->send(400, "text/plain", "Missing parameters: to, cmd");
      return;
    }
    
    String to = r->getParam("to", true)->value();
    String cmd = r->getParam("cmd", true)->value();
    
    // Handle different command types
    
    // Simple value parameter
    if (r->hasParam("value", true)) {
      cmd += "," + r->getParam("value", true)->value();
    }
    
    // RGB color command (for lights)
    if (cmd == "LIGHT_COLOR" && r->hasParam("r", true) && r->hasParam("g", true) && r->hasParam("b", true)) {
      cmd += "," + r->getParam("r", true)->value();
      cmd += "," + r->getParam("g", true)->value();
      cmd += "," + r->getParam("b", true)->value();
    }
    
    // RGB color command (for guns - SET_RGB1 or SET_RGB2)
    if ((cmd == "SET_RGB1" || cmd == "SET_RGB2") && r->hasParam("r", true) && r->hasParam("g", true) && r->hasParam("b", true)) {
      cmd += "," + r->getParam("r", true)->value();
      cmd += "," + r->getParam("g", true)->value();
      cmd += "," + r->getParam("b", true)->value();
    }
    
    sendCommand(to, cmd);
    r->send(200, "text/plain", "OK");
  });

  // Favicon (prevent 404)
  server.on("/favicon.ico", HTTP_GET, [](AsyncWebServerRequest *r) {
    r->send(204);
  });

  server.begin();
  Serial.println("[Web] Server started: http://" + WiFi.localIP().toString());
  Serial.println("\n=== READY! Supports 30+ slaves ===\n");
}

// =================== LOOP ===================
void loop() {
  static uint32_t lastCheck = 0;
  static uint32_t lastDiscovery = 0;
  
  // Send discovery broadcast every 5 seconds to find new devices
  if (millis() - lastDiscovery > 5000) {
    String discoveryMsg = "all,PING";
    esp_now_send(broadcastMac, (uint8_t*)discoveryMsg.c_str(), discoveryMsg.length());
    lastDiscovery = millis();
  }
  
  // Check device timeouts every 30 seconds
  if (millis() - lastCheck > 30000) {
    int offlineCount = 0;
    
    for (auto &p : devices) {
      // Mark offline if no heartbeat for 60 seconds
      if (millis() - p.second.lastSeen > 60000) {
        if (p.second.online) {
          p.second.online = false;
          offlineCount++;
          Serial.println("[OFFLINE] " + p.second.name);
        }
      }
    }
    
    if (offlineCount > 0) {
      Serial.printf("[CHECK] %d device(s) went offline\n", offlineCount);
    }
    
    lastCheck = millis();
  }
  
  delay(10);
}
