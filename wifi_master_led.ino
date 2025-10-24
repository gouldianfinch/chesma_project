#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESPAsyncWebServer.h>
#include <map>

const char* ssid = "WhyNot";
const char* password = "whynotmadeit";

const int udpPort = 12345;
WiFiUDP udp;

std::map<String, IPAddress> slaves;

struct Params {
  unsigned long duration = 800;
  uint8_t startR = 255, startG = 165, startB = 0;
  uint8_t endR = 255, endG = 255, endB = 0;
};

std::map<String, Params> slaveParams;

AsyncWebServer server(80);

void setup() {
  Serial.begin(115200);
  
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
  Serial.print("Master IP: ");
  Serial.println(WiFi.localIP());

  udp.begin(udpPort);
  Serial.println("UDP listener started for registrations, heartbeats, and confirms");

  // Main page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    String html = R"(
      <!DOCTYPE html>
      <html>
      <head>
        <title>ESP32 Shot Control</title>
        <style>
          body { font-family: Arial; margin: 20px; background: #1a1a1a; color: #fff; }
          .slave-row { 
            background: #2a2a2a; margin: 20px 0; padding: 15px; border-radius: 10px; 
            border-left: 5px solid #ff6600;
          }
          .params { margin: 10px 0; }
          .params label { display: inline-block; width: 120px; }
          .params input { margin: 5px; padding: 5px; width: 60px; }
          .shoot-btn { 
            background: #ff4444; color: white; border: none; padding: 10px 20px; 
            font-size: 16px; border-radius: 5px; cursor: pointer; margin: 10px 0;
          }
          .shoot-btn:hover { background: #cc0000; }
          .status { margin-top: 10px; padding: 5px; background: #333; border-radius: 3px; }
          .status.success { background: #228B22; }
          .status.error { background: #8B0000; }
        </style>
      </head>
      <body>
        <h1>🎯 Shot Simulator Control</h1>
        <div id="slaves-container"></div>
        
        <script>
          function loadSavedInputs(slaveID, row) {
            const saved = sessionStorage.getItem('inputs-' + slaveID);
            if (saved) {
              const inputs = JSON.parse(saved);
              row.querySelector('.duration').value = inputs.duration || '';
              row.querySelector('.startR').value = inputs.startR || '';
              row.querySelector('.startG').value = inputs.startG || '';
              row.querySelector('.startB').value = inputs.startB || '';
              row.querySelector('.endR').value = inputs.endR || '';
              row.querySelector('.endG').value = inputs.endG || '';
              row.querySelector('.endB').value = inputs.endB || '';
            }
          }

          function saveInputs(slaveID, row) {
            const inputs = {
              duration: row.querySelector('.duration').value,
              startR: row.querySelector('.startR').value,
              startG: row.querySelector('.startG').value,
              startB: row.querySelector('.startB').value,
              endR: row.querySelector('.endR').value,
              endG: row.querySelector('.endG').value,
              endB: row.querySelector('.endB').value
            };
            sessionStorage.setItem('inputs-' + slaveID, JSON.stringify(inputs));
          }

          function updateSlave(slaveID, rowIndex) {
            const row = document.getElementById('slave' + rowIndex);
            const duration = row.querySelector('.duration').value;
            const startR = row.querySelector('.startR').value;
            const startG = row.querySelector('.startG').value;
            const startB = row.querySelector('.startB').value;
            const endR = row.querySelector('.endR').value;
            const endG = row.querySelector('.endG').value;
            const endB = row.querySelector('.endB').value;
            
            const status = document.getElementById('status' + rowIndex);
            status.textContent = 'Updating...';
            status.className = 'status';
            
            fetch('/update', {
              method: 'POST',
              headers: {'Content-Type': 'application/x-www-form-urlencoded'},
              body: 'slaveID=' + encodeURIComponent(slaveID) + 
                    '&duration=' + duration +
                    '&startR=' + startR + '&startG=' + startG + '&startB=' + startB +
                    '&endR=' + endR + '&endG=' + endG + '&endB=' + endB
            })
            .then(response => response.text())
            .then(data => {
              if (data === 'OK') {
                status.textContent = 'Parameters updated';
                status.className = 'status success';
                sessionStorage.removeItem('inputs-' + slaveID);
              } else {
                status.textContent = 'Error: ' + data;
                status.className = 'status error';
              }
            })
            .catch(error => {
              status.textContent = 'Network error';
              status.className = 'status error';
            });
          }
          
          function shootSlave(slaveID, rowIndex) {
            const status = document.getElementById('status' + rowIndex);
            status.textContent = 'Triggering shot...';
            status.className = 'status';
            
            fetch('/shoot', {
              method: 'POST',
              headers: {'Content-Type': 'application/x-www-form-urlencoded'},
              body: 'slaveID=' + encodeURIComponent(slaveID)
            })
            .then(response => response.text())
            .then(data => {
              if (data === 'OK') {
                status.textContent = 'Shot triggered!';
                status.className = 'status success';
              } else {
                status.textContent = 'Error: ' + data;
                status.className = 'status error';
              }
            })
            .catch(error => {
              status.textContent = 'Network error';
              status.className = 'status error';
            });
          }

          function updateSlaveRow(slave, slaveIndex) {
            const container = document.getElementById('slaves-container');
            let row = document.getElementById('slave' + slaveIndex);
            if (!row) {
              row = document.createElement('div');
              row.className = 'slave-row';
              row.id = 'slave' + slaveIndex;
              container.appendChild(row);
            }
            row.innerHTML = `
              <h3>${slave.id} Controls</h3>
              <div class="params">
                <label>Duration (ms):</label>
                <input type="number" class="duration" value="${slave.duration}">
                <label>Start RGB:</label>
                <input type="number" class="startR" min="0" max="255" value="${slave.startR}">
                <input type="number" class="startG" min="0" max="255" value="${slave.startG}">
                <input type="number" class="startB" min="0" max="255" value="${slave.startB}">
                <label>End RGB:</label>
                <input type="number" class="endR" min="0" max="255" value="${slave.endR}">
                <input type="number" class="endG" min="0" max="255" value="${slave.endG}">
                <input type="number" class="endB" min="0" max="255" value="${slave.endB}">
                <button onclick='updateSlave("${slave.id}", ${slaveIndex})'>Update Params</button>
              </div>
              <button onclick='shootSlave("${slave.id}", ${slaveIndex})' class="shoot-btn">🔫 SHOOT</button>
              <div class="status" id="status${slaveIndex}">Ready</div>
            `;
            loadSavedInputs(slave.id, row);
            row.querySelectorAll('input').forEach(input => {
              input.addEventListener('input', () => saveInputs(slave.id, row));
            });
          }

          function refreshSlaves() {
            fetch('/get_slaves')
            .then(response => response.json())
            .then(data => {
              data.forEach((slave, index) => {
                updateSlaveRow(slave, index + 1);
              });
            });
          }
          
          setInterval(refreshSlaves, 5000);
          refreshSlaves();
        </script>
      </body>
      </html>
    )";
    request->send(200, "text/html", html);
  });

  server.on("/get_slaves", HTTP_GET, [](AsyncWebServerRequest *request){
    String json = "[";
    bool first = true;
    for (auto& pair : slaves) {
      if (!first) json += ",";
      String slaveID = pair.first;
      Params p = slaveParams[slaveID];
      json += "{";
      json += "\"id\":\"" + slaveID + "\",";
      json += "\"duration\":" + String(p.duration) + ",";
      json += "\"startR\":" + String(p.startR) + ",";
      json += "\"startG\":" + String(p.startG) + ",";
      json += "\"startB\":" + String(p.startB) + ",";
      json += "\"endR\":" + String(p.endR) + ",";
      json += "\"endG\":" + String(p.endG) + ",";
      json += "\"endB\":" + String(p.endB);
      json += "}";
      first = false;
    }
    json += "]";
    request->send(200, "application/json", json);
  });

  server.on("/update", HTTP_POST, [](AsyncWebServerRequest *request){
    String slaveID = request->getParam("slaveID", true)->value();
    if (slaves.find(slaveID) == slaves.end()) {
      request->send(400, "text/plain", "Slave not found");
      return;
    }
    
    Params& p = slaveParams[slaveID];
    if (request->hasParam("duration", true)) p.duration = request->getParam("duration", true)->value().toInt();
    if (request->hasParam("startR", true)) p.startR = request->getParam("startR", true)->value().toInt();
    if (request->hasParam("startG", true)) p.startG = request->getParam("startG", true)->value().toInt();
    if (request->hasParam("startB", true)) p.startB = request->getParam("startB", true)->value().toInt();
    if (request->hasParam("endR", true)) p.endR = request->getParam("endR", true)->value().toInt();
    if (request->hasParam("endG", true)) p.endG = request->getParam("endG", true)->value().toInt();
    if (request->hasParam("endB", true)) p.endB = request->getParam("endB", true)->value().toInt();
    
    IPAddress slaveIP = slaves[slaveID];
    udp.beginPacket(slaveIP, udpPort);
    String msg = "update," + String(p.duration) + "," +
                 String(p.startR) + "," + String(p.startG) + "," + String(p.startB) + "," +
                 String(p.endR) + "," + String(p.endG) + "," + String(p.endB);
    udp.print(msg);
    udp.endPacket();
    
    Serial.print("Update sent to ");
    Serial.println(slaveID);
    request->send(200, "text/plain", "OK");
  });

  server.on("/shoot", HTTP_POST, [](AsyncWebServerRequest *request){
    String slaveID = request->getParam("slaveID", true)->value();
    if (slaves.find(slaveID) == slaves.end()) {
      request->send(400, "text/plain", "Slave not found");
      return;
    }
    
    Params p = slaveParams[slaveID];
    IPAddress slaveIP = slaves[slaveID];
    udp.beginPacket(slaveIP, udpPort);
    String msg = "shoot," + String(p.duration) + "," +
                 String(p.startR) + "," + String(p.startG) + "," + String(p.startB) + "," +
                 String(p.endR) + "," + String(p.endG) + "," + String(p.endB);
    udp.print(msg);
    udp.endPacket();
    
    Serial.print("Shoot sent to ");
    Serial.println(slaveID);
    request->send(200, "text/plain", "OK");
  });

  server.begin();
  Serial.println("Web server started");
}

void loop() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    char packetBuffer[255];
    int len = udp.read(packetBuffer, 255);
    if (len > 0) packetBuffer[len] = 0;
    
    String msg = String(packetBuffer);
    if (msg.startsWith("register,") || msg.startsWith("heartbeat,")) {
      int comma1 = msg.indexOf(',', msg.startsWith("register,") ? 9 : 10);
      String slaveID = msg.substring(msg.startsWith("register,") ? 9 : 10, comma1);
      int comma2 = msg.indexOf(',', comma1 + 1);
      int comma3 = msg.indexOf(',', comma2 + 1);
      int comma4 = msg.indexOf(',', comma3 + 1);
      int comma5 = msg.indexOf(',', comma4 + 1);
      int comma6 = msg.indexOf(',', comma5 + 1);
      int comma7 = msg.indexOf(',', comma6 + 1);
      int comma8 = msg.indexOf(',', comma7 + 1);
      
      Params& p = slaveParams[slaveID];
      p.duration = msg.substring(comma1 + 1, comma2).toInt();
      p.startR = msg.substring(comma2 + 1, comma3).toInt();
      p.startG = msg.substring(comma3 + 1, comma4).toInt();
      p.startB = msg.substring(comma4 + 1, comma5).toInt();
      p.endR = msg.substring(comma5 + 1, comma6).toInt();
      p.endG = msg.substring(comma6 + 1, comma7).toInt();
      p.endB = msg.substring(comma7 + 1, comma8).toInt();
      
      IPAddress senderIP = udp.remoteIP();
      slaves[slaveID] = senderIP;
      Serial.print(msg.startsWith("register,") ? "Registered" : "Heartbeat from");
      Serial.print(" slave: ");
      Serial.print(slaveID);
      Serial.print(" at IP: ");
      Serial.println(senderIP);
      Serial.println("With params: duration=" + String(p.duration) + ", etc.");
    } else if (msg.startsWith("confirm,")) {
      int comma1 = msg.indexOf(',', 8);
      String slaveID = msg.substring(8, comma1);
      if (slaves.find(slaveID) != slaves.end()) {
        int comma2 = msg.indexOf(',', comma1 + 1);
        int comma3 = msg.indexOf(',', comma2 + 1);
        int comma4 = msg.indexOf(',', comma3 + 1);
        int comma5 = msg.indexOf(',', comma4 + 1);
        int comma6 = msg.indexOf(',', comma5 + 1);
        int comma7 = msg.indexOf(',', comma6 + 1);
        int comma8 = msg.indexOf(',', comma7 + 1);
        
        Params& p = slaveParams[slaveID];
        p.duration = msg.substring(comma1 + 1, comma2).toInt();
        p.startR = msg.substring(comma2 + 1, comma3).toInt();
        p.startG = msg.substring(comma3 + 1, comma4).toInt();
        p.startB = msg.substring(comma4 + 1, comma5).toInt();
        p.endR = msg.substring(comma5 + 1, comma6).toInt();
        p.endG = msg.substring(comma6 + 1, comma7).toInt();
        p.endB = msg.substring(comma7 + 1, comma8).toInt();
        
        Serial.print("Confirm received from ");
        Serial.print(slaveID);
        Serial.println(" with updated params");
      }
    }
  }
}