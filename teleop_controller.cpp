#include <Arduino.h>
#include <esp32-hal.h>
#include <esp32-hal-gpio.h>
#include <esp32-hal-ledc.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>

float lx, ly, rx, ry, lt, rt;
int a, b, x, y;

const char* webpage = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>HAMR Joystick</title>
  <style>
    body { font-family: sans-serif; text-align: center; background: #f0f0f0; }
    #joystickZone { width: 200px; height: 200px; margin: auto; margin-top: 50px; background: #ccc; border-radius: 50%; position: relative; touch-action: none; }
    #stick { width: 50px; height: 50px; background: #555; border-radius: 50%; position: absolute; top: 75px; left: 75px; }
  </style>
</head>
<body>
  <h2>HAMR Virtual Joystick</h2>
  <div id="joystickZone">
    <div id="stick"></div>
  </div>
  <script>
    const stick = document.getElementById('stick');
    const zone = document.getElementById('joystickZone');

    let dragging = false;

    zone.addEventListener('touchstart', startDrag);
    zone.addEventListener('touchmove', drag);
    zone.addEventListener('touchend', endDrag);
    zone.addEventListener('mousedown', startDrag);
    zone.addEventListener('mousemove', drag);
    zone.addEventListener('mouseup', endDrag);

    function startDrag(e) {
      dragging = true;
    }

    function drag(e) {
      if (!dragging) return;
      e.preventDefault();

      let x = (e.touches ? e.touches[0].clientX : e.clientX) - zone.getBoundingClientRect().left;
      let y = (e.touches ? e.touches[0].clientY : e.clientY) - zone.getBoundingClientRect().top;

      let dx = x - 100;
      let dy = y - 100;

      let dist = Math.min(Math.sqrt(dx*dx + dy*dy), 75);
      let angle = Math.atan2(dy, dx);

      let stickX = Math.cos(angle) * dist + 75;
      let stickY = Math.sin(angle) * dist + 75;

      stick.style.left = stickX + 'px';
      stick.style.top = stickY + 'px';

      // Normalize and send
      let normX = ((stickX - 75) / 75).toFixed(2);
      let normY = ((stickY - 75) / 75).toFixed(2);

      fetch(`/move?x=${normX}&y=${normY}`);
    }

    function endDrag() {
      dragging = false;
      stick.style.left = '75px';
      stick.style.top = '75px';
      fetch(`/move?x=0&y=0`);
    }
  </script>
</body>
</html>
)rawliteral";

WebServer server(80);

// WIFI CREDENTIALS
const char* ssid = "HAMR";
const char* password = "123571113";

// UDP SETUP
WiFiUDP udp;
const int port = 12345;  // Port to listen on
char incoming[255];  // Buffer for incoming data


void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 Ready");

  WiFi.softAP(ssid, password, 6, 0, 2);
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("ESP IP: ");

  server.on("/", HTTP_GET, []() {
    server.send(200, "text/html", webpage);  // Serve the HTML page
  });

  server.on("/move", HTTP_GET, []() {
    String xVal = server.arg("x");
    String yVal = server.arg("y");
    float x = xVal.toFloat();
    float y = -yVal.toFloat();
    Serial.printf("Joystick X: %.2f, Y: %.2f\n", x, y);
    server.send(200, "text/plain", "OK");
  });

  server.onNotFound([]() {
    server.send(404, "text/plain", "404 Not Found");
  });

  server.begin();
  Serial.println("HTTP server started");
    

  // Connect to WiFi
  // WiFi.begin(ssid, password);
  // Serial.print("Connecting to WiFi");
  // while (WiFi.status() != WL_CONNECTED) {
  //   delay(500);
  //   Serial.print(".");
  // }
  // Serial.println("\nWiFi connected!");
  Serial.print("ESP IP: "); 
  // Serial.println(WiFi.localIP());
  Serial.println(myIP);
  udp.begin(port);
  Serial.printf("Listening for UDP on port %d\n", port);
}

void loop() {
  // if (Serial.available()) {
//   String msg = Serial.readStringUntil('\n');
  server.handleClient(); // Handle HTTP requests
  int len = udp.parsePacket();
  if (len > 0) {
    udp.read(incoming, sizeof(incoming));
    incoming[len] = '\0';  // null-terminate
    // Serial.printf("Received: %s\n", incoming);
    String msg = String(incoming); 
    

    // Basic parsing example — you might want to improve parsing robustness
    int lxIndex = msg.indexOf("LX:");
    int lyIndex = msg.indexOf("LY:");
    int rxIndex = msg.indexOf("RX:");
    int ryIndex = msg.indexOf("RY:");
    int ltIndex = msg.indexOf("LT:");
    int rtIndex = msg.indexOf("RT:");
    int aIndex  = msg.indexOf("A:");
    int bIndex  = msg.indexOf("B:");
    int xIndex  = msg.indexOf("X:");
    int yIndex  = msg.indexOf("Y:");

    if (lxIndex == -1 || lyIndex == -1 || rxIndex == -1 || ryIndex == -1 ||
        ltIndex == -1 || rtIndex == -1 || aIndex == -1 || bIndex == -1 ||
        xIndex == -1 || yIndex == -1) {
      Serial.println("ESP: Parsing error!");
      return;
    }

    // lx = msg.substring(lxIndex + 3, lyIndex).toFloat();
    ly = msg.substring(lyIndex + 3, rxIndex).toFloat();
    rx = msg.substring(rxIndex + 3, ryIndex).toFloat();
    // ry = msg.substring(ryIndex + 3, ltIndex).toFloat();
    lt = msg.substring(ltIndex + 3, rtIndex).toFloat();
    rt = msg.substring(rtIndex + 3, aIndex).toFloat();

    a = msg.substring(aIndex + 2, bIndex).toInt();
    // b = msg.substring(bIndex + 2, xIndex).toInt();
    // x = msg.substring(xIndex + 2, yIndex).toInt();
    // y = msg.substring(yIndex + 2).toInt();

    Serial.printf("ESP:  LY: %.2f RX: %.2f LT: %.2f RT: %.2f A: %d \n", ly, rx, lt, rt, a);
    // Serial.printf("Joystick X: %.2f, Y: %.2f\n", x, y);
    // Serial.printf("ESP: LX: %.2f LY: %.2f RX: %.2f RY: %.2f LT: %.2f RT: %.2f A: %d B: %d X: %d Y: %d\n",
    //               lx, ly, rx, ry, lt, rt, a, b, x, y);
  }
}