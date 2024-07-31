#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

// Thông tin kết nối Wi-Fi
const char* ssid = "nuxauxa";
const char* password = "nunununu";

// Khởi tạo máy chủ web trên cổng 80
ESP8266WebServer server(80);

// Biến toàn cục để lưu trữ dữ liệu gửi
String data = "0";

void handleRoot() {
  String html = "<!DOCTYPE html><html><head><title>Car Control</title>";
  html += "<style>body {font-family: Arial, sans-serif; text-align: center;} h1 {font-size: 36px; font-weight: bold;} .control {display: inline-block; margin: 20px; padding: 20px;} .button {display: inline-block; padding: 30px 60px; margin: 15px; font-size: 50px; font-weight: bold; background-color: #4CAF50; color: white; border: none; border-radius: 15px; cursor: pointer;} .button:hover {background-color: #45a049;} .button-row {margin: 15px;}";
  html += ".stop-button {display: inline-block; padding: 30px 60px; margin: 15px; font-size: 50px; font-weight: bold; background-color: #f44336; color: white; border: none; border-radius: 15px; cursor: pointer;} .stop-button:hover {background-color: #d32f2f;}";
  html += ".inline-buttons {display: flex; justify-content: center; align-items: center;}";
  html += "</style></head>";
  html += "<body><h1>CAR Control</h1>";
  html += "<div class='control'>";
  html += "<div class='button-row'><button class='stop-button' id='stopBtn'>Stop</button></div>";
  html += "<div class='button-row'><button class='button' id='diagLeftUpBtn'>&#x2196;</button><button class='button' id='diagRightUpBtn'>&#x2197;</button></div>";
  html += "<div class='button-row'><button class='button' id='upBtn'>&uarr;</button></div>";
  html += "<div class='button-row'><button class='button' id='leftBtn'>&larr;</button><button class='button' id='rightBtn'>&rarr;</button></div>";
  html += "<div class='button-row'><button class='button' id='downBtn'>&darr;</button></div>";
  html += "<div class='button-row'><button class='button' id='diagLeftDownBtn'>&#x2199;</button><button class='button' id='diagRightDownBtn'>&#x2198;</button></div>";
  html += "<div class='button-row inline-buttons'><button class='button' id='leftTurnBtn'>&#x21ba; Turn Left</button><button class='button' id='rightTurnBtn'>Turn Right &#x21bb;</button></div>";
  html += "<script>let lastCommand = '0'; function sendCommand(command) { if (lastCommand !== command) { var xhttp = new XMLHttpRequest(); xhttp.open('POST', '/send', true); xhttp.setRequestHeader('Content-type', 'application/x-www-form-urlencoded'); xhttp.send('data=' + encodeURIComponent(command)); lastCommand = command; } }";
  html += "document.getElementById('upBtn').addEventListener('mousedown', function() { sendCommand('WW'); });";
  html += "document.getElementById('leftBtn').addEventListener('mousedown', function() { sendCommand('AA'); });";
  html += "document.getElementById('downBtn').addEventListener('mousedown', function() { sendCommand('SS'); });";
  html += "document.getElementById('rightBtn').addEventListener('mousedown', function() { sendCommand('DD'); });";
  html += "document.getElementById('leftTurnBtn').addEventListener('mousedown', function() { sendCommand('LL'); });";
  html += "document.getElementById('rightTurnBtn').addEventListener('mousedown', function() { sendCommand('RR'); });";
  html += "document.getElementById('diagLeftUpBtn').addEventListener('mousedown', function() { sendCommand('QQ'); });";
  html += "document.getElementById('diagRightUpBtn').addEventListener('mousedown', function() { sendCommand('EE'); });";
  html += "document.getElementById('diagLeftDownBtn').addEventListener('mousedown', function() { sendCommand('ZZ'); });";
  html += "document.getElementById('diagRightDownBtn').addEventListener('mousedown', function() { sendCommand('CC'); });";
  html += "document.getElementById('stopBtn').addEventListener('mousedown', function() { sendCommand('00'); });";
  html += "</script></body></html>";
  server.send(200, "text/html", html);
}


void handleSend() {
  if (server.method() == HTTP_POST) {
    data = server.arg("data"); // Cập nhật biến data với giá trị nhận được
    Serial.println(data);
    // Gửi dữ liệu qua UART đến STM32
    Serial1.print(data);
    Serial1.print("\n"); // Gửi ký tự newline để phân tách dữ liệu
    server.send(200, "text/plain", "Data sent to UART: " + data);
    data = "0"; // Đặt biến data về giá trị mặc định
  } else {
    server.send(405, "text/plain", "Method Not Allowed");
  }
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  server.on("/", handleRoot);
  server.on("/send", handleSend);
  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  server.handleClient();
}
