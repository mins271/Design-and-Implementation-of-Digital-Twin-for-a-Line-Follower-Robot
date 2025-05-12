#include <WiFi.h>
#include <WiFiUdp.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>

// Function prototypes
void setMotorSpeed(int leftForwardPWM, int leftBackwardPWM, int rightForwardPWM, int rightBackwardPWM);
void applyMotorControl();

const char* ssid = "Apple";
const char* password = "Apple@123";

const char* matlabIP = "192.168.79.113";
const int leftSensorPort = 5000;
const int rightSensorPort = 5001;

const int rotateRPort = 8000;
const int rotateLPort = 8001;
const int speedPort = 8002;

#define IN1 25
#define IN2 26
#define IN3 27
#define IN4 14

#define IR_LEFT 15
#define IR_RIGHT 5
#define IR_CENTER 13

#define LEFT_SPEED_SENSOR 18
#define RIGHT_SPEED_SENSOR 19

const float WHEEL_DIAMETER = 0.066;
const float WHEELBASE = 0.137;
const float PULSES_PER_REVOLUTION = 20.0;
const float WHEEL_CIRCUMFERENCE = PI * WHEEL_DIAMETER;

WiFiUDP udpRotateR;
WiFiUDP udpRotateL;
WiFiUDP udpSpeed;
WiFiUDP udpSensors;

AsyncWebServer server(80);

// Store physical and virtual robot data as JSON strings
String physicalData = "[]";
String virtualData = "[]";

float rotateR = 0.0;
float rotateL = 0.0;
float speed = 100.0;

bool receivedRotateR = false;
bool receivedRotateL = false;
bool receivedSpeed = false;

unsigned long rotateRPacketCount = 0;
unsigned long rotateLPacketCount = 0;
unsigned long speedPacketCount = 0;

bool isConnectedToMATLAB = false;
unsigned long lastCommandTime = 0;
const unsigned long commandTimeout = 1000;

unsigned long lastSensorSendTime = 0;
const unsigned long sensorSendInterval = 50;

unsigned long simulationStartTime = 0;
bool simulationStarted = false;
const unsigned long logInterval = 5000;
unsigned long lastLogTime = 0;

volatile unsigned long leftPulseCount = 0;
volatile unsigned long rightPulseCount = 0;
unsigned long lastSpeedCalcTime = 0;
float leftWheelRPM = 0.0;
float rightWheelRPM = 0.0;
float leftWheelSpeed = 0.0;
float rightWheelSpeed = 0.0;
float linearSpeed = 0.0;
float angularVelocity = 0.0;

struct PerformanceData {
  unsigned long time;
  int deviation;
  float pathAccuracy;
  float centeredPathAccuracy;
  float errorRate;
  int stability;
  float linearSpeed;
  float angularVelocity;
};

PerformanceData performanceData[60];
int dataIndex = 0;

unsigned long onPathCount = 0;
unsigned long centeredCount = 0;
unsigned long totalSamples = 0;

float avgPathAccuracy = 0.0;
float avgCenteredPathAccuracy = 0.0;
float avgErrorRate = 0.0;
float avgLinearSpeed = 0.0;
float avgAngularVelocity = 0.0;

// Variables to store the current PWM values applied to the motors
int currentLeftForwardPWM = 0;
int currentLeftBackwardPWM = 0;
int currentRightForwardPWM = 0;
int currentRightBackwardPWM = 0;

// Virtual sensor values (received from Unity 3D)
uint8_t virtualLeftSensor = 0;
uint8_t virtualRightSensor = 0;
uint8_t virtualCenterSensor = 0;

void IRAM_ATTR leftSpeedSensorISR() {
  leftPulseCount++;
}

void IRAM_ATTR rightSpeedSensorISR() {
  rightPulseCount++;
}

void setup() {
  Serial.begin(115200);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  setMotorSpeed(0, 0, 0, 0);

  pinMode(IR_LEFT, INPUT);
  pinMode(IR_RIGHT, INPUT);
  pinMode(IR_CENTER, INPUT);

  pinMode(LEFT_SPEED_SENSOR, INPUT_PULLUP);
  pinMode(RIGHT_SPEED_SENSOR, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LEFT_SPEED_SENSOR), leftSpeedSensorISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_SPEED_SENSOR), rightSpeedSensorISR, FALLING);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("IP Address: " + WiFi.localIP().toString());

  udpRotateR.begin(rotateRPort);
  udpRotateL.begin(rotateLPort);
  udpSpeed.begin(speedPort);
  udpSensors.begin(0);

  // Route for root (webpage with tabs)
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>Robot Data Dashboard</title>
  <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
  <style>
    body { font-family: Arial, sans-serif; margin: 20px; }
    .tab { overflow: hidden; border: 1px solid #ccc; background-color: #f1f1f1; }
    .tab button { background-color: inherit; float: left; border: none; outline: none; cursor: pointer; padding: 14px 16px; transition: 0.3s; }
    .tab button:hover { background-color: #ddd; }
    .tab button.active { background-color: #ccc; }
    .tabcontent { display: none; padding: 6px 12px; border: 1px solid #ccc; border-top: none; }
    table { width: 100%; border-collapse: collapse; margin-top: 20px; }
    th, td { border: 1px solid #ddd; padding: 8px; text-align: left; }
    th { background-color: #f2f2f2; }
    canvas { margin-top: 20px; max-width: 100%; }
    .averages { margin-top: 20px; }
    .averages h4 { margin-bottom: 5px; }
    .averages p { margin: 5px 0; }
  </style>
</head>
<body>
  <h1>Robot Data Dashboard</h1>

  <!-- Tabs -->
  <div class="tab">
    <button class="tablinks" onclick="openTab(event, 'Physical')" id="defaultOpen">Physical</button>
    <button class="tablinks" onclick="openTab(event, 'Virtual')">Virtual</button>
    <button class="tablinks" onclick="openTab(event, 'Compare')">Compare</button>
  </div>

  <!-- Physical Tab -->
  <div id="Physical" class="tabcontent">
    <h2>Physical Robot Data</h2>
    <table id="physicalTable">
      <thead>
        <tr>
          <th>Time (s)</th>
          <th>Deviation</th>
          <th>Path Accuracy (%)</th>
          <th>Centered Path Accuracy (%)</th>
          <th>Error Rate (%)</th>
          <th>Stability</th>
          <th>Linear Speed (m/s)</th>
          <th>Angular Velocity (rad/s)</th>
        </tr>
      </thead>
      <tbody id="physicalTableBody"></tbody>
    </table>
    <h3>Graphs</h3>
    <canvas id="physicalPathAccuracyChart"></canvas>
    <canvas id="physicalCenteredPathAccuracyChart"></canvas>
    <canvas id="physicalErrorRateChart"></canvas>
    <canvas id="physicalLinearSpeedChart"></canvas>
    <canvas id="physicalAngularVelocityChart"></canvas>
  </div>

  <!-- Virtual Tab -->
  <div id="Virtual" class="tabcontent">
    <h2>Virtual Robot Data</h2>
    <table id="virtualTable">
      <thead>
        <tr>
          <th>Time (s)</th>
          <th>Deviation</th>
          <th>Path Accuracy (%)</th>
          <th>Centered Path Accuracy (%)</th>
          <th>Error Rate (%)</th>
          <th>Stability</th>
          <th>Linear Speed (m/s)</th>
          <th>Angular Velocity (rad/s)</th>
        </tr>
      </thead>
      <tbody id="virtualTableBody"></tbody>
    </table>
    <h3>Graphs</h3>
    <canvas id="virtualPathAccuracyChart"></canvas>
    <canvas id="virtualCenteredPathAccuracyChart"></canvas>
    <canvas id="virtualErrorRateChart"></canvas>
    <canvas id="virtualLinearSpeedChart"></canvas>
    <canvas id="virtualAngularVelocityChart"></canvas>
  </div>

  <!-- Compare Tab -->
  <div id="Compare" class="tabcontent">
    <h2>Compare Physical vs Virtual</h2>
    <div id="compareMessage"></div>
    <div id="compareAverages" class="averages" style="display: none;">
      <h3>Average Metrics</h3>
      <div id="physicalAverages">
        <h4>Physical Robot</h4>
        <p>Average Path Accuracy (%): <span id="physicalAvgPathAccuracy">0.0</span></p>
        <p>Average Centered Path Accuracy (%): <span id="physicalAvgCenteredPathAccuracy">0.0</span></p>
        <p>Average Error Rate (%): <span id="physicalAvgErrorRate">0.0</span></p>
      </div>
      <div id="virtualAverages">
        <h4>Virtual Robot</h4>
        <p>Average Path Accuracy (%): <span id="virtualAvgPathAccuracy">0.0</span></p>
        <p>Average Centered Path Accuracy (%): <span id="virtualAvgCenteredPathAccuracy">0.0</span></p>
        <p>Average Error Rate (%): <span id="virtualAvgErrorRate">0.0</span></p>
      </div>
    </div>
    <div id="compareGraphs" style="display: none;">
      <h3>Graphs</h3>
      <canvas id="comparePathAccuracyChart"></canvas>
      <canvas id="compareCenteredPathAccuracyChart"></canvas>
      <canvas id="compareErrorRateChart"></canvas>
      <canvas id="compareLinearSpeedChart"></canvas>
      <canvas id="compareAngularVelocityChart"></canvas>
    </div>
  </div>

  <script>
    // Tab functionality
    function openTab(evt, tabName) {
      var i, tabcontent, tablinks;
      tabcontent = document.getElementsByClassName("tabcontent");
      for (i = 0; i < tabcontent.length; i++) {
        tabcontent[i].style.display = "none";
      }
      tablinks = document.getElementsByClassName("tablinks");
      for (i = 0; i < tablinks.length; i++) {
        tablinks[i].className = tablinks[i].className.replace(" active", "");
      }
      document.getElementById(tabName).style.display = "block";
      evt.currentTarget.className += " active";

      if (tabName === 'Physical') loadPhysicalData();
      if (tabName === 'Virtual') loadVirtualData();
      if (tabName === 'Compare') loadCompareData();
    }

    // Open default tab
    document.getElementById("defaultOpen").click();

    // Load Physical Data
    function loadPhysicalData() {
      fetch('/physical').then(response => response.json()).then(data => {
        console.log("Fetched physical data:", data);
        let tableBody = document.getElementById('physicalTableBody');
        tableBody.innerHTML = '';
        data.forEach(item => {
          let row = `<tr>
            <td>${item.time.toFixed(1)}</td>
            <td>${item.deviation}</td>
            <td>${item.pathAccuracy.toFixed(1)}</td>
            <td>${item.centeredPathAccuracy.toFixed(1)}</td>
            <td>${item.errorRate.toFixed(1)}</td>
            <td>${item.stability}</td>
            <td>${item.linearSpeed.toFixed(2)}</td>
            <td>${item.angularVelocity.toFixed(2)}</td>
          </tr>`;
          tableBody.innerHTML += row;
        });

        createChart('physicalPathAccuracyChart', 'Path Accuracy (%) over Time', data, 'pathAccuracy', 'rgba(54, 162, 235, 0.2)', 'rgba(54, 162, 235, 1)');
        createChart('physicalCenteredPathAccuracyChart', 'Centered Path Accuracy (%) over Time', data, 'centeredPathAccuracy', 'rgba(75, 192, 192, 0.2)', 'rgba(75, 192, 192, 1)');
        createChart('physicalErrorRateChart', 'Error Rate (%) over Time', data, 'errorRate', 'rgba(255, 206, 86, 0.2)', 'rgba(255, 206, 86, 1)');
        createChart('physicalLinearSpeedChart', 'Linear Speed (m/s) over Time', data, 'linearSpeed', 'rgba(75, 192, 192, 0.2)', 'rgba(75, 192, 192, 1)');
        createChart('physicalAngularVelocityChart', 'Angular Velocity (rad/s) over Time', data, 'angularVelocity', 'rgba(153, 102, 255, 0.2)', 'rgba(153, 102, 255, 1)');
      }).catch(error => {
        console.error("Error fetching physical data:", error);
      });
    }

    // Load Virtual Data
    function loadVirtualData() {
      fetch('/virtual').then(response => response.json()).then(data => {
        console.log("Fetched virtual data:", data);
        let tableBody = document.getElementById('virtualTableBody');
        tableBody.innerHTML = '';
        data.forEach(item => {
          let row = `<tr>
            <td>${item.time.toFixed(1)}</td>
            <td>${item.deviation}</td>
            <td>${item.normalPathAccuracy.toFixed(1)}</td>
            <td>${item.centeredPathAccuracy.toFixed(1)}</td>
            <td>${item.errorRate.toFixed(1)}</td>
            <td>${item.stability}</td>
            <td>${item.linearSpeed.toFixed(2)}</td>
            <td>${item.angularVelocity.toFixed(2)}</td>
          </tr>`;
          tableBody.innerHTML += row;
        });

        createChart('virtualPathAccuracyChart', 'Path Accuracy (%) over Time', data, 'normalPathAccuracy', 'rgba(54, 162, 235, 0.2)', 'rgba(54, 162, 235, 1)');
        createChart('virtualCenteredPathAccuracyChart', 'Centered Path Accuracy (%) over Time', data, 'centeredPathAccuracy', 'rgba(75, 192, 192, 0.2)', 'rgba(75, 192, 192, 1)');
        createChart('virtualErrorRateChart', 'Error Rate (%) over Time', data, 'errorRate', 'rgba(255, 206, 86, 0.2)', 'rgba(255, 206, 86, 1)');
        createChart('virtualLinearSpeedChart', 'Linear Speed (m/s) over Time', data, 'linearSpeed', 'rgba(75, 192, 192, 0.2)', 'rgba(75, 192, 192, 1)');
        createChart('virtualAngularVelocityChart', 'Angular Velocity (rad/s) over Time', data, 'angularVelocity', 'rgba(153, 102, 255, 0.2)', 'rgba(153, 102, 255, 1)');
      }).catch(error => {
        console.error("Error fetching virtual data:", error);
      });
    }

    // Load Compare Data
    function loadCompareData() {
      Promise.all([
        fetch('/physical').then(response => response.json()),
        fetch('/virtual').then(response => response.json())
      ]).then(([physical, virtual]) => {
        let messageDiv = document.getElementById('compareMessage');
        let averagesDiv = document.getElementById('compareAverages');
        let graphsDiv = document.getElementById('compareGraphs');

        if (physical.length > 0 && virtual.length > 0) {
          let physicalAvgPathAccuracy = physical.reduce((sum, item) => sum + item.pathAccuracy, 0) / physical.length;
          let physicalAvgCenteredPathAccuracy = physical.reduce((sum, item) => sum + item.centeredPathAccuracy, 0) / physical.length;
          let physicalAvgErrorRate = physical.reduce((sum, item) => sum + item.errorRate, 0) / physical.length;

          let virtualAvgPathAccuracy = virtual.reduce((sum, item) => sum + item.normalPathAccuracy, 0) / virtual.length;
          let virtualAvgCenteredPathAccuracy = virtual.reduce((sum, item) => sum + item.centeredPathAccuracy, 0) / virtual.length;
          let virtualAvgErrorRate = virtual.reduce((sum, item) => sum + item.errorRate, 0) / virtual.length;

          document.getElementById('physicalAvgPathAccuracy').textContent = physicalAvgPathAccuracy.toFixed(1);
          document.getElementById('physicalAvgCenteredPathAccuracy').textContent = physicalAvgCenteredPathAccuracy.toFixed(1);
          document.getElementById('physicalAvgErrorRate').textContent = physicalAvgErrorRate.toFixed(1);

          document.getElementById('virtualAvgPathAccuracy').textContent = virtualAvgPathAccuracy.toFixed(1);
          document.getElementById('virtualAvgCenteredPathAccuracy').textContent = virtualAvgCenteredPathAccuracy.toFixed(1);
          document.getElementById('virtualAvgErrorRate').textContent = virtualAvgErrorRate.toFixed(1);

          let physicalCount = physical.length;
          let virtualCount = virtual.length;
          let commonCount = Math.min(physicalCount, virtualCount);

          if (commonCount === 0) {
            messageDiv.innerHTML = '<p>No overlapping data points found between Physical and Virtual datasets.</p>';
            averagesDiv.style.display = 'none';
            graphsDiv.style.display = 'none';
            return;
          }

          let filteredPhysical = physical.slice(0, commonCount);
          let filteredVirtual = virtual.slice(0, commonCount);

          let commonTimes = filteredPhysical.map(item => item.time);
          for (let i = 0; i < commonCount; i++) {
            if (filteredPhysical[i].time !== filteredVirtual[i].time) {
              messageDiv.innerHTML = '<p>Timestamps do not align between Physical and Virtual datasets.</p>';
              averagesDiv.style.display = 'none';
              graphsDiv.style.display = 'none';
              return;
            }
          }

          messageDiv.innerHTML = '';
          averagesDiv.style.display = 'block';
          graphsDiv.style.display = 'block';

          createCompareChart('comparePathAccuracyChart', 'Path Accuracy (%) over Time', filteredPhysical, filteredVirtual, 'pathAccuracy', 'normalPathAccuracy', commonTimes);
          createCompareChart('compareCenteredPathAccuracyChart', 'Centered Path Accuracy (%) over Time', filteredPhysical, filteredVirtual, 'centeredPathAccuracy', 'centeredPathAccuracy', commonTimes);
          createCompareChart('compareErrorRateChart', 'Error Rate (%) over Time', filteredPhysical, filteredVirtual, 'errorRate', 'errorRate', commonTimes);
          createCompareChart('compareLinearSpeedChart', 'Linear Speed (m/s) over Time', filteredPhysical, filteredVirtual, 'linearSpeed', 'linearSpeed', commonTimes);
          createCompareChart('compareAngularVelocityChart', 'Angular Velocity (rad/s) over Time', filteredPhysical, filteredVirtual, 'angularVelocity', 'angularVelocity', commonTimes);
        } else {
          messageDiv.innerHTML = '<p>Waiting for both Physical and Virtual data to compare.</p>';
          averagesDiv.style.display = 'none';
          graphsDiv.style.display = 'none';
        }
      }).catch(error => {
        document.getElementById('compareMessage').innerHTML = '<p>Error loading data for comparison: ' + error + '</p>';
        document.getElementById('compareAverages').style.display = 'none';
        document.getElementById('compareGraphs').style.display = 'none';
      });
    }

    function createChart(canvasId, label, data, key, backgroundColor, borderColor) {
      const ctx = document.getElementById(canvasId).getContext('2d');
      new Chart(ctx, {
        type: 'line',
        data: {
          labels: data.map(item => item.time),
          datasets: [{
            label: label,
            data: data.map(item => item[key]),
            backgroundColor: backgroundColor,
            borderColor: borderColor,
            borderWidth: 1,
            fill: true
          }]
        },
        options: {
          scales: {
            x: { title: { display: true, text: 'Time (s)' } },
            y: { title: { display: true, text: label.split(' ')[0] } }
          }
        }
      });
    }

    function createCompareChart(canvasId, label, physicalData, virtualData, physicalKey, virtualKey, commonTimes) {
      const ctx = document.getElementById(canvasId).getContext('2d');
      new Chart(ctx, {
        type: 'line',
        data: {
          labels: commonTimes,
          datasets: [
            {
              label: 'Physical ' + label,
              data: physicalData.map(item => item[physicalKey]),
              borderColor: 'rgba(255, 99, 132, 1)',
              backgroundColor: 'rgba(255, 99, 132, 0.2)',
              borderWidth: 1,
              fill: false
            },
            {
              label: 'Virtual ' + label,
              data: virtualData.map(item => item[virtualKey]),
              borderColor: 'rgba(54, 162, 235, 1)',
              backgroundColor: 'rgba(54, 162, 235, 0.2)',
              borderWidth: 1,
              fill: false
            }
          ]
        },
        options: {
          scales: {
            x: { title: { display: true, text: 'Time (s)' } },
            y: { title: { display: true, text: label.split(' ')[0] } }
          }
        }
      });
    }
  </script>
</body>
</html>
)rawliteral";
    request->send(200, "text/html", html);
  });

  // Route to get physical data
  server.on("/physical", HTTP_GET, [](AsyncWebServerRequest *request){
    Serial.println("Physical data requested: " + physicalData);
    request->send(200, "application/json", physicalData);
  });

  // Route to get virtual data
  server.on("/virtual", HTTP_GET, [](AsyncWebServerRequest *request){
    Serial.println("Virtual data requested: " + virtualData);
    request->send(200, "application/json", virtualData);
  });

  // Route to receive virtual data via POST
  server.on("/virtual", HTTP_POST, [](AsyncWebServerRequest *request){
    // Handled in onBody
  }, NULL, [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total){
    String body = "";
    for (size_t i = 0; i < len; i++) {
      body += (char)data[i];
    }

    DynamicJsonDocument doc(2048); // Increased size for incoming JSON
    DeserializationError error = deserializeJson(doc, body);
    if (error) {
      Serial.println("Parsing JSON failed: " + String(error.c_str()));
      request->send(400, "text/plain", "Invalid JSON");
      return;
    }

    // Update virtual sensor values
    if (doc.containsKey("leftSensor")) {
      virtualLeftSensor = doc["leftSensor"].as<uint8_t>();
    }
    if (doc.containsKey("rightSensor")) {
      virtualRightSensor = doc["rightSensor"].as<uint8_t>();
    }
    if (doc.containsKey("centerSensor")) {
      virtualCenterSensor = doc["centerSensor"].as<uint8_t>();
    }

    // Append to virtualData
    DynamicJsonDocument virtualDoc(8192); // Increased size for larger arrays
    error = deserializeJson(virtualDoc, virtualData);
    if (error) {
      Serial.println("Failed to parse existing virtualData: " + String(error.c_str()));
      virtualDoc.clear();
      virtualDoc.to<JsonArray>();
    }

    JsonArray virtualArray = virtualDoc.as<JsonArray>();
    JsonObject newData = virtualArray.createNestedObject();
    newData["time"] = doc["time"].as<float>();
    newData["deviation"] = doc["deviation"].as<int>();
    newData["normalPathAccuracy"] = doc["normalPathAccuracy"].as<float>();
    newData["centeredPathAccuracy"] = doc["centeredPathAccuracy"].as<float>();
    newData["errorRate"] = 100.0 - doc["normalPathAccuracy"].as<float>(); // Error rate is 100 - normal path accuracy
    newData["stability"] = doc["stability"].as<int>();
    newData["linearSpeed"] = doc["linearSpeed"].as<float>();
    newData["angularVelocity"] = doc["angularVelocity"].as<float>();
    newData["leftSensor"] = virtualLeftSensor;
    newData["rightSensor"] = virtualRightSensor;
    newData["centerSensor"] = virtualCenterSensor;

    serializeJson(virtualArray, virtualData);

    Serial.println("Received virtual data: " + body);
    Serial.println("Updated virtualData: " + virtualData);

    request->send(200, "text/plain", "Data received");
  });

  // Debug endpoint to inspect virtualData
  server.on("/debug/virtualData", HTTP_GET, [](AsyncWebServerRequest *request){
    String debugInfo = "Virtual Data: " + virtualData + "\n";
    debugInfo += "Virtual Left Sensor: " + String(virtualLeftSensor) + "\n";
    debugInfo += "Virtual Right Sensor: " + String(virtualRightSensor) + "\n";
    debugInfo += "Virtual Center Sensor: " + String(virtualCenterSensor) + "\n";
    Serial.println("Debug virtualData requested: " + virtualData);
    request->send(200, "text/plain", debugInfo);
  });

  server.begin();
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.reconnect();
    while (WiFi.status() != WL_CONNECTED) {
      delay(100);
    }
    Serial.println("Reconnected to WiFi: " + WiFi.localIP().toString());
  }

  unsigned long currentTime = millis();
  if (currentTime - lastSensorSendTime >= sensorSendInterval) {
    uint8_t left = digitalRead(IR_LEFT);
    uint8_t right = digitalRead(IR_RIGHT);
    uint8_t center = digitalRead(IR_CENTER);

    // Send only left and right sensor data to MATLAB
    uint8_t leftBuffer[1] = {left};
    if (udpSensors.beginPacket(matlabIP, leftSensorPort)) {
      udpSensors.write(leftBuffer, 1);
      udpSensors.endPacket();
    }

    uint8_t rightBuffer[1] = {right};
    if (udpSensors.beginPacket(matlabIP, rightSensorPort)) {
      udpSensors.write(rightBuffer, 1);
      udpSensors.endPacket();
    }

    // Update counts for path accuracies
    // Centered Path Accuracy: deviation = 0 if center sensor is on black path (0), else 1
    if (center == 0) {
      centeredCount++;
    }

    // Normal Path Accuracy: deviation = 0 if at least one sensor is on (1), else 1
    if (left == 1 || right == 1) {
      onPathCount++;
    }

    totalSamples++;

    lastSensorSendTime = currentTime;
  }

  if (currentTime - lastSpeedCalcTime >= 100) {
    float deltaTime = (currentTime - lastSpeedCalcTime) / 1000.0;

    float leftPulsesPerSecond = leftPulseCount / deltaTime;
    float rightPulsesPerSecond = rightPulseCount / deltaTime;
    leftWheelRPM = (leftPulsesPerSecond * 60.0) / PULSES_PER_REVOLUTION;
    rightWheelRPM = (rightPulsesPerSecond * 60.0) / PULSES_PER_REVOLUTION;

    leftWheelSpeed = (leftWheelRPM / 60.0) * WHEEL_CIRCUMFERENCE;
    rightWheelSpeed = (rightWheelRPM / 60.0) * WHEEL_CIRCUMFERENCE;

    linearSpeed = (leftWheelSpeed + rightWheelSpeed) / 2.0;
    angularVelocity = (rightWheelSpeed - leftWheelSpeed) / WHEELBASE;

    leftPulseCount = 0;
    rightPulseCount = 0;
    lastSpeedCalcTime = currentTime;
  }

  int packetSize = udpRotateR.parsePacket();
  if (packetSize) {
    char packetBuffer[8];
    int len = udpRotateR.read(packetBuffer, 7);
    if (len > 0) {
      packetBuffer[len] = '\0';
      rotateR = String(packetBuffer).toFloat();
      receivedRotateR = true;
      isConnectedToMATLAB = true;
      lastCommandTime = millis();
      rotateRPacketCount++;

      if (!simulationStarted) {
        physicalData = "[]";
        // Only reset virtualData if no virtual data has been received recently
        if (virtualData == "[]") {
          virtualData = "[]";
        }
        simulationStartTime = millis();
        simulationStarted = true;
        dataIndex = 0;
        onPathCount = 0;
        centeredCount = 0;
        totalSamples = 0;
        avgPathAccuracy = 0.0;
        avgCenteredPathAccuracy = 0.0;
        avgErrorRate = 0.0;
        avgLinearSpeed = 0.0;
        avgAngularVelocity = 0.0;
        Serial.println("Simulation started, physical data reset for a fresh start.");
      }
    }
  }

  packetSize = udpRotateL.parsePacket();
  if (packetSize) {
    char packetBuffer[8];
    int len = udpRotateL.read(packetBuffer, 7);
    if (len > 0) {
      packetBuffer[len] = '\0';
      rotateL = String(packetBuffer).toFloat();
      receivedRotateL = true;
      isConnectedToMATLAB = true;
      lastCommandTime = millis();
      rotateLPacketCount++;
    }
  }

  packetSize = udpSpeed.parsePacket();
  if (packetSize) {
    char packetBuffer[11];
    int len = udpSpeed.read(packetBuffer, 10);
    if (len > 0) {
      packetBuffer[len] = '\0';
      speed = String(packetBuffer).toFloat();
      receivedSpeed = true;
      isConnectedToMATLAB = true;
      lastCommandTime = millis();
      speedPacketCount++;
    }
  }

  if (isConnectedToMATLAB && receivedRotateR && receivedRotateL && receivedSpeed) {
    applyMotorControl();
  } else {
    setMotorSpeed(0, 0, 0, 0);
  }

  if (isConnectedToMATLAB && (millis() - lastCommandTime > commandTimeout)) {
    rotateR = 0.0;
    rotateL = 0.0;
    applyMotorControl();
    receivedRotateR = false;
    receivedRotateL = false;
    receivedSpeed = false;
    isConnectedToMATLAB = false;

    if (simulationStarted) {
      float sumPathAccuracy = 0.0, sumCenteredPathAccuracy = 0.0, sumErrorRate = 0.0;
      float sumLinearSpeed = 0.0, sumAngularVelocity = 0.0;
      for (int i = 0; i < dataIndex; i++) {
        sumPathAccuracy += performanceData[i].pathAccuracy;
        sumCenteredPathAccuracy += performanceData[i].centeredPathAccuracy;
        sumErrorRate += performanceData[i].errorRate;
        sumLinearSpeed += performanceData[i].linearSpeed;
        sumAngularVelocity += performanceData[i].angularVelocity;
      }
      avgPathAccuracy = (dataIndex > 0) ? (sumPathAccuracy / dataIndex) : 0.0;
      avgCenteredPathAccuracy = (dataIndex > 0) ? (sumCenteredPathAccuracy / dataIndex) : 0.0;
      avgErrorRate = (dataIndex > 0) ? (sumErrorRate / dataIndex) : 0.0;
      avgLinearSpeed = (dataIndex > 0) ? (sumLinearSpeed / dataIndex) : 0.0;
      avgAngularVelocity = (dataIndex > 0) ? (sumAngularVelocity / dataIndex) : 0.0;
      simulationStarted = false;
    }
  }

  if (simulationStarted && (currentTime - lastLogTime >= logInterval)) {
    unsigned long elapsedTime = (currentTime - simulationStartTime) / 1000;

    int deviation = (digitalRead(IR_LEFT) == 1 || digitalRead(IR_RIGHT) == 1) ? 0 : 1;
    float pathAccuracy = (totalSamples > 0) ? (onPathCount * 100.0 / totalSamples) : 0.0;
    float centeredPathAccuracy = (totalSamples > 0) ? (centeredCount * 100.0 / totalSamples) : 0.0;
    float errorRate = 100.0 - pathAccuracy; // Error rate is 100 - path accuracy
    int stability = (currentLeftForwardPWM > 0 || currentLeftBackwardPWM > 0 || currentRightForwardPWM > 0 || currentRightBackwardPWM > 0) ? 1 : 0;

    if (dataIndex < 60) {
      performanceData[dataIndex].time = elapsedTime;
      performanceData[dataIndex].deviation = deviation;
      performanceData[dataIndex].pathAccuracy = pathAccuracy;
      performanceData[dataIndex].centeredPathAccuracy = centeredPathAccuracy;
      performanceData[dataIndex].errorRate = errorRate;
      performanceData[dataIndex].stability = stability;
      performanceData[dataIndex].linearSpeed = linearSpeed;
      performanceData[dataIndex].angularVelocity = angularVelocity;

      DynamicJsonDocument doc(1024);
      doc["time"] = (double)elapsedTime;
      doc["deviation"] = deviation;
      doc["pathAccuracy"] = pathAccuracy;
      doc["centeredPathAccuracy"] = centeredPathAccuracy;
      doc["errorRate"] = errorRate;
      doc["stability"] = stability;
      doc["linearSpeed"] = linearSpeed;
      doc["angularVelocity"] = angularVelocity;

      DynamicJsonDocument physicalDoc(8192); // Increased size for larger arrays
      DeserializationError error = deserializeJson(physicalDoc, physicalData);
      if (error) {
        Serial.println("Failed to parse existing physicalData: " + String(error.c_str()));
        physicalDoc.clear();
        physicalDoc.to<JsonArray>();
      }

      JsonArray physicalArray = physicalDoc.as<JsonArray>();
      physicalArray.add(doc.as<JsonObject>());
      serializeJson(physicalArray, physicalData);

      dataIndex++;
    }

    onPathCount = 0;
    centeredCount = 0;
    totalSamples = 0;

    lastLogTime = currentTime;
  }
}

void setMotorSpeed(int leftForwardPWM, int leftBackwardPWM, int rightForwardPWM, int rightBackwardPWM) {
  leftForwardPWM = constrain(leftForwardPWM, 0, 255);
  leftBackwardPWM = constrain(leftBackwardPWM, 0, 255);
  rightForwardPWM = constrain(rightForwardPWM, 0, 255);
  rightBackwardPWM = constrain(rightBackwardPWM, 0, 255);

  currentLeftForwardPWM = leftForwardPWM;
  currentLeftBackwardPWM = leftBackwardPWM;
  currentRightForwardPWM = rightForwardPWM;
  currentRightBackwardPWM = rightBackwardPWM;

  analogWrite(IN1, leftForwardPWM);
  analogWrite(IN2, leftBackwardPWM);
  analogWrite(IN3, rightForwardPWM);
  analogWrite(IN4, rightBackwardPWM);
}

void applyMotorControl() {
  uint8_t leftSensor = digitalRead(IR_LEFT);
  uint8_t rightSensor = digitalRead(IR_RIGHT);

  if (leftSensor == 1 && rightSensor == 1) {
    setMotorSpeed(int(speed), 0, int(speed), 0);
  } else if (leftSensor == 0 && rightSensor == 0) {
    setMotorSpeed(0, 0, 0, 0);
  } else if (leftSensor == 1 && rightSensor == 0) {
    setMotorSpeed(int(speed), 0, 0, int(speed));
  } else if (leftSensor == 0 && rightSensor == 1) {
    setMotorSpeed(0, int(speed), int(speed), 0);
  }
}