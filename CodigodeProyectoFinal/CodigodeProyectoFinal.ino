#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <WebServer.h>
#include <PubSubClient.h>
#include <math.h>

// ---------------- CONFIGURACIÓN ----------------

// WiFi
#define WIFI_SSID      "GUAMAL"
#define WIFI_PASSWORD  "Mariposa2641+*"

// Broker MQTT (HiveMQ público)
#define MQTT_SERVER "broker.hivemq.com"
#define MQTT_PORT   8883   // MQTT sobre TLS

// Tópicos
#define MQTT_TOPIC_INSTRUCTIONS "car/instructions"
#define MQTT_TOPIC_SENSOR       "car/sensor"

// Motores (puente H, ENA/ENB fijos a VCC en el módulo)
#define IN1 32
#define IN2 33
#define IN3 26
#define IN4 27

// Sensor HC-SR04
#define TRIG_PIN 13
#define ECHO_PIN 36

// true = simulado, false = HC-SR04 real
#define SENSOR_SIMULADO false

// *** CALIBRACIÓN DE MOVIMIENTO / MAPA ***

// Distancia lógica que el mapa asume por cada "paso" (un comando adelante/atrás)
const float ODO_STEP_CM       = 10.0f;   // Ajusta si quieres que el mapa use otra distancia

// Tiempo real que el motor está activo para intentar recorrer ODO_STEP_CM
// Si ves que en la vida real se mueve más de 10 cm, BAJA este valor.
// Si se mueve menos de 10 cm, SÚBELO.
const unsigned long FORWARD_STEP_MS   = 350;    // ms adelante
const unsigned long BACKWARD_STEP_MS  = 350;    // ms atrás

// Para bloqueo de pared (cuando la distancia es <= este margen)
const float BLOCK_MARGIN_CM   = 10.0f;  // si el sensor mide <= 10cm no avanza

// GIRO ~90°
const int TURN_MS_90 = 420;             // ms para girar ~90° (ya lo tenías calibrado)

// Distancia mínima válida para considerar una pared / obstáculo
const float MIN_VALID_DIST_CM = 1.0f;

// ---------------- CERTIFICADO CA (TLS MQTT) ----------------
// Usamos setInsecure() más abajo, así que este CA no se usa realmente.
// Lo dejamos por si luego quieres poner el certificado real de HiveMQ.

const char* ca_cert = R"EOF(-----BEGIN CERTIFICATE-----
MIIDQTCCAimgAwIBAgITBmyfz5m/jAo54vB4ikPmljZbyjANBgkqhkiG9w0BAQsF
ADA5MQswCQYDVQQGEwJVUzEPMA0GA1UEChMGQW1hem9uMRkwFwYDVQQDExBBbWF6
b24gUm9vdCBDQSAxMB4XDTE1MDUyNjAwMDAwMFoXDTM4MDExNzAwMDAwMFowOTEL
MAkGA1UEBhMCVVMxDzANBgNVBAoTBkFtYXpvbjEZMBcGA1UEAxMQQW1hem9uIFJv
b3QgQ0EgMTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBALJ4gHHKeNXj
ca9HgFB0fW7Y14h29Jlo91ghYPl0hAEvrAIthtOgQ3pOsqTQNroBvo3bSMgHFzZM
9O6II8c+6zf1tRn4SWiw3te5djgdYZ6k/oI2peVKVuRF4fn9tBb6dNqcmzU5L/qw
IFAGbHrQgLKm+a/sRxmPUDgH3KKHOVj4utWp+UhnMJbulHheb4mjUcAwhmahRWa6
VOujw5H5SNz/0egwLX0tdHA114gk957EWW67c4cX8jJGKLhD+rcdqsq08p8kDi1L
93FcXmn/6pUCyziKrlA4b9v7LWIbxcceVOF34GfID5yHI9Y/QCB/IIDEgEw+OyQm
jgSubJrIqg0CAwEAAaNCMEAwDwYDVR0TAQH/BAUwAwEB/zAOBgNVHQ8BAf8EBAMC
AYYwHQYDVR0OBBYEFIQYzIU07LwMlJQuCFmcx7IQTgoIMA0GCSqGSIb3DQEBCwUA
A4IBAQCY8jdaQZChGsV2USggNiMOruYou6r4lK5IpDB/G/wkjUu0yKG9rbxenDI
U5PMCCjjmCXPI6T53iHTfIUJrU6adTrCC2qJeHZERxhlbI1Bjjt/msv0tadQ1wUs
N+gDS63pYaACbvXy8MWy7Vu33PqUXHeeE6V/Uq2V8viTO96LXFvKWlJbYK8U90vv
o/ufQJVtMVT8QtPHRh8jrdkPSHCa2XV4cdFyQzR1bldZwgJcJmApzyMZFo6IQ6XU
5MsI+yMRQ+hDKXJioaldXgjUkK642M4UwtBV8ob2xJNDd2ZhwLnoQdeXeGADbkpy
rqXRfboQnoZsG4q5WTP468SQvvG5
-----END CERTIFICATE-----)EOF";

// ---------------- OBJETOS ----------------

WiFiClientSecure secureClient;
PubSubClient mqttClient(secureClient);
WebServer server(80);

// flag: true cuando los motores están en movimiento
bool isMoving = false;
// tiempo a partir del cual el sensor puede volver a publicar
unsigned long sensorResumeTime = 0;

// ---------------- ESTADO DEL CARRO / MAPA ----------------

float posX = 0.0f;
float posY = 0.0f;
float headingDeg = 0.0f;   // 0° = hacia +Y

const int MAX_POINTS = 300;
float obsX[MAX_POINTS];
float obsY[MAX_POINTS];
int   obsCount = 0;

float lastObsX = NAN;
float lastObsY = NAN;
float lastObsAngle = NAN;

unsigned long lastSensorPublish = 0;

// ---------------- PÁGINA HTML (WALL-E, sin duración, con brújula y más zoom) ----------------

const char MAIN_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8" />
  <title>WALL-E</title>
  <style>
    body {
      font-family: Arial, sans-serif;
      background: #111;
      color: #eee;
      text-align: center;
    }
    #map {
      border: 1px solid #555;
      background: #000;
      margin-top: 10px;
    }
    .controls {
      margin-top: 10px;
    }
    .btn {
      padding: 6px 12px;
      margin: 3px;
      border: 1px solid #555;
      background: #222;
      color: #eee;
      cursor: pointer;
    }
    .btn:hover {
      background: #333;
    }
    #mqttStatus {
      margin-top: 8px;
      font-size: 0.9rem;
      color: #0f0;
    }
    #alertMessage {
      margin-top: 6px;
      font-size: 0.95rem;
      color: #ff6666;
      min-height: 1.2em;
    }
    .layout {
      display: flex;
      justify-content: center;
      gap: 20px;
      flex-wrap: wrap;
      margin-top: 10px;
    }
  </style>
</head>
<body>
  <h2>WALL-E</h2>

  <div class="controls">
    <!-- Quitamos input de duración: usamos pasos fijos en el backend -->
    <button class="btn" onclick="sendMove('forward')">⬆ Adelante</button><br />
    <button class="btn" onclick="sendMove('left')">⬅ Izquierda</button>
    <button class="btn" onclick="sendMove('stop')">⏹ Stop</button>
    <button class="btn" onclick="sendMove('right')">➡ Derecha</button><br />
    <button class="btn" onclick="sendMove('backward')">⬇ Atrás</button>
  </div>

  <!-- Distancia frontal justo debajo de los controles -->
  <p>Distancia frontal: <span id="dist">--.-</span> cm</p>

  <!-- Botón para mostrar / ocultar líneas de medición -->
  <button class="btn" id="toggleRaysBtn" onclick="toggleRays()">
    Mostrar líneas de medición: OFF
  </button>

  <p id="lastMove">Último movimiento: ninguno</p>
  <p id="mqttStatus">MQTT: inicializando...</p>
  <p id="alertMessage"></p>

  <div class="layout">
    <div>
      <canvas id="map" width="600" height="600"></canvas>
      <p>Posición: <span id="pos">cargando...</span></p>
    </div>
    <div>
      <canvas id="compass" width="140" height="140"></canvas>
      <p>Orientación: <span id="headingText">0° (N)</span></p>
    </div>
  </div>

  <script>
    var SENSOR_TOPIC = "car/sensor";

    var canvas = document.getElementById("map");
    var ctx = canvas.getContext("2d");
    var posSpan = document.getElementById("pos");
    var distSpan = document.getElementById("dist");
    var lastMoveSpan = document.getElementById("lastMove");
    var mqttStatusSpan = document.getElementById("mqttStatus");
    var alertSpan = document.getElementById("alertMessage");

    var compass = document.getElementById("compass");
    var cctx = compass.getContext("2d");
    var headingTextSpan = document.getElementById("headingText");

    var obstacles = [];
    var robot = { x: 0, y: 0, heading: 0 };

    var showRays = false;

    var ws = null;
    var mqttReady = false;
    var reconnectTimeout = null;
    var clientId = "webcar-" + Math.random().toString(16).substr(2, 8);
    var nextPacketId = 1;

    function encodeString(str) {
      var len = str.length;
      var bytes = new Uint8Array(2 + len);
      bytes[0] = (len >> 8) & 0xFF;
      bytes[1] = len & 0xFF;
      for (var i = 0; i < len; i++) {
        bytes[2 + i] = str.charCodeAt(i);
      }
      return bytes;
    }

    function stringToBytes(str) {
      var len = str.length;
      var bytes = new Uint8Array(len);
      for (var i = 0; i < len; i++) {
        bytes[i] = str.charCodeAt(i);
      }
      return bytes;
    }

    function concatArrays(list) {
      var total = 0;
      for (var i = 0; i < list.length; i++) total += list[i].length;
      var result = new Uint8Array(total);
      var offset = 0;
      for (var j = 0; j < list.length; j++) {
        result.set(list[j], offset);
        offset += list[j].length;
      }
      return result;
    }

    function encodeRemainingLength(length) {
      var enc = [];
      do {
        var digit = length % 128;
        length = Math.floor(length / 128);
        if (length > 0) digit |= 0x80;
        enc.push(digit);
      } while (length > 0);
      return new Uint8Array(enc);
    }

    function decodeRemainingLength(bytes, pos) {
      var multiplier = 1;
      var value = 0;
      var digit;
      do {
        digit = bytes[pos++];
        value += (digit & 127) * multiplier;
        multiplier *= 128;
      } while ((digit & 128) != 0);
      return { value: value, next: pos };
    }

    function bytesToString(bytes) {
      var s = "";
      for (var i = 0; i < bytes.length; i++) s += String.fromCharCode(bytes[i]);
      return s;
    }

    function getPacketId() {
      nextPacketId++;
      if (nextPacketId > 65535) nextPacketId = 1;
      return nextPacketId;
    }

    function sendConnectPacket() {
      if (!ws || ws.readyState !== 1) return;
      var protocolName = encodeString("MQTT");
      var vhRest = new Uint8Array(4);
      vhRest[0] = 4;
      vhRest[1] = 2;
      vhRest[2] = 0;
      vhRest[3] = 60;
      var varHeader = concatArrays([protocolName, vhRest]);
      var payload = encodeString(clientId);
      var remLen = varHeader.length + payload.length;
      var rem = encodeRemainingLength(remLen);
      var header = new Uint8Array(1 + rem.length);
      header[0] = 0x10;
      header.set(rem, 1);
      var packet = concatArrays([header, varHeader, payload]);
      ws.send(packet);
    }

    function sendSubscribe(topic, qos) {
      if (!ws || ws.readyState !== 1) return;
      var topicBytes = encodeString(topic);
      var pid = getPacketId();
      var varHeader = new Uint8Array(2);
      varHeader[0] = (pid >> 8) & 0xff;
      varHeader[1] = pid & 0xff;
      var payload = concatArrays([topicBytes, new Uint8Array([qos || 0])]);
      var remLen = varHeader.length + payload.length;
      var rem = encodeRemainingLength(remLen);
      var header = new Uint8Array(1 + rem.length);
      header[0] = 0x82;
      header.set(rem, 1);
      var packet = concatArrays([header, varHeader, payload]);
      ws.send(packet);
    }

    function sendPing() {
      if (!ws || ws.readyState !== 1 || !mqttReady) return;
      var packet = new Uint8Array(2);
      packet[0] = 0xC0;
      packet[1] = 0x00;
      ws.send(packet);
    }

    function handleMqttMessage(data) {
      if (!(data instanceof ArrayBuffer)) return;
      var bytes = new Uint8Array(data);
      var pos = 0;
      var header = bytes[pos++];
      var packetType = header >> 4;
      var rl = decodeRemainingLength(bytes, pos);
      var remLen = rl.value;
      pos = rl.next;

      if (packetType === 2) {
        if (remLen >= 2) {
          pos++;
          var rc = bytes[pos++];
          if (rc === 0) {
            mqttReady = true;
            mqttStatusSpan.textContent = "MQTT: conectado (wss)";
            sendSubscribe(SENSOR_TOPIC, 0);
          } else {
            mqttStatusSpan.textContent = "MQTT: CONNACK rc=" + rc;
          }
        }
      } else if (packetType === 3) {
        var topicLen = (bytes[pos] << 8) + bytes[pos + 1];
        pos += 2;
        var topicBytes = bytes.subarray(pos, pos + topicLen);
        pos += topicLen;
        var topic = bytesToString(topicBytes);

        var payloadLen = remLen - (2 + topicLen);
        if (payloadLen < 0) payloadLen = 0;
        var payloadBytes = bytes.subarray(pos, pos + payloadLen);
        var payload = bytesToString(payloadBytes);

        if (topic === SENSOR_TOPIC) {
          try {
            var dataObj = JSON.parse(payload);

            if (typeof dataObj.distance_cm === "number") {
              if (dataObj.distance_cm < 0) {
                distSpan.textContent = "--.-";
              } else {
                distSpan.textContent = dataObj.distance_cm.toFixed(1);
              }
            }

            if (dataObj.robot) robot = dataObj.robot;

            if (dataObj.obstacle) {
              obstacles.push(dataObj.obstacle);
            }

            if (dataObj.blocked) {
              alertSpan.textContent =
                dataObj.message || "No puedes avanzar: hay un objeto al frente.";
            } else if (alertSpan.textContent !== "") {
              alertSpan.textContent = "";
            }

            drawMap();
          } catch (e) {
            console.log("Error JSON sensor:", e, payload);
          }
        }
      }
    }

    function connectMqtt() {
      if (ws && (ws.readyState === 0 || ws.readyState === 1)) return;
      mqttReady = false;
      mqttStatusSpan.textContent = "MQTT: conectando (wss)...";
      try {
        ws = new WebSocket("wss://broker.hivemq.com:8884/mqtt");
      } catch (e) {
        console.error("Error creando WebSocket", e);
        mqttStatusSpan.textContent = "MQTT: error WebSocket";
        return;
      }

      ws.binaryType = "arraybuffer";

      ws.onopen = function () {
        sendConnectPacket();
      };
      ws.onclose = function () {
        mqttReady = false;
        mqttStatusSpan.textContent = "MQTT: desconectado (reintentando...)";
        ws = null;
        if (reconnectTimeout === null) {
          reconnectTimeout = setTimeout(function () {
            reconnectTimeout = null;
            connectMqtt();
          }, 3000);
        }
      };
      ws.onerror = function (e) {
        console.error("WS error", e);
        mqttStatusSpan.textContent = "MQTT: error";
      };
      ws.onmessage = function (event) {
        handleMqttMessage(event.data);
      };
    }

    setInterval(sendPing, 30000);

    // Enviar movimiento al API REST: usamos duración fija = 1 paso (stop = 0)
    function sendMove(dir) {
      var duration = 1;
      if (dir === "stop") duration = 0;

      var body =
        "direction=" + encodeURIComponent(dir) +
        "&duration=" + encodeURIComponent(duration);

      fetch("/api/v1/move", {
        method: "POST",
        headers: { "Content-Type": "application/x-www-form-urlencoded" },
        body: body
      })
      .then(r => r.json())
      .then(data => {
        lastMoveSpan.textContent =
          "Último movimiento: " + data.direction + " (pasos=" + data.duration + ")";
      })
      .catch(err => {
        console.error("Error en /api/v1/move:", err);
        lastMoveSpan.textContent = "Error enviando movimiento";
      });
    }

    window.sendMove = sendMove;

    function toggleRays() {
      showRays = !showRays;
      var btn = document.getElementById("toggleRaysBtn");
      btn.textContent = showRays
        ? "Mostrar líneas de medición: ON"
        : "Mostrar líneas de medición: OFF";
      drawMap();
    }

    window.toggleRays = toggleRays;

    function headingToCardinal(h) {
      var a = ((h % 360) + 360) % 360;
      if (a >= 315 || a < 45) return "N";
      if (a >= 45 && a < 135) return "E";
      if (a >= 135 && a < 225) return "S";
      return "O"; // Oeste
    }

    function drawCompass() {
      var w = compass.width;
      var h = compass.height;
      var cx = w / 2;
      var cy = h / 2;
      var r = 50;

      cctx.clearRect(0, 0, w, h);

      cctx.fillStyle = "#111";
      cctx.fillRect(0, 0, w, h);

      cctx.strokeStyle = "#555";
      cctx.beginPath();
      cctx.arc(cx, cy, r, 0, Math.PI * 2);
      cctx.stroke();

      cctx.fillStyle = "#eee";
      cctx.font = "12px Arial";
      cctx.textAlign = "center";
      cctx.textBaseline = "middle";
      cctx.fillText("N", cx, cy - r + 10);
      cctx.fillText("S", cx, cy + r - 10);
      cctx.fillText("O", cx - r + 10, cy);
      cctx.fillText("E", cx + r - 10, cy);

      var ang = robot.heading * Math.PI / 180;
      var len = 35;
      var dx = Math.sin(ang);
      var dy = -Math.cos(ang); // negativo porque en canvas Y crece hacia abajo

      var ex = cx + dx * len;
      var ey = cy + dy * len;

      cctx.strokeStyle = "#0ff";
      cctx.beginPath();
      cctx.moveTo(cx, cy);
      cctx.lineTo(ex, ey);
      cctx.stroke();

      cctx.fillStyle = "#0ff";
      cctx.beginPath();
      cctx.arc(ex, ey, 3, 0, Math.PI * 2);
      cctx.fill();

      var card = headingToCardinal(robot.heading);
      headingTextSpan.textContent =
        robot.heading.toFixed(0) + "° (" + card + ")";
    }

    function drawMap() {
      var w = canvas.width;
      var h = canvas.height;
      ctx.fillStyle = "#000";
      ctx.fillRect(0, 0, w, h);

      var minX = -150, maxX = 150, minY = -150, maxY = 150;

      for (var i = 0; i < obstacles.length; i++) {
        var p = obstacles[i];
        if (p.x < minX) minX = p.x;
        if (p.x > maxX) maxX = p.x;
        if (p.y < minY) minY = p.y;
        if (p.y > maxY) maxY = p.y;
      }
      if (robot.x < minX) minX = robot.x;
      if (robot.x > maxX) maxX = robot.x;
      if (robot.y < minY) minY = robot.y;
      if (robot.y > maxY) maxY = robot.y;

      var margin = 20;
      var scaleX = (w - 2 * margin) / (maxX - minX || 1);
      var scaleY = (h - 2 * margin) / (maxY - minY || 1);
      var scale = (scaleX < scaleY) ? scaleX : scaleY;

      // 🔍 Un poco de zoom extra
      scale *= 1.4;

      function toScreen(px, py) {
        var sx = (px - minX) * scale + margin;
        var sy = h - ((py - minY) * scale + margin);
        return [sx, sy];
      }

      ctx.strokeStyle = "#333";
      ctx.beginPath();
      ctx.moveTo(margin, h / 2);
      ctx.lineTo(w - margin, h / 2);
      ctx.moveTo(w / 2, margin);
      ctx.lineTo(w / 2, h - margin);
      ctx.stroke();

      ctx.strokeStyle = "#0f0";
      var wallHalfLen = 8;
      for (var j = 0; j < obstacles.length; j++) {
        var ob = obstacles[j];
        var angDeg = (typeof ob.angle === "number") ? ob.angle : 0;
        var ang = (angDeg * Math.PI) / 180;
        var sx1, sy1, sx2, sy2;

        if (Math.abs(Math.sin(ang)) >= Math.abs(Math.cos(ang))) {
          var x = ob.x;
          var y0 = ob.y - wallHalfLen;
          var y1 = ob.y + wallHalfLen;
          var s1 = toScreen(x, y0);
          var s2 = toScreen(x, y1);
          sx1 = s1[0]; sy1 = s1[1];
          sx2 = s2[0]; sy2 = s2[1];
        } else {
          var y = ob.y;
          var x0 = ob.x - wallHalfLen;
          var x1 = ob.x + wallHalfLen;
          var s1h = toScreen(x0, y);
          var s2h = toScreen(x1, y);
          sx1 = s1h[0]; sy1 = s1h[1];
          sx2 = s2h[0]; sy2 = s2h[1];
        }

        ctx.beginPath();
        ctx.moveTo(sx1, sy1);
        ctx.lineTo(sx2, sy2);
        ctx.stroke();
      }

      if (showRays) {
        ctx.strokeStyle = "#0ff";
        ctx.fillStyle = "#0ff";
        ctx.font = "10px Arial";

        for (var k = 0; k < obstacles.length; k++) {
          var ob2 = obstacles[k];
          if (typeof ob2.origin_x !== "number" ||
              typeof ob2.origin_y !== "number") {
            continue;
          }
          var sOrigin = toScreen(ob2.origin_x, ob2.origin_y);
          var sWall   = toScreen(ob2.x, ob2.y);

          ctx.beginPath();
          ctx.moveTo(sOrigin[0], sOrigin[1]);
          ctx.lineTo(sWall[0], sWall[1]);
          ctx.stroke();

          var mx = (sOrigin[0] + sWall[0]) / 2;
          var my = (sOrigin[1] + sWall[1]) / 2;
          var d = (typeof ob2.ray_cm === "number") ? ob2.ray_cm : null;
          if (d !== null) {
            ctx.fillText(d.toFixed(1) + "cm", mx + 3, my - 3);
          }
        }
      }

      var angle = (robot.heading * Math.PI) / 180;
      var len = 6;
      var halfLen = 3;
      var halfWidth = 2.5;
      var vx = Math.sin(angle);
      var vy = Math.cos(angle);
      var nx = -vy;
      var ny = vx;

      var fx = robot.x + vx * len;
      var fy = robot.y + vy * len;
      var lbx = robot.x - vx * halfLen + nx * halfWidth;
      var lby = robot.y - vy * halfLen + ny * halfWidth;
      var rbx = robot.x - vx * halfLen - nx * halfWidth;
      var rby = robot.y - vy * halfLen - ny * halfWidth;

      var fScr = toScreen(fx, fy);
      var lbScr = toScreen(lbx, lby);
      var rbScr = toScreen(rbx, rby);

      ctx.fillStyle = "#ff0";
      ctx.beginPath();
      ctx.moveTo(fScr[0], fScr[1]);
      ctx.lineTo(lbScr[0], lbScr[1]);
      ctx.lineTo(rbScr[0], rbScr[1]);
      ctx.closePath();
      ctx.fill();

      posSpan.textContent =
        "x=" + robot.x.toFixed(1) + "cm, y=" + robot.y.toFixed(1) +
        "cm, heading=" + robot.heading.toFixed(0) + "°";

      drawCompass();
    }

    drawMap();
    connectMqtt();
  </script>
</body>
</html>
)rawliteral";

// ---------------- MOTORES / SENSOR (C++) ----------------

void stopMotores() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void adelante() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void atras() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void izquierda() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void derecha() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

float leerDistancia() {
  if (SENSOR_SIMULADO) {
    return random(10, 200) + random(0, 99) / 100.0f;
  } else {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    long duration = pulseIn(ECHO_PIN, HIGH, 20000);
    float distance = duration * 0.034f / 2.0f;
    return distance;
  }
}

void agregarPuntoMapa(float distanciaCm) {
  if (distanciaCm < MIN_VALID_DIST_CM) return;

  float rad = headingDeg * PI / 180.0f;
  float ox = posX + distanciaCm * sin(rad);
  float oy = posY + distanciaCm * cos(rad);
  lastObsX = ox;
  lastObsY = oy;
  lastObsAngle = headingDeg;
  if (obsCount >= MAX_POINTS) return;
  obsX[obsCount] = ox;
  obsY[obsCount] = oy;
  obsCount++;
}

void actualizarPosicion(const String& direction, int steps) {
  float distance = ODO_STEP_CM * steps;
  float rad = headingDeg * PI / 180.0f;

  if (direction == "forward") {
    posX += distance * sin(rad);
    posY += distance * cos(rad);
  } else if (direction == "backward") {
    posX -= distance * sin(rad);
    posY -= distance * cos(rad);
  } else if (direction == "left") {
    headingDeg += 90.0f;
    if (headingDeg >= 360.0f) headingDeg -= 360.0f;
  } else if (direction == "right") {
    headingDeg -= 90.0f;
    if (headingDeg < 0.0f) headingDeg += 360.0f;
  }
}

// Publica sensor + mapa (no mide mientras se mueve)
void publicarSensor() {
  unsigned long now = millis();

  if (isMoving) return;
  if (now < sensorResumeTime) return;
  if (now - lastSensorPublish < 3000) return;
  lastSensorPublish = now;

  float distancia = leerDistancia();
  float originX = posX;
  float originY = posY;

  String payload = "{";

  if (distancia < MIN_VALID_DIST_CM) {
    payload += "\"distance_cm\":-1";
    payload += ",\"robot\":{\"x\":" + String(posX, 2) +
               ",\"y\":" + String(posY, 2) +
               ",\"heading\":" + String(headingDeg, 1) + "}";
    payload += "}";
  } else {
    agregarPuntoMapa(distancia);
    payload += "\"distance_cm\":" + String(distancia, 2);
    payload += ",\"robot\":{\"x\":" + String(posX, 2) +
               ",\"y\":" + String(posY, 2) +
               ",\"heading\":" + String(headingDeg, 1) + "}";
    payload += ",\"obstacle\":{";
    payload += "\"x\":" + String(lastObsX, 2);
    payload += ",\"y\":" + String(lastObsY, 2);
    payload += ",\"angle\":" + String(lastObsAngle, 1);
    payload += ",\"origin_x\":" + String(originX, 2);
    payload += ",\"origin_y\":" + String(originY, 2);
    payload += ",\"ray_cm\":" + String(distancia, 2);
    payload += "}";
    payload += "}";
  }

  if (mqttClient.connected()) {
    mqttClient.publish(MQTT_TOPIC_SENSOR, payload.c_str());
    Serial.println("MQTT SENSOR -> " + payload);
  }
}

// --------- EJECUCIÓN DE MOVIMIENTOS ---------

bool executeMove(const String& dir, int steps) {
  if (steps < 0) steps = 0;
  if (steps > 5) steps = 5;

  if (dir == "stop") {
    stopMotores();
    isMoving = false;
    sensorResumeTime = millis() + 500;
    return false;
  }

  // Giros: ignoramos steps, siempre 90°
  if (dir == "left" || dir == "right") {
    isMoving = true;
    if (dir == "left") izquierda();
    else               derecha();

    delay(TURN_MS_90);
    stopMotores();
    isMoving = false;
    sensorResumeTime = millis() + 500;
    actualizarPosicion(dir, 1);
    return true;
  }

  // Adelante: chequeo de bloqueo
  if (dir == "forward") {
    float dist = leerDistancia();
    float originX = posX;
    float originY = posY;

    if (dist >= MIN_VALID_DIST_CM) {
      agregarPuntoMapa(dist);

      if (dist <= BLOCK_MARGIN_CM) {
        String payload = "{";
        payload += "\"distance_cm\":" + String(dist, 2);
        payload += ",\"robot\":{\"x\":" + String(posX, 2) +
                   ",\"y\":" + String(posY, 2) +
                   ",\"heading\":" + String(headingDeg, 1) + "}";
        payload += ",\"obstacle\":{";
        payload += "\"x\":" + String(lastObsX, 2);
        payload += ",\"y\":" + String(lastObsY, 2);
        payload += ",\"angle\":" + String(lastObsAngle, 1);
        payload += ",\"origin_x\":" + String(originX, 2);
        payload += ",\"origin_y\":" + String(originY, 2);
        payload += ",\"ray_cm\":" + String(dist, 2);
        payload += "}";
        payload += ",\"blocked\":true";
        payload += ",\"message\":\"No puedes avanzar: objeto al frente a ";
        payload += String(dist, 1);
        payload += " cm\"";
        payload += "}";

        if (mqttClient.connected()) {
          mqttClient.publish(MQTT_TOPIC_SENSOR, payload.c_str());
          Serial.println("MQTT BLOQUEO -> " + payload);
        }
        Serial.println("Movimiento bloqueado: obstáculo demasiado cerca");
        return false;
      }
    }

    // Si pasó el bloqueo, avanzamos steps veces
    isMoving = true;
    for (int i = 0; i < steps; ++i) {
      adelante();
      delay(FORWARD_STEP_MS);
      stopMotores();
      actualizarPosicion(dir, 1);
    }
    isMoving = false;
    sensorResumeTime = millis() + 500;
    return true;
  }

  // Atrás: sin sensor frontal
  if (dir == "backward") {
    isMoving = true;
    for (int i = 0; i < steps; ++i) {
      atras();
      delay(BACKWARD_STEP_MS);
      stopMotores();
      actualizarPosicion(dir, 1);
    }
    isMoving = false;
    sensorResumeTime = millis() + 500;
    return true;
  }

  stopMotores();
  isMoving = false;
  sensorResumeTime = millis() + 500;
  return false;
}

// ---------------- MQTT CALLBACK ----------------

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String topicStr = String(topic);
  String msg;
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];

  Serial.print("MQTT [");
  Serial.print(topicStr);
  Serial.print("] ");
  Serial.println(msg);

  if (topicStr == MQTT_TOPIC_INSTRUCTIONS) {
    int sep = msg.indexOf('|');
    if (sep > 0) {
      String dir = msg.substring(0, sep);
      int duration = msg.substring(sep + 1).toInt();
      (void)executeMove(dir, duration);
    }
  }
}

// ---------------- WIFI / MQTT ----------------

void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Conectando a WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("WiFi conectado. IP: ");
  Serial.println(WiFi.localIP());
}

void reconnectMQTT() {
  while (!mqttClient.connected()) {
    Serial.print("Conectando a MQTT (TLS)...");
    if (mqttClient.connect("ESP32CarTLS")) {
      Serial.println("Conectado!");
      mqttClient.subscribe(MQTT_TOPIC_INSTRUCTIONS);
      Serial.println("Suscrito a " MQTT_TOPIC_INSTRUCTIONS);
    } else {
      Serial.print("Fallo rc=");
      Serial.print(mqttClient.state());
      Serial.println(" intentando de nuevo en 2s...");
      delay(2000);
    }
  }
}

// ---------------- HTTP HANDLERS ----------------

void handleRootPage() {
  server.send_P(200, "text/html", MAIN_PAGE);
}

void handleStatus() {
  server.send(200, "application/json", "{\"status\":\"online\"}");
}

void handleHealthcheck() {
  bool wifi = (WiFi.status() == WL_CONNECTED);
  bool mqtt = mqttClient.connected();
  String json = "{";
  json += "\"status\":\"ok\"";
  json += ",\"wifi_connected\":"; json += wifi ? "true" : "false";
  json += ",\"mqtt_connected\":"; json += mqtt ? "true" : "false";
  json += ",\"ip\":\"" + WiFi.localIP().toString() + "\"";
  json += ",\"uptime_ms\":" + String(millis());
  json += "}";
  server.send(200, "application/json", json);
}

void handleMoveAPI() {
  if (!server.hasArg("direction") || !server.hasArg("duration")) {
    server.send(400, "application/json",
                "{\"error\":\"Missing direction or duration\"}");
    return;
  }

  String direction = server.arg("direction");
  int duration = server.arg("duration").toInt();

  // API → manda instrucción por MQTT (la ESP actúa al recibir del broker)
  String msg = direction + "|" + String(duration);
  bool published = false;
  if (mqttClient.connected()) {
    published = mqttClient.publish(MQTT_TOPIC_INSTRUCTIONS, msg.c_str());
  }

  String json = "{";
  json += "\"direction\":\"" + direction + "\"";
  json += ",\"duration\":" + String(duration);
  json += ",\"published_to_mqtt\":"; json += published ? "true" : "false";
  json += "}";
  server.send(200, "application/json", json);
}

// ---------------- SETUP / LOOP ----------------

void setup() {
  Serial.begin(115200);
  randomSeed(analogRead(0));

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  stopMotores();

  if (!SENSOR_SIMULADO) {
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
  }

  connectWiFi();

  // Para no fallar por CA incorrecto
  secureClient.setInsecure();
  // Si luego quieres usar CA real:
  // secureClient.setCACert(ca_cert);

  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);
  reconnectMQTT();

  server.on("/", handleRootPage);
  server.on("/status", handleStatus);
  server.on("/api/v1/healthcheck", HTTP_GET, handleHealthcheck);
  server.on("/api/v1/move", HTTP_ANY, handleMoveAPI);

  server.begin();
  Serial.println("Servidor HTTP listo!");
}

void loop() {
  if (!mqttClient.connected()) {
    reconnectMQTT();
  }
  mqttClient.loop();
  server.handleClient();
  publicarSensor();
}
