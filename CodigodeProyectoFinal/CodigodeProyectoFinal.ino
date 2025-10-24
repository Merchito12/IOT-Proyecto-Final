#include <WiFi.h>
#include <WebServer.h>
#include <PubSubClient.h>

// ====== CONFIGURACIONES GLOBALES ======

// ---- WiFi ----
#define WIFI_SSID "ChikiNet"
#define WIFI_PASSWORD "12345678"

// ---- MQTT ----
#define MQTT_SERVER "broker.hivemq.com"
#define MQTT_PORT 1883
#define MQTT_TOPIC_INSTRUCTIONS "car/instructions"
#define MQTT_TOPIC_SENSOR "car/sensor"

// ---- Pines motores (puente H) ----
#define ENA 32
#define IN1 25
#define IN2 26
#define ENB 33
#define IN3 14
#define IN4 27

// ---- Pines sensor (si se usa físico) ----
#define TRIG_PIN 5
#define ECHO_PIN 18

// ---- Configuración de simulación ----
#define SENSOR_SIMULADO true   

// ====== OBJETOS ======
WiFiClient espClient;
PubSubClient client(espClient);
WebServer server(80);

// ====== FUNCIONES DE MOVIMIENTO ======
void stopMotores() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

void adelante(int velocidad) {
  analogWrite(ENA, velocidad);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENB, velocidad);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void atras(int velocidad) {
  analogWrite(ENA, velocidad);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENB, velocidad);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void izquierda(int velocidad) {
  analogWrite(ENA, velocidad);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENB, velocidad);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void derecha(int velocidad) {
  analogWrite(ENA, velocidad);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENB, velocidad);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

// ====== SIMULACIÓN / SENSOR REAL ======
float leerDistancia() {
  if (SENSOR_SIMULADO) {
    // Genera un número aleatorio entre 10 y 200 cm
    float simulated = random(10, 200) + random(0, 99) / 100.0;
    return simulated;
  } else {
    // Modo real con HC-SR04
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    long duration = pulseIn(ECHO_PIN, HIGH, 20000);
    float distance = duration * 0.034 / 2;
    return distance;
  }
}

// ====== PUBLICACIÓN SENSOR ======
unsigned long lastSensorPublish = 0;
void publicarSensor() {
  if (millis() - lastSensorPublish > 3000) {  // cada 3 segundos
    float distancia = leerDistancia();
    String payload = "{\"distance_cm\":" + String(distancia, 2) + "}";
    client.publish(MQTT_TOPIC_SENSOR, payload.c_str());
    Serial.println("MQTT Sensor -> " + payload);
    lastSensorPublish = millis();
  }
}

// ====== HANDLERS HTTP ======
void handleMove() {
  if (!server.hasArg("direction") || !server.hasArg("speed") || !server.hasArg("duration")) {
    server.send(400, "text/plain", "Missing parameters");
    return;
  }

  String direction = server.arg("direction");
  int speed = server.arg("speed").toInt();
  int duration = server.arg("duration").toInt();
  if (duration > 5) duration = 5;

  String clientIP = server.client().remoteIP().toString();
  Serial.printf("Movimiento: %s Vel: %d Dur: %d Cliente: %s\n",
                direction.c_str(), speed, duration, clientIP.c_str());

  if (direction == "forward") adelante(speed);
  else if (direction == "backward") atras(speed);
  else if (direction == "left") izquierda(speed);
  else if (direction == "right") derecha(speed);
  else stopMotores();

  String payload = "{\"direction\":\"" + direction + "\",\"speed\":" + String(speed) +
                   ",\"duration\":" + String(duration) + ",\"client_ip\":\"" + clientIP + "\"}";
  client.publish(MQTT_TOPIC_INSTRUCTIONS, payload.c_str());
  Serial.println("MQTT Movimiento -> " + payload);

  delay(duration * 1000);
  stopMotores();

  server.send(200, "application/json", "{\"status\":\"ok\"}");
}

void handleStatus() {
  server.send(200, "application/json", "{\"status\":\"online\"}");
}

// ====== CONEXIÓN MQTT ======
void reconnectMQTT() {
  while (!client.connected()) {
    Serial.print("Conectando a MQTT...");
    if (client.connect("ESP32Car")) {
      Serial.println("Conectado a MQTT!");
    } else {
      Serial.print("Fallo, rc=");
      Serial.print(client.state());
      delay(2000);
    }
  }
}

// ====== SETUP ======
void setup() {
  Serial.begin(115200);
  randomSeed(analogRead(0));

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  stopMotores();

  if (!SENSOR_SIMULADO) {
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
  }

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Conectando a WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi conectado. IP: " + WiFi.localIP().toString());

  client.setServer(MQTT_SERVER, MQTT_PORT);

  server.on("/move", handleMove);
  server.on("/status", handleStatus);
  server.begin();
  Serial.println("Servidor HTTP listo!");
}

// ====== LOOP para reconectar ======
void loop() {
  if (!client.connected()) reconnectMQTT();
  client.loop();
  server.handleClient();
  publicarSensor();
}
