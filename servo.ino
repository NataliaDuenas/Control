#include <WiFi.h>
#include <WebSocketsServer.h>
#include "Adafruit_VL53L0X.h"
#include <ESP32Servo.h>

// ---------- CONFIG WiFi ----------
const char *ssid = "Galaxy A52A281";
const char *password = "0987654321";

// ---------- Pines ----------
#define SDA 21
#define SCL 22
const int potPin = 32;
const int pinPulsador = 2;
const int servoPin = 4;

// ---------- Objetos y variables ----------
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
Servo servo;
WiFiServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);
String header;

float distance;        // Distancia real
float setpoint = 20.0; // Setpoint inicial
int x1, x2, x3, x4, x5;
bool ultimoEstadoBoton = HIGH;

// ---------- PID ----------
float error, prev_error = 0;
float P, I, D, U;
float I_term = 0;
float T = 0.1;
unsigned long t_prev = 0;
float Kp = 1.5;
float Ki = 0.1;
float Kd = 0.4;

// ---------- SETUP ----------
void setup() {
  Serial.begin(115200);
  pinMode(pinPulsador, INPUT_PULLUP);

  servo.setPeriodHertz(50);
  servo.attach(servoPin, 1000, 2000);
  servo.write(110);  // Reposo

  if (!lox.begin()) {
    Serial.println(F("❌ VL53L0X no detectado"));
    while (1);
  }

  WiFi.begin(ssid, password);
  Serial.print("Conectando a WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n✅ Conectado!");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  server.begin();
  webSocket.begin();
  webSocket.onEvent(onWebSocketEvent);
}

// ---------- LOOP PRINCIPAL ----------
void loop() {
  webSocket.loop();
  WiFiClient client = server.available();

  distance = MedirLaser();

  // Ejecutar PID si hay distancia válida
  if (distance > 0) {
    unsigned long t_now = millis();
    if ((t_now - t_prev) >= T * 1000) {
      ejecutarPID();
      t_prev = t_now;
    }
  } else {
    servo.write(110);
  }

  // WebSocket: enviar distancia real
  static unsigned long lastSend = 0;
  if (millis() - lastSend > 200) {
    lastSend = millis();
    String mensaje = (distance <= 0) ? "Fuera de rango" : String(int(distance));
    webSocket.broadcastTXT(mensaje);
  }

  // WebSocket: enviar valor del potenciómetro
  static unsigned long lastPotSend = 0;
  if (millis() - lastPotSend > 300) {
    lastPotSend = millis();
    int potValue = MedirPot(false);
    webSocket.broadcastTXT("analog:" + String(potValue));
  }

  // Pulsador físico para cambiar setpoint
  bool estadoBoton = digitalRead(pinPulsador);
  if (ultimoEstadoBoton == HIGH && estadoBoton == LOW) {
    Serial.println("🔘 Pulsador presionado");
    int potValue = MedirPot(true);
    setpoint = potValue;
  }
  ultimoEstadoBoton = estadoBoton;

  // Manejar slider HTTP
  if (client) {
    String currentLine = "";
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        header += c;

        if (c == '\n') {
          if (currentLine.length() == 0) {
            int index = header.indexOf("GET /distanciaweb/");
            if (index >= 0) {
              int fin = header.indexOf(' ', index + 18);
              String valorStr = header.substring(index + 18, fin);
              int distanciaRecibida = valorStr.toInt();

              Serial.println("====================================");
              Serial.println("🌐 Valor recibido desde la web (slider):");
              Serial.print("🔹 Distancia seleccionada: ");
              Serial.print(distanciaRecibida);
              setpoint=distanciaRecibida;
              Serial.println(" cm");
              Serial.println("====================================");

              setpoint = distanciaRecibida;
            }

            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();
            client.println("<html><body><h1>Dato recibido</h1></body></html>");
            break;
          } else {
            currentLine = "";
          }
        } else if (c != '\r') {
          currentLine += c;
        }
      }
    }
    header = "";
    client.stop();
  }
}

// ---------- FUNCIONES ----------
float MedirLaser() {
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);

  x5 = x4; x4 = x3; x3 = x2; x2 = x1; x1 = measure.RangeMilliMeter;
  int x = (x1 + x2 + x3 + x4 + x5) / 5;

  float d = 0.1002 * x - 3.4301;
  if (d > 6)  d = 0.0939 * x - 2.301;
  if (d > 20) d = 0.0939 * x - 1.9301;
  if (measure.RangeStatus == 4 || d >= 40) d = -1;

  return d;
}

int MedirPot(bool imprimir) {
  int raw = analogRead(potPin);
  int dist = map(raw, 0, 4095, 0, 45);
  if (imprimir) {
    Serial.println("====================================");
    Serial.println("🔵 Valor recibido desde potenciometro:");
    Serial.print("🔹 Distancia seleccionada: ");
    Serial.print(dist);
    setpoint=dist;
    Serial.println(" cm");
    Serial.println("====================================");
  }
  return dist;
}

void ejecutarPID() {
  error = setpoint - distance;
  P = Kp * error;
  I_term += Ki * error * T;
  D = Kd * (error - prev_error) / T;
  U = P + I_term + D;

  U = constrain(U, -30, 30);
  int angulo;

  if (abs(U) < 2) {
    angulo = 110;
  } else if (U < 0) {
    angulo = map(U, -30, -3, 170, 125);
  } else {
    angulo = map(U, 3, 30, 95, 50);
  }

  angulo = constrain(angulo, 50, 170);
  servo.write(angulo);
  prev_error = error;
}

void onWebSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  if (type == WStype_CONNECTED) {
    IPAddress ip = webSocket.remoteIP(num);
    Serial.printf("🟢 Cliente %u conectado desde %d.%d.%d.%d\n", num, ip[0], ip[1], ip[2], ip[3]);
  } else if (type == WStype_DISCONNECTED) {
    Serial.printf("🔴 Cliente %u desconectado\n", num);
  } else if (type == WStype_TEXT) {
    String msg = String((char*)payload);
    Serial.printf("📨 Mensaje del cliente %u: %s\n", num, msg.c_str());

    if (msg == "wifi") {
      int d = MedirPot(true);
      setpoint = d;
    }
  }
}
