#include <WiFi.h>
#include <WebSocketsServer.h>
#include "Adafruit_VL53L0X.h"

// ---------- CONFIG WiFi ----------
const char *ssid = "Galaxy A52A281";
const char *password = "0987654321";

// ---------- Pines ----------
#define SDA 21
#define SCL 22
const int potPin = 32;
const int pinPulsador = 2;

// ---------- Variables ----------
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
WiFiServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);
String header;
float distanceReal;
int x1, x2, x3, x4, x5;
bool ultimoEstadoBoton = HIGH;  // Para detectar flanco de bajada

// ---------- SETUP ----------
void setup() {
  Serial.begin(115200);
  pinMode(pinPulsador, INPUT_PULLUP);  // Activar pull-up interno para el pulsador

  // Iniciar sensor láser
  if (!lox.begin()) {
    Serial.println(F("No se pudo inicializar VL53L0X"));
    while (1);
  }

  // Conectar a WiFi
  WiFi.begin(ssid, password);
  Serial.print("Conectando a WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConectado!");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  // Iniciar servidores
  server.begin();
  webSocket.begin();
  webSocket.onEvent(onWebSocketEvent);
}

// ---------- LOOP PRINCIPAL ----------
void loop() {
  webSocket.loop();
  distanceReal = MedirLaser();

  // Enviar distancia real cada 200 ms
  static unsigned long lastSend = 0;
  if (millis() - lastSend > 200) {
    lastSend = millis();
    String mensaje = (distanceReal <= 0) ? "Fuera de rango" : String(int(distanceReal));
    webSocket.broadcastTXT(mensaje);
  }

  // Enviar valor del potenciómetro cada 300 ms
  static unsigned long lastPotSend = 0;
  if (millis() - lastPotSend > 300) {
    lastPotSend = millis();
    int potValue = MedirPot(false);  // Solo para enviar, sin imprimir
    webSocket.broadcastTXT("analog:" + String(potValue));
  }

  // Detectar si se presionó el pulsador
  bool estadoBoton = digitalRead(pinPulsador);
  if (ultimoEstadoBoton == HIGH && estadoBoton == LOW) {
    Serial.println("🔘 Pulsador presionado");
    int potValue = MedirPot(true);  // Con imprimir en consola
  }
  ultimoEstadoBoton = estadoBoton;

  // Manejar solicitud HTTP desde slider
  WiFiClient client = server.available();
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
              Serial.println("🔵 Valor recibido desde la web (slider):");
              Serial.print("🔹 Distancia seleccionada: ");
              Serial.print(distanciaRecibida);
              Serial.println(" cm");
              Serial.println("====================================");
            }

            // Responder al navegador
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

double MedirPot(bool imprimir) {
  int rawValue = analogRead(potPin);
  float distancia = map(rawValue, 0, 4095, 0, 45);
  distancia = int(distancia);  // redondear

  if (imprimir) {
    Serial.println("====================================");
    Serial.println("🔵 Valor recibido desde potenciometro:");
    Serial.print("🔹 Distancia seleccionada: ");
    Serial.print(distancia);
    Serial.println(" cm");
    Serial.println("====================================");
  }

  return distancia;
}

double MedirLaser() {
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);

  x5 = x4; x4 = x3; x3 = x2; x2 = x1; x1 = measure.RangeMilliMeter;
  int x = (x1 + x2 + x3 + x4 + x5) / 5;

  distanceReal = 0.1002 * x - 3.4301;
  if (distanceReal > 6)  distanceReal = 0.0939 * x - 2.301;
  if (distanceReal > 20) distanceReal = 0.0939 * x - 1.9301;
  if (measure.RangeStatus == 4 || distanceReal >= 40) distanceReal = -1;

  return distanceReal;
}

void onWebSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  if (type == WStype_CONNECTED) {
    IPAddress ip = webSocket.remoteIP(num);
    Serial.printf("Cliente %u conectado desde %d.%d.%d.%d\n", num, ip[0], ip[1], ip[2], ip[3]);
  } else if (type == WStype_DISCONNECTED) {
    Serial.printf("Cliente %u desconectado\n", num);
  } else if (type == WStype_TEXT) {
    Serial.printf("Mensaje del cliente %u: %s\n", num, payload);
  }
}
