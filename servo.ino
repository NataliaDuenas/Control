#include <WiFi.h>
#include <WebSocketsServer.h>
#include "Adafruit_VL53L0X.h"

const char *ssid = "Galaxy A52A281";
const char *password = "0987654321";

// Pines I2C
#define SDA 21
#define SCL 22

Adafruit_VL53L0X lox = Adafruit_VL53L0X();
WiFiServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);

String header;
float distanceReal;
int x1, x2, x3, x4, x5;
int x = 0;

void setup() {
  Serial.begin(115200);

  // Inicializa sensor
  if (!lox.begin()) {
    Serial.println(F("No se pudo inicializar VL53L0X"));
    while (1);
  }

  // Conexión WiFi
  WiFi.begin(ssid, password);
  Serial.print("Conectando a WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConectado!");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  // Servidores
  server.begin();
  webSocket.begin();
  webSocket.onEvent(onWebSocketEvent);
}

void loop() {
  // Medir y enviar distancia cada 200 ms
  distanceReal = MedirLaser();
  webSocket.loop();

  static unsigned long lastSend = 0;
  if (millis() - lastSend > 200) {
    lastSend = millis();
    String mensaje = (distanceReal <= 0) ? "Fuera de rango" : String(int(distanceReal));
    webSocket.broadcastTXT(mensaje);
  }

  // Atender solicitudes HTTP
  WiFiClient client = server.available();
  if (client) {
    String currentLine = "";
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        header += c;

        if (c == '\n') {
          if (currentLine.length() == 0) {
            // Detectar ruta: /distanciaweb/<valor>
            int index = header.indexOf("GET /distanciaweb/");
            if (index >= 0) {
              int fin = header.indexOf(' ', index + 18);
              String valorStr = header.substring(index + 18, fin);
              int distanciaRecibida = valorStr.toInt();
              Serial.print("Distancia WiFi recibida: ");
              Serial.println(distanciaRecibida);
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

// ------------------- SENSOR VL53L0X ----------------------
double MedirLaser() {
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);

  x5 = x4; x4 = x3; x3 = x2; x2 = x1; x1 = measure.RangeMilliMeter;
  x = (x1 + x2 + x3 + x4 + x5) / 5;

  distanceReal = 0.1002 * x - 3.4301;
  if (distanceReal > 6) distanceReal = 0.0939 * x - 2.301;
  if (distanceReal > 20) distanceReal = 0.0939 * x - 1.9301;
  if (measure.RangeStatus == 4 || distanceReal >= 40) distanceReal = -1;

  return distanceReal;
}

// ------------------- EVENTOS WEBSOCKET ----------------------
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
