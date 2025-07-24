#include "Adafruit_VL53L0X.h"
#include <ESP32Servo.h>

#define SDA 21
#define SCL 22

Servo servo;

int x1, x2, x3, x4, x5;
int x = 0;

float distance;
float setpoint = 20.0;  // Distancia objetivo en cm
float error, prev_error = 0;
float P, I, D, U;
float I_term = 0;
float T = 0.1;  // Tiempo de muestreo en segundos
unsigned long t_prev = 0;

float Kp = 1.5;
float Ki = 0.1;
float Kd = 0.4;

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

void setup() {
  Serial.begin(115200);
  servo.attach(4);
  servo.write(110);  // Zona muerta (reposo)

  if (!lox.begin()) {
    Serial.println(F("❌ VL53L0X no detectado"));
    while (1);
  }

  Serial.println(F("✅ VL53L0X listo\n"));
}

void PID() {
  error = setpoint - distance;

  P = Kp * error;
  I_term += Ki * error * T;
  D = Kd * (error - prev_error) / T;

  U = P + I_term + D;

  // Limitar U entre -30 y 30
  U = constrain(U, -30, 30);

  int angulo;

  if (abs(U) < 2) {
    // Error pequeño → no mover (zona muerta)
    angulo = 110;
  } else if (U < 0) {
    // Alejarse (U negativo) → mover hacia derecha
    angulo = map(U, -30, -3, 170, 125);  // Más lejos de la zona muerta
  } else {
    // Acercarse (U positivo) → mover hacia izquierda
    angulo = map(U, 3, 30, 95, 50);  // Más lejos de la zona muerta
  }

  angulo = constrain(angulo, 50, 170);
  servo.write(angulo);

  Serial.print("📏 Distancia: "); Serial.print(distance); Serial.print(" cm\t");
  Serial.print("🔁 Error: "); Serial.print(error); Serial.print("\t");
  Serial.print("⚙️ U: "); Serial.print(U); Serial.print("\t");
  Serial.print("⏱ Ángulo: "); Serial.println(angulo);

  prev_error = error;
}


void loop() {
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);

  if (measure.RangeStatus != 4) {
    x5 = x4; x4 = x3; x3 = x2; x2 = x1;
    x1 = measure.RangeMilliMeter;
    x = (x1 + x2 + x3 + x4 + x5) / 5;

    distance = 0.1002 * x - 3.4301;
    if (distance > 20) {
      distance = 0.0949 * x - 3.4301;
    }

    unsigned long t_now = millis();
    if (t_now - t_prev >= T * 1000) {
      PID();
      t_prev = t_now;
    }
  } else {
    Serial.println("🚫 Fuera de rango");
    servo.write(110);  // Zona muerta segura
  }
}
