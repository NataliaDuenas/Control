# Ball & Beam Control System (ESP32 + PID)

Un sistema clásico de control **Ball & Beam** implementado con un **ESP32**.  
El proyecto estabiliza una bola en una posición deseada sobre una viga basculante utilizando un **controlador PID** digital.  
El sistema permite control vía **WiFi (WebSocket)** o control manual mediante un **potenciómetro**.

https://github.com/user-attachments/assets/69de2c22-bdfc-48aa-8a52-8cd0728301c9

---

## Características Principales

### Control PID
- PID digital con tiempo de muestreo **T = 0.1 s**.
- Sintonización realizada con MATLAB PID Tuner.
- Ganancias obtenidas:
  - **Kp = 2.7**
  - **Ki = 0.2**
  - **Kd = 0.8**

### Modos de Operación
- **Modo WiFi**: Control del setpoint desde una interfaz web en tiempo real mediante WebSockets.  
- **Modo Analógico**: Control manual con un potenciómetro.

### Monitoreo en Tiempo Real
- Visualización de la posición real y del error por WebSocket.

---

## Modelo Matemático

La función de transferencia del sistema es:

$$
P(s) = \frac{R(s)}{\Theta(s)} = 
-\frac{mgd}{L\left(\frac{J}{R^2} + m\right)} \cdot \frac{1}{s^2}
$$

Donde:

- **m**: masa de la bola  
- **R**: radio de la bola  
- **g**: gravedad  
- **L**: longitud de la viga  
- **J**: momento de inercia  

---

## BOM (Bill of Materials)

| Componente        | Descripción             |
|------------------|--------------------------|
| Microcontrolador | ESP32 Dev Module         |
| Actuador         | Servo MG996R             |
| Sensor           | Adafruit VL53L0X         |
| Interfaz manual  | Potenciómetro 10 kΩ      |
| Selector         | Pulsador                 |
| Estructura       | Viga impresa en 3D       |

---

## Instalación y Uso

### Librerías necesarias (Arduino IDE)
- `Adafruit_VL53L0X`
- `ESP32Servo`
- `WebSockets` (por Markus Sattler)

### Configuración
1. Abre el archivo `.ino` en Arduino IDE.
2. Edita tus credenciales WiFi:

```cpp
const char *ssid = "TU_RED_WIFI";
const char *password = "TU_CONTRASEÑA";
```

3. Sube el código al ESP32.
4. Abre el Monitor Serial (115200 baudios) para ver la IP asignada
(ejemplo: `192.168.1.15`).

### Interfaz Web
1. Abre el archivo distancia.html.
2. Cambia la IP del ESP32 en la línea:
```js
const ESP32_IP = "192.168.1.15";
```
3. Guarda y abre index.html en tu navegador.
