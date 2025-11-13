// ####### INICIAL CONFIG #######################################

#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <ArduinoJson.h> // Compatible amb versió 7.4.2
#include <ESP32Servo.h>

// Device ID
const char *deviceId = "G3_Servos";

// Wi-Fi credentials
const char *ssid = "Robotics_UB";
const char *password = "rUBot_xx";

// UDP settings
IPAddress receiverESP32IP(192, 168, 1, 51);
IPAddress receiverComputerIP(192, 168, 1, 35);
const int udpPort = 12345;
WiFiUDP udp;

// Servo settings
Servo servo_yaw;
Servo servo_pitch;
Servo servo_roll1;
Servo servo_roll2;

// Pins
const int PIN_ANALOG_YAW = 36;
const int PIN_SIGNAL_YAW = 32;
const int PIN_ANALOG_PITCH = 39;
const int PIN_SIGNAL_PITCH = 33;
const int PIN_ANALOG_ROLL1 = 34;
const int PIN_SIGNAL_ROLL1 = 25;
const int PIN_ANALOG_ROLL2 = 35;
const int PIN_SIGNAL_ROLL2 = 27;

const float Rshunt = 1.6;

// Variables
float Gri_roll = 0.0, Gri_pitch = 0.0, Gri_yaw = 0.0;
float Torque_roll1 = 0.0, Torque_roll2 = 0.0, Torque_pitch = 0.0, Torque_yaw = 0.0;
float prevRoll1 = 0, prevRoll2 = 0, prevPitch = 0, prevYaw = 0;
float sumRoll1 = 0, sumRoll2 = 0, sumPitch = 0, sumYaw = 0;
float OldValueRoll = 0, OldValuePitch = 0, OldValueYaw = 0; // we do not use
float roll = 0, pitch = 0, yaw = 0;
int s1 = 1, s2 = 1;
float YawInit = 0, prevGriYaw = 0, ant_Yaw = 0, deltaYaw = 0;
bool yaw_initialized = false;

// ####### CONNECT TO WIFI #######################################
void connectToWiFi() {
  Serial.print("Connecting to Wi-Fi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("\nWi-Fi connected!");
  Serial.println("IP Address: " + WiFi.localIP().toString());
  Serial.print("ESP32 MAC Address: ");
  Serial.println(WiFi.macAddress());
}


// ####### RECIEVE DATA FROM GRIPPER #######################################
void receiveOrientationUDP() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    byte packetBuffer[512];
    int len = udp.read(packetBuffer, 512);
    if (len > 0) {
      packetBuffer[len] = '\0';
      Serial.print("Received packet size: ");
      Serial.println(packetSize);
      Serial.print("Received: ");
      Serial.println((char*)packetBuffer);

      JsonDocument doc;  // ✅ Versió 7
      DeserializationError error = deserializeJson(doc, packetBuffer);
      if (error) {
        Serial.print(F("deserializeJson() failed: "));
        Serial.println(error.f_str());
        return;
      }

      const char* device = doc["device"];
      if (strcmp(device, "G3_Gri") == 0) {
        Gri_roll = round(doc["roll"].as<float>());
        Gri_pitch = round(doc["pitch"].as<float>());
        Gri_yaw = round(doc["yaw"].as<float>());
        s1 = doc["s1"];
        s2 = doc["s2"];
        Serial.print("Gri_Roll: "); Serial.print(Gri_roll);
        Serial.print(" Gri_Pitch: "); Serial.print(Gri_pitch);
        Serial.print(" Gri_Yaw: "); Serial.println(Gri_yaw);
        Serial.print("S1: "); Serial.print(s1);
        Serial.print(" S2: "); Serial.println(s2);
      } else {
        Serial.println("Unknown device.");
      }
    }
  }
}


// ####### DEFINE I (current) AND T (TORQUE) #######################################
float getCurrent(uint32_t integrationTimeMs, int pin) {
  uint32_t startTime = millis();
  float integratedCurrent = 0;
  while (millis() < startTime + integrationTimeMs) {
    uint16_t adcValue = analogRead(pin);
    integratedCurrent += ((float)adcValue / 4095.0 * 3.3) / Rshunt;
  }
  return integratedCurrent;
}

float getTorque(float& sum, int analogPin, float& previous) {
  float current = getCurrent(20, analogPin);
  sum += current;
  float diff = abs(sum - previous);
  previous = sum;
  return diff;
}

// ####### TRY TO NORMALIZE ANGLES (NO WRAP) #######################################
float normalizeAngle(float angle) {
  angle = fmod(angle, 360.0f);      // Range 0-360
  if (angle > 180.0f) angle -= 360.0f; // Converts from -180 to +180
  return angle;
}

// ####### CONTROL MOTORS ACCORDING TO RPY #######################################
void moveServos() {
  //// --- Get the received orientation values ---
  //roll = Gri_roll;
  //pitch = Gri_pitch;
  //yaw = Gri_yaw;
  //// --- Normalize roll & pitch to [-180, 180] range ---
  //auto normalize = [](float angle) {
  //  angle = fmod(angle + 180.0f, 360.0f);
  //  if (angle < 0) angle += 360.0f;
  //  return angle - 180.0f;
  //};
  //roll  = normalize(roll);
  //pitch = normalize(pitch);
  //// yaw stays as-is (0–360)

  roll = normalizeAngle(Gri_roll);
  pitch = normalizeAngle(Gri_pitch);

  // // ---------------------V1---------------------
  // if (!yaw_initialized) {
  //   YawInit = Gri_yaw; // definir un yaw inicial (no varia)
  //   yaw_initialized = true;
  // }
  // yaw = Gri_yaw - YawInit; // yaw final = yaw mesurat - yaw inicial

  // // ---------------------V2---------------------
  if (!yaw_initialized) {
    prevGriYaw = Gri_yaw; // primera iteració: yaw relatiu = yaw mesurat
    ant_Yaw = 0; // yaw mesurat en la iteració anterior = 0
    yaw_initialized = true;
  }
  deltaYaw = Gri_yaw - prevGriYaw; // diferencia entre yaw mesurat i "yaw mesurat en la iteració anterior"
  yaw = ant_Yaw + deltaYaw; // yaw final = "yaw final de la iteració anterior" + diferencia d'angles
  ant_Yaw = yaw; // definir "yaw final de la iteració anterior" per la propera iteració
  prevGriYaw = Gri_yaw; // definir "yaw mesurat en la iteració anterior"

  // // ---------------------V3---------------------
  // if (!yaw_initialized) {
  //   prevGriYaw = Gri_yaw; 
  //   yaw_initialized = true;
  // }
  // deltaYaw = Gri_yaw - prevGriYaw;
  // yaw = deltaYaw; // el yaw és la diferencia d'angles
  // prevGriYaw = Gri_yaw;


   // --- Optional gripper open offset (S1 button) ---
   float delta = 0;
   if (s1 == 0) {
     delta = 20; // small safe offset for opening
     Serial.println("S1 pressed → Opening gripper");
  }

  // --- Map normalized angles to servo range (0–180°) ---
  auto mapToServo = [](float angle) {
    // maps [-180,180] → [0,180]; 0° stays at 90°
    return constrain(90 + (angle / 2.0f), 0, 180);
  };

  int servoRoll1Angle = mapToServo( roll + delta);
  int servoRoll2Angle = mapToServo(-roll - delta);
  int servoPitchAngle = mapToServo( pitch );
  int servoYawAngle   = constrain(90 + yaw, 0, 180);  // yaw stays linear 0–180

  // --- Write to servos ---
  servo_roll1.write(servoRoll1Angle);
  servo_roll2.write(servoRoll2Angle);
  servo_pitch.write(servoPitchAngle);
  servo_yaw.write(servoYawAngle);
}

// ####### SEND TORQUE VALUES TO GRIPPER #######################################
void sendTorquesUDP() {
  JsonDocument doc;
  doc["device"] = deviceId;
  doc["Torque_roll1"] = Torque_roll1;
  doc["Torque_roll2"] = Torque_roll2;
  doc["Torque_pitch"] = Torque_pitch;
  doc["Torque_yaw"] = Torque_yaw;

  char jsonBuffer[512];
  serializeJson(doc, jsonBuffer, sizeof(jsonBuffer));
  
  // Send Json to Gripper
  udp.beginPacket(receiverESP32IP, udpPort);
  udp.write((const uint8_t*)jsonBuffer, strlen(jsonBuffer));
  udp.endPacket();

  // Send Json to Computer
  udp.beginPacket(receiverComputerIP, udpPort);
  udp.write((const uint8_t*)jsonBuffer, strlen(jsonBuffer));
  udp.endPacket();
}



void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(2000);

  connectToWiFi();
  udp.begin(udpPort);
  Serial.println("UDP initialized");

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  servo_yaw.setPeriodHertz(50);
  servo_pitch.setPeriodHertz(50);
  servo_roll1.setPeriodHertz(50);
  servo_roll2.setPeriodHertz(50);

  servo_yaw.attach(PIN_SIGNAL_YAW);
  servo_pitch.attach(PIN_SIGNAL_PITCH);
  servo_roll1.attach(PIN_SIGNAL_ROLL1);
  servo_roll2.attach(PIN_SIGNAL_ROLL2);

  pinMode(PIN_ANALOG_YAW, INPUT);
  pinMode(PIN_ANALOG_PITCH, INPUT);
  pinMode(PIN_ANALOG_ROLL1, INPUT);
  pinMode(PIN_ANALOG_ROLL2, INPUT);

  servo_yaw.write(90);
  servo_pitch.write(90);
  servo_roll1.write(90);
  servo_roll2.write(90);
}

void loop() {
  receiveOrientationUDP();
  moveServos();
  
  // NEW: llegim torques from ADCs
  Torque_roll1 = getTorque(sumRoll1, PIN_ANALOG_ROLL1, prevRoll1);
  Torque_roll2 = getTorque(sumRoll2, PIN_ANALOG_ROLL2, prevRoll2);
  Torque_pitch = getTorque(sumPitch, PIN_ANALOG_PITCH, prevPitch);
  Torque_yaw = getTorque(sumYaw, PIN_ANALOG_YAW, prevYaw);
  // NEW: enviem els torques amb la nova funció creada
  sendTorquesUDP();
  
  delay(10);
}
