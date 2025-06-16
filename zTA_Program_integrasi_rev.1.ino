#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "DFRobot_MAX30102.h"

// === WIFI & MQTT CONFIG ===
const char* WIFI_SSID = "hepibanget";
const char* WIFI_PASSWORD = "halo1234";
const char* MQTT_SERVER = "iotmqtt.online";
const int MQTT_PORT = 1883;
const char* MQTT_STATUS = "serv-s/status";
const char* MQTT_STATUS_PENGUKURAN = "serv-s/status-pengukuran";
const char* MQTT_CMD = "serv-s/cmd";
const char* MQTT_DATA = "serv-s/sistem/data";
const char* MQTT_PREDICT = "serv-s/sistem/predict";
const char* MQTT_CLIENT_ID = "ESP32Client";
const char* MQTT_USERNAME = "public";
const char* MQTT_PASSWORD = "1";

// Variabel button pengukuran
bool startMeasurement = false;  // Flag kontrol dari Node-RED

// Variebel timer pengukuran 
unsigned long measurementStartTime = 0;
const unsigned long measurementDuration = 60000; // 1 menit

// Variebel Flex Sensor
const int flexPin = 34; // Sensor Flex
int flexValue;
int threshold = 0;
const int calibrationDuration = 5000;
const int thresholdOffset = 80;

WiFiClient espClient;
PubSubClient client(espClient);

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (int i = 0; i < length; i++) msg += (char)payload[i];

  Serial.print("Pesan MQTT pada topik ["); Serial.print(topic); Serial.print("]: ");
  Serial.println(msg);

  if (String(topic) == MQTT_CMD) {
    if (msg.indexOf("start") >= 0) {
      //Trigger mulai pengukuran
      kalibrasiFlexSensor();
      startMeasurement = true;
      measurementStartTime = millis();
      publishStatus("Mulai Pengukuran");
      Serial.println("Pengukuran dimulai!");
    } else if (msg.indexOf("stop") >= 0) {
      //Trigger berhenti pengukuran
      Serial.println("Pengukuran dihentikan!");
      publishStatus("Menunggu");
    }
  }
}


void setup_wifi() {
  delay(10);
  Serial.print("Menghubungkan ke ");
  Serial.println(WIFI_SSID);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi terhubung!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Menghubungkan ke MQTT...");
    if (client.connect(MQTT_CLIENT_ID, MQTT_USERNAME, MQTT_PASSWORD, MQTT_STATUS, 0, true, "Disconnected")) {
      Serial.println("Berhasil!");
      client.publish(MQTT_STATUS, "Connected", true);
    } else {
      Serial.print("Gagal, rc=");
      Serial.print(client.state());
      Serial.println(" Coba lagi dalam 5 detik");
      delay(5000);
    }
  }
}

void kalibrasiFlexSensor() {
  long calibrationSum = 0;
  int calibrationCount = 0;
  unsigned long calibrationStart = millis();
  publishStatus("Kalibrasi");
  Serial.println("Kalibrasi flex... Diamkan perut dalam posisi rileks.");
  delay(500);
  while (millis() - calibrationStart < calibrationDuration) {
    flexValue = analogRead(flexPin);
    calibrationSum += flexValue;
    calibrationCount++;
    delay(10);
  }
  threshold = (calibrationSum / calibrationCount) + thresholdOffset;
  Serial.print("Threshold baru: "); Serial.println(threshold);
}

void publishStatus(const String& status) {
  client.publish(MQTT_STATUS_PENGUKURAN, status.c_str(), true);
  Serial.print("Status Pengukuran: ");
  Serial.println(status);
}

// === SENSOR CONFIG ===
DFRobot_MAX30102 sensor;

unsigned long currentTime = 0, breathTime = 0, previousTime = 0;
int breathCount = 0, respiratoryRate = 0;
const int debounceTime = 1000;
const int timeWindow = 60000;
const int numReadings = 10;
int readings[numReadings], readIndex = 0, total = 0, average = 0;

int32_t SPO2; // MAX30102
int8_t SPO2Valid;
int32_t heartRate;
int8_t heartRateValid;

float m_hr = 1.140, b_hr = -12.904;
float m_spo2 = 0.917, b_spo2 = 6.194;

float hrSum = 0, spo2Sum = 0;
int validCount = 0;

void setup() {
  Serial.begin(115200);
  delay(100);

  Wire.begin(21, 22);  // <<< PERHATIKAN INI: gunakan pin I2C yang sama untuk MAX30102
  setup_wifi();
  client.setServer(MQTT_SERVER, MQTT_PORT);
  client.setCallback(mqttCallback);
  reconnect();

  client.subscribe(MQTT_CMD); // Button Start - (Payload = "start")

  // Inisialisasi MAX30102
  while (!sensor.begin()) {
    Serial.println("MAX30102 tidak ditemukan.");
    delay(1000);
  }
  sensor.sensorConfiguration(50, SAMPLEAVG_4, MODE_MULTILED, SAMPLERATE_100, PULSEWIDTH_411, ADCRANGE_16384);

  publishStatus("")
}

void loop() {
  if (!client.connected()) reconnect();
  client.loop();

  if (!startMeasurement) return;

  currentTime = millis();

  // Timer Pengukuran Selama 1 Menit
  if (millis() - measurementStartTime >= measurementDuration) {
    startMeasurement = false;

    // Hitung rata-rata HR dan SpO2
    float hrAvg = (validCount > 0) ? hrSum / validCount : 0;
    float spo2Avg = (validCount > 0) ? spo2Sum / validCount : 0;

    // Kirim hasil akhir (data prediksi)
    String finalPayload = String("{\"HR_avg\":") + hrAvg +
                          ",\"SpO2_avg\":" + spo2Avg +
                          ",\"RR\":" + respiratoryRate + "}";
    client.publish(MQTT_PREDICT, finalPayload.c_str());

    // Reset akumulasi
    hrSum = 0;
    spo2Sum = 0;
    validCount = 0;
    respiratoryRate = 0;

    Serial.println("Pengukuran Sudah Dilakukan Selama 1 Menit");
    return;
  }

  // Sensor Flex
  flexValue = analogRead(flexPin);
  total = total - readings[readIndex];
  readings[readIndex] = flexValue;
  total += readings[readIndex];
  readIndex = (readIndex + 1) % numReadings;
  average = total / numReadings;

  if (average > threshold && (currentTime - breathTime > debounceTime)) {
    breathCount++;
    breathTime = currentTime;
  }

  if (currentTime - previousTime >= timeWindow) {
    respiratoryRate = breathCount;
    breathCount = 0;
    previousTime = currentTime;
  }

  // MAX30102
  sensor.heartrateAndOxygenSaturation(&SPO2, &SPO2Valid, &heartRate, &heartRateValid);

  // Kalibrasi
  float hr_cal = m_hr * heartRate + b_hr;
  float spo2_cal = m_spo2 * SPO2 + b_spo2;

  if (heartRateValid && SPO2Valid) {
    hrSum += hr_cal;
    spo2Sum += spo2_cal;
    validCount++;
  }

  // ===== OUTPUT SERIAL =====
  Serial.println("======= DATA HASIL =======");
  Serial.print("HR: "); if (heartRateValid) Serial.print(hr_cal, 1); else Serial.print("Invalid");
  Serial.print(" bpm | SpO2: "); if (SPO2Valid) Serial.print(spo2_cal, 1); else Serial.print("Invalid");
  Serial.print(" % | RR: "); Serial.print(respiratoryRate); Serial.println(" bpm");
  Serial.println("==========================");

  // Real-time publish tiap detik
  String payload = String("{\"HR\":") + hr_cal +
                   ",\"SpO2\":" + spo2_cal +
                   ",\"RR_now\":" + breathCount + "}";
  client.publish(MQTT_DATA, payload.c_str());

  delay(1000); // sampling 1 detik
}
