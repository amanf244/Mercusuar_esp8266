#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <ESP8266HTTPClient.h>  
#include <Servo.h>
#include <ArduinoJson.h>  

// ======================= Konfigurasi WiFi dan MQTT =======================
const char* ssid = "kelompokempat";
const char* password = "kelompokempat";

// HiveMQ Cloud details
const char* mqtt_server = "a3f850802ac34230b60106b86aaa6ae8.s1.eu.hivemq.cloud";
const int mqtt_port = 8883;
const char* mqtt_user = "hivemq.webclient.1745768022480";
const char* mqtt_pass = "K1zTM:0g!roXYdJ74w;>";

const char* mqtt_topic_status = "iot/sensor";
const char* mqtt_topic_threshold = "iot/sensor/set";
const char* mqtt_topic_lamp_control = "iot/actuator/lamp";
const char* mqtt_topic_fan_control = "iot/actuator/fan";

WiFiClientSecure espClient;
PubSubClient client(espClient);

// ===================== DHT & Servo Setup =====================
#define DHTPIN D5
#define DHTTYPE DHT22
#define SERVO_PIN D6
#define LAMP_PIN D7
#define FAN_PIN D1
#define LED_WIFI D4
#define LED_MQTT D3

float TEMP_THRESHOLD = 31.00;
const float HYSTERESIS = 0.5; // Hysteresis untuk mencegah kedipan

// Variabel kontrol manual
bool MANUAL_LAMP_CONTROL = false;
bool MANUAL_FAN_CONTROL = false;
bool LAMP_STATE = false;
bool FAN_STATE = false;

// ====================== Servo Sweep Variables =====================
int currentServoAngle = 0;       // Current angle position
int servoDirection = 1;           // 1 = increasing, -1 = decreasing
const int SERVO_MIN = 0;          // Minimum servo angle
const int SERVO_MAX = 180;        // Maximum servo angle
const int SERVO_STEP = 5;         // Step size per movement (ditingkatkan)
unsigned long lastServoMove = 0;  // Last movement time
const unsigned long SERVO_INTERVAL = 10; // Movement interval (ms) (dikurangi)

enum SensorStatus { SENSOR_READY, SENSOR_ERROR };

class DHTSensor {
  private:
    DHT* dht;
    unsigned long lastReadTime = 0;
    const unsigned long readInterval = 2000;
    float humidity = 0;
    float temperature = 0;
    SensorStatus status = SENSOR_READY;

  public:
    DHTSensor(uint8_t pin) { dht = new DHT(pin, DHTTYPE); }
    void begin() { dht->begin(); }
    void update() {
      unsigned long currentMillis = millis();
      if (currentMillis - lastReadTime >= readInterval) {
        lastReadTime = currentMillis;
        humidity = dht->readHumidity();
        temperature = dht->readTemperature();
        status = (isnan(humidity) || isnan(temperature)) ? SENSOR_ERROR : SENSOR_READY;
      }
    }
    float getHumidity() { return humidity; }
    float getTemperature() { return temperature; }
    SensorStatus getStatus() { return status; }
};

class ServoController {
  private:
    Servo servo;
    int currentAngle = -1;

  public:
    void begin(int pin) { servo.attach(pin); }
    void update(int angle) {
      if (angle != currentAngle) {
        servo.write(angle);
        currentAngle = angle;
      }
    }
};

// ====================== Inisialisasi =======================
DHTSensor dhtSensor(DHTPIN);
ServoController servoController;

bool setup_wifi() {
  delay(10);
  Serial.print("Menghubungkan ke WiFi...");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    digitalWrite(LED_WIFI, LOW);
    delay(250);
    digitalWrite(LED_WIFI, HIGH);
    delay(250);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    digitalWrite(LED_WIFI, LOW);
    Serial.println("\nWiFi terhubung");
    return true;
  } else {
    digitalWrite(LED_WIFI, HIGH);
    Serial.println("\nGagal terhubung WiFi!");
    return false;
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.printf("Pesan MQTT diterima [%s]: %s\n", topic, message.c_str());

 if (String(topic) == mqtt_topic_threshold) {
    // Ganti koma menjadi titik untuk format desimal
    message.replace(',', '.');
    
    // Gunakan fungsi parsing yang lebih robust
    char* endptr;
    float newThreshold = strtof(message.c_str(), &endptr);
    
    // Cek apakah konversi berhasil
    if (endptr != message.c_str() && *endptr == '\0' && 
        newThreshold >= 10.0 && newThreshold <= 100.0) {
        TEMP_THRESHOLD = newThreshold;
        Serial.printf("Threshold baru: %.2f °C\n", TEMP_THRESHOLD);
        client.publish("iot/sensor/confirm", ("Threshold updated: " + String(TEMP_THRESHOLD, 1)).c_str());
    } else {
        Serial.println("Nilai threshold tidak valid (10-100)!");
        // Kirim pesan error
        client.publish("iot/sensor/error", "Invalid threshold value");
    }
  }
  // Handle manual lamp control
  else if (String(topic) == mqtt_topic_lamp_control) {
    if (message == "ON") {
      MANUAL_LAMP_CONTROL = true;
      LAMP_STATE = true;
      digitalWrite(LAMP_PIN, HIGH);  // Nyala (active-low)
      Serial.println("Lampu manual NYALA");
    } else if (message == "OFF") {
      MANUAL_LAMP_CONTROL = true;
      LAMP_STATE = false;
      digitalWrite(LAMP_PIN, LOW); // Mati
      Serial.println("Lampu manual MATI");
    } else if (message == "AUTO") {
      MANUAL_LAMP_CONTROL = false;
      Serial.println("Lampu kembali AUTO");
    }
  }
  // Handle manual fan control
  else if (String(topic) == mqtt_topic_fan_control) {
    if (message == "ON") {
      MANUAL_FAN_CONTROL = true;
      FAN_STATE = true;
      digitalWrite(FAN_PIN, HIGH); // Kipas nyala
      Serial.println("Kipas manual NYALA");
    } else if (message == "OFF") {
      MANUAL_FAN_CONTROL = true;
      FAN_STATE = false;
      digitalWrite(FAN_PIN, LOW);  // Kipas mati
      Serial.println("Kipas manual MATI");
    } else if (message == "AUTO") {
      MANUAL_FAN_CONTROL = false;
      Serial.println("Kipas kembali AUTO");
    }
  }
}

bool isInternetConnected() {
  WiFiClient client;
  const int timeout = 5000;
  const char* host = "www.google.com";
  const int port = 80;

  if (!client.connect(host, port)) {
    unsigned long start = millis();
    while (!client.connected() && millis() - start < timeout) {
      delay(100);
    }
  }

  bool result = client.connected();
  client.stop();
  return result;
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Menghubungkan ke MQTT...");
    digitalWrite(LED_MQTT, HIGH);

    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("\nWiFi terputus! Mencoba reconnect...");
      digitalWrite(LED_WIFI, HIGH);
      if (!setup_wifi()) {
        Serial.println("Gagal reconnect WiFi!");
        delay(5000);
        continue;
      }
    }
    
    if (!isInternetConnected()) {
      Serial.println("Tidak ada internet!");
      digitalWrite(LED_MQTT, LOW);
      delay(5000);
      continue;
    }

    String clientId = "ESP8266-" + String(ESP.getChipId());
    if (client.connect(clientId.c_str(), mqtt_user, mqtt_pass)) {
      Serial.println("terhubung");
      client.subscribe(mqtt_topic_threshold);
      client.subscribe(mqtt_topic_lamp_control);
      client.subscribe(mqtt_topic_fan_control);
      digitalWrite(LED_MQTT, HIGH);
    } else {
      Serial.print("Gagal, rc=");
      Serial.print(client.state());
      Serial.println(" mencoba lagi dalam 5 detik");

      unsigned long startTime = millis();
      while (millis() - startTime < 250) {
        digitalWrite(LED_MQTT, !digitalRead(LED_MQTT));
        delay(250);
        yield();
      }
    }
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(LAMP_PIN, OUTPUT);
  pinMode(FAN_PIN, OUTPUT);
  digitalWrite(FAN_PIN, LOW);
  digitalWrite(LAMP_PIN, LOW);
  pinMode(LED_WIFI, OUTPUT);
  pinMode(LED_MQTT, OUTPUT);
  digitalWrite(LED_WIFI, LOW);
  digitalWrite(LED_MQTT, LOW);

  // Uji LED
  digitalWrite(LED_WIFI, LOW);
  delay(1000);
  digitalWrite(LED_WIFI, HIGH);
  delay(1000);
  digitalWrite(LED_WIFI, LOW);
  delay(1000);
  digitalWrite(LED_WIFI, HIGH);

  setup_wifi();
  espClient.setInsecure();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  dhtSensor.begin();
  servoController.begin(SERVO_PIN);
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi putus. Coba reconnect...");
    if (!setup_wifi()) {
      digitalWrite(LED_WIFI, HIGH);
      return;
    }
  }

  if (!isInternetConnected()) {
    Serial.println("Internet tidak tersedia.");
    digitalWrite(LED_MQTT, HIGH);
  }

  if (!client.connected()) {
    reconnect();
  }

  client.loop();

  // ======== Servo Sweep Mechanism ========
  unsigned long currentMillis = millis();
  if (currentMillis - lastServoMove >= SERVO_INTERVAL) {
    lastServoMove = currentMillis;
    
    // Update angle position
    currentServoAngle += servoDirection * SERVO_STEP;
    
    // Reverse direction at limits
    if (currentServoAngle >= SERVO_MAX) {
      servoDirection = -1;
      currentServoAngle = SERVO_MAX; // Pastikan tidak melebihi batas
    } else if (currentServoAngle <= SERVO_MIN) {
      servoDirection = 1;
      currentServoAngle = SERVO_MIN; // Pastikan tidak melebihi batas
    }
    
    servoController.update(currentServoAngle);
  }

  dhtSensor.update();

  if (dhtSensor.getStatus() == SENSOR_READY) {
    float temperature = dhtSensor.getTemperature();
    float humidity = dhtSensor.getHumidity();

    // Kontrol lampu dengan hysteresis untuk mencegah kedipan
    if (!MANUAL_LAMP_CONTROL) {
      static bool lastLampState = LAMP_STATE;
      
      if (lastLampState) {
        // Jika lampu sedang nyala, matikan hanya jika suhu >= threshold
        if (temperature >= TEMP_THRESHOLD) {
          LAMP_STATE = false;
          lastLampState = false;
        }
      } else {
        // Jika lampu sedang mati, nyalakan hanya jika suhu < threshold - hysteresis
        if (temperature < TEMP_THRESHOLD - HYSTERESIS) {
          LAMP_STATE = true;
          lastLampState = true;
        }
      }
      digitalWrite(LAMP_PIN, LAMP_STATE ? HIGH : LOW);
    }

    // Kontrol kipas dengan hysteresis
    if (!MANUAL_FAN_CONTROL) {
      static bool lastFanState = FAN_STATE;
      
      if (lastFanState) {
        // Jika kipas sedang nyala, matikan hanya jika suhu < threshold
        if (temperature < TEMP_THRESHOLD) {
          FAN_STATE = false;
          lastFanState = false;
        }
      } else {
        // Jika kipas sedang mati, nyalakan hanya jika suhu >= threshold + hysteresis
        if (temperature >= TEMP_THRESHOLD + HYSTERESIS) {
          FAN_STATE = true;
          lastFanState = true;
        }
      }
      digitalWrite(FAN_PIN, FAN_STATE ? HIGH : LOW);
    }

    // Membuat JSON
    StaticJsonDocument<200> doc;
    doc["Suhu"] = temperature;
    doc["Kelembapan"] = humidity;
    doc["StatusLampu"] = LAMP_STATE;
    doc["StatusKipas"] = FAN_STATE;
    doc["Threshold"] = TEMP_THRESHOLD;
    doc["ServoAngle"] = currentServoAngle; // Gunakan sudut sweep
    doc["ManualModeLamp"] = MANUAL_LAMP_CONTROL;
    doc["ManualModeFan"] = MANUAL_FAN_CONTROL;

    char jsonBuffer[512];
    serializeJson(doc, jsonBuffer);

    // Mengirim data JSON melalui MQTT
    client.publish(mqtt_topic_status, jsonBuffer);

    Serial.printf("Suhu: %.2f °C | Humidity: %.2f %% | Threshold: %.2f | Lamp: %s | Fan: %s | ManualLamp: %s | ManualFan: %s | Servo: %d°\n", 
                  temperature, humidity, TEMP_THRESHOLD, 
                  LAMP_STATE ? "Nyala" : "Mati", 
                  FAN_STATE ? "Nyala" : "Mati",
                  MANUAL_LAMP_CONTROL ? "ON" : "AUTO",
                  MANUAL_FAN_CONTROL ? "ON" : "AUTO",
                  currentServoAngle);
  
  } else {
    Serial.println("Gagal membaca sensor DHT!");
  }

  delay(10); // Small delay for stability
}