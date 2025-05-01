#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <Servo.h>
#include <ArduinoJson.h>  // Pastikan kamu sudah menginstal ArduinoJson

// ======================= Konfigurasi WiFi dan MQTT =======================
const char* ssid = "NAILY";
const char* password = "naji2010";

// HiveMQ Cloud details
const char* mqtt_server = "a3f850802ac34230b60106b86aaa6ae8.s1.eu.hivemq.cloud";  // Ganti dengan broker yang diberikan HiveMQ Cloud
const int mqtt_port = 8883;  // Port dengan TLS
const char* mqtt_user = "hivemq.webclient.1745768022480";  // Ganti dengan username dari HiveMQ Cloud
const char* mqtt_pass = "K1zTM:0g!roXYdJ74w;>";  // Ganti dengan password dari HiveMQ Cloud

const char* mqtt_topic_status = "iot/sensor";        // Untuk kirim suhu/beban
const char* mqtt_topic_threshold = "iot/sensor/set";   // Untuk terima update threshold

WiFiClientSecure espClient;
PubSubClient client(espClient);

// ===================== DHT & Servo Setup =====================
#define DHTPIN D5
#define DHTTYPE DHT22
#define SERVO_PIN D6
#define LAMP_PIN D7
#define FAN_PIN D1  // Pin untuk kipas (jika ada)

float TEMP_THRESHOLD = 31.00;  // Dapat diubah via MQTT

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

void setup_wifi() {
  delay(10);
  Serial.print("Menghubungkan ke WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); Serial.print(".");
  }
  Serial.println("\nWiFi terhubung");
}

void callback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.printf("Pesan MQTT diterima [%s]: %s\n", topic, message.c_str());

  // Handle update threshold
  if (String(topic) == mqtt_topic_threshold) {
    float newThreshold = message.toFloat();
    
    // Validasi nilai threshold
    if (newThreshold >= 10.0 && newThreshold <= 100.0) {
      TEMP_THRESHOLD = newThreshold;
      Serial.printf("Threshold baru: %.2f °C\n", TEMP_THRESHOLD);
      
      // Konfirmasi update via MQTT (opsional)
      client.publish("iot/sensor/confirm", ("Threshold updated: " + String(TEMP_THRESHOLD)).c_str());
    } else {
      Serial.println("Nilai threshold tidak valid (10-100)!");
    }
  }
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Menghubungkan ke MQTT...");
    String clientId = "ESP8266-" + String(ESP.getChipId());  // ID unik
    
    if (client.connect(clientId.c_str(), mqtt_user, mqtt_pass)) {
      Serial.println("terhubung");
      client.subscribe(mqtt_topic_threshold);  // Subscribe ke topik threshold
    } else {
      Serial.print("Gagal, rc=");
      Serial.print(client.state());
      Serial.println(" mencoba lagi dalam 5 detik");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  setup_wifi();
  espClient.setInsecure(); // Skip certificate verification (untuk testing)
// atau gunakan root CA yang valid:
// const char* ca_cert = "-----BEGIN CERTIFICATE-----\n...";
// espClient.setCACert(ca_cert);
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  dhtSensor.begin();
  servoController.begin(SERVO_PIN);

  pinMode(LAMP_PIN, OUTPUT);
  pinMode(FAN_PIN, OUTPUT);  // Pin untuk kipas (jika ada)
  digitalWrite(FAN_PIN, LOW);  // Kipas mati default
  digitalWrite(LAMP_PIN, LOW);  // Lampu nyala default
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  dhtSensor.update();

  if (dhtSensor.getStatus() == SENSOR_READY) {
    float temperature = dhtSensor.getTemperature();
    float humidity = dhtSensor.getHumidity();
    int angle = map(temperature, 0, 40, 0, 180);
    servoController.update(angle);

    bool lampStatus = temperature < TEMP_THRESHOLD;  // Status lampu (nyala/mati)

    // Membuat JSON
    StaticJsonDocument<200> doc;
    doc["SuhuLampu"] = temperature;
    doc["Kelembapan"] = humidity;
    doc["StatusLampu"] = lampStatus;
    doc["StatusKipas"] = !lampStatus;  // Misalnya, kipas mati, ganti logika sesuai kebutuhan
    doc["Threshold"] = TEMP_THRESHOLD;
    doc["ServoAngle"] = angle;

    // doc["ServoStatus"] = (angle == -1) ? "Error" : "OK";  // Status servo
    // doc["SensorStatus"] = (dhtSensor.getStatus() == SENSOR_ERROR) ? "Error" : "OK";  // Status sensor

    // Serialize JSON ke string
    char jsonBuffer[512];
    serializeJson(doc, jsonBuffer);

    // Mengirim data JSON melalui MQTT
    client.publish(mqtt_topic_status, jsonBuffer);

    // Mengontrol lampu berdasarkan suhu
    if (lampStatus) {
      digitalWrite(FAN_PIN, LOW);  // Kipas nyala
      Serial.println("Kipas Mati");
      digitalWrite(LAMP_PIN, LOW);  // Lampu nyala
      Serial.println("Lampu Nyala");
    } else {
      digitalWrite(FAN_PIN, HIGH);  // Kipas mati
      Serial.println("Kipas Mati");
      digitalWrite(LAMP_PIN, HIGH);  // Lampu mati
      Serial.println("Lampu Mati");
    }

    Serial.printf("Suhu: %.2f °C | Humidity: %.2f %% | Threshold: %.2f | StatusLampu: %s\n", 
                  temperature, humidity, TEMP_THRESHOLD, lampStatus ? "Nyala" : "Mati");
  } else {
    Serial.println("Gagal membaca sensor DHT!");
  }

  delay(1000);  // Delay 1 detik untuk loop
}
