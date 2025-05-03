#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <ESP8266HTTPClient.h>  
#include <Servo.h>
#include <ArduinoJson.h>  

// ======================= Konfigurasi WiFi dan MQTT =======================
const char* ssid = "Kelompok4";
const char* password = "Kelompok4";

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
#define LED_WIFI D4    // LED hijau untuk status WiFi
#define LED_MQTT D3    // LED kuning untuk status MQTT


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

bool setup_wifi() {
  delay(10);
  Serial.print("Menghubungkan ke WiFi...");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    digitalWrite(LED_WIFI, LOW);   // Nyala saat mencoba (active-low)
    delay(250);
    digitalWrite(LED_WIFI, HIGH);  // Mati
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

// Fungsi untuk mengecek koneksi internet
bool isInternetConnected() {
  WiFiClient client;
  const int timeout = 5000; // Timeout 5 detik
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
    digitalWrite(LED_MQTT, HIGH);  // Reset state

    // Cek WiFi & Internet
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("\nWiFi terputus! Mencoba reconnect...");
      digitalWrite(LED_WIFI, HIGH);  // Mati
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

    // Koneksi MQTT
    String clientId = "ESP8266-" + String(ESP.getChipId());
    if (client.connect(clientId.c_str(), mqtt_user, mqtt_pass)) {
      Serial.println("terhubung");
      client.subscribe(mqtt_topic_threshold);
      digitalWrite(LED_MQTT, HIGH);  // Nyala saat terhubung
    } else {
      Serial.print("Gagal, rc=");
      Serial.print(client.state());
      Serial.println(" mencoba lagi dalam 5 detik");

      // Blink LED_MQTT dengan non-blocking
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
  pinMode(FAN_PIN, OUTPUT);  // Pin untuk kipas (jika ada)
  digitalWrite(FAN_PIN, LOW);  // Kipas mati default
  digitalWrite(LAMP_PIN, HIGH);  // Lampu nyala default
  pinMode(LED_WIFI, OUTPUT);
   pinMode(LED_MQTT, OUTPUT);
digitalWrite(LED_WIFI, LOW);   // Mulai dalam keadaan mati
digitalWrite(LED_MQTT, LOW);


  setup_wifi();
  espClient.setInsecure(); // Skip certificate verification (untuk testing)
// atau gunakan root CA yang valid:
// const char* ca_cert = "-----BEGIN CERTIFICATE-----\n...";
// espClient.setCACert(ca_cert);
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  dhtSensor.begin();
  servoController.begin(SERVO_PIN);

  
 // Uji LED langsung
// Uji LED
digitalWrite(LED_WIFI, LOW);  // Nyala
delay(1000);
digitalWrite(LED_WIFI, HIGH); // Mati
delay(1000);
digitalWrite(LED_WIFI, LOW);  // Nyala lagi
delay(1000);
digitalWrite(LED_WIFI, HIGH); // Mati

}

void loop() {
  // Cek WiFi
  // if (WiFi.status() != WL_CONNECTED) {
  //   Serial.println("WiFi putus. Coba reconnect...");
  //   if (!setup_wifi()) {
  //     digitalWrite(LED_WIFI, HIGH);
  //     return;
  //   }
  // }

  // // Cek Internet
  // if (!isInternetConnected()) {
  //   Serial.println("Internet tidak tersedia.");
  //   digitalWrite(LED_MQTT, HIGH);
  //   // Lanjutkan kontrol lokal (jangan return!)
  // }

  // MQTT reconnect jika perlu
  if (!client.connected()) {
    reconnect();
  }

  client.loop();

  dhtSensor.update();

  if (dhtSensor.getStatus() == SENSOR_READY) {
    // Mengambil data dari sensor DHT
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

  //delay(1000);  // Delay 1 detik untuk loop
}
