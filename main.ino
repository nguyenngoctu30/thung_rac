//servo 2 là vô cơ

#include <WiFi.h>
#include <PubSubClient.h>
#include <ESP32Servo.h>

const char* ssid = "677 5G";
const char* password = "10101010";

const char* mqtt_server = "broker.hivemq.com";
const int mqtt_port = 1883;
const char* topic_sub     = "esp32/servo/control";
const char* topic_sensor1 = "esp32/ultrasonic/sensor1";
const char* topic_sensor2 = "esp32/ultrasonic/sensor2";
const char* topic_status1 = "esp32/ultrasonic/status1";
const char* topic_status2 = "esp32/ultrasonic/status2";
const char* topic_buzzer  = "esp32/buzzer/control";   

const float DISTANCE_THRESHOLD = 15.0f;

const int BUZZER_PIN = 26;

const unsigned long BEEP_ON  = 200;
const unsigned long BEEP_OFF = 200;

bool buzzerActive       = false;
bool buzzerState        = false;
unsigned long buzzerLast = 0;
bool buzzerMuted        = false;   

WiFiClient espClient;
PubSubClient client(espClient);

Servo servo1;
Servo servo2;

const int servo1Pin = 19;
const int servo2Pin = 25;

bool servo1Busy = false;
bool servo2Busy = false;

unsigned long servo1Start = 0;
unsigned long servo2Start = 0;

const int TRIG1 = 5;
const int ECHO1 = 18;
const int TRIG2 = 22;
const int ECHO2 = 23;

const unsigned long SENSOR_INTERVAL = 500;
unsigned long lastSensorRead = 0;

float measureDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000);
  if (duration == 0) return -1;

  float distance = duration * 0.0343f / 2.0f;
  if (distance < 2.0f || distance > 400.0f) return -1;

  return distance;
}

void handleBuzzer() {
  if (!buzzerActive) {
    digitalWrite(BUZZER_PIN, LOW);
    return;
  }

  unsigned long now = millis();
  unsigned long interval = buzzerState ? BEEP_ON : BEEP_OFF;

  if (now - buzzerLast >= interval) {
    buzzerState = !buzzerState;
    digitalWrite(BUZZER_PIN, buzzerState ? HIGH : LOW);
    buzzerLast = now;
  }
}

void publishSensors() {
  char buf[16];
  bool full1 = false;
  bool full2 = false;

  float d1 = measureDistance(TRIG1, ECHO1);
  if (d1 >= 0) {
    dtostrf(d1, 5, 1, buf);
    client.publish(topic_sensor1, buf);
    if (d1 < DISTANCE_THRESHOLD) {
      client.publish(topic_status1, "day");
      full1 = true;
    } else {
      client.publish(topic_status1, "khong day");
    }
  } else {
    client.publish(topic_sensor1, "-1");
    client.publish(topic_status1, "khong day");
  }

  delay(10);

  float d2 = measureDistance(TRIG2, ECHO2);
  if (d2 >= 0) {
    dtostrf(d2, 5, 1, buf);
    client.publish(topic_sensor2, buf);
    if (d2 < DISTANCE_THRESHOLD) {
      client.publish(topic_status2, "day");
      full2 = true;
    } else {
      client.publish(topic_status2, "khong day");
    }
  } else {
    client.publish(topic_sensor2, "-1");
    client.publish(topic_status2, "khong day");
  }

  if (full1 || full2) {
    // NEW: chỉ bật còi nếu chưa bị mute và còi chưa đang kêu
    if (!buzzerMuted && !buzzerActive) {
      buzzerActive = true;
      buzzerState  = true;
      buzzerLast   = millis();
      digitalWrite(BUZZER_PIN, HIGH);
    }
  } else {
    // Thùng đã được dọn → reset mute để lần đầy tiếp theo còi kêu lại bình thường
    buzzerActive = false;
    buzzerState  = false;
    buzzerMuted  = false;   // NEW
    digitalWrite(BUZZER_PIN, LOW);
  }
}

void setupWiFi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
}

void reconnect() {
  while (!client.connected()) {
    String clientId = "ESP32Servo-";
    clientId += String(random(0xffff), HEX);

    if (client.connect(clientId.c_str())) {
      client.subscribe(topic_sub);
      client.subscribe(topic_buzzer);   // NEW: subscribe topic tắt còi
    } else {
      delay(2000);
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (unsigned int i = 0; i < length; i++) {
    msg += (char)payload[i];
  }

  // ── Điều khiển servo ──────────────────────────────────────
  if (strcmp(topic, topic_sub) == 0) {
    if (msg == "1" && !servo1Busy) {
      servo1.write(90);
      servo1Start = millis();
      servo1Busy = true;
      lastSensorRead = millis();
    }
    if (msg == "0" && !servo2Busy) {
      servo2.write(90);
      servo2Start = millis();
      servo2Busy = true;
      lastSensorRead = millis();
    }
  }

  // ── Tắt còi (NEW) ────────────────────────────────────────
  // Gửi "off" đến esp32/buzzer/control để tắt còi đang kêu.
  // Còi sẽ tự kêu lại khi thùng được dọn rồi đầy trở lại.
  if (strcmp(topic, topic_buzzer) == 0) {
    if (msg == "off" && buzzerActive) {
      buzzerActive = false;
      buzzerState  = false;
      buzzerMuted  = true;          // đánh dấu đã mute thủ công
      digitalWrite(BUZZER_PIN, LOW);
    }
  }
}

void setup() {
  servo1.setPeriodHertz(50);
  servo2.setPeriodHertz(50);
  servo1.attach(servo1Pin, 500, 2400);
  servo2.attach(servo2Pin, 500, 2400);
  servo1.write(0);
  servo2.write(0);

  pinMode(TRIG1, OUTPUT);
  pinMode(ECHO1, INPUT);
  pinMode(TRIG2, OUTPUT);
  pinMode(ECHO2, INPUT);

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  setupWiFi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  if (servo1Busy && millis() - servo1Start >= 3000) {
    servo1.write(0);
    servo1Busy = false;
  }
  if (servo2Busy && millis() - servo2Start >= 3000) {
    servo2.write(0);
    servo2Busy = false;
  }

  if (!servo1Busy && !servo2Busy) {
    if (millis() - lastSensorRead >= SENSOR_INTERVAL) {
      lastSensorRead = millis();
      publishSensors();
    }
  }

  handleBuzzer();
}
