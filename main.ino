#include <WiFi.h>
#include <PubSubClient.h>
#include <ESP32Servo.h>
#include <HardwareSerial.h>
#include <JQ6500_Serial.h>
#include <TinyGPS++.h>

// ============================================================
// --- Cấu hình WiFi ---
// ============================================================
const char* ssid = "677 5G";
const char* password = "10101010";

// ============================================================
// --- Cấu hình MQTT ---
// ============================================================
const char* mqtt_server = "broker.hivemq.com";
const int mqtt_port = 1883;

const char* topic_sub_servo  = "esp32/servo/control";
const char* topic_sub_audio  = "esp32/audio/control";

const char* topic_pub_dist1   = "esp32/bin1/distance";
const char* topic_pub_dist2   = "esp32/bin2/distance";
const char* topic_pub_status1 = "esp32/bin1/status";
const char* topic_pub_status2 = "esp32/bin2/status";

// Cảm biến tiệm cận hồng ngoại
const char* topic_pub_ir1     = "esp32/ir1/status";
const char* topic_pub_ir2     = "esp32/ir2/status";

// GPS
const char* topic_pub_gps_lat = "esp32/gps/latitude";
const char* topic_pub_gps_lng = "esp32/gps/longitude";
const char* topic_pub_gps_spd = "esp32/gps/speed";
const char* topic_pub_gps_alt = "esp32/gps/altitude";
const char* topic_pub_gps_sat = "esp32/gps/satellites";

WiFiClient   espClient;
PubSubClient client(espClient);

// ============================================================
// --- Cấu hình Servo ---
// ============================================================
Servo servo1;
Servo servo2;

const int servo1Pin = 22;
const int servo2Pin = 15;

// ============================================================
// --- Cảm biến siêu âm (HC-SR04/05) ---
// ============================================================
const int trig1Pin = 5;
const int echo1Pin = 18;
const int trig2Pin = 19;
const int echo2Pin = 21;

// ============================================================
// --- Cảm biến tiệm cận hồng ngoại ---
// Chân OUTPUT của module IR nối vào 2 chân này.
// Mức LOW = có vật, mức HIGH = không có vật (thường gặp với
// module FC-51 / TCRT5000). Điều chỉnh logic nếu module của
// bạn ngược lại.
// ============================================================
const int ir1Pin = 34;   // GPIO34 – input only, phù hợp cảm biến
const int ir2Pin = 35;   // GPIO35 – input only

// ============================================================
// --- Cấu hình JQ6500 – chuyển sang Serial1 (chân 4, 2) ---
// TX ESP32 → RX JQ6500 : GPIO4
// RX ESP32 ← TX JQ6500 : GPIO2  (thường không cần)
// ============================================================
HardwareSerial jqSerial(1);          // UART1
JQ6500_Serial  mp3(jqSerial);

// ============================================================
// --- Cấu hình GPS (NEO-7M) – giữ Serial2 (chân 16, 17) ---
// ============================================================
TinyGPSPlus    gps;
HardwareSerial GPSserial(2);         // UART2

const int GPS_RX = 16;
const int GPS_TX = 17;
const uint32_t GPS_BAUD = 9600;

// ============================================================
// --- Biến trạng thái ---
// ============================================================
bool isBin1Full     = false;
bool isBin2Full     = false;
bool isAudioEnabled = true;

int  action = -1;

// IR trạng thái trước (tránh publish liên tục)
bool lastIR1State = false;
bool lastIR2State = false;

// ============================================================
// --- Timing ---
// ============================================================
unsigned long lastServo1Time      = 0;
unsigned long lastServo2Time      = 0;
unsigned long lastMovementTime    = 0;
unsigned long lastSensorCheckTime = 0;
unsigned long lastReconnectAttempt= 0;
unsigned long lastAlert1Time      = 0;
unsigned long lastAlert2Time      = 0;
unsigned long lastGPSPublishTime  = 0;

const unsigned long cooldownPeriod   = 5000;
const unsigned long waitBeforeMeasure= 3000;
const unsigned long alertInterval    = 10000;
const unsigned long gpsPublishInterval = 5000;  // Publish GPS mỗi 5 giây

// ============================================================
// --- Hàm tiện ích ---
// ============================================================

// Delay không làm rớt MQTT
void smartDelay(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    // Đọc GPS trong lúc chờ
    while (GPSserial.available() > 0)
      gps.encode(GPSserial.read());

    if (client.connected())
      client.loop();

    delay(10);
  }
}

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Đang kết nối WiFi: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi đã kết nối thành công!");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
}

// ============================================================
// --- MQTT Callback ---
// ============================================================
void callback(char* topic, byte* payload, unsigned int length) {
  String msg = "";
  for (unsigned int i = 0; i < length; i++)
    msg += (char)payload[i];

  Serial.print("Nhận MQTT [");
  Serial.print(topic);
  Serial.print("]: ");
  Serial.println(msg);

  // Điều khiển Servo
  if (strcmp(topic, topic_sub_servo) == 0) {
    if (msg == "0") {
      if (lastServo1Time == 0 || millis() - lastServo1Time >= cooldownPeriod)
        action = 0;
      else
        Serial.println("-> Bỏ qua: Servo 1 đang cooldown!");
    }
    else if (msg == "1") {
      if (lastServo2Time == 0 || millis() - lastServo2Time >= cooldownPeriod)
        action = 1;
      else
        Serial.println("-> Bỏ qua: Servo 2 đang cooldown!");
    }
  }

  // Bật/Tắt âm thanh
  else if (strcmp(topic, topic_sub_audio) == 0) {
    if (msg == "0") {
      isAudioEnabled = false;
      Serial.println("-> Đã TẮT âm thanh cảnh báo.");
    }
    else if (msg == "1") {
      isAudioEnabled = true;
      Serial.println("-> Đã BẬT âm thanh cảnh báo.");
    }
  }
}

// ============================================================
// --- Reconnect MQTT ---
// ============================================================
void reconnect() {
  Serial.print("Đang kết nối lại MQTT...");
  String clientId = "ESP32Client-";
  clientId += String(random(0xffff), HEX);

  if (client.connect(clientId.c_str())) {
    Serial.println("Thành công!");
    client.subscribe(topic_sub_servo);
    client.subscribe(topic_sub_audio);
  }
  else {
    Serial.print("Thất bại, mã lỗi = ");
    Serial.println(client.state());
  }
}

// ============================================================
// --- Đo khoảng cách ---
// ============================================================
float getDistance(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  long duration = pulseIn(echo, HIGH, 30000);
  if (duration == 0) return 999.0;
  return duration * 0.034 / 2.0;
}

// ============================================================
// --- Publish GPS lên MQTT ---
// ============================================================
void publishGPS() {
  if (!gps.location.isValid()) {
    Serial.println("GPS: Chưa có tín hiệu hợp lệ.");
    client.publish(topic_pub_gps_lat, "NO_FIX");
    client.publish(topic_pub_gps_lng, "NO_FIX");
    return;
  }

  // Latitude / Longitude (6 chữ số thập phân)
  char buf[20];

  dtostrf(gps.location.lat(), 10, 6, buf);
  client.publish(topic_pub_gps_lat, buf);

  dtostrf(gps.location.lng(), 10, 6, buf);
  client.publish(topic_pub_gps_lng, buf);

  // Tốc độ km/h
  if (gps.speed.isValid()) {
    dtostrf(gps.speed.kmph(), 6, 1, buf);
    client.publish(topic_pub_gps_spd, buf);
  }

  // Độ cao (m)
  if (gps.altitude.isValid()) {
    dtostrf(gps.altitude.meters(), 7, 1, buf);
    client.publish(topic_pub_gps_alt, buf);
  }

  // Số vệ tinh
  if (gps.satellites.isValid()) {
    client.publish(topic_pub_gps_sat,
                   String(gps.satellites.value()).c_str());
  }

  Serial.printf("GPS → Lat: %.6f | Lng: %.6f | Spd: %.1f km/h | Alt: %.1f m | Sat: %d\n",
                gps.location.lat(),
                gps.location.lng(),
                gps.speed.isValid()     ? gps.speed.kmph()       : 0.0,
                gps.altitude.isValid()  ? gps.altitude.meters()  : 0.0,
                gps.satellites.isValid()? gps.satellites.value() : 0);
}

// ============================================================
// --- SETUP ---
// ============================================================
void setup() {
  Serial.begin(115200);

  // Siêu âm
  pinMode(trig1Pin, OUTPUT);
  pinMode(echo1Pin, INPUT);
  pinMode(trig2Pin, OUTPUT);
  pinMode(echo2Pin, INPUT);

  // Cảm biến hồng ngoại (INPUT_PULLUP nếu module kéo lên, 
  // hoặc INPUT nếu module đã có trở kéo trên board)
  pinMode(ir1Pin, INPUT);
  pinMode(ir2Pin, INPUT);

  // JQ6500 – UART1, TX=GPIO4, RX=GPIO2
  jqSerial.begin(9600, SERIAL_8N1, 2, 4);
  mp3.reset();
  mp3.setVolume(25);
  mp3.setSource(MP3_SRC_BUILTIN);

  // GPS – UART2, RX=16, TX=17
  GPSserial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
  Serial.println("GPS UART2 khởi động...");

  // Servo
  servo1.attach(servo1Pin);
  servo2.attach(servo2Pin);
  servo1.write(0);
  servo2.write(0);

  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
}

// ============================================================
// --- LOOP ---
// ============================================================
void loop() {
  // --- Đọc GPS liên tục ---
  while (GPSserial.available() > 0)
    gps.encode(GPSserial.read());

  // --- Duy trì kết nối MQTT ---
  if (!client.connected()) {
    if (millis() - lastReconnectAttempt > 5000) {
      lastReconnectAttempt = millis();
      reconnect();
    }
  }
  else {
    client.loop();
  }

  // ============================================================
  // XỬ LÝ SERVO
  // ============================================================
  if (action == 0) {
    Serial.println("-> Mở Servo 1, phát bài 1...");
    mp3.playFileByIndexNumber(1);

    for (int a = 0; a <= 180; a += 5) { servo1.write(a); smartDelay(50); }
    smartDelay(5000);
    for (int a = 180; a >= 0; a -= 5) { servo1.write(a); smartDelay(50); }

    mp3.pause();
    action          = -1;
    lastServo1Time  = millis();
    lastMovementTime= millis();
  }
  else if (action == 1) {
    Serial.println("-> Mở Servo 2, phát bài 2...");
    mp3.playFileByIndexNumber(2);

    for (int a = 0; a <= 180; a += 5) { servo2.write(a); smartDelay(50); }
    smartDelay(5000);
    for (int a = 180; a >= 0; a -= 5) { servo2.write(a); smartDelay(50); }

    mp3.pause();
    action          = -1;
    lastServo2Time  = millis();
    lastMovementTime= millis();
  }

  // ============================================================
  // ĐO KHOẢNG CÁCH + KIỂM TRA IR + PUBLISH
  // ============================================================
  if (action == -1 && (millis() - lastMovementTime >= waitBeforeMeasure)) {
    if (millis() - lastSensorCheckTime >= 1000) {
      lastSensorCheckTime = millis();

      // --- Siêu âm ---
      float dist1 = getDistance(trig1Pin, echo1Pin);
      float dist2 = getDistance(trig2Pin, echo2Pin);

      if (dist1 < 999.0) client.publish(topic_pub_dist1, String(dist1, 1).c_str());
      if (dist2 < 999.0) client.publish(topic_pub_dist2, String(dist2, 1).c_str());

      // Thùng 1
      if (dist1 > 0 && dist1 < 7.0) {
        client.publish(topic_pub_status1, "FULL");
        if (!isBin1Full || millis() - lastAlert1Time >= alertInterval) {
          Serial.printf("-> Thùng 1 ĐẦY! (%.1f cm)\n", dist1);
          if (isAudioEnabled) mp3.playFileByIndexNumber(3);
          isBin1Full    = true;
          lastAlert1Time= millis();
        }
      }
      else {
        client.publish(topic_pub_status1, "EMPTY");
        isBin1Full = false;
      }

      // Thùng 2
      if (dist2 > 0 && dist2 < 7.0) {
        client.publish(topic_pub_status2, "FULL");
        if (!isBin2Full || millis() - lastAlert2Time >= alertInterval) {
          Serial.printf("-> Thùng 2 ĐẦY! (%.1f cm)\n", dist2);
          if (isAudioEnabled) mp3.playFileByIndexNumber(4);
          isBin2Full    = true;
          lastAlert2Time= millis();
        }
      }
      else {
        client.publish(topic_pub_status2, "EMPTY");
        isBin2Full = false;
      }

      // --- Cảm biến hồng ngoại ---
      // Module FC-51/TCRT5000: LOW = phát hiện vật, HIGH = không có vật
      bool ir1Detected = (digitalRead(ir1Pin) == LOW);
      bool ir2Detected = (digitalRead(ir2Pin) == LOW);

      // Chỉ publish khi trạng thái thay đổi (giảm traffic MQTT)
      if (ir1Detected != lastIR1State) {
        client.publish(topic_pub_ir1, ir1Detected ? "DETECTED" : "CLEAR");
        Serial.printf("-> IR1: %s\n", ir1Detected ? "DETECTED" : "CLEAR");
        lastIR1State = ir1Detected;
      }
      if (ir2Detected != lastIR2State) {
        client.publish(topic_pub_ir2, ir2Detected ? "DETECTED" : "CLEAR");
        Serial.printf("-> IR2: %s\n", ir2Detected ? "DETECTED" : "CLEAR");
        lastIR2State = ir2Detected;
      }
    }
  }

  // ============================================================
  // PUBLISH GPS (mỗi 5 giây, độc lập với servo/sensor)
  // ============================================================
  if (millis() - lastGPSPublishTime >= gpsPublishInterval) {
    lastGPSPublishTime = millis();
    if (client.connected()) {
      publishGPS();
    }
  }
}
