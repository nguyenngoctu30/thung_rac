# Hệ Thống Thùng Rác Thông Minh

Ngắn gọn: dự án IoT minh họa hệ thống phân loại rác và giám sát mức đầy thùng bằng ESP32, cảm biến siêu âm, servo và mô-đun nhận diện hình ảnh (YOLO) trên Raspberry Pi.

**Repository structure**
- [main.ino](main.ino) : Firmware ESP32 (MQTT client, servo, cảm biến siêu âm, buzzer).
- [index.html](index.html) : Dashboard web (MQTT over WebSocket, hiển thị trạng thái thùng, logs, alert).
- [t.py](t.py) : Đoạn mã Python cho Raspberry Pi (YOLOv8 dual-model, weighted NMS, gửi lệnh MQTT để điều khiển servo).

## Tổng quan hoạt động
- Camera + Raspberry Pi chạy `t.py` phát hiện `organic` / `inorganic` và gửi `1` hoặc `0` lên topic MQTT `esp32/servo/control`.
- ESP32 (firmware trong `main.ino`) đọc cảm biến siêu âm rồi publish khoảng cách và trạng thái lên các topic `esp32/ultrasonic/sensorX` và `esp32/ultrasonic/statusX`.
- Dashboard (`index.html`) kết nối tới broker MQTT (mặc định sử dụng HiveMQ public) qua WebSocket để hiển thị trạng thái thời gian thực.

## MQTT (mặc định)
- Broker web client: `wss://broker.hivemq.com:8884/mqtt` (được cấu hình trong `index.html`).
- ESP32 kết nối tới `broker.hivemq.com:1883` (xem `main.ino`).
- Topics chính:
  - `esp32/servo/control` — payload `"1"` = mở servo thùng hữu cơ, `"0"` = mở servo thùng vô cơ.
  - `esp32/ultrasonic/sensor1` — giá trị float khoảng cách cảm biến 1 (cm).
  - `esp32/ultrasonic/sensor2` — giá trị float khoảng cách cảm biến 2 (cm).
  - `esp32/ultrasonic/status1` / `status2` — trạng thái `day` hoặc `khong day` (full / not full).

## Phần cứng (tóm tắt)
- ESP32 (ví dụ: ESP32 DevKit)
- 2x cảm biến siêu âm HC-SR04 (hoặc tương tự) -> pins trong `main.ino`: TRIG1=5, ECHO1=18, TRIG2=22, ECHO2=23.
- 2x servo (servo1 pin 19, servo2 pin 25) — dùng thư viện `ESP32Servo`.
- Buzzer pin: 26.

## Firmware ESP32 (`main.ino`) — hướng dẫn nhanh
- Thư viện cần cài: `PubSubClient`, `ESP32Servo` (IDE Arduino hoặc PlatformIO).
- Trước khi upload, chỉnh SSID/Password Wi‑Fi ở đầu file `main.ino`.
- Cổng và topics có thể tùy chỉnh trong file.

## Raspberry Pi / Computer (mô-đun nhận diện) — `t.py`
- Yêu cầu Python packages: `ultralytics`, `paho-mqtt`, `opencv-python`, `numpy`.
- Lưu ý model: cập nhật `MODEL_A_PATH`/`MODEL_B_PATH` trong `t.py` tới file `.pt` của bạn.
- Chạy trong virtualenv (ví dụ `yolovenv`):

  `source yolovenv/bin/activate`

  `python3 t.py`

- `t.py` có các tính năng: dual-model YOLO, weighted NMS, motion-trigger, adaptive skip-frame để tối ưu trên Pi.

## Dashboard web (`index.html`)
- Mở `index.html` trực tiếp trên trình duyệt (hoặc host qua webserver). File sử dụng MQTT over WebSocket (client lib từ CDN).
- Nếu dùng broker khác, thay `BROKER_URL` trong file.

## Lưu ý bảo mật & vận hành
- Broker HiveMQ công khai dùng cho demo; với triển khai thực tế hãy dùng broker riêng, xác thực TLS.
- Không commit thông tin Wi‑Fi hoặc khoá nhạy cảm vào repo.

## Ghi chú cuối
- Yêu cầu của bạn: chỉ xem mã và không thay đổi mã — README được thêm như tài liệu dự án, không chỉnh sửa mã nguồn.
- File README này đã được tạo ở gốc repo: [README.md](README.md)

Nếu muốn, tôi có thể: liệt kê các bước upload firmware (Arduino CLI), tạo script nhỏ để host dashboard, hoặc soạn file `requirements.txt` cho `t.py`.
