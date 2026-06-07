#!/usr/bin/env python3
"""
YOLOv8 Dual-Model + Weighted NMS + MQTT - Raspberry Pi 5 + USB Camera
Tối ưu độ trễ: async NMS, skip-frame thích ứng, warmup, motion trigger.
- Detect "organic"   → gửi "1" lên MQTT → ESP32 quay servo
- Detect "inorganic" → gửi "0" lên MQTT
Chạy : python3 detect_camera.py
Thoát: 'q' | Chụp: 's'

THAY ĐỔI SO VỚI BẢN CŨ:
- ROI mở rộng toàn frame (X: 0%–100%, Y: 0%–100%) — tắt hoàn toàn lọc ROI
- MAX_BOX_AREA_RATIO tăng lên 0.95 (nhận vật gần như toàn frame)
- MIN_BOX_AREA_RATIO giảm xuống 0.0005 (nhận vật nhỏ hơn)
- MAX_ASPECT_RATIO nới lỏng lên 8.0
- Bỏ rule cắt cạnh trái (x1 <= 2)
- SKIP_IDLE = 0 — không bỏ frame nào, luôn infer liên tục
- CONF_THRESH giảm xuống 0.25 — nhận cả detection confidence thấp
- IMG_SIZE tăng lên 480 — giữ detail hơn khi resize
"""

import cv2
import time
import sys
import threading
import numpy as np
from pathlib import Path
from collections import deque

try:
    from ultralytics import YOLO
except ImportError:
    print("[LỖI] Chưa cài ultralytics. Chạy: pip install ultralytics")
    sys.exit(1)

try:
    import paho.mqtt.client as mqtt
except ImportError:
    print("[LỖI] Chưa cài paho-mqtt. Chạy: pip install paho-mqtt")
    sys.exit(1)


# ══════════════════════════════════════════════════════════
#  CẤU HÌNH
# ══════════════════════════════════════════════════════════
MODEL_A_PATH   = "/home/pi/tu/run_cv/best.pt"
MODEL_B_PATH   = "/home/pi/tu/best.pt"
CAMERA_INDEX   = 0
CONF_THRESH    = 0.60          # giảm mạnh: nhận detection confidence thấp (chai trong suốt, góc nghiêng)
IMG_SIZE       = 480           # tăng từ 320→480: giữ detail hơn, ít bỏ sót vật lớn
CAM_WIDTH      = 640
CAM_HEIGHT     = 480
CAM_FPS        = 30
NMS_IOU_THRESH = 0.45

# ── Lọc box ──────────────────────────────────────────────
MAX_BOX_AREA_RATIO = 0.55      # nhận vật chiếm gần toàn frame
MIN_BOX_AREA_RATIO = 0.0005    # nhận vật rất nhỏ
MAX_ASPECT_RATIO   = 8.0       # nới lỏng tối đa

# Vùng ROI: TẮT HOÀN TOÀN — nhận toàn bộ frame
ROI_X_MIN = 0.0
ROI_X_MAX = 1.0
ROI_Y_MIN = 0.0
ROI_Y_MAX = 1.0

# ── Skip-frame thích ứng ──────────────────────────────────
SKIP_IDLE      = 0             # không bỏ frame nào — luôn infer liên tục
SKIP_ACTIVE    = 0
MOTION_THRESH  = 1200          # pixel diff để coi là "có chuyển động"

# ── MQTT ─────────────────────────────────────────────────
MQTT_SERVER    = "broker.hivemq.com"
MQTT_PORT      = 1883
MQTT_TOPIC     = "esp32/servo/control"
MQTT_COOLDOWN  = 2.0


# ══════════════════════════════════════════════════════════
#  MQTT
# ══════════════════════════════════════════════════════════
class MQTTSender:
    def __init__(self, broker, port, topic):
        self.topic     = topic
        self.last_sent = 0.0
        self.last_msg  = None
        self._lock     = threading.Lock()

        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.client.on_connect    = self._on_connect
        self.client.on_disconnect = self._on_disconnect
        self._connected = False

        try:
            self.client.connect_async(broker, port, keepalive=60)
            self.client.loop_start()
        except Exception as e:
            print(f"[MQTT] Không kết nối được: {e}")

    def _on_connect(self, client, userdata, flags, rc, props=None):
        if rc == 0:
            self._connected = True
            print(f"[MQTT] Kết nối thành công → {MQTT_SERVER}:{MQTT_PORT}")
        else:
            print(f"[MQTT] Lỗi kết nối rc={rc}")

    def _on_disconnect(self, client, userdata, rc, props=None, reason=None):
        self._connected = False
        print("[MQTT] Mất kết nối, đang thử lại...")

    def send(self, message: str):
        with self._lock:
            now = time.time()
            if message == self.last_msg and now - self.last_sent < MQTT_COOLDOWN:
                return
            if not self._connected:
                print(f"[MQTT] Chưa kết nối, bỏ qua: {message}")
                return
            self.client.publish(self.topic, message)
            self.last_sent = now
            self.last_msg  = message
            print(f"[MQTT] Gửi → topic={self.topic}  msg={message}")

    def stop(self):
        self.client.loop_stop()
        self.client.disconnect()


# ══════════════════════════════════════════════════════════
#  WEIGHTED NMS
# ══════════════════════════════════════════════════════════
def iou(b1, b2):
    ix1 = max(b1[0], b2[0]); iy1 = max(b1[1], b2[1])
    ix2 = min(b1[2], b2[2]); iy2 = min(b1[3], b2[3])
    inter = max(0, ix2 - ix1) * max(0, iy2 - iy1)
    if inter == 0:
        return 0.0
    a1 = (b1[2] - b1[0]) * (b1[3] - b1[1])
    a2 = (b2[2] - b2[0]) * (b2[3] - b2[1])
    return inter / (a1 + a2 - inter)


def weighted_nms(boxes_a, boxes_b, names_a, names_b, wa, wb, iou_thresh):
    combined = []
    for (x1, y1, x2, y2, conf, cls) in boxes_a:
        combined.append((x1, y1, x2, y2, conf, conf * wa, names_a.get(cls, str(cls))))
    for (x1, y1, x2, y2, conf, cls) in boxes_b:
        combined.append((x1, y1, x2, y2, conf, conf * wb, names_b.get(cls, str(cls))))

    combined.sort(key=lambda x: x[5], reverse=True)
    suppressed = [False] * len(combined)
    kept = []
    for i in range(len(combined)):
        if suppressed[i]: continue
        kept.append(combined[i])
        for j in range(i + 1, len(combined)):
            if not suppressed[j] and iou(combined[i], combined[j]) >= iou_thresh:
                suppressed[j] = True
    return [(b[0], b[1], b[2], b[3], b[4], b[6]) for b in kept]
def filter_boxes(boxes_raw, frame_w, frame_h):
    """
    Lọc box — thêm kiểm tra area ratio để loại false positive toàn frame
    """
    frame_area = frame_w * frame_h
    kept = []
    for b in boxes_raw:
        x1, y1, x2, y2 = b[0], b[1], b[2], b[3]
        bw = x2 - x1
        bh = y2 - y1

        if bw <= 0 or bh <= 0:
            continue

        # --- THÊM MỚI: loại box quá to (background/container bị nhận nhầm)
        box_area = bw * bh
        area_ratio = box_area / frame_area
        if area_ratio > MAX_BOX_AREA_RATIO:
            continue
        if area_ratio < MIN_BOX_AREA_RATIO:
            continue

        aspect = max(bw, bh) / max(min(bw, bh), 1)
        if aspect > MAX_ASPECT_RATIO:
            continue

        kept.append(b)
    return kept


# ══════════════════════════════════════════════════════════
#  FRAME BUFFER — chỉ giữ frame mới nhất
# ══════════════════════════════════════════════════════════
class FrameBuffer:
    """Lock-free double-buffer: writer không bao giờ block."""
    def __init__(self):
        self._frame    = None
        self._frame_id = 0
        self._lock     = threading.Lock()

    def write(self, frame):
        with self._lock:
            self._frame_id += 1
            self._frame = frame

    def read(self):
        with self._lock:
            if self._frame is None:
                return 0, None
            return self._frame_id, self._frame.copy()


class ResultBuffer:
    def __init__(self):
        self._boxes    = []
        self._frame_id = -1
        self._lock     = threading.Lock()

    def write(self, boxes, frame_id):
        with self._lock:
            self._boxes    = boxes
            self._frame_id = frame_id

    def read(self):
        with self._lock:
            return self._frame_id, list(self._boxes)


# ══════════════════════════════════════════════════════════
#  CAPTURE THREAD
# ══════════════════════════════════════════════════════════
class CaptureThread(threading.Thread):
    def __init__(self, cap, buf, stop_evt):
        super().__init__(daemon=True)
        self.cap  = cap
        self.buf  = buf
        self.stop = stop_evt

    def run(self):
        while not self.stop.is_set():
            ret, frame = self.cap.read()
            if ret:
                self.buf.write(frame)
            else:
                time.sleep(0.005)


# ══════════════════════════════════════════════════════════
#  INFER THREAD
# ══════════════════════════════════════════════════════════
class InferThread(threading.Thread):
    def __init__(self, model, frame_buf, result_buf, stop_evt, skip_ref, name=""):
        super().__init__(daemon=True, name=name)
        self.model      = model
        self.frame_buf  = frame_buf
        self.result_buf = result_buf
        self.stop       = stop_evt
        self.skip_ref   = skip_ref
        self.infer_fps  = 0.0
        self._skip_cnt  = 0

    def run(self):
        prev     = time.time()
        last_fid = -1

        while not self.stop.is_set():
            fid, frame = self.frame_buf.read()

            if frame is None or fid == last_fid:
                time.sleep(0.003)
                continue

            skip = self.skip_ref[0]
            if skip > 0 and self._skip_cnt < skip:
                self._skip_cnt += 1
                last_fid = fid
                continue
            self._skip_cnt = 0
            last_fid = fid

            results = self.model.predict(
                source=frame,
                conf=CONF_THRESH,
                imgsz=IMG_SIZE,
                verbose=False,
                device="cpu",
                half=False,
                augment=False,
            )

            boxes = []
            for r in results:
                for box in r.boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    boxes.append((x1, y1, x2, y2, float(box.conf[0]), int(box.cls[0])))

            self.result_buf.write(boxes, fid)
            now = time.time()
            self.infer_fps = 1.0 / max(now - prev, 1e-6)
            prev = now


# ══════════════════════════════════════════════════════════
#  NMS THREAD
# ══════════════════════════════════════════════════════════
class NMSThread(threading.Thread):
    def __init__(self, result_a, result_b, model_a, model_b,
                 wa, wb, shared_boxes, shared_lock, stop_evt,
                 frame_buf, skip_ref, mqtt_sender):
        super().__init__(daemon=True)
        self.ra         = result_a
        self.rb         = result_b
        self.names_a    = model_a.names
        self.names_b    = model_b.names
        self.wa         = wa
        self.wb         = wb
        self.boxes_out  = shared_boxes
        self.lock       = shared_lock
        self.stop       = stop_evt
        self.frame_buf  = frame_buf
        self.skip_ref   = skip_ref
        self.mqtt       = mqtt_sender
        self._prev_fid_a = -1
        self._prev_fid_b = -1

    def run(self):
        while not self.stop.is_set():
            fid_a, boxes_a = self.ra.read()
            fid_b, boxes_b = self.rb.read()

            if fid_a == self._prev_fid_a and fid_b == self._prev_fid_b:
                time.sleep(0.005)
                continue

            self._prev_fid_a = fid_a
            self._prev_fid_b = fid_b

            _, frame = self.frame_buf.read()
            if frame is None:
                continue
            fh, fw = frame.shape[:2]

            fa = filter_boxes(boxes_a, fw, fh)
            fb = filter_boxes(boxes_b, fw, fh)

            new_boxes = weighted_nms(
                fa, fb,
                self.names_a, self.names_b,
                self.wa, self.wb, NMS_IOU_THRESH,
            )

            # Chỉ giữ box có nhãn hợp lệ (organic / inorganic)
            # Loại bỏ false positive không liên quan
            VALID_LABELS = ("organic", "inorganic")
            new_boxes = [
                b for b in new_boxes
                if any(v in b[5].lower() for v in VALID_LABELS)
            ]

            with self.lock:
                self.boxes_out.clear()
                self.boxes_out.extend(new_boxes)

            self.skip_ref[0] = SKIP_IDLE if not new_boxes else SKIP_ACTIVE

            decide_mqtt(new_boxes, self.mqtt)


# ══════════════════════════════════════════════════════════
#  VẼ
# ══════════════════════════════════════════════════════════
# BGR color map theo nhãn — dễ nhớ, không bị lẫn xanh dương
LABEL_COLORS = {
    "organic":   (0, 200, 0),      # xanh lá
    "inorganic": (0, 80, 255),     # cam
}
COLOR_DEFAULT = (0, 255, 255)      # vàng — cho nhãn lạ/không xác định


def get_label_color(label: str):
    low = label.lower()
    for key, col in LABEL_COLORS.items():
        if key in low:
            return col
    return COLOR_DEFAULT


def draw(frame, boxes_with_label):
    for (x1, y1, x2, y2, conf, label) in boxes_with_label:
        color = get_label_color(label)
        text  = f"{label}: {conf:.2f}"
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
        ty = max(y1, th + 6)
        cv2.rectangle(frame, (x1, ty - th - 6), (x1 + tw + 4, ty), color, -1)
        cv2.putText(frame, text, (x1 + 2, ty - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
    return frame


# ══════════════════════════════════════════════════════════
#  MQTT LOGIC
# ══════════════════════════════════════════════════════════
def decide_mqtt(boxes_with_label, mqtt_sender: MQTTSender):
    """
    Khi conflict organic vs inorganic trong cùng frame:
    - Lấy box có confidence CAO NHẤT, bỏ qua box kia
    """
    if not boxes_with_label:
        return

    # Sắp xếp theo confidence giảm dần, lấy nhãn của box tốt nhất
    best = max(boxes_with_label, key=lambda b: b[4])
    label = best[5].lower()

    if "inorganic" in label:
        mqtt_sender.send("0")
    elif "organic" in label:
        mqtt_sender.send("1")


# ══════════════════════════════════════════════════════════
#  MOTION DETECTOR
# ══════════════════════════════════════════════════════════
class MotionDetector:
    def __init__(self, threshold=MOTION_THRESH):
        self._prev  = None
        self.thresh = threshold

    def has_motion(self, frame) -> bool:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (7, 7), 0)
        if self._prev is None:
            self._prev = gray
            return False
        diff  = cv2.absdiff(self._prev, gray)
        score = int(np.sum(diff > 25))
        self._prev = gray
        return score > self.thresh


# ══════════════════════════════════════════════════════════
#  HELPERS
# ══════════════════════════════════════════════════════════
def load_model(path):
    p = Path(path)
    if not p.exists():
        print(f"[LỖI] Không tìm thấy: {path}")
        sys.exit(1)
    print(f"[INFO] Tải model: {path}")
    m = YOLO(path)
    print(f"[INFO]   → {len(m.names)} class: {list(m.names.values())}")
    return m


def warmup_model(model, name="model"):
    print(f"[INFO] Warmup {name}...")
    dummy = np.zeros((CAM_HEIGHT, CAM_WIDTH, 3), dtype=np.uint8)
    model.predict(source=dummy, conf=CONF_THRESH, imgsz=IMG_SIZE,
                  verbose=False, device="cpu")
    print(f"[INFO] Warmup {name} xong.")


def compute_weights(model_a, model_b):
    na = len(model_a.names); nb = len(model_b.names)
    total = na + nb
    wa = round(na / total * 2, 3); wb = round(nb / total * 2, 3)
    print(f"[INFO] Model A: {na} class → weight {wa}")
    print(f"[INFO] Model B: {nb} class → weight {wb}")
    return wa, wb


# ══════════════════════════════════════════════════════════
#  MAIN
# ══════════════════════════════════════════════════════════
def main():
    model_a = load_model(MODEL_A_PATH)
    model_b = load_model(MODEL_B_PATH)

    warmup_model(model_a, "Model A")
    warmup_model(model_b, "Model B")

    wa, wb = compute_weights(model_a, model_b)

    mqtt_sender = MQTTSender(MQTT_SERVER, MQTT_PORT, MQTT_TOPIC)

    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        print(f"[LỖI] Không mở được camera {CAMERA_INDEX}")
        sys.exit(1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  CAM_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS,          CAM_FPS)
    cap.set(cv2.CAP_PROP_BUFFERSIZE,   1)
    print(f"[INFO] Camera OK ({int(cap.get(3))}x{int(cap.get(4))})")

    stop       = threading.Event()
    frame_buf  = FrameBuffer()
    result_a   = ResultBuffer()
    result_b   = ResultBuffer()
    skip_ref   = [SKIP_IDLE]

    shared_boxes = []
    shared_lock  = threading.Lock()

    motion_det = MotionDetector()

    capture_t = CaptureThread(cap, frame_buf, stop)
    infer_a   = InferThread(model_a, frame_buf, result_a, stop, skip_ref, "InferA")
    infer_b   = InferThread(model_b, frame_buf, result_b, stop, skip_ref, "InferB")
    nms_t     = NMSThread(result_a, result_b, model_a, model_b,
                          wa, wb, shared_boxes, shared_lock, stop,
                          frame_buf, skip_ref, mqtt_sender)

    capture_t.start()
    infer_a.start()
    infer_b.start()
    nms_t.start()

    print("[INFO] Đang chạy... 'q' thoát | 's' chụp ảnh")

    prev_time = time.time()
    snap_id   = 0

    while True:
        _, frame = frame_buf.read()
        if frame is None:
            time.sleep(0.005)
            continue

        if motion_det.has_motion(frame):
            skip_ref[0] = SKIP_ACTIVE

        with shared_lock:
            cur_boxes = list(shared_boxes)

        frame = draw(frame, cur_boxes)

        now       = time.time()
        ui_fps    = 1.0 / max(now - prev_time, 1e-6)
        prev_time = now

        skip_label = "ACTIVE" if skip_ref[0] == 0 else f"IDLE(skip={skip_ref[0]})"
        cv2.putText(frame,
                    f"UI:{ui_fps:.0f}  A:{infer_a.infer_fps:.0f}fps  B:{infer_b.infer_fps:.0f}fps  [{skip_label}]",
                    (8, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 220, 255), 2)
        cv2.putText(frame, f"Objects: {len(cur_boxes)}",
                    (8, 56), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 220, 255), 2)

       # cv2.imshow("Dual-Model NMS - Pi5", frame)

       # key = cv2.waitKey(1) & 0xFF
       # if key == ord("q"):
       #     break
       # elif key == ord("s"):
       #     fname = f"snap_{snap_id:04d}.jpg"
       #     cv2.imwrite(fname, frame)
       #     print(f"[INFO] Lưu: {fname}")
       #     snap_id += 1

    stop.set()
    for t in [capture_t, infer_a, infer_b, nms_t]:
        t.join(timeout=2)
    cap.release()
    #cv2.destroyAllWindows()
    mqtt_sender.stop()
    print("[INFO] Đã thoát.")


if __name__ == "__main__":
    main()
