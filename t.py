#!/usr/bin/env python3
#tệp chạy /home/pi/tu/run_cv/t.py
#source truoc khi chạy: source yolovenv/bin/activate

"""
YOLOv8 Dual-Model + Weighted NMS + MQTT - Raspberry Pi 5 + USB Camera
Tá»‘i Æ°u Ä‘á»™ trá»…: async NMS, skip-frame thĂ­ch á»©ng, warmup, motion trigger.
- Detect "organic"   â†’ gá»­i "1" lĂªn MQTT â†’ ESP32 quay servo
- Detect "inorganic" â†’ gá»­i "0" lĂªn MQTT
Cháº¡y : python3 detect_camera.py
ThoĂ¡t: 'q' | Chá»¥p: 's'
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
    print("[Lá»–I] ChÆ°a cĂ i ultralytics. Cháº¡y: pip install ultralytics")
    sys.exit(1)

try:
    import paho.mqtt.client as mqtt
except ImportError:
    print("[Lá»–I] ChÆ°a cĂ i paho-mqtt. Cháº¡y: pip install paho-mqtt")
    sys.exit(1)


# â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
#  Cáº¤U HĂŒNH
# â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
MODEL_A_PATH   = "/home/pi/tu/run_cv/best.pt"
MODEL_B_PATH   = "/home/pi/tu/best.pt"
CAMERA_INDEX   = 0
CONF_THRESH    = 0.45          # tÄƒng nháº¹ Ä‘á»ƒ loáº¡i false positive yáº¿u
IMG_SIZE       = 320
CAM_WIDTH      = 640
CAM_HEIGHT     = 480
CAM_FPS        = 30
NMS_IOU_THRESH     = 0.45

# â”€â”€ Lá»c box â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
MAX_BOX_AREA_RATIO = 0.25      # giáº£m tá»« 0.35 â†’ 0.25: loáº¡i box chiáº¿m > 1/4 frame
MIN_BOX_AREA_RATIO = 0.001     # loáº¡i box quĂ¡ nhá» (nhiá»…u Ä‘iá»ƒm áº£nh)
MAX_ASPECT_RATIO   = 4.0       # loáº¡i box quĂ¡ dĂ i/dáº¹t (thÆ°á»ng lĂ  cáº¡nh tÆ°á»ng)
# VĂ¹ng ROI: chá»‰ nháº­n box cĂ³ tĂ¢m náº±m trong vĂ¹ng nĂ y (0.0â€“1.0)
# Cáº¯t bá» viá»n trĂ¡i 20% vĂ  pháº£i 5% nÆ¡i hay bá»‹ nháº­n nháº§m tÆ°á»ng
ROI_X_MIN = 0.20               # bá» 20% bĂªn trĂ¡i
ROI_X_MAX = 0.95               # bá» 5% bĂªn pháº£i
ROI_Y_MIN = 0.05
ROI_Y_MAX = 0.95

# â”€â”€ Skip-frame thĂ­ch á»©ng â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
# Náº¿u khĂ´ng cĂ³ váº­t (boxes rá»—ng N frame liĂªn tiáº¿p) â†’ tÄƒng skip
# Ngay khi detect tháº¥y gĂ¬ â†’ giáº£m vá» 0 skip
SKIP_IDLE      = 2            # bá» qua N frame khi khĂ´ng cĂ³ váº­t
SKIP_ACTIVE    = 0            # khĂ´ng bá» frame khi Ä‘ang detect
MOTION_THRESH  = 1200         # pixel diff Ä‘á»ƒ coi lĂ  "cĂ³ chuyá»ƒn Ä‘á»™ng"

# â”€â”€ MQTT â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
MQTT_SERVER    = "broker.hivemq.com"
MQTT_PORT      = 1883
MQTT_TOPIC     = "esp32/servo/control"
MQTT_COOLDOWN  = 2.0


# â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
#  MQTT
# â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
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
            print(f"[MQTT] KhĂ´ng káº¿t ná»‘i Ä‘Æ°á»£c: {e}")

    def _on_connect(self, client, userdata, flags, rc, props=None):
        if rc == 0:
            self._connected = True
            print(f"[MQTT] Káº¿t ná»‘i thĂ nh cĂ´ng â†’ {MQTT_SERVER}:{MQTT_PORT}")
        else:
            print(f"[MQTT] Lá»—i káº¿t ná»‘i rc={rc}")

    def _on_disconnect(self, client, userdata, rc, props=None, reason=None):
        self._connected = False
        print("[MQTT] Máº¥t káº¿t ná»‘i, Ä‘ang thá»­ láº¡i...")

    def send(self, message: str):
        with self._lock:
            now = time.time()
            if message == self.last_msg and now - self.last_sent < MQTT_COOLDOWN:
                return
            if not self._connected:
                print(f"[MQTT] ChÆ°a káº¿t ná»‘i, bá» qua: {message}")
                return
            self.client.publish(self.topic, message)
            self.last_sent = now
            self.last_msg  = message
            print(f"[MQTT] Gá»­i â†’ topic={self.topic}  msg={message}")

    def stop(self):
        self.client.loop_stop()
        self.client.disconnect()


# â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
#  WEIGHTED NMS
# â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
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
    Lá»c false-positive báº±ng 4 tiĂªu chĂ­:
    1. Diá»‡n tĂ­ch box / frame trong [MIN_BOX_AREA_RATIO, MAX_BOX_AREA_RATIO]
    2. Aspect ratio khĂ´ng quĂ¡ dĂ i/dáº¹t (trĂ¡nh nháº­n cáº¡nh tÆ°á»ng)
    3. TĂ¢m box náº±m trong vĂ¹ng ROI (cáº¯t viá»n hay bá»‹ nháº§m)
    4. Box khĂ´ng dĂ­nh sĂ¡t cáº¡nh trĂ¡i frame
    """
    frame_area = frame_w * frame_h
    kept = []
    for b in boxes_raw:
        x1, y1, x2, y2 = b[0], b[1], b[2], b[3]
        bw   = x2 - x1
        bh   = y2 - y1
        area = bw * bh

        # 1. Diá»‡n tĂ­ch
        ratio = area / frame_area
        if ratio > MAX_BOX_AREA_RATIO or ratio < MIN_BOX_AREA_RATIO:
            continue

        # 2. Aspect ratio (cáº¡nh tÆ°á»ng thÆ°á»ng ráº¥t dĂ i/dáº¹t)
        aspect = max(bw, bh) / max(min(bw, bh), 1)
        if aspect > MAX_ASPECT_RATIO:
            continue

        # 3. TĂ¢m box pháº£i náº±m trong ROI
        cx = (x1 + x2) / 2 / frame_w
        cy = (y1 + y2) / 2 / frame_h
        if not (ROI_X_MIN <= cx <= ROI_X_MAX and ROI_Y_MIN <= cy <= ROI_Y_MAX):
            continue

        # 4. KhĂ´ng dĂ­nh sĂ¡t cáº¡nh trĂ¡i (thÆ°á»ng lĂ  tÆ°á»ng/ná»n)
        if x1 <= 2:
            continue

        kept.append(b)
    return kept


# â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
#  FRAME BUFFER â€” chá»‰ giá»¯ frame má»›i nháº¥t
# â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
class FrameBuffer:
    """Lock-free double-buffer: writer khĂ´ng bao giá» block."""
    def __init__(self):
        self._frame    = None
        self._frame_id = 0
        self._lock     = threading.Lock()

    def write(self, frame):
        with self._lock:
            self._frame_id += 1
            self._frame = frame          # chá»‰ giá»¯ frame Má»I NHáº¤T

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


# â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
#  CAPTURE THREAD
# â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
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
                # ÄĂƒ Sá»¬A: dĂ¹ng toĂ n bá»™ frame thay vĂ¬ chá»‰ ná»­a trĂªn
                self.buf.write(frame)
            else:
                time.sleep(0.005)


# â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
#  INFER THREAD â€” luĂ´n láº¥y frame Má»I NHáº¤T, bá» frame cÅ©
# â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
class InferThread(threading.Thread):
    def __init__(self, model, frame_buf, result_buf, stop_evt, skip_ref, name=""):
        super().__init__(daemon=True, name=name)
        self.model      = model
        self.frame_buf  = frame_buf
        self.result_buf = result_buf
        self.stop       = stop_evt
        self.skip_ref   = skip_ref   # list[int] dĂ¹ng chung Ä‘á»ƒ Ä‘á»c skip count
        self.infer_fps  = 0.0
        self._skip_cnt  = 0

    def run(self):
        prev     = time.time()
        last_fid = -1

        while not self.stop.is_set():
            fid, frame = self.frame_buf.read()

            # KhĂ´ng cĂ³ frame má»›i
            if frame is None or fid == last_fid:
                time.sleep(0.003)
                continue

            # Skip-frame thĂ­ch á»©ng
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
                half=False,       # Pi5 khĂ´ng cĂ³ FP16 HW â€” bá» half
                augment=False,    # táº¯t TTA Ä‘á»ƒ nhanh hÆ¡n
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


# â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
#  NMS THREAD â€” tĂ¡ch riĂªng Ä‘á»ƒ main loop khĂ´ng chá»
# â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
class NMSThread(threading.Thread):
    """
    Äá»c káº¿t quáº£ tá»« cáº£ 2 InferThread, cháº¡y weighted NMS,
    ghi vĂ o shared list. KhĂ´ng yĂªu cáº§u fid_a == fid_b ná»¯a â€”
    cháº¥p nháº­n káº¿t quáº£ má»›i nháº¥t tá»« má»—i model (lá»‡ch â‰¤ 1 frame lĂ  á»•n).
    """
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

            # Chá» Ă­t nháº¥t 1 model cĂ³ káº¿t quáº£ má»›i
            if fid_a == self._prev_fid_a and fid_b == self._prev_fid_b:
                time.sleep(0.005)
                continue

            self._prev_fid_a = fid_a
            self._prev_fid_b = fid_b

            # Láº¥y kĂ­ch thÆ°á»›c frame hiá»‡n táº¡i Ä‘á»ƒ filter
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

            with self.lock:
                self.boxes_out.clear()
                self.boxes_out.extend(new_boxes)

            # ThĂ­ch á»©ng skip rate
            self.skip_ref[0] = SKIP_IDLE if not new_boxes else SKIP_ACTIVE

            # Gá»­i MQTT
            decide_mqtt(new_boxes, self.mqtt)


# â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
#  Váº¼
# â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
COLORS = [
    (0, 255, 0), (255, 80, 80), (80, 80, 255), (255, 255, 0),
    (0, 255, 255), (255, 0, 255), (128, 255, 0), (255, 128, 0), (0, 128, 255),
]


def draw(frame, boxes_with_label):
    fh, fw = frame.shape[:2]

    # Váº½ vĂ¹ng ROI (viá»n tráº¯ng má») Ä‘á»ƒ dá»… debug/cÄƒn chá»‰nh
    rx1 = int(ROI_X_MIN * fw); ry1 = int(ROI_Y_MIN * fh)
    rx2 = int(ROI_X_MAX * fw); ry2 = int(ROI_Y_MAX * fh)
    overlay = frame.copy()
    # TĂ´ má» vĂ¹ng ngoĂ i ROI
    cv2.rectangle(overlay, (0, 0), (rx1, fh), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.35, frame, 0.65, 0, frame)
    # Viá»n ROI
    cv2.rectangle(frame, (rx1, ry1), (rx2, ry2), (200, 200, 200), 1)

    for i, (x1, y1, x2, y2, conf, label) in enumerate(boxes_with_label):
        color = COLORS[i % len(COLORS)]
        text  = f"{label}: {conf:.2f}"
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
        cv2.rectangle(frame, (x1, y1 - th - 6), (x1 + tw + 4, y1), color, -1)
        cv2.putText(frame, text, (x1 + 2, y1 - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
    return frame


# â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
#  MQTT LOGIC
# â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
def decide_mqtt(boxes_with_label, mqtt_sender: MQTTSender):
    labels = [label.lower() for (_, _, _, _, _, label) in boxes_with_label]
    if any("organic" in l and "in" not in l for l in labels):
        mqtt_sender.send("1")
    elif any("inorganic" in l for l in labels):
        mqtt_sender.send("0")


# â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
#  MOTION DETECTOR â€” Ä‘á»ƒ trigger infer nhanh hÆ¡n khi cĂ³ váº­t má»›i
# â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
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


# â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
#  HELPERS
# â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
def load_model(path):
    p = Path(path)
    if not p.exists():
        print(f"[Lá»–I] KhĂ´ng tĂ¬m tháº¥y: {path}")
        sys.exit(1)
    print(f"[INFO] Táº£i model: {path}")
    m = YOLO(path)
    print(f"[INFO]   â†’ {len(m.names)} class: {list(m.names.values())}")
    return m


def warmup_model(model, name="model"):
    """Cháº¡y 1 láº§n predict áº£nh Ä‘en Ä‘á»ƒ JIT/compile xong trÆ°á»›c khi dĂ¹ng tháº­t."""
    print(f"[INFO] Warmup {name}...")
    dummy = np.zeros((CAM_HEIGHT // 2, CAM_WIDTH, 3), dtype=np.uint8)
    model.predict(source=dummy, conf=CONF_THRESH, imgsz=IMG_SIZE,
                  verbose=False, device="cpu")
    print(f"[INFO] Warmup {name} xong.")


def compute_weights(model_a, model_b):
    na = len(model_a.names); nb = len(model_b.names)
    total = na + nb
    wa = round(na / total * 2, 3); wb = round(nb / total * 2, 3)
    print(f"[INFO] Model A: {na} class â†’ weight {wa}")
    print(f"[INFO] Model B: {nb} class â†’ weight {wb}")
    return wa, wb


# â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
#  MAIN
# â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
def main():
    model_a = load_model(MODEL_A_PATH)
    model_b = load_model(MODEL_B_PATH)

    # Warmup: trĂ¡nh láº§n infer Ä‘áº§u bá»‹ cháº­m do JIT
    warmup_model(model_a, "Model A")
    warmup_model(model_b, "Model B")

    wa, wb = compute_weights(model_a, model_b)

    mqtt_sender = MQTTSender(MQTT_SERVER, MQTT_PORT, MQTT_TOPIC)

    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        print(f"[Lá»–I] KhĂ´ng má»Ÿ Ä‘Æ°á»£c camera {CAMERA_INDEX}")
        sys.exit(1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  CAM_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS,          CAM_FPS)
    cap.set(cv2.CAP_PROP_BUFFERSIZE,   1)   # giáº£m buffer camera, trĂ¡nh frame cÅ©
    print(f"[INFO] Camera OK ({int(cap.get(3))}x{int(cap.get(4))})")

    stop       = threading.Event()
    frame_buf  = FrameBuffer()
    result_a   = ResultBuffer()
    result_b   = ResultBuffer()

    # skip_ref[0]: sá»‘ frame bá» qua giá»¯a 2 láº§n infer
    # DĂ¹ng list Ä‘á»ƒ NMSThread cĂ³ thá»ƒ ghi, InferThread Ä‘á»c (shared mutable)
    skip_ref = [SKIP_IDLE]

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

    print("[INFO] Äang cháº¡y... 'q' thoĂ¡t | 's' chá»¥p áº£nh")

    prev_time = time.time()
    snap_id   = 0

    while True:
        _, frame = frame_buf.read()
        if frame is None:
            time.sleep(0.005)
            continue

        # Náº¿u cĂ³ chuyá»ƒn Ä‘á»™ng â†’ reset skip Ä‘á»ƒ infer ngay
        if motion_det.has_motion(frame):
            skip_ref[0] = SKIP_ACTIVE

        # Láº¥y boxes thread-safe
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

        cv2.imshow("Dual-Model NMS - Pi5", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        elif key == ord("s"):
            fname = f"snap_{snap_id:04d}.jpg"
            cv2.imwrite(fname, frame)
            print(f"[INFO] LÆ°u: {fname}")
            snap_id += 1

    stop.set()
    for t in [capture_t, infer_a, infer_b, nms_t]:
        t.join(timeout=2)
    cap.release()
    cv2.destroyAllWindows()
    mqtt_sender.stop()
    print("[INFO] ÄĂ£ thoĂ¡t.")


if __name__ == "__main__":
    main()
