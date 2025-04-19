from picamera2 import Picamera2
import cv2
import numpy as np
import serial
import time
import math
import json

# --- Cấu hình UART ---
try:
    ser = serial.Serial('/dev/ttyS0', 115200, timeout=1)
    time.sleep(2)
    print("Serial kết nối thành công.")
    ser.write(b"Hello\n")
except Exception as e:
    ser = None
    print(f"Không thể mở cổng Serial: {e}")

# --- Cấu hình camera ---
picam2 = Picamera2()
picam2.preview_configuration.main.size = (640, 480)
picam2.configure("preview")
picam2.start()
time.sleep(1)

# --- Tham số ---
kernel = np.ones((5, 5), np.uint8)
send_interval = 0.2  # giây
last_send_time = 0
current_command = 0  # 0 = idle

# --- Ngưỡng màu ---
# RED (#FC2A1B) -> HSV khoảng [0-5] và [175-180]
lower_red1 = np.array([0, 120, 120])
upper_red1 = np.array([5, 255, 255])
lower_red2 = np.array([175, 120, 120])
upper_red2 = np.array([180, 255, 255])

# YELLOW (#FAFF37) -> HSV khoảng [25-35]
lower_yellow = np.array([25, 150, 150])
upper_yellow = np.array([35, 255, 255])

# BLUE (#3C00FB) -> HSV khoảng [115-130]
lower_blue = np.array([115, 150, 150])
upper_blue = np.array([130, 255, 255])

# --- Track circle detection ---
detected_circle = None
tracked_center = None
last_detected_circle_command = None
lost_counter = 0
MAX_LOST_FRAMES = 5

# --- Lắng nghe lệnh từ mạch FC ---
def listen_for_fc_command():
    global current_command
    if ser and ser.in_waiting > 0:
        try:
            line = ser.readline().decode().strip()
            if line.isdigit():
                cmd = int(line)
                if 0 <= cmd <= 6:
                    current_command = cmd
                    print(f"Nhận lệnh từ FC: {cmd}")
        except Exception as e:
            print("Lỗi đọc UART:", e)

# --- Gửi offset về FC ---
def send_target_offset(x, y, command_code):
    global last_send_time
    now = time.time()
    if now - last_send_time >= send_interval:
        height, width, _ = frame.shape
        center_x, center_y = width // 2, height // 2
        dx = center_x - x
        dy = y - center_y

        data = {"cmd": command_code, "dx": dx, "dy": dy}
        message = json.dumps(data) + "\n"

        if ser:
            ser.write(message.encode())
            print("Tọa độ gửi:", message.strip())

        last_send_time = now

# --- Process mask for a specific color ---
def process_mask(mask, bgr_color, expected_command, frame):
    global detected_circle, tracked_center, current_command, last_detected_circle_command, lost_counter

    # Nếu nhận lệnh 0, ngừng phát hiện và theo dõi
    if current_command == 0:
        detected_circle = None
        expected_command = None
        tracked_center = None
        last_detected_circle_command = None
        lost_counter = 0  # Reset lost_counter khi ngừng
        print("[CMD 0] Ngừng phát hiện và theo dõi")
        return  # Ngừng xử lý ngay lập tức nếu lệnh là 0

    # Nếu không có frame hoặc mask, không thực hiện gì thêm
    if frame is None or mask is None:
        return

    # Nếu lệnh không phải là lệnh cần thiết, bỏ qua xử lý
    if current_command != expected_command:
        return

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    height, width, _ = frame.shape
    image_center = (width // 2, height // 2)

    candidates = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 500:
            (x, y), radius = cv2.minEnclosingCircle(cnt)
            center = (int(x), int(y))
            candidates.append((center, radius))

    if not candidates:
        if last_detected_circle_command == expected_command:
            lost_counter += 1
            if lost_counter > MAX_LOST_FRAMES:
                detected_circle = None
                tracked_center = None
                last_detected_circle_command = None
                print(f"[CMD {expected_command}] Mất vòng tròn quá {MAX_LOST_FRAMES} frame")
            else:
                print(f"[CMD {expected_command}] Mất tạm thời ({lost_counter})")
        return

    best_candidate = None

    if detected_circle is None or last_detected_circle_command != expected_command:
        if expected_command in [1, 3, 5, 6]:
            best_distance = float('inf')
            for center, radius in candidates:
                distance = math.hypot(center[0] - image_center[0], center[1] - image_center[1])
                if distance < best_distance:
                    best_distance = distance
                    best_candidate = (center, radius)

        elif expected_command in [2, 4]:
            if tracked_center:
                best_distance = 0
                for center, radius in candidates:
                    distance = math.hypot(center[0] - tracked_center[0], center[1] - tracked_center[1])
                    if distance > best_distance:
                        best_distance = distance
                        best_candidate = (center, radius)
            else:
                best_distance = 0
                for center, radius in candidates:
                    distance = math.hypot(center[0] - image_center[0], center[1] - image_center[1])
                    if distance > best_distance:
                        best_distance = distance
                        best_candidate = (center, radius)

        if best_candidate:
            center, radius = best_candidate
            detected_circle = (center, radius, bgr_color, expected_command)
            tracked_center = center
            last_detected_circle_command = expected_command
            lost_counter = 0
            print(f"[CMD {expected_command}] Chọn vòng tại {center}")

    elif last_detected_circle_command == expected_command:
        best_candidate = None
        best_distance = float('inf')

        for center, radius in candidates:
            distance = math.hypot(center[0] - tracked_center[0], center[1] - tracked_center[1])
            if distance < best_distance:
                best_distance = distance
                best_candidate = (center, radius)

        if best_candidate and best_distance < 50:
            center, radius = best_candidate
            tracked_center = center
            detected_circle = (center, radius, bgr_color, expected_command)
            lost_counter = 0

    if detected_circle and last_detected_circle_command == expected_command:
        tracked_center, tracked_radius, _, _ = detected_circle
        draw_radius = int(tracked_radius) + 10  # Tăng thêm cho dễ nhìn
        cv2.circle(frame, tracked_center, draw_radius, bgr_color, 2)
        cv2.putText(frame, f"CMD {expected_command}", (tracked_center[0] - 20, tracked_center[1] - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, bgr_color, 2)
        send_target_offset(tracked_center[0], tracked_center[1], expected_command)

# --- Vòng lặp chính ---
# --- Vòng lặp chính ---
while True:
    listen_for_fc_command()
    frame = picam2.capture_array()
    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    mask_red = cv2.inRange(hsv, lower_red1, upper_red1) | cv2.inRange(hsv, lower_red2, upper_red2)
    mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

    mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)
    mask_yellow = cv2.morphologyEx(mask_yellow, cv2.MORPH_OPEN, kernel)
    mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_OPEN, kernel)

    if current_command in [1, 2, 5]:
        process_mask(mask_yellow, (0, 255, 255), current_command, frame)
    elif current_command in [3, 4]:
        process_mask(mask_red, (0, 0, 255), current_command, frame)
    elif current_command == 6:
        process_mask(mask_blue, (255, 0, 0), current_command, frame)
    elif current_command == 0:
        print("[CMD 0] Không thực hiện xử lý")

    height, width, _ = frame.shape
    center_x, center_y = width // 2, height // 2
    cv2.drawMarker(frame, (center_x, center_y), (255, 255, 255),
                   markerType=cv2.MARKER_CROSS, markerSize=10)
    cv2.imshow("Target Alignment", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

if ser:
    ser.close()
cv2.destroyAllWindows()
