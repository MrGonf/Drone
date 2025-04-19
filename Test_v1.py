from picamera2 import Picamera2
import cv2
import numpy as np
import serial
import time

last_send_time = 0
send_interval = 1  # giây

# --- Cấu hình UART ---
try:
    ser = serial.Serial('/dev/ttyS0', 115200, timeout=1)
    time.sleep(2)
    print("Serial kết nối thành công.")
except:
    ser = None
    print("Không thể mở cổng Serial.")

def send_target_offset(x, y, color):
    global last_send_time
    now = time.time()
    if now - last_send_time >= send_interval:
        height, width, _ = frame.shape
        center_x, center_y = width // 2, height // 2  # Tính trung tâm của ảnh
        # center_x, center_y = 320, 240
        dx = center_x - x
        dy = y - center_y
        message = f"{color}:{dx},{dy}\n"
        if ser:
            ser.write(message.encode())
            print("Toa do:", message.strip())
        last_send_time = now


# # --- Khởi động camera ---
# picam2 = Picamera2()
# picam2.preview_configuration.main.size = (640, 480)
# # picam2.preview_configuration.main.format = "RGB888"
# picam2.configure("preview")
# picam2.start()
# time.sleep(1)
# --- Khởi động video từ file ---
video_path = 'Vid_1.mp4'  # Đường dẫn đến video file
cap = cv2.VideoCapture(video_path)

if not cap.isOpened():
    print("Không thể mở video.")
    exit()
kernel = np.ones((5, 5), np.uint8)

while True:
    # frame = picam2.capture_array()
    ret, frame = cap.read()
    if not ret:
        print("Đã đến cuối video hoặc có lỗi khi đọc video.")
        break
    # frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # --- Cập nhật vùng màu đã tinh chỉnh thực tế ---
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([180, 255, 255])

    lower_yellow = np.array([18, 100, 100])
    upper_yellow = np.array([35, 255, 255])

    lower_blue = np.array([90, 100, 100])
    upper_blue = np.array([130, 255, 255])

    # --- Tạo mask và lọc nhiễu ---
    mask_red = cv2.inRange(hsv, lower_red1, upper_red1) | cv2.inRange(hsv, lower_red2, upper_red2)
    mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

    mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)
    mask_yellow = cv2.morphologyEx(mask_yellow, cv2.MORPH_OPEN, kernel)
    mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_OPEN, kernel)

    def process_mask(mask, color_name, bgr_color):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 500:
                (x, y), radius = cv2.minEnclosingCircle(cnt)
                center = (int(x), int(y))
                cv2.circle(frame, center, int(radius), bgr_color, 2)
                cv2.putText(frame, color_name, (center[0]-20, center[1]-20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, bgr_color, 2)
                send_target_offset(center[0], center[1], color_name)
                break

    process_mask(mask_red, "Red", (0, 0, 255))
    process_mask(mask_yellow, "Yellow", (0, 255, 255))
    process_mask(mask_blue, "Blue", (255, 0, 0))
    height, width, _ = frame.shape
    center_x, center_y = width // 2, height // 2 
    cv2.drawMarker(frame, (center_x, center_y), (255, 255, 255), markerType=cv2.MARKER_CROSS, markerSize=10)
    cv2.imshow("Target Alignment", frame)
    # cv2.setMouseCallback("Target Alignment", mouse_callback)  # DEBUG HSV

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

if ser:
    ser.close()
cv2.destroyAllWindows()
