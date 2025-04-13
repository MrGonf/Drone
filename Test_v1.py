import cv2
import numpy as np
import serial
import time

last_send_time = 0
send_interval = 1  # giây


# --- Cấu hình UART ---
try:
    # ser = serial.Serial('/dev/ttyS0', 115200, timeout=1)  # hoặc '/dev/ttyUSB0' nếu dùng USB-UART
    ser = serial.Serial('COM1', 115200, timeout=1)  # hoặc '/dev/ttyUSB0' nếu dùng USB-UART

    time.sleep(2)
    print("✅ Serial kết nối thành công.")
except:
    ser = None
    print("⚠️ Không thể mở cổng Serial.")

# --- Hàm gửi offset mục tiêu ---
# def send_target_offset(x, y, color):
#     center_x, center_y = 320, 240  # Tâm khung hình
#     dx = x - center_x
#     dy = y - center_y
#     message = f"{color}:{dx},{dy}\n"
#     if ser:
#         ser.write(message.encode())
#         print("📤 Gửi offset:", message.strip())

def send_target_offset(x, y, color):
    global last_send_time
    now = time.time()
    if now - last_send_time >= send_interval:
        center_x, center_y = 320, 240
        dx = x - center_x
        dy = y - center_y
        message = f"{color}:{dx},{dy}\n"
        if ser:
            ser.write(message.encode())
            print("📤 Gửi offset:", message.strip())
        last_send_time = now  # cập nhật thời gian gửi gần nhất

# --- Mở camera ---
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        print("❌ Không đọc được khung hình.")
        break

    frame = cv2.resize(frame, (640, 480))
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # --- Vùng màu ---
    lower_red1 = np.array([0, 120, 70])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 120, 70])
    upper_red2 = np.array([180, 255, 255])
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([30, 255, 255])
    lower_blue = np.array([100, 150, 0])
    upper_blue = np.array([140, 255, 255])

    # --- Mặt nạ ---
    mask_red = cv2.inRange(hsv, lower_red1, upper_red1) | cv2.inRange(hsv, lower_red2, upper_red2)
    mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

    # --- Hàm xử lý từng màu ---
    def process_mask(mask, color_name, bgr_color):
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 500:
                (x, y), radius = cv2.minEnclosingCircle(cnt)
                center = (int(x), int(y))
                cv2.circle(frame, center, int(radius), bgr_color, 2)
                cv2.putText(frame, color_name, (center[0]-20, center[1]-20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, bgr_color, 2)

                # Gửi độ lệch mục tiêu
                send_target_offset(center[0], center[1], color_name)
                break  # chỉ gửi 1 mục tiêu mỗi màu

    process_mask(mask_red, "Red", (0, 0, 255))
    process_mask(mask_yellow, "Yellow", (0, 255, 255))
    process_mask(mask_blue, "Blue", (255, 0, 0))

    # Vẽ tâm ảnh
    cv2.drawMarker(frame, (320, 240), (255, 255, 255), markerType=cv2.MARKER_CROSS, markerSize=10)

    cv2.imshow("Target Alignment", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
if ser:
    ser.close()
cv2.destroyAllWindows()

