import cv2
import numpy as np
import serial
from simple_pid import PID

# Arduino seri port bağlantısı
arduino = serial.Serial('COM3', 9600, timeout=1)

# Kamera ayarları
cap = cv2.VideoCapture(1)
cap.set(3, 640)  # Genişlik
cap.set(4, 480)  # Yükseklik

# Masa boyutları (cm)
table_width_cm = 18
table_height_cm = 25

# Piksel/cm dönüşüm
pixels_per_cm_x = 640 / table_width_cm
pixels_per_cm_y = 480 / table_height_cm

# Merkez (px)
center_x, center_y = 640 // 2, 480 // 2
default_pwm = 75

# Hedef başlangıcı
target_x, target_y = center_x, center_y

# PID kontrolörleri
pid_x = PID(0.2, 0.01, 15.5, setpoint=0)
pid_y = PID(0.2, 0.01, 15.5, setpoint=0)
pid_x.output_limits = (-40, 40)
pid_y.output_limits = (-40, 40)

# Kontur filtresi ve filtre sabiti
MIN_CONTOUR_AREA = 4000
alpha = 0.9

# PWM yumuşatma limiti
max_pwm_step = 3

# Önceki PWM'ler
previous_pwm_a = default_pwm
previous_pwm_b = default_pwm
previous_pwm_c = default_pwm
previous_pwm_d = default_pwm

# HSV eşik değerleri (beyaz top)
LOWER_H = 0
UPPER_H = 180
LOWER_S = 0
UPPER_S = 30
LOWER_V = 200
UPPER_V = 255

# PWM komutu gönderme
def send_motor_commands(pwm_a, pwm_b, pwm_c, pwm_d):
    command = f"A:{pwm_a},B:{pwm_b},C:{pwm_c},D:{pwm_d}\n"
    arduino.write(command.encode())

# Hedef seçimi
def select_target(event, x, y, flags, param):
    global target_x, target_y
    if event == cv2.EVENT_LBUTTONDOWN:
        target_x, target_y = x, y
        print(f"Yeni hedef seçildi: ({target_x / pixels_per_cm_x:.2f} cm, {target_y / pixels_per_cm_y:.2f} cm)")

# PWM rampası
def smooth_pwm(previous, target, step=max_pwm_step):
    if target > previous:
        return min(previous + step, target)
    elif target < previous:
        return max(previous - step, target)
    else:
        return target

# Görüntü penceresi
cv2.namedWindow("Frame")
cv2.setMouseCallback("Frame", select_target)

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Görüntü işleme
        frame_blur = cv2.GaussianBlur(frame, (5, 5), 0)
        hsv = cv2.cvtColor(frame_blur, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (LOWER_H, LOWER_S, LOWER_V), (UPPER_H, UPPER_S, UPPER_V))
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=2)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest_contour) > MIN_CONTOUR_AREA:
                (x, y), radius = cv2.minEnclosingCircle(largest_contour)
                current_x, current_y = int(x), int(y)

                # Hedefle fark (cm)
                delta_x = (target_x - current_x) / pixels_per_cm_x
                delta_y = (target_y - current_y) / pixels_per_cm_y

                # PID çıktısı
                pwm_delta_x = pid_x(delta_x)
                pwm_delta_y = pid_y(delta_y)

                # PWM hedefleri
                target_pwm_a = int(default_pwm + pwm_delta_y + pwm_delta_x)
                target_pwm_b = int(default_pwm + pwm_delta_y - pwm_delta_x)
                target_pwm_c = int(default_pwm - pwm_delta_y + pwm_delta_x)
                target_pwm_d = int(default_pwm - pwm_delta_y - pwm_delta_x)

                # Sınırla
                target_pwm_a = max(75, min(255, target_pwm_a))
                target_pwm_b = max(75, min(255, target_pwm_b))
                target_pwm_c = max(75, min(255, target_pwm_c))
                target_pwm_d = max(75, min(255, target_pwm_d))

                # PWM rampası
                pwm_a = smooth_pwm(previous_pwm_a, target_pwm_a)
                pwm_b = smooth_pwm(previous_pwm_b, target_pwm_b)
                pwm_c = smooth_pwm(previous_pwm_c, target_pwm_c)
                pwm_d = smooth_pwm(previous_pwm_d, target_pwm_d)

                # Filtre
                pwm_a = int(alpha * pwm_a + (1 - alpha) * previous_pwm_a)
                pwm_b = int(alpha * pwm_b + (1 - alpha) * previous_pwm_b)
                pwm_c = int(alpha * pwm_c + (1 - alpha) * previous_pwm_c)
                pwm_d = int(alpha * pwm_d + (1 - alpha) * previous_pwm_d)

                # Önceki değerleri güncelle
                previous_pwm_a, previous_pwm_b = pwm_a, pwm_b
                previous_pwm_c, previous_pwm_d = pwm_c, pwm_d

                # Arduino'ya gönder
                send_motor_commands(pwm_a, pwm_b, pwm_c, pwm_d)

                # Görselleştirme
                cv2.circle(frame, (target_x, target_y), 5, (0, 255, 0), -1)
                cv2.circle(frame, (current_x, current_y), int(radius), (255, 0, 0), 2)

        # Gösterim
        cv2.imshow("Frame", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("Klavye ile durduruldu.")

finally:
    cap.release()
    cv2.destroyAllWindows()
    arduino.close()
