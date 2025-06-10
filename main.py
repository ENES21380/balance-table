import cv2
import numpy as np
import serial
from simple_pid import PID

# Arduino seri portuna bağlan
arduino = serial.Serial('COM3', 9600, timeout=1)

# Kamera ayarları
cap = cv2.VideoCapture(0)  # Kamera indexi 0 olabilir, sisteminize göre değişebilir
cap.set(3, 640)  # Genişlik
cap.set(4, 480)  # Yükseklik

# Masa boyutları (cm cinsinden)
table_width_cm = 18
table_height_cm = 25

# Piksel/cm dönüşüm oranları
pixels_per_cm_x = 640 / table_width_cm  # 21.33
pixels_per_cm_y = 480 / table_height_cm  # 16.0

# Masa merkez referansı
center_x, center_y = 640 // 2, 480 // 2
default_pwm = 75

# Hedef pozisyon
target_x, target_y = center_x, center_y

# PID kontrolörleri
pid_x = PID(0.3, 0.05, 0.02, setpoint=0)
pid_y = PID(0.3, 0.05, 0.02, setpoint=0)
pid_x.output_limits = (-30, 30)
pid_y.output_limits = (-30, 30)

# Minimum kontur alanı
MIN_CONTOUR_AREA = 5000

# Düşük geçiren filtre katsayısı
alpha = 0.9

# PWM değerlerini saklamak için başlangıç değerleri
previous_pwm_a = default_pwm
previous_pwm_b = default_pwm
previous_pwm_c = default_pwm
previous_pwm_d = default_pwm

# HSV değerleri (beyaz renk için)
LOWER_H, UPPER_H = 0, 180
LOWER_S, UPPER_S = 0, 30
LOWER_V, UPPER_V = 200, 255

# Motor komutlarını Arduino'ya gönder
def send_motor_commands(pwm_a, pwm_b, pwm_c, pwm_d):
    command = f"A:{pwm_a},B:{pwm_b},C:{pwm_c},D:{pwm_d}\n"
    arduino.write(command.encode())

# Kullanıcının hedef pozisyonu seçmesi için fare geri çağırma fonksiyonu
def select_target(event, x, y, flags, param):
    global target_x, target_y
    if event == cv2.EVENT_LBUTTONDOWN:
        target_x, target_y = x, y
        print(f"Yeni hedef seçildi: ({target_x / pixels_per_cm_x:.2f} cm, {target_y / pixels_per_cm_y:.2f} cm)")

cv2.namedWindow("Frame")
cv2.setMouseCallback("Frame", select_target)

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Gaussian Blur ile gürültüyü azalt
        frame_blur = cv2.GaussianBlur(frame, (5, 5), 0)

        # HSV maskeleme
        lower_white = np.array([LOWER_H, LOWER_S, LOWER_V])
        upper_white = np.array([UPPER_H, UPPER_S, UPPER_V])

        hsv = cv2.cvtColor(frame_blur, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_white, upper_white)

        # Erozyon ve dilatasyon işlemleri
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)

            if cv2.contourArea(largest_contour) > MIN_CONTOUR_AREA:
                (x, y), radius = cv2.minEnclosingCircle(largest_contour)
                current_x, current_y = int(x), int(y)

                # Sapmaları hesapla
                delta_x = (target_x - current_x) / pixels_per_cm_x
                delta_y = (target_y - current_y) / pixels_per_cm_y

                # PID kontrolünden PWM farklarını al
                pwm_delta_x = pid_x(delta_x)
                pwm_delta_y = pid_y(delta_y)

                # Her bacak için PWM hesapla
                pwm_a = int(default_pwm + pwm_delta_y + pwm_delta_x)
                pwm_b = int(default_pwm + pwm_delta_y - pwm_delta_x)
                pwm_c = int(default_pwm - pwm_delta_y + pwm_delta_x)
                pwm_d = int(default_pwm - pwm_delta_y - pwm_delta_x)

                # PWM değerlerini sınırla
                pwm_a = max(75, min(255, pwm_a))
                pwm_b = max(75, min(255, pwm_b))
                pwm_c = max(75, min(255, pwm_c))
                pwm_d = max(75, min(255, pwm_d))

                # Düşük geçiren filtre ile yumuşatma
                pwm_a = int(alpha * pwm_a + (1 - alpha) * previous_pwm_a)
                pwm_b = int(alpha * pwm_b + (1 - alpha) * previous_pwm_b)
                pwm_c = int(alpha * pwm_c + (1 - alpha) * previous_pwm_c)
                pwm_d = int(alpha * pwm_d + (1 - alpha) * previous_pwm_d)

                previous_pwm_a, previous_pwm_b, previous_pwm_c, previous_pwm_d = pwm_a, pwm_b, pwm_c, pwm_d

                send_motor_commands(pwm_a, pwm_b, pwm_c, pwm_d)

                # Görüntüye hedef ve mevcut pozisyonları çiz
                cv2.circle(frame, (target_x, target_y), 5, (0, 255, 0), -1)
                cv2.circle(frame, (current_x, current_y), int(radius), (255, 0, 0), 2)

            else:
                # Eğer top küçükse varsayı
::contentReference[oaicite:12]{index=12}