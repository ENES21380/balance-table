import cv2
import numpy as np
import serial
from simple_pid import PID

# Arduino seri port bağlantısı
arduino = serial.Serial('COM3', 9600, timeout=1)

# Kamera ayarları
cap = cv2.VideoCapture(1)
cap.set(3, 640)  # Genişlik (px)
cap.set(4, 480)  # Yükseklik (px)

# Masa boyutları (cm)
table_width_cm = 18
table_height_cm = 25

# Piksel/cm dönüşüm oranları
pixels_per_cm_x = 640 / table_width_cm  # 35.56 px/cm
pixels_per_cm_y = 480 / table_height_cm  # 19.2 px/cm

# Masa merkez koordinatları (px)
center_x, center_y = 640 // 2, 480 // 2
default_pwm = 75

# Hedef konum başlangıcı (px)
target_x, target_y = center_x, center_y

# PID kontrolör tanımları
pid_x = PID(0.3, 0.05, 0.07, setpoint=0)
pid_y = PID(0.3, 0.05, 0.07, setpoint=0)
pid_x.output_limits = (-30, 30)
pid_y.output_limits = (-30, 30)

# Kontur filtresi ve yumuşatma katsayısı
MIN_CONTOUR_AREA = 5000
alpha = 0.9

# Önceki PWM değerleri (filtreleme için)
previous_pwm_a = default_pwm
previous_pwm_b = default_pwm
previous_pwm_c = default_pwm
previous_pwm_d = default_pwm

# HSV sınır değerleri doğrudan sabit olarak tanımlandı
LOWER_H = 0
UPPER_H = 180
LOWER_S = 0
UPPER_S = 30
LOWER_V = 200
UPPER_V = 255

# Arduino'ya PWM komutlarını gönder
def send_motor_commands(pwm_a, pwm_b, pwm_c, pwm_d):
    command = f"A:{pwm_a},B:{pwm_b},C:{pwm_c},D:{pwm_d}\n"
    arduino.write(command.encode())

# Fare ile hedef seçme fonksiyonu
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

        # Görüntüyü bulanıklaştır
        frame_blur = cv2.GaussianBlur(frame, (5, 5), 0)

        # HSV formatına çevir
        hsv = cv2.cvtColor(frame_blur, cv2.COLOR_BGR2HSV)

        # Beyaz top maskesi oluştur
        mask = cv2.inRange(hsv,
                           (LOWER_H, LOWER_S, LOWER_V),
                           (UPPER_H, UPPER_S, UPPER_V))

        # Gürültü temizleme: erozyon ve genişleme
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)

            if cv2.contourArea(largest_contour) > MIN_CONTOUR_AREA:
                (x, y), radius = cv2.minEnclosingCircle(largest_contour)
                current_x, current_y = int(x), int(y)

                # Hedefle top konumu arasındaki fark (cm cinsinden)
                delta_x = (target_x - current_x) / pixels_per_cm_x
                delta_y = (target_y - current_y) / pixels_per_cm_y

                # PID kontrol çıktısı
                pwm_delta_x = pid_x(delta_x)
                pwm_delta_y = pid_y(delta_y)

                # Solenoid PWM hesaplama
                pwm_a = int(default_pwm + pwm_delta_y + pwm_delta_x)
                pwm_b = int(default_pwm + pwm_delta_y - pwm_delta_x)
                pwm_c = int(default_pwm - pwm_delta_y + pwm_delta_x)
                pwm_d = int(default_pwm - pwm_delta_y - pwm_delta_x)

                # PWM sınırlarını uygula
                pwm_a = max(75, min(255, pwm_a))
                pwm_b = max(75, min(255, pwm_b))
                pwm_c = max(75, min(255, pwm_c))
                pwm_d = max(75, min(255, pwm_d))

                # Düşük geçiren filtre uygulama
                pwm_a = int(alpha * pwm_a + (1 - alpha) * previous_pwm_a)
                pwm_b = int(alpha * pwm_b + (1 - alpha) * previous_pwm_b)
                pwm_c = int(alpha * pwm_c + (1 - alpha) * previous_pwm_c)
                pwm_d = int(alpha * pwm_d + (1 - alpha) * previous_pwm_d)

                previous_pwm_a, previous_pwm_b, previous_pwm_c, previous_pwm_d = pwm_a, pwm_b, pwm_c, pwm_d

                # Arduino'ya komut gönder
                send_motor_commands(pwm_a, pwm_b, pwm_c, pwm_d)

                # Görselleştirme
                cv2.circle(frame, (target_x, target_y), 5, (0, 255, 0), -1)
                cv2.circle(frame, (current_x, current_y), int(radius), (255, 0, 0), 2)

        # Ekrana görüntü ver
        cv2.imshow("Frame", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("Klavye ile durduruldu.")

finally:
    cap.release()
    cv2.destroyAllWindows()
    arduino.close()
