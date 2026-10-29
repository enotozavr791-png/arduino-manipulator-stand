import serial
import time
import math

ARDUINO_PORT = 'COM4'   # <-- ВАШ ПОРТ

try:
    arduino = serial.Serial(ARDUINO_PORT, 115200, timeout=0.1)
    time.sleep(2)
    print(f"[*] Arduino подключена на {ARDUINO_PORT}")
except Exception as e:
    print(f"[!] Arduino не подключена: {e}")
    exit()

# --- Калибровка ---
OFFSET_SHOULDER = 45  # <-- СТАВИМ 45 ДЛЯ ТЕСТА

# Симулируем "нулевые" углы (как будто рука смотрит прямо)
t0 = 0.0
t1 = 0.0
t2 = 0.0

a0 = int(90 + math.degrees(t0))      # База
a1 = int(90 - math.degrees(t1) + OFFSET_SHOULDER) # Плечо
a2 = int(90 + math.degrees(t2))      # Локоть

print(f"Отправляем на Arduino: {a0}, {a1}, {a2}")
arduino.write(f"{a0},{a1},{a2}\n".encode())

time.sleep(2)
arduino.close()