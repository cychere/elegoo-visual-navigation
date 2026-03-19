import time
import serial

def send_speed(ser, left, right):
    ser.write(f"{left} {right}\n".encode("ascii"))
    ser.flush()

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)
time.sleep(2)

send_speed(ser, -200, -200)
time.sleep(0.5)
send_speed(ser, 0, 0)