import time
import serial

def send_angle(ser, angle):
    ser.write(f"{angle}\n".encode("ascii"))
    ser.flush()

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)
time.sleep(2)

send_angle(ser, 100)