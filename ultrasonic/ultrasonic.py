import time
import serial

def get_distance(ser):
    ser.reset_input_buffer()
    ser.write(b'GET\n')
    ser.flush()

    response = ser.readline().decode('ascii', errors='ignore').strip()
    if not response:
        raise TimeoutError('Timed out waiting for ultrasonic reading')

    return int(response)

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)
time.sleep(2)

print("Distance: ", get_distance(ser), "cm")