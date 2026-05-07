import serial
import time

ser = serial.Serial('COM3', 115200, timeout=1)

time.sleep(5000)

data = "1"
data += "\r\n"
ser.write(data.encode())


time.sleep(5000)

while True:
    try:
        if ser.in_waiting > 0 :
            line = ser.readline().decode('utf-8').strip()
            print('Line: ', line)

    except Exception as e:
        print(e)
        pass