import serial
import time
from pymodbus.client import ModbusSerialClient

client = ModbusSerialClient( 
    port='COM4',  # Revisar puerto
    baudrate=2400,
    parity='N',
    stopbits=1,
    bytesize=8,
    timeout=1
)

client.connect()

SLAVE = 1

# --- Set frequency (30 Hz) ---
freq = 1500
time.sleep(1)
client.write_register(0x0002, 2000, device_id=SLAVE)
print('Frequency set')



ser = serial.Serial('COM3', 115200, timeout=1)

time.sleep(5)

data = "1"
data += "\r\n"
ser.write(data.encode())


while True:
    try:
        if ser.in_waiting > 0 :
            line = ser.readline().decode('utf-8').strip()
            if 'Zeros ready' in line:
                # # --- RUN forward ---
                client.write_register(0x0001, 1, device_id=SLAVE)
                print('Start drive')
            print('Line: ', line)

    except Exception as e:
        print(e)
        break