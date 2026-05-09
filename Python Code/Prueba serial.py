import serial
import time
from pymodbus.client import ModbusSerialClient
import threading, queue

# client = ModbusSerialClient( 
#     port='COM4',  # Revisar puerto
#     baudrate=2400,
#     parity='N',
#     stopbits=1,
#     bytesize=8,
#     timeout=1
# )

# client.connect()

# SLAVE = 1

# # --- Set frequency (30 Hz) ---
# freq = 1500
# time.sleep(1)
# client.write_register(0x0002, 1500, device_id=SLAVE)
# print('Frequency set')



ser = serial.Serial('COM3', 115200, timeout=1)

time.sleep(1)

data = "1"
data += "\r\n"
ser.write(data.encode())

# Normal thread serial reading

while True:
    try:
        line = ser.readline().decode('utf-8').strip()
        if 'Zeros ready' in line:
            # # --- RUN forward ---
            # client.write_register(0x0001, 1, device_id=SLAVE)
            print('Start drive')
        print('Line: ', line)
        time.sleep(0.010)

    except Exception as e:
        print(e)
        break

# Polling with threads

# # Arduino polling function

# ard_data_queue = queue.Queue()

# def read_serial_data():
#     while True:
#         try:
#             line = ser.readline()
#             if line:
#                 line = line.decode('utf-8').strip()
#                 ard_data_queue.put(line)
#         except Exception as e:
#             print("Thread error:", e)
#             break
#         # Prevent CPU hogging
#         time.sleep(0.001)


# # Start thread
# serial_thread = threading.Thread(target=read_serial_data, daemon=True)
# serial_thread.start()

# while True:
#     if not ard_data_queue.empty():
#         # Get data from thread queue
#         line = ard_data_queue.get()
#         data = line.split(',')
#         print('Data: ', data)
#         time.sleep(0.05)
