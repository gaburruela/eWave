import tkinter as tk
import serial
import threading
import queue
from pymodbus.client import ModbusSerialClient
import time
import string

ARDPORT = "COM6" # COM6 Gabriel
BAUDRATE = 115200


ser = serial.Serial(ARDPORT, BAUDRATE)

time.sleep(2)

ser.reset_input_buffer()   # discard anything already received
ser.reset_output_buffer()  # optional

data = "1"
data += "\r\n"
ser.write(data.encode())
time.sleep(1)

ard_data_queue = queue.Queue()
vfd_data_queue = queue.Queue()

# --- ARDUINO THREAD FUNCTION ---
def read_serial_thread():
    while True:
        data = ser.readline().decode('utf-8').strip()
        ard_data_queue.put(data)
        print("Data: ", data)
        # time.sleep(0.02)

# --- VFD THREAD FUNCTION ---
def command_VFD_thread():
    client = ModbusSerialClient(
        port='COM8',  # Revisar puerto
        baudrate=2400,
        parity='N',
        stopbits=1,
        bytesize=8,
        timeout=1
    )

    client.connect()
    SLAVE = 1

    while True:
        if not vfd_data_queue.empty():
            cmd = vfd_data_queue.get()
            if cmd[0] == 'set_freq':
                freq = cmd[1]
                client.write_register(0x0002, freq, device_id=SLAVE)
                print('Frequency set')

            elif cmd[0] == 'start':
                client.write_register(0x0001, 1, device_id=SLAVE)
                print('Start drive')

            elif cmd[0] == 'stop':
                client.write_register(0x0001, 0, device_id=SLAVE)
                print('Stop drive')

            else:
                print('Codigo de VFD no soportado')

# Start threads
threading.Thread(target=read_serial_thread, daemon=True).start()
threading.Thread(target=command_VFD_thread, daemon=True).start()

time.sleep(4)

# --- TKINTER ---
root = tk.Tk()
label = tk.Label(root, text="Waiting...", font=("Arial", 16))
label.pack(padx=20, pady=20)
root.geometry('800x100')

# # --- Set frequency (20 Hz) ---
# client.write_register(0x0002, 2000, device_id=SLAVE)
# print('Frequency set')

counter = 0

def update_gui():
    global counter
    if not ard_data_queue.empty():
        print('data queue length: ', ard_data_queue.qsize())
        counter += 1
        data = ard_data_queue.get()
        # print('Data type: ', type(data))
        label.config(text=data)
        # if counter == 1:
        #     vfd_data_queue.put(['set_freq',1500])

        # if "Zeros ready" in data:
        #     # --- RUN forward ---
        #     vfd_data_queue.put(['start'])

        if counter >= 30:
        #     vfd_data_queue.put(['stop'])
            print('30 datapoints')

    root.after(20, update_gui)

update_gui()
root.mainloop()

print('This is the end')
# client.write_register(0x0001, 0, device_id=SLAVE)
print('Stop drive')
