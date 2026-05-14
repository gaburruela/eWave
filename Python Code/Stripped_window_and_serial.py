import tkinter as tk
import serial
import threading
import queue
from pymodbus.client import ModbusSerialClient
import time

ARDPORT = "COM6" # COM6 Gabriel
BAUDRATE = 115200

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


ser = serial.Serial(ARDPORT, BAUDRATE)

data = "1"
data += "\r\n"
ser.write(data.encode())
time.sleep(1)

data_queue = queue.Queue()

# --- THREAD FUNCTION ---
def read_serial_thread():
    while True:
        data = ser.readline().decode('utf-8').strip()
        data_queue.put(data)

# Start thread
threading.Thread(target=read_serial_thread, daemon=True).start()

# --- TKINTER ---
root = tk.Tk()
label = tk.Label(root, text="Waiting...", font=("Arial", 16))
label.pack(padx=20, pady=20)

# --- Set frequency (20 Hz) ---
client.write_register(0x0002, 2000, device_id=SLAVE)
print('Frequency set')

def update_gui():
    if not data_queue.empty():
        data = data_queue.get()
        # print('Data type: ', type(data))
        label.config(text=data)
        if "Zeros ready" in data:
            # --- RUN forward ---
            client.write_register(0x0001, 1, device_id=SLAVE)
            print('Start drive')
        # if data == 5:
        #     # --- STOP ---
        #     client.write_register(0x0001, 0, device_id=SLAVE)
        #     print('Stop drive')

    root.after(100, update_gui)

update_gui()
root.mainloop()

print('This is the end')
client.write_register(0x0001, 0, device_id=SLAVE)
print('Stop drive')
