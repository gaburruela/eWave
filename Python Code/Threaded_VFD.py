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


# Combined coms thread
def comb_serial_thread():
    polling_coms_counter = 0
    ard_coms_counter = 0
    print('Starting modbus client coms')
    client = ModbusSerialClient(
        port='COM8',  # Revisar puerto
        baudrate=115200,
        parity='N',
        stopbits=1,
        bytesize=8,
        timeout=0.02
    )

    client.connect()
    SLAVE = 1

    while True:
        polling_coms_counter += 1
        if ser.in_waiting > 0:
            ard_coms_counter =+ 1
            data = ser.readline().decode('utf-8').strip()
            ard_data_queue.put(data)
            # print("Data: ", data)
            # time.sleep(0.02)

        if not vfd_data_queue.empty():
            VFD_start_time = time.time()
            cmd = vfd_data_queue.get()
            print('start time: ', time.time())
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
            
            print('VFD coms exec time: ', time.time() - VFD_start_time)

        if ard_coms_counter%10 == 0 and ard_coms_counter != 0: 
            print('Ard coms still going ', ard_coms_counter)
        if polling_coms_counter%500 == 0 and polling_coms_counter != 0: 
            print('Polling thread still going ', polling_coms_counter)

        else:
            time.sleep(0.01)




# --- ARDUINO THREAD FUNCTION ---
def read_serial_thread():
    while True:
        if ser.in_waiting > 0:
            data = ser.readline().decode('utf-8').strip()
            ard_data_queue.put(data)
            # print("Data: ", data)
            # time.sleep(0.02)
        else:
            time.sleep(0.01)

# --- VFD THREAD FUNCTION ---
def command_VFD_thread():
    print('Starting modbus client coms')
    client = ModbusSerialClient(
        port='COM8',  # Revisar puerto
        baudrate=115200,
        parity='N',
        stopbits=1,
        bytesize=8,
        timeout=0.02
    )

    client.connect()
    SLAVE = 1

    while True:
        if not vfd_data_queue.empty():
            cmd = vfd_data_queue.get()
            print('start time: ', time.time())
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
            
            print('end time: ', time.time())

        else:
            time.sleep(0.01)

# Start threads
threading.Thread(target=comb_serial_thread, daemon=True).start()
# threading.Thread(target=read_serial_thread, daemon=True).start()
# threading.Thread(target=command_VFD_thread, daemon=True).start()

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
window_counter = 0

def update_gui():
    start_time = time.time()
    global counter, window_counter
    if not ard_data_queue.empty():
        # print('data queue length: ', ard_data_queue.qsize())
        counter += 1
        data = ard_data_queue.get()
        # print('Data type: ', type(data))
        label.config(text=data)
        if counter == 1:
            vfd_data_queue.put(['set_freq',1500])

        if "Zeros ready" in data:
            # --- RUN forward ---
            vfd_data_queue.put(['start'])

        if counter == 100:
            vfd_data_queue.put(['stop'])
            # print('30 datapoints')

        # print('Window exec time with data: ', time.time() - start_time)
    
    window_counter += 1
    if(window_counter%250 == 0):
        print('Still alive bb ', window_counter)
    root.after(20, update_gui)

update_gui()
root.mainloop()

# # Testing just polling thread
# while True:
#     start_time = time.time()
#     if not ard_data_queue.empty():
#         print(ard_data_queue.get())
#         print('Gets ard data exec time: ', time.time() - start_time)
#     else:
#         print('Window exec time: ', time.time() - start_time)
#         time.sleep(0.01)

        


print('This is the end')
# client.write_register(0x0001, 0, device_id=SLAVE)
print('Stop drive')
