import threading
import queue
from pymodbus.client import ModbusSerialClient
import serial
import time

ard_data_queue = queue.Queue()
VFD_data_queue = queue.Queue()

class Comms:
    def __init__(self):
        self.ard_port = None
        self.VFD_port = None

        self.ser = None
        self.client = None

        self.ard_baudrate = 115200
        self.VFD_baudrate = 19200

    def set_ports(self, ard_port, VFD_port):
        self.ard_port = ard_port
        self.VFD_port = VFD_port

    def connect(self):
        self.client = ModbusSerialClient(
            port=self.VFD_port,
            baudrate=19200,
            parity="N",
            stopbits=1,
            bytesize=8,
            timeout=1
        )


        if not self.client.connect():
            raise RuntimeError(f"Failed to connect VFD on {self.VFD_port}")

        self.client.socket.reset_input_buffer()
        self.client.socket.reset_output_buffer()

        self.ser = serial.Serial(
            self.ard_port,
            115200,
            timeout=0.02
        )

        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

        time.sleep(2)



def Serial_coms_thread():

    print('Clearing serial buffers')
    serial_comms.ser.reset_input_buffer()
    serial_comms.ser.reset_output_buffer()

    time.sleep(2)  # Give 2 seconds to let arduino boot up

    # Start automatic arduino meassurements process
    data = "Start\r\n"
    serial_comms.ser.write(data.encode())

    while True:
        if serial_comms.ser.in_waiting > 0:
            data = serial_comms.ser.readline().decode('utf-8').strip()
            ard_data_queue.put(data)
            print("Data: ", data)
            # time.sleep(0.02)

        if not VFD_data_queue.empty():
            cmd = VFD_data_queue.get()
            print('Command: ', cmd)

            if cmd[0] == 'set_freq':
                freq = cmd[1] * 100
                serial_comms.client.write_register(0x0002, freq, device_id=1, no_response_expected=True)
                print('Frequency set')   

            elif cmd[0] == 'start':
                serial_comms.client.write_register(0x0001, 1, device_id=1, no_response_expected=True)
                print('Start drive')

            elif cmd[0] == 'stop':
                serial_comms.client.write_register(0x0001, 0, device_id=1, no_response_expected=True)
                print('Stop drive')

            else:
                print('Codigo de VFD no soportado')

            time.sleep(2)

        else:
            time.sleep(0.01)


serial_comms = Comms()

serial_comms.set_ports("COM6","COM8")
serial_comms.connect()

time.sleep(1)

threading.Thread(target=Serial_coms_thread, daemon=True).start()
# VFD_data_queue.put(["set_freq", 17])
# VFD_data_queue.put(['start'])
# VFD_data_queue.put(['stop'])

# Serial_coms_thread()



VFD_data_queue.put(["set_freq", 17])

# time.sleep(2)

VFD_data_queue.put(['start'])

time.sleep(6)

VFD_data_queue.put(['stop'])

print('Just vibing')
time.sleep(10)
