import pymodbus
from pymodbus.client import ModbusSerialClient
import time

# Start stop register = 0001
#   Revisar opcion H5-12 para ver formato de start stop y direccion
# Frequency reference register = 0002

# o1-03 gives units, set at 0 for 0.01 Hz


client = ModbusSerialClient(
    port='COM8',  # Revisar puerto
    baudrate=9600,
    parity='N',
    stopbits=1,
    bytesize=8,
    timeout=1
)

client.connect()

SLAVE = 1

# result = client.read_holding_registers(
#     address=0x0000,
#     count=2,
#     device_id=SLAVE
# )

# if result.isError():
#     print("Error:", result)
# else:
#     print("OK:", result.registers)

# --- Set frequency (30 Hz) ---
freq = 2000
time.sleep(1)
client.write_register(0x0002, 2000, device_id=SLAVE)

time.sleep(1)
print('Frequency set')
# --- RUN forward ---
client.write_register(0x0001, 1, device_id=SLAVE)
print('Start drive')

time.sleep(5)

# client.write_register(0x0002, 3000, device_id=SLAVE)

# time.sleep(10)

# --- STOP ---
client.write_register(0x0001, 0, device_id=SLAVE)
print('Stop drive')
time.sleep(1)

# time.sleep(1)

# result = client.read_holding_registers(0x0000, 2, device_id=SLAVE)
# print(result.registers)

client.close()
