import asyncio
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

# --- Set frequency (30 Hz) ---
freq = 1500
time.sleep(1)
client.write_register(0x0002, 1500, device_id=SLAVE)
print('Frequency set')


time.sleep(1)
# --- RUN forward ---
client.write_register(0x0001, 1, device_id=SLAVE)
print('Start drive')


time.sleep(10)

client.write_register(0x0002, 2000, device_id=SLAVE)
print('Frequency change')

# time.sleep(10)

# --- STOP ---
client.write_register(0x0001, 0, device_id=SLAVE)
print('Stop drive')
time.sleep(1)

# Reading registers for error detection
# 0021 Fault description 1
# 0022 Fault contents
# 0029 Fault description 2
# 0022 Fault contents
# 0029 Minor fault description 2
# 0020 Drive Status 2

# 003D Comunications  

# time.sleep(1)


client.close()

# --------------------------------------------------------------------------------

# import time
# from pymodbus.client import ModbusSerialClient

# SLAVE = 1

# # Mapping of bits to error descriptions
# ERROR_BITS = {
#     0: "CRC Error",
#     1: "Data Length Error",
#     3: "Parity Error",
#     4: "Overrun Error",
#     5: "Framing Error",
#     6: "Timeout",
# }

# def read_error_register(client):
#     result = client.read_holding_registers(
#         address=0x003D,
#         count=1,
#         device_id=SLAVE
#     )

#     if result.isError():
#         print("Modbus error:", result)
#         return

#     value = result.registers[0]
#     print(f"\nRaw register value: {value} (0x{value:04X})")

#     print("Detected errors:")
#     error_found = False

#     for bit, description in ERROR_BITS.items():
#         if value & (1 << bit):
#             print(f"- Bit {bit}: {description}")
#             error_found = True

#     if not error_found:
#         print("No errors detected")


# # --- Client configuration (UNCHANGED) ---
# client = ModbusSerialClient(
#     port='COM8',  # Revisar puerto
#     baudrate=2400,
#     parity='N',
#     stopbits=1,
#     bytesize=8,
#     timeout=1
# )

# # --- Connect ---
# client.connect()

# print("Connected to VFD")

# # --- Set frequency (15 Hz → 1500) ---
# time.sleep(1)
# client.write_register(0x0002, 1500, device_id=SLAVE)
# print("Frequency set to 15 Hz")

# time.sleep(1)

# # --- Start drive ---
# client.write_register(0x0001, 1, device_id=SLAVE)
# print("Drive started")

# time.sleep(1)

# # --- Monitor errors while running ---
# for _ in range(5):
#     read_error_register(client)
#     time.sleep(2)

# # --- Change frequency ---
# client.write_register(0x0002, 2000, device_id=SLAVE)
# print("Frequency set to 20 Hz")

# time.sleep(5)

# # --- Stop drive ---
# client.write_register(0x0001, 0, device_id=SLAVE)
# print("Drive stopped")

# time.sleep(1)

# # --- Final error check ---
# read_error_register(client)

# # --- Close ---
# client.close()
# print("Connection closed")