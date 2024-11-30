import serial
import csv
import time

# Configura el puerto serie (ajusta el nombre del puerto según sea necesario)
# En Windows, el puerto puede ser 'COM3', 'COM4', etc.
# En Linux/Mac, el puerto suele ser '/dev/ttyUSB0' o '/dev/ttyACM0'
port = 'COM10'  # COM9 para Andrés / COM10 para Daniel
baudrate = 115200
i = 0
max_measurements = 1000

try:
    ser = serial.Serial(port, baudrate)
    print(f"Conectado al puerto {port} a {baudrate} baudios.")
except serial.SerialException as e:
    print(f"No se pudo abrir el puerto {port}: {e}")
    exit()

# Nombre del archivo CSV
crank_pos = str(input('Crank Position: '))
motor_freq = input('Motor Frequency (Hz): ')

print('Ready to start measurements!')

csv_path = r'C:\Users\Daniel Q\Documents\TEC\2024 - II Semestre\eWave\eWave\Datasets\\' # Para Daniel
#csv_path = r'C:\Users\Lenovo\Documents\eWave\eWave\Datasets\\' # Para Andrés

csv_filename = csv_path + crank_pos + str(motor_freq) + '.csv'

# Espera unos segundos para asegurarse de que la conexión esté establecida
time.sleep(2)

# Abre el archivo CSV en modo de escritura
with open(csv_filename, mode='w', newline='') as file:
    writer = csv.writer(file)
    # Escribe los encabezados del archivo CSV
    writer.writerow(["Time (s)", "Accel_x (m2/s)", "Accel_y (m2/s)", "Accel_z (m2/s)", "RPM", "Humidity (percentage)", "Amb_Temp (C)", "Water_Temp (C)", "Motor_Temp (C)", "Height 1 (mm)", "Height 2 (mm)"])

    try:
        while i < max_measurements:
            if ser.in_waiting > 0:
                # Lee una línea del puerto serie
                line = ser.readline().decode('utf-8').strip()
                # Divide la línea en tiempo y temperatura
                data = line.split(',')
                if len(data) == 11:
                    # Escribe los datos en el archivo CSV
                    writer.writerow(data)
                    #print(f"Datos guardados: {data}")
                    i += 1
    except KeyboardInterrupt:
        print("Deteniendo la lectura de datos.")

# Cierra la conexión serie
ser.close()
