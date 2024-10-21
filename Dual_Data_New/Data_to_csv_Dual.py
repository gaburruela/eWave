import serial
import csv
import time

# Configura el puerto serie (ajusta el nombre del puerto según sea necesario)
# En Windows, el puerto puede ser 'COM3', 'COM4', etc.
# En Linux/Mac, el puerto suele ser '/dev/ttyUSB0' o '/dev/ttyACM0'
port = 'COM9'  # Reemplaza 'COM3' con el puerto correcto
baudrate = 9600
i = 0

try:
    ser = serial.Serial(port, baudrate)
    print(f"Conectado al puerto {port} a {baudrate} baudios.")
except serial.SerialException as e:
    print(f"No se pudo abrir el puerto {port}: {e}")
    exit()

# Nombre del archivo CSV
csv_filename = 'Dual_30.csv'

# Espera unos segundos para asegurarse de que la conexión esté establecida
time.sleep(2)

# Abre el archivo CSV en modo de escritura
with open(csv_filename, mode='w', newline='') as file:
    writer = csv.writer(file)
    # Escribe los encabezados del archivo CSV
    writer.writerow(["Tiempo (ms)", "Distancia 1 (mm)", "Distancia 2 (mm)"])

    try:
        while i < 400:
            if ser.in_waiting > 0:
                # Lee una línea del puerto serie
                line = ser.readline().decode('utf-8').strip()
                # Divide la línea en tiempo y temperatura
                data = line.split(',')
                if len(data) == 3:
                    # Escribe los datos en el archivo CSV
                    writer.writerow(data)
                    #print(f"Datos guardados: {data}")
                    i += 1
    except KeyboardInterrupt:
        print("Deteniendo la lectura de datos.")

# Cierra la conexión serie
ser.close()
