import serial
import csv

port = 'COM10'  # COM9 para Andrés / COM10 para Daniel
baudrate = 115200

maxWaves = 400
graph_maxWaves = 8

def visual_interface(update_stat):
    # Recibe código para actualizar variables estadísticas
    

# Connect to serial port
try:
    ser = serial.Serial(port, baudrate)
    print(f"Conectado al puerto {port} a {baudrate} baudios.")
except serial.SerialException as e:
    print(f"No se pudo abrir el puerto {port}: {e}")
    exit()

# Nombre del archivo CSV
crank_pos = str(input('Crank Position: '))
motor_freq = input('Motor Frequency (Hz): ')

print('\nReady to start measurements!')

csv_path = r'C:\Users\Daniel Q\Documents\TEC\2024 - II Semestre\eWave\eWave\Datasets\\' # Para Daniel
#csv_path = r'C:\Users\Lenovo\Documents\eWave\eWave\Datasets\\' # Para Andrés

csv_filename = csv_path + crank_pos + str(motor_freq) + '.csv'

# Espera unos segundos para asegurarse de que la conexión esté establecida
time.sleep(2)

i = 0

update_stat = False # Update graph stats values

# Flags to ignore first wave
Bond_first_wave = True
notBond_first_wave = True

# Array to store half a period of the wave
Bond_half_period = []
notBond_half_period = []

Bond_avg_height
noBond_avg_height




'''
save in array all values
calculate max or min at sign change
depending on sign change value (-1 or +1) calculate max or min of array
'''

def 



# Abre el archivo CSV en modo de escritura
with open(csv_filename, mode='w', newline='') as file:
    writer = csv.writer(file)
    # Escribe los encabezados del archivo CSV
    writer.writerow(["Time (s)", "Accel_x (m/s2)", "Accel_y (m/s2)", "Accel_z (m/s2)", "RPM", "Humidity (percentage)", "Amb_Temp (C)", "Water_Temp (C)", "Motor_Temp (C)", "Height 1 (mm)", "Height 2 (mm)"])

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
                    if first_wave:
                        
                    
                    #print(f"Datos guardados: {data}")
                    i += 1
    except KeyboardInterrupt:
        print("Deteniendo la lectura de datos.")


# Cierra la conexión serie
ser.close()
