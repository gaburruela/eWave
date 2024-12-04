import serial
import time
import csv
import statistics

port = 'COM10'  # COM9 para Andrés / COM10 para Daniel
baudrate = 115200

# Wave variables
max_waves = 400
graph_max_waves = 8
wave_counter = 0

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


# Graph stuff
update_stat = False # Update graph stats values

'''
def visual_interface(update_stat):
    # Recibe código para actualizar variables estadísticas
'''



# Variables I care about right now

# Flags to ignore first wave
first_wave = True

# Store half a period of the wave
half_period = []
half_period_counter = 0

# Minimum and maximum of an individual period - Just momentary variables
max_height = 0
min_height = 0

# Store all peak-peak measurements - Has no size cap
pp = []

# Store averages and standard deviations - Using all past data from peak-peak
avg = 0
stdev = 0


# Abre el archivo CSV en modo de escritura
with open(csv_filename, mode='w', newline='') as file:
    writer = csv.writer(file)
    # Escribe los encabezados del archivo CSV
    writer.writerow(["Time (s)", "Accel_x (m/s2)", "Accel_y (m/s2)", "Accel_z (m/s2)", "RPM", "Humidity (percentage)", "Amb_Temp (C)", "Water_Temp (C)", "Motor_Temp (C)", "Height 1 (mm)", "Height 2 (mm)"])

    try:
        while wave_counter < max_waves:
            if ser.in_waiting > 0:
                # Read serial port string
                line = ser.readline().decode('utf-8').strip()
                # Split the whole serial string into values
                data = line.split(',')
                if len(data) == 11: # Right stage of Arduino code
                    writer.writerow(data) # Save everything into csv file

                    # OBTAIN PEAK-PEAK MEASUREMENTS
                    # The current height value is data[9]
                    height = float(data[9])
                    
                    # Not an empty array
                    if len(half_period) >= 1:
                        # New zero crossings found
                        if half_period[-1] * height < 0:
                            # Ignore first wave
                            if first_wave == True: first_wave = False
                            else:
                                # Get new maximum or new minimum
                                if half_period[-1] > 0: max_height = max(half_period)
                                else: min_height = min(half_period)
                                
                                # Calculate peak-peak
                                if max_height != 0 and min_height != 0: # Both have been updated at least once
                                    pp.append(max_height - min_height)

                                # Calculate new average and standard deviation
                                if len(pp) >= 2:
                                    avg = statistics.mean(pp)
                                    stdev = statistics.stdev(pp)
                                    
                                    print('Average: ', avg, ', Standard deviation: ', stdev)

                                # Update wave counter
                                wave_counter += 0.5
                            print('\nHalf period: \n', half_period)
                            half_period = []
                    
                    half_period.append(height)
                else: print(line)

    except KeyboardInterrupt:
        print("Deteniendo la lectura de datos.")


# Cierra la conexión serie
ser.close()
