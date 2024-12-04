import serial
import time
import csv
import statistics

port = 'COM10'  # COM9 para Andrés / COM10 para Daniel
baudrate = 115200

# Wave variables
max_waves = 400
graph_max_waves = 8

# Connect to serial port
try:
    ser = serial.Serial(port, baudrate)
    print(f"Conectado al puerto {port} a {baudrate} baudios.")
except serial.SerialException as e:
    print(f"No se pudo abrir el puerto {port}: {e}")
    exit()

# Nombre del archivo CSV
crank_pos = str(input('Crank Position: '))
motor_noBond_freq = input('Motor noBond_frequency (Hz): ')

print('\nReady to start measurements!')

csv_path = r'C:\Users\Daniel Q\Documents\TEC\2024 - II Semestre\eWave\eWave\Datasets\\' # Para Daniel
#csv_path = r'C:\Users\Lenovo\Documents\eWave\eWave\Datasets\\' # Para Andrés

csv_filename = csv_path + crank_pos + str(motor_noBond_freq) + '.csv'

# Espera unos segundos para asegurarse de que la conexión esté establecida
time.sleep(2)


# Graph stuff
update_stat = False # Update graph stats values

'''
def visual_interface(update_stat):
    # Recibe código para actualizar variables estadísticas
'''



# Variables for getting wave parameters

# Flags to ignore first wave
noBond_first_wave = True
Bond_first_wave = True

# Store half a period of the wave
noBond_half_period = []
noBond_wave_counter = 0
Bond_half_period = []
Bond_wave_counter = 0

# Momentary variables
noBond_max_height = 0
noBond_min_height = 0
noBond_prev_time = 0
Bond_max_height = 0
Bond_min_height = 0
Bond_prev_time = 0

# Store all parameters - Has no size cap
noBond_pp = []
noBond_freq = []
Bond_pp = []
Bond_freq = []

# Store averages and standard deviations - Using all past data
noBond_pp_avg = 0
noBond_pp_stdev = 0
noBond_freq_avg = 0
noBond_freq_stdev = 0
Bond_pp_avg = 0
Bond_pp_stdev = 0
Bond_freq_avg = 0
Bond_freq_stdev = 0
    


# Abre el archivo CSV en modo de escritura
with open(csv_filename, mode='w', newline='') as file:
    writer = csv.writer(file)
    # Escribe los encabezados del archivo CSV
    writer.writerow(["Time (s)", "Accel_x (m/s2)", "Accel_y (m/s2)", "Accel_z (m/s2)", "RPM", "Humidity (percentage)", "Amb_Temp (C)", "Water_Temp (C)", "Motor_Temp (C)", "noBond_height 1 (mm)", "noBond_height 2 (mm)"])

    try:
        while Bond_wave_counter < max_waves:
            if ser.in_waiting > 0:
                # Read serial port string
                line = ser.readline().decode('utf-8').strip()
                # Split the whole serial string into values
                data = line.split(',')
                if len(data) == 11: # Right stage of Arduino code
                    writer.writerow(data) # Save everything into csv file

                    # The current time value is data[0]
                    time = float(data[0])

                    # NO BOND
                    # No Bond value is data[9]
                    noBond_height = float(data[9])
                    
                    # Not an empty array
                    if len(noBond_half_period) >= 1:
                        # New zero crossing found
                        if noBond_half_period[-1] * noBond_height < 0:
                            
                            # Ignore first wave
                            if noBond_first_wave == True:
                                noBond_first_wave = False
                                noBond_prev_time = time
                            else:
                                # PEAK-PEAK
                                # Get new maximum or new minimum
                                if noBond_half_period[-1] > 0: noBond_max_height = max(noBond_half_period)
                                else: noBond_min_height = min(noBond_half_period)
                                
                                # Calculate peak-peak
                                if noBond_max_height != 0 and noBond_min_height != 0: # Both have been updated at least once
                                    noBond_pp.anoBond_ppend(noBond_max_height - noBond_min_height)

                                # Calculate new average and standard deviation
                                if len(noBond_pp) >= 2:
                                    noBond_pp_avg = statistics.mean(noBond_pp)
                                    noBond_pp_stdev = statistics.stdev(noBond_pp)

                                    print('No Bond:\nPeak-Peak:')
                                    print('Average: ', noBond_pp_avg, ', Standard deviation: ', noBond_pp_stdev)

                                # PERIOD
                                if isinstance(noBond_wave_counter, int) == False:
                                    noBond_freq.append(1/(time - noBond_prev_time))
                                    noBond_prev_time = time

                                    if len(noBond_freq) >= 2:
                                        noBond_freq_avg = statistics.mean(noBond_freq)
                                        noBond_freq_stdev = statistics.stdev(noBond_freq)
                                        
                                        print('Frequency:')
                                        print('Average: ', noBond_freq_avg, ', Standard deviation: ', noBond_freq_stdev)

                                # Update period counter
                                noBond_wave_counter += 0.5
                            print('\nHalf period: \n', noBond_half_period)
                            noBond_half_period = []
                    
                    noBond_half_period.append(noBond_height)

                    # BOND
                    # No Bond value is data[10]
                    Bond_height = float(data[10])
                    
                    # Not an empty array
                    if len(Bond_half_period) >= 1:
                        # New zero crossing found
                        if Bond_half_period[-1] * Bond_height < 0:
                            
                            # Ignore first wave
                            if Bond_first_wave == True:
                                Bond_first_wave = False
                                Bond_prev_time = time
                            else:
                                # PEAK-PEAK
                                # Get new maximum or new minimum
                                if Bond_half_period[-1] > 0: Bond_max_height = max(Bond_half_period)
                                else: Bond_min_height = min(Bond_half_period)
                                
                                # Calculate peak-peak
                                if Bond_max_height != 0 and Bond_min_height != 0: # Both have been updated at least once
                                    Bond_pp.append(Bond_max_height - Bond_min_height)

                                # Calculate new average and standard deviation
                                if len(Bond_pp) >= 2:
                                    Bond_pp_avg = statistics.mean(Bond_pp)
                                    Bond_pp_stdev = statistics.stdev(Bond_pp)

                                    print('Bond:\nPeak-Peak:')
                                    print('Average: ', Bond_pp_avg, ', Standard deviation: ', Bond_pp_stdev)

                                # PERIOD
                                if isinstance(Bond_wave_counter, int) == False:
                                    Bond_freq.append(1/(time - Bond_prev_time))
                                    Bond_prev_time = time

                                    if len(Bond_freq) >= 2:
                                        Bond_freq_avg = statistics.mean(Bond_freq)
                                        Bond_freq_stdev = statistics.stdev(Bond_freq)
                                        
                                        print('Frequency:')
                                        print('Average: ', Bond_freq_avg, ', Standard deviation: ', Bond_freq_stdev)

                                # Update period counter
                                Bond_wave_counter += 0.5
                            print('\nHalf period: \n', Bond_half_period)
                            Bond_half_period = []
                    
                    Bond_half_period.append(Bond_height)
                    
                else: print(line)

    except KeyboardInterrupt:
        print("Deteniendo la lectura de datos.")


# Cierra la conexión serie
ser.close()
