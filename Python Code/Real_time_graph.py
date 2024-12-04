import time
import serial
import time
import csv
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import argparse
import os
import statistics

plt.ion()  # turning interactive mode on

port = 'COM9'  # COM9 para Andrés / COM10 para Daniel
baudrate = 115200

max_measurements = 1000
graph_max = 100
time_csv = [0]
# ax = fig.add_subplot(1, 1, 1)
Bond_measurements = [550]
notBond_meausurements = []
fig = plt.plot(time_csv,Bond_measurements)[0]

# def animate(a, time_csv, Bond_measurements):

    
##    Bond_measurements = Bond_measurements[-100:]
##    time_csv = time_csv[-100:]
##    # notBond_measurments = notBond_measurments[-20:]
    
    
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
##crank_pos = str(input('Crank Position: '))
##motor_freq = input('Motor Frequency (Hz): ')

crank_pos = 'Z'
motor_freq = 60

print('\nReady to start measurements!')

# csv_path = r'C:\Users\Daniel Q\Documents\TEC\2024 - II Semestre\eWave\eWave\Datasets\\' # Para Daniel
csv_path = r'C:\Users\Lenovo\Documents\eWave\eWave\Datasets\\' # Para Andrés

csv_filename = csv_path + crank_pos + str(motor_freq) + '.csv'

# Espera unos segundos para asegurarse de que la conexión esté establecida
time.sleep(2)

i = 0

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

                if len(data) == 11:
                    # Escribe los datos en el archivo CSV
                    # print(data)
                    # print("Hello")
                    writer.writerow(data)
                    

                    Bond_measurements.append(float(data[10]))
                    time_csv.append(float(data[0]))
                    
                    Bond_measurments = np.asarray(Bond_measurements)
                    # time_csv = np.asarray(time_csv)
                    
                    # removing the older graph
                    fig.remove()
                    
                    # plotting newer graph
                    fig = plt.plot(time_csv[-100:],Bond_measurements[-100:],color = 'g')[0]
                    #
                    if len(time_csv)>=graph_max:
                        plt.xlim(time_csv[-graph_max], time_csv[-1])   
                    # calling pause function for 20 milliseconds
                    plt.pause(0.02)

                    i+=1 

                    
                    
                    
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
# ani = animation.FuncAnimation(fig, animate, fargs=(time_csv, Bond_measurements), interval=40, save_count=50, cache_frame_data=False)
# plt.show()
# Abre el archivo CSV en modo de escritura



# Cierra la conexión serie
ser.close()


