import time
import serial
import csv
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import argparse
import os

port = 'COM9'  # COM9 para Andrés / COM10 para Daniel
baudrate = 115200

max_measurements = 1000
graph_maxWaves = 8
time_csv = [0]
fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
Bond_measurements = [0]
notBond_meausurements = []
prueba=[]

def animate(a, time_csv, Bond_measurements):
    Bond_measurements = Bond_measurements[-20:]
    # notBond_measurments = notBond_measurments[-20:]

    ax.clear()
    ax.plot(time_csv, Bond_measurements,'-+')
    ax.set_xlim(0,np.max(time_csv))
    ax.set_ylim(0,np.max(Bond_measurements))
    

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

# csv_path = r'C:\Users\Daniel Q\Documents\TEC\2024 - II Semestre\eWave\eWave\Datasets\\' # Para Daniel
csv_path = r'C:\Users\Lenovo\Documents\eWave\eWave\Datasets\\' # Para Andrés

csv_filename = csv_path + crank_pos + str(motor_freq) + '.csv'

# Espera unos segundos para asegurarse de que la conexión esté establecida
time.sleep(2)

i = 0

update_stat = False # Update graph stats values

flag_graph = False

# Abre el archivo CSV en modo de escritura
with open(csv_filename, mode='w', newline='') as file:
    writer = csv.writer(file)
    # Escribe los encabezados del archivo CSV
    writer.writerow(["Time (s)", "Accel_x (m/s2)", "Accel_y (m/s2)", "Accel_z (m/s2)", "RPM", "Humidity (percentage)", "Amb_Temp (C)", "Water_Temp (C)", "Motor_Temp (C)", "Height 1 (mm)", "Height 2 (mm)"])

    ani = animation.FuncAnimation(fig, animate, fargs=(time_csv, Bond_measurements), interval=40, blit=True, save_count=50, cache_frame_data=False)
    plt.show()
    try:
        while i < max_measurements:
            if ser.in_waiting > 0:
                # Lee una línea del puerto serie
                line = ser.readline().decode('utf-8').strip()
                # Divide la línea en tiempo y temperatura
                data = line.split(',')
                print('Ready for next state')

                
                
                if len(data) == 11:
                    # Escribe los datos en el archivo CSV
                    # print(data)
                    writer.writerow(data)
                    print(Bond_measurements)

                    Bond_measurements.append(float(data[10]))
                    time_csv.append(float(data[0]))
                    
                    Bond_measurments = np.asarray(Bond_measurements)
                    time_csv = np.asarray(time_csv)
                    

                    i+=1 
    except KeyboardInterrupt:
        print("Deteniendo la lectura de datos.")


# Cierra la conexión serie
ser.close()


