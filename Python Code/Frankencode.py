import time
import serial
import csv
import numpy as np
import argparse
import os
import statistics
import random
from PIL import Image, ImageTk # Images
import tkinter as tk # Interface
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from matplotlib import style
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# SERIAL COMMUNICATION

port = 'COM10'  # COM9 para Andrés / COM10 para Daniel
baudrate = 115200


# Comment out when using simulated data
# Connect to serial port
try:
    ser = serial.Serial(port, baudrate)
    print(f"Conectado al puerto {port} a {baudrate} baudios.")
except serial.SerialException as e:
    print(f"No se pudo abrir el puerto {port}: {e}")
    exit()

# Nombre del archivo CSV
crank_pos = str(input('Crank Position: '))
motor_noBond_freq = input('Motor Frequency (Hz): ')

print('\nReady to start measurements!')

csv_path = r'C:\Users\Daniel Q\Documents\TEC\2024 - II Semestre\eWave\eWave\Datasets\\' # Para Daniel
#csv_path = r'C:\Users\Lenovo\Documents\eWave\eWave\Datasets\\' # Para Andrés
#csv_path = r'C:\Users\garab\ewave Repo\eWave\Datasets\\' # Para Gabriel

csv_filename = csv_path + crank_pos + str(motor_noBond_freq) + '.csv'

# Espera unos segundos para asegurarse de que la conexión esté establecida
time.sleep(2)


# GENERAL VARIABLES

# Graph variables
max_measurements = 100
graph_max = 20
time_csv = []
Bond_measurements = []
NotBond_measurements = []
AmbTemp_value = 0
WaterTemp_value = 0
Humidity_value = 0
MotorTemp_value = 0
AngularVelocity_value = 0

# Wave variables
max_waves = 10

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
# Peak-Peak
noBond_max_height = 0
noBond_min_height = 0
Bond_max_height = 0
Bond_min_height = 0
# Frequency
noBond_prev_time = 0
Bond_prev_time = 0
# Wavelength
sign_cross = 0 # Just care about the sign
time_diff = 0

# Store all parameters - Has no size cap
noBond_pp = []
Bond_pp = []
noBond_freq = []
Bond_freq = []
wavelength = []

# Store averages and standard deviations - Using all past data
# Peak-Peak
noBond_pp_avg = 0
noBond_pp_stdev = 0
Bond_pp_avg = 0
Bond_pp_stdev = 0
# Frequency
noBond_freq_avg = 0
noBond_freq_stdev = 0
Bond_freq_avg = 0
Bond_freq_stdev = 0
# Wavelength
wavelength_avg = 0
wavelength_stdev = 0

# Wavelength thingies
sensor_dist = 2.22
crest_flag = True


# INTERFACE ANTESALA
window = tk.Tk() # The main Tkinter window
window.title('Plotting in Tkinter')
window.configure(bg="#80e0a7")

# Computer screen width and height
screen_width = window.winfo_screenwidth()
screen_height = window.winfo_screenheight()

window.geometry(f"{screen_width}x{screen_height}")

image = Image.open("images.png")
photo = ImageTk.PhotoImage(image)

image_label = tk.Label(window, image = photo, bg="#80e0a7")
image_label.place(relx = 0.983, rely = 0.92, anchor = 'se', x=-20, y=-20)


plt.ion() # turning interactive mode on


# Create graph and axis Bond
Bond_graph, Bond_axis = plt.subplots(figsize = (11, 4.1))
Bond_axis.set_ylabel('Amplitud (mm)')
Bond_axis.set_title("Mediciones de amplitud de la onda", fontsize = 16, fontweight = 'bold')
plt.close(Bond_graph) # Close the base graph
Bond_line, = Bond_axis.plot([],[]) # Graph line

# Insert the graph into tkinter window
Bond_canvas = FigureCanvasTkAgg(Bond_graph, master = window)
Bond_canvas.get_tk_widget().pack()

# Place the graph
Bond_canvas.get_tk_widget().place(x=20, y=20)

# Create graph and axis Not Bond
NotBond_graph, NotBond_axis = plt.subplots(figsize = (11, 4.1))
NotBond_axis.set_xlabel('Time (s)')
NotBond_axis.set_ylabel('Amplitud (mm)')
plt.close(NotBond_graph)
NotBond_line, = NotBond_axis.plot([],[])

# Insert the graph into tkinter window
NotBond_canvas = FigureCanvasTkAgg(NotBond_graph, master = window)
NotBond_canvas.get_tk_widget().pack()

# Place the graph
NotBond_canvas.get_tk_widget().place(x=20, y=429)



# FUNCTION TIME! - FOR PARAMETERS

# Return max and min
def PP(half_period, max_height, min_height, pp):
    # PEAK-PEAK
    # Get new maximum or new minimum
    if half_period[-1] > 0: max_height = max(half_period)
    else: min_height = min(half_period)
    
    # Calculate peak-peak
    if max_height != 0 and min_height != 0: # Both have been updated at least once
        pp.append(max_height - min_height)

    return max_height, min_height


def Freq(wave_counter, time, prev_time, freq):
    # FREQUENCY
    freq.append(1/(time - prev_time))


def Wavelength(wavelength):
    period = 1/noBond_freq[-1]

    # Add a full period per crest
    percentage = time_diff/period + (crests - 1)

    # Make sure phase is in sync - if not take away half a period
    if sign_cross * Bond_height < 0:
        percentage -= 0.5
    
    wavelength.append(sensor_dist/percentage)


def Stats(var):
    # Calculate new average and standard deviation
    avg = statistics.mean(var)
    stdev = statistics.stdev(var)

    return avg, stdev


# Simulated sine stuff
# Function to generate random sine wave for testing wihtout tank ouputs
def Simulated_sine():
    # Simulated sine wave parameters
    data_points = 300 # Number of datapoints
    total_periods = 10 # Total of periods 
    simulated_time = np.linspace(0, 2*total_periods*np.pi, data_points)

    # simulated_sine = np.sin(simulated_time) + np.random.normal(scale=0.1, size=data_points)
    simulated_sine = np.sin(simulated_time)

    # Variables to use simulated data
    sine_counter = 0
    sim_offset = 25 # Plays the part of the phase shift

    # Plot the graph to see the resulting sine wave
    plt.plot(simulated_time, simulated_sine)
    plt.plot(simulated_time[sim_offset], simulated_sine[sim_offset], 'or')
    plt.plot(simulated_time, np.zeros(data_points), 'm')
    plt.show()

    zero_crossings = int(input('\nZero crossings between sensors: '))


# ACTUAL CODE

def update_graphs():
    global time_csv, Bond_measurements, Bond_line, NotBond_measurements, NotBond_line, WaterTemp_value
    # Abre el archivo CSV en modo de escritura
    with open(csv_filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        # Escribe los encabezados del archivo CSV
        writer.writerow(["Time (s)", "Accel_x (m/s2)", "Accel_y (m/s2)", "Accel_z (m/s2)", "RPM", "Humidity (percentage)", "Amb_Temp (C)", "Water_Temp (C)", "Motor_Temp (C)", "noBond_height 1 (mm)", "noBond_height 2 (mm)"])
        
        try:
            while Bond_wave_counter < max_waves:
                #if sine_counter + sim_offset < len(simulated_sine): # Usar esta linea para usar datos simulados
                if ser.in_waiting > 0:
                    # Read serial port string
                    line = ser.readline().decode('utf-8').strip()
                    # Split the whole serial string into values
                    data = line.split(',')

                    # Right stage of Arduino code
                    #if sine_counter + sim_offset < len(simulated_sine): # Usar esta linea para usar datos simulados
                    if len(data) == 11:

                        # Escribe los datos en el archivo CSV
                        writer.writerow(data)

                        # Get serial data into variables
                        time = float(data[0])
                        noBond_height = float(data[9])
                        Bond_height = float(data[10])
                        
                        # Update tkinter window
                        Bond_measurements.append(Bond_height)
                        NotBond_measurements.append(noBond_height)
                        time_csv.append(time)

                        AmbTemp_value = (float(data[6]))
                        AmbTemp_valuetext.config(text = f"Temperatura del ambiente (°C): {AmbTemp_value:.2f}")
                        
                        WaterTemp_value = (float(data[7]))
                        WaterTemp_valuetext.config(text = f"Temperatura del agua (°C): {WaterTemp_value:.2f}")

                        Humidity_value = (float(data[5]))
                        Humidity_valuetext.config(text = f"Humedad (%): {Humidity_value:.2f}")

                        MotorTemp_value = (float(data[8]))
                        MotorTemp_valuetext.config(text = f"Temperatura del motor (°C): {MotorTemp_value:.2f}")

                        AngularVelocity_value = (float(data[4]))
                        AngularVelocity_valuetext.config(text = f"Velocidad angular (rpm): {AngularVelocity_value:.2f}")

                        # NO BOND
                        # Not an empty array
                        if len(noBond_half_period) >= 1:
                            # New zero crossing found
                            if noBond_half_period[-1] * noBond_height < 0:
                                # Ignore first wave
                                if noBond_first_wave == True:
                                    noBond_first_wave = False
                                    noBond_prev_time = time
                                    sign_cross = noBond_height
                                else:
                                    print('No Bond Wave counter:', noBond_wave_counter)
                                    # Peak-Peak
                                    noBond_max_height, noBond_min_height = PP(noBond_half_period, noBond_max_height, noBond_min_height, noBond_pp)
                                    
                                    if len(noBond_pp) >= 2:
                                        noBond_pp_avg, noBond_pp_stdev = Stats(noBond_pp)
                                        print('No Bond Peak-Peak:')
                                        print('Average: ', noBond_pp_avg, ', Standard deviation: ', noBond_pp_stdev)

                                    # Frequency
                                    if noBond_wave_counter % 1 == 0.5: # Only once per period (non integers)
                                        Freq(noBond_wave_counter, time, noBond_prev_time, noBond_freq)
                                        noBond_prev_time = time
                                        sign_cross = noBond_height

                                        if len(noBond_freq) >= 2:
                                            noBond_freq_avg, noBond_freq_stdev = Stats(noBond_freq)
                                            print('No Bond Frequency:')
                                            print('Average: ', noBond_freq_avg, ', Standard deviation: ', noBond_freq_stdev)

                                        # Wavelength calculations
                                        Wavelength(wavelength)
                                        print('Wavelength:' , wavelength)

                                        if len(wavelength) >= 2:
                                            wavelength_avg, wavelength_stdev = Stats(wavelength)
                                            print('Wavelength:')
                                            print('Average: ', wavelength_avg, ', Standard deviation: ', wavelength_stdev)

                                    # Update period counter
                                    noBond_wave_counter += 0.5

                                #print('\nNo Bond:\nHalf period: \n', noBond_half_period)
                                noBond_half_period = []
                        noBond_half_period.append(noBond_height)

                        # BOND
                        # Not an empty array
                        if len(Bond_half_period) >= 1:
                            # New zero crossing found
                            if Bond_half_period[-1] * Bond_height < 0:
                                # Ignore first wave
                                if Bond_first_wave == True:
                                    Bond_first_wave = False
                                    Bond_prev_time = time
                                else:
                                    print('Bond Wave counter:', Bond_wave_counter)
                                    # Peak-Peak
                                    Bond_max_height, Bond_min_height = PP(Bond_half_period, Bond_max_height, Bond_min_height, Bond_pp)
                                    
                                    if len(Bond_pp) >= 2:
                                        Bond_pp_avg, Bond_pp_stdev = Stats(Bond_pp)
                                        print('Bond Peak-Peak:')
                                        print('Average: ', Bond_pp_avg, ', Standard deviation: ', Bond_pp_stdev)

                                    # Frequency
                                    if Bond_wave_counter % 1 == 0.5:
                                        Freq(Bond_wave_counter, time, Bond_prev_time, Bond_freq)
                                        Bond_prev_time = time
                                        #print('Freq test: ', Bond_freq)
                                    
                                    # No Bond has had first crossing (Bond comes after), calculates time diff at Bond zero crossing
                                    else:
                                        time_diff = time - noBond_prev_time
                                    
                                    if len(Bond_freq) >= 2:
                                        Bond_freq_avg, Bond_freq_stdev = Stats(Bond_freq)
                                        print('Bond Frequency:')
                                        print('Average: ', Bond_freq_avg, ', Standard deviation: ', Bond_freq_stdev)
                                    
                                    # Update period counter
                                    Bond_wave_counter += 0.5

                                #print('\nBond:\nHalf period: \n', Bond_half_period)
                                Bond_half_period = []

                        Bond_half_period.append(Bond_height)

                        #sine_counter += 1

                        '''
                        # GRAPHING TIME
                        time_graph = np.append(time_graph, time)
                        noBond_graph = np.append(noBond_graph, noBond_height)
                        Bond_graph = np.append(Bond_graph, Bond_height)

                        # Calling graphing function - Only graphs Bond for now
                        Graph(time_graph, noBond_graph, Bond_graph)
                        '''
                        
                    elif line.find('Ambient humidity')!=-1 and crest_flag:
                        crests = int(input('\nCrests between sensors: '))
                        crest_flag = False
                        print(line)
                    else: print(line)

        except KeyboardInterrupt:
            print("Deteniendo la lectura de datos.")


# Cierra la conexión serie
ser.close()


# Correct for wrong number of crests
if input('\nWas the wavelength correct? (y/n): ') == 'n':
    corr = int(input('How many crests do you need to add?: '))
    for i in range(len(wavelength)):
        wavelength[i] = 1/(1/wavelength[i] + corr/2.2)

wavelength_avg, wavelength_stdev = Stats(wavelength)
