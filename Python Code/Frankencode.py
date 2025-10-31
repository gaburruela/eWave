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
import winsound

# SERIAL COMMUNICATION

port = 'COM3'  # COM9 para Andrés / COM10 (o COM3) para Daniel
baudrate = 115200

winsound.Beep(350,500)

# Comment out when using simulated data
# Connect to serial port
try:
    ser = serial.Serial(port, baudrate)
    print(f"Conectado al puerto {port} a {baudrate} baudios.")
except serial.SerialException as e:
    print(f"No se pudo abrir el puerto {port}: {e}")
    exit()

# Nombre del archivo CSV
crank_pos = input('Crank Position: ') + " mm - "
motor_freq = input('Motor Frequency (Hz): ') + " Hz"

if input('Are you sure? (y/n): ') == 'n':
    crank_pos = input('Crank Position: ') + " mm - "
    motor_freq = input('Motor Frequency (Hz): ') + " Hz"

print('\nReady to start measurements!')

#csv_path = r'C:\Users\Daniel Q\Documents\TEC\2025 - II Semestre\eWave\eWave\Datasets\\' # Para Daniel
csv_path = r'C:\eWave\eWave\Datasets\II Semester 2025\Raw_Data\\' # Para Andrés
#csv_path = r'C:\Users\garab\ewave Repo\eWave\Datasets\\' # Para Gabriel

csv_filename = csv_path + crank_pos + motor_freq + '.csv'

# Waits a couple of seconds to establish a conection with the arduino
time.sleep(2)


# GENERAL VARIABLES

# Wave variables
max_waves = 120
graph_max = 20 # Data points, not waves, gets calculated automatically on first period of wave
points_per_period_flag = True

# Graph variables
time_csv = []
Bond_measurements = []
noBond_measurements = []
# Useless variables
AmbTemp_value = 0
WaterTemp_value = 0
Humidity_value = 0
MotorTemp_value = 0
AngularVelocity_value = 0



# Variables for getting wave parameters

# Flags to ignore first wave
noBond_first_wave = True
Bond_first_wave = True

# Flags to avoid multiple zero crossings
noBond_anti_ripple = 0
Bond_anti_ripple = 0

# Other flags
time_start_flag = True
amplitude_flag = 0

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
noBond_sign_cross = 0 # Just care about the sign
Bond_sign_cross = 0 # Just care about the sign
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
crests = 0

# Others
noBond_real_zero = 465 # measured height at zero
Bond_real_zero = 495
anti_ripple = 7 # crests to ignore

# INTERFACE ANTESALA
window = tk.Tk() # The main Tkinter window
window.title('Plotting in Tkinter')
window.configure(bg="#80e0a7")

# Computer screen width and height
screen_width = 1536
screen_height = 864

window.geometry(f"{screen_width}x{screen_height}")

image = Image.open("eWave logo.png")
photo = ImageTk.PhotoImage(image)

image_label = tk.Label(window, image = photo, bg="#80e0a7")
image_label.place(relx = 0.983, rely = 0.92, anchor = 'se', x=-20, y=-20)


plt.ion() # turning interactive mode on
x=0

# Create graph and axis Not Bond
graph_update = False

noBond_graph, noBond_axis = plt.subplots(figsize = (11, 4.1))
noBond_axis.set_title("Mediciones de amplitud de la onda", fontsize = 16, fontweight = 'bold')
noBond_axis.set_ylabel('No Bond Amplitud (mm)')
plt.close(noBond_graph)
noBond_line, = noBond_axis.plot([],[], linestyle = 'None', marker = 'o')

# Insert the graph into tkinter window
noBond_canvas = FigureCanvasTkAgg(noBond_graph, master = window)
noBond_canvas.get_tk_widget().pack()

# Place the graph
noBond_canvas.get_tk_widget().place(x=20, y=20)

# Create graph and axis Bond
Bond_graph, Bond_axis = plt.subplots(figsize = (11, 4.1))
Bond_axis.set_xlabel('Time (s)')
Bond_axis.set_ylabel('Bond Amplitud (mm)')
plt.close(Bond_graph) # Close the base graph
Bond_line, = Bond_axis.plot([],[], linestyle = 'None', marker = 'o') # Graph line

# Insert the graph into tkinter window
Bond_canvas = FigureCanvasTkAgg(Bond_graph, master = window)
Bond_canvas.get_tk_widget().pack()

# Place the graph
Bond_canvas.get_tk_widget().place(x=20, y=429)


# CREATE LABELS AND TEXT BOXES

# Environment condition box
T = tk.Text(window, height = 33, width = 45)
T.pack()
T.place(x = 1511, y = 43, anchor = 'ne')

# Environment conditions
font_size = 11
xpos = 1150
ystart = 0.021
linespace = 0.038

title = tk.Label(window, text = "Condiciones Ambientales")
title.config(font =("Courier", 14))
title.pack()
title.place(x = 1332, y = 22, anchor = 'center')

Humidity_valuetext = tk.Label(window)
Humidity_valuetext.config(font =("Courier", font_size))
Humidity_valuetext.pack()
Humidity_valuetext.place(x = xpos, rely = ystart + linespace, anchor = 'nw')

AmbTemp_valuetext = tk.Label(window)
AmbTemp_valuetext.config(font =("Courier", font_size))
AmbTemp_valuetext.pack()
AmbTemp_valuetext.place(x = xpos, rely = ystart + linespace*2, anchor = 'nw')

WaterTemp_valuetext = tk.Label(window)
WaterTemp_valuetext.config(font = ("Courier", font_size))
WaterTemp_valuetext.pack()
WaterTemp_valuetext.place(x = xpos, rely = ystart + linespace*3, anchor = 'nw')

MotorTemp_valuetext = tk.Label(window)
MotorTemp_valuetext.config(font =("Courier", font_size))
MotorTemp_valuetext.pack()
MotorTemp_valuetext.place(x = xpos, rely = ystart + linespace*4, anchor = 'nw')

AngularVelocity_valuetext = tk.Label(window)
AngularVelocity_valuetext.config(font =("Courier", font_size))
AngularVelocity_valuetext.pack()
AngularVelocity_valuetext.place(x = xpos, rely = ystart + linespace*5, anchor = 'nw')

noBond_pp_avg_text = tk.Label(window)
noBond_pp_avg_text.config(font =("Courier", font_size))
noBond_pp_avg_text.pack()
noBond_pp_avg_text.place(x = xpos, rely = ystart + linespace*6, anchor = 'nw')

noBond_pp_stdev_text = tk.Label(window)
noBond_pp_stdev_text.config(font =("Courier", font_size))
noBond_pp_stdev_text.pack()
noBond_pp_stdev_text.place(x = xpos, rely = ystart + linespace*7, anchor = 'nw')

Bond_pp_avg_text = tk.Label(window)
Bond_pp_avg_text.config(font =("Courier", font_size))
Bond_pp_avg_text.pack()
Bond_pp_avg_text.place(x = xpos, rely = ystart + linespace*8, anchor = 'nw')

Bond_pp_stdev_text = tk.Label(window)
Bond_pp_stdev_text.config(font =("Courier", font_size))
Bond_pp_stdev_text.pack()
Bond_pp_stdev_text.place(x = xpos, rely = ystart + linespace*9, anchor = 'nw')

noBond_freq_avg_text = tk.Label(window)
noBond_freq_avg_text.config(font =("Courier", font_size))
noBond_freq_avg_text.pack()
noBond_freq_avg_text.place(x = xpos, rely = ystart + linespace*10, anchor = 'nw')

noBond_freq_stdev_text = tk.Label(window)
noBond_freq_stdev_text.config(font =("Courier", font_size))
noBond_freq_stdev_text.pack()
noBond_freq_stdev_text.place(x = xpos, rely = ystart + linespace*11, anchor = 'nw')

Bond_freq_avg_text = tk.Label(window)
Bond_freq_avg_text.config(font =("Courier", font_size))
Bond_freq_avg_text.pack()
Bond_freq_avg_text.place(x = xpos, rely = ystart + linespace*12, anchor = 'nw')

Bond_freq_stdev_text = tk.Label(window)
Bond_freq_stdev_text.config(font =("Courier", font_size))
Bond_freq_stdev_text.pack()
Bond_freq_stdev_text.place(x = xpos, rely = ystart + linespace*13, anchor = 'nw')

wavelength_avg_text = tk.Label(window)
wavelength_avg_text.config(font =("Courier", font_size))
wavelength_avg_text.pack()
wavelength_avg_text.place(x = xpos, rely = ystart + linespace*14, anchor = 'nw')

wavelength_stdev_text = tk.Label(window)
wavelength_stdev_text.config(font =("Courier", font_size))
wavelength_stdev_text.pack()
wavelength_stdev_text.place(x = xpos, rely = ystart + linespace*15, anchor = 'nw')

wave_number_text = tk.Label(window)
wave_number_text.config(font =("Courier", font_size))
wave_number_text.pack()
wave_number_text.place(x = xpos, rely = ystart + linespace*16, anchor = 'nw')



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


def Freq(wave_counter, ttime, prev_time, freq):
    # FREQUENCY
    if ttime - prev_time != 0:
        freq.append(1/(ttime - prev_time))


def Wavelength(wavelength):
    period = 1/noBond_freq[-1]

    # Add a full period per crest
    percentage = time_diff/period + (crests - 1)

    # Make sure phase is in sync - if not take away half a period
    if noBond_sign_cross * Bond_sign_cross < 0:
        percentage -= 0.5

    if abs(percentage) > 0.001:
        wavelength.append(sensor_dist/percentage)


def Stats(var):
    # Calculate new average and standard deviation
    avg = statistics.mean(var)
    stdev = statistics.stdev(var)

    return avg, stdev


# Simulated sine stuff
'''
# Generate random sine wave for testing wihtout tank ouputs
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

crests = int(input('\nCrests between sensors: '))
data = [0,1,2,3,4,5,6,7,8]
'''

# ACTUAL CODE

def Update_graphs():
    global time_csv, Bond_measurements, Bond_line, noBond_measurements, noBond_line
    global noBond_first_wave, Bond_first_wave, time_start_flag, noBond_anti_ripple, Bond_anti_ripple, amplitude_flag
    global noBond_half_period, noBond_wave_counter, Bond_half_period, Bond_wave_counter
    global noBond_max_height, noBond_min_height, Bond_max_height, Bond_min_height
    global noBond_prev_time, Bond_prev_time, noBond_sign_cross, Bond_sign_cross, time_diff
    global noBond_pp, Bond_pp, noBond_freq, Bond_freq, wavelength
    global noBond_pp_avg, noBond_pp_stdev, Bond_pp_avg, Bond_pp_stdev
    global noBond_freq_avg, noBond_freq_stdev, Bond_freq_avg, Bond_freq_stdev, wavelength_avg, wavelength_stdev, crest_flag, crests
    global graph_update, graph_max, points_per_period_flag
    #global sine_counter
    
    # Abre el archivo CSV en modo de escritura
    with open(csv_filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        # Escribe los encabezados del archivo CSV
        writer.writerow(["Time (s)", "Accel_x (m/s2)", "Accel_y (m/s2)", "Accel_z (m/s2)", "RPM", "Humidity (percentage)", "Amb_Temp (C)", "Water_Temp (C)", "Motor_Temp (C)", "noBond_height 1 (mm)", "Bond_height 2 (mm)"])
        
        try:
            while Bond_wave_counter < max_waves:
                #if sine_counter + sim_offset < len(simulated_sine): # Usar esta linea para usar datos simulados
                if ser.in_waiting > 0:
                    # Read serial port string
                    line = ser.readline().decode('utf-8').strip()
                    #print(line)
                    # Split the whole serial string into values
                    data = line.split(',')

                    # Right stage of Arduino code
                    #if sine_counter + sim_offset < len(simulated_sine): # Usar esta linea para usar datos simulados
                    if len(data) == 11:
                        
                        # Escribe los datos en el archivo CSV
                        writer.writerow(data)

                        # Simutaled stuff
                        '''
                        ttime = simulated_time[sine_counter]
                        Bond_height = simulated_sine[sine_counter]
                        noBond_height = simulated_sine[sine_counter + sim_offset]
                        '''

                        # Get time starting at zero
                        if time_start_flag == True:
                            time_start_flag = False
                            time_start = float(data[0])

                        # If at any point the current time is less than the starting time reset everything
                        if float(data[0]) < time_start:
                            time_start = float(data[0])
                            noBond_first_wave = True
                            Bond_first_wave = True
                            noBond_half_period = []
                            noBond_wave_counter = 0
                            Bond_half_period = []
                            Bond_wave_counter = 0
                            noBond_pp = []
                            Bond_pp = []
                            noBond_freq = []
                            Bond_freq = []
                            wavelength = []
                        
                        # Get serial data into variables (offset made with real measurements)
                        ttime = float(data[0]) - time_start
                        noBond_height = float(data[9])
                        Bond_height = float(data[10])
                        
                        # noBond_height = float(data[9]) + noBond_offset
                        # Bond_height = float(data[10]) + Bond_offset
                        
                        # Update tkinter window
                        Humidity_value = (float(data[5]))
                        Humidity_valuetext.config(text = f"Humedad (%): {Humidity_value:.2f}")
                        
                        AmbTemp_value = (float(data[6]))
                        AmbTemp_valuetext.config(text = f"Temperatura ambiente (°C): {AmbTemp_value:.2f}")
                        
                        WaterTemp_value = (float(data[7]))
                        WaterTemp_valuetext.config(text = f"Temperatura del agua (°C): {WaterTemp_value:.2f}")

                        MotorTemp_value = (float(data[8]))
                        MotorTemp_valuetext.config(text = f"Temperatura del motor (°C): {MotorTemp_value:.2f}")

                        AngularVelocity_value = (float(data[4]))
                        AngularVelocity_valuetext.config(text = f"Velocidad angular (rpm): {AngularVelocity_value:.2f}")

                        noBond_pp_avg_text.config(text = f"No Bond Peak-Peak Avg (mm): {noBond_pp_avg:.2f}")
                        noBond_pp_stdev_text.config(text = f"No Bond Peak-Peak StDev (mm): {noBond_pp_stdev:.2f}")

                        Bond_pp_avg_text.config(text = f"Bond Peak-Peak Avg (mm): {Bond_pp_avg:.2f}")
                        Bond_pp_stdev_text.config(text = f"Bond Peak-Peak StDev (mm): {Bond_pp_stdev:.2f}")

                        noBond_freq_avg_text.config(text = f"No Bond Frequency Avg (Hz): {noBond_freq_avg:.3f}")
                        noBond_freq_stdev_text.config(text = f"No Bond Frequency StDev (Hz): {noBond_freq_stdev:.3f}")
                        
                        Bond_freq_avg_text.config(text = f"Bond Frequency Avg (Hz): {Bond_freq_avg:.3f}")
                        Bond_freq_stdev_text.config(text = f"Bond Frequency StDev (Hz): {Bond_freq_stdev:.3f}")

                        wavelength_avg_text.config(text = f"Wavelength Avg (m): {wavelength_avg:.3f}")
                        wavelength_stdev_text.config(text = f"Wavelength StDev (m): {wavelength_stdev:.3f}")

                        wave_number_text.config(text = f"Wave number: {int(Bond_wave_counter)}")

                        
                        # NO BOND
                        # Not an empty array
                        if len(noBond_half_period) >= 1:
                            if noBond_anti_ripple != 0:
                                if noBond_anti_ripple > anti_ripple:
                                    noBond_anti_ripple = 0
                                else:
                                    noBond_anti_ripple += 1
                                
                            # New zero crossing found
                            if noBond_measurements[-1] * noBond_height < 0 and noBond_anti_ripple == 0 or noBond_height == 0:
                                noBond_anti_ripple += 1
                                # Ignore first wave
                                if noBond_first_wave == True:
                                    noBond_first_wave = False
                                    noBond_prev_time = ttime
                                    noBond_sign_cross = noBond_height
                                else:
                                    #print('No Bond Wave counter:', noBond_wave_counter)
                                    # Peak-Peak
                                    noBond_max_height, noBond_min_height = PP(noBond_half_period, noBond_max_height, noBond_min_height, noBond_pp)
                                    
                                    if len(noBond_pp) >= 2:
                                        noBond_pp_avg, noBond_pp_stdev = Stats(noBond_pp)
                                        #print('No Bond Peak-Peak:')
                                        #print('Average: ', noBond_pp_avg, ', Standard deviation: ', noBond_pp_stdev)

                                    # Frequency
                                    if noBond_wave_counter % 1 == 0.5: # Only once per period (non integers)
                                        Freq(noBond_wave_counter, ttime, noBond_prev_time, noBond_freq)
                                        noBond_prev_time = ttime
                                        noBond_sign_cross = noBond_height

                                        if len(noBond_freq) >= 2:
                                            noBond_freq_avg, noBond_freq_stdev = Stats(noBond_freq)
                                            #print('No Bond Frequency:')
                                            #print('Average: ', noBond_freq_avg, ', Standard deviation: ', noBond_freq_stdev)

                                        # Wavelength calculations
                                        Wavelength(wavelength)
                                        #print('Wavelength:' , wavelength)

                                        if len(wavelength) >= 2:
                                            wavelength_avg, wavelength_stdev = Stats(wavelength)
                                            #print('Wavelength:')
                                            #print('Average: ', wavelength_avg, ', Standard deviation: ', wavelength_stdev)

                                        if points_per_period_flag:
                                            graph_max = int(1.4*len(noBond_measurements))
                                            points_per_period_flag = False

                                    # Update period counter
                                    noBond_wave_counter += 0.5

                                #print('\nNo Bond:\nHalf period: \n', noBond_half_period)
                                noBond_half_period = []
                        # Only add a measurement every 3 measurements
                        if amplitude_flag == 0:
                            noBond_half_period.append(noBond_height)

                        # BOND
                        # Not an empty array
                        if len(Bond_half_period) >= 1:
                            if Bond_anti_ripple != 0:
                                if Bond_anti_ripple > anti_ripple:
                                    Bond_anti_ripple = 0
                                else:
                                    Bond_anti_ripple += 1
                            
                            # New zero crossing found
                            if Bond_measurements[-1] * Bond_height < 0 and Bond_anti_ripple == 0 or Bond_height == 0:
                                Bond_anti_ripple = 1
                                # Ignore first wave
                                if Bond_first_wave == True:
                                    if noBond_first_wave == False: # No Bond should already be done with its first wave
                                        Bond_first_wave = False
                                        Bond_prev_time = ttime
                                else:
                                    #print('Bond Wave counter:', Bond_wave_counter)
                                    # Peak-Peak
                                    Bond_max_height, Bond_min_height = PP(Bond_half_period, Bond_max_height, Bond_min_height, Bond_pp)
                                    
                                    if len(Bond_pp) >= 2:
                                        Bond_pp_avg, Bond_pp_stdev = Stats(Bond_pp)
                                        #print('Bond Peak-Peak:')
                                        #print('Average: ', Bond_pp_avg, ', Standard deviation: ', Bond_pp_stdev)

                                    # Frequency
                                    if Bond_wave_counter % 1 == 0.5:
                                        Freq(Bond_wave_counter, ttime, Bond_prev_time, Bond_freq)
                                        Bond_prev_time = ttime
                                        #print('Freq test: ', Bond_freq)
                                    
                                    # No Bond has had first crossing (Bond comes after), calculates time diff at Bond zero crossing
                                    else:
                                        time_diff = ttime - noBond_prev_time
                                        Bond_sign_cross = Bond_height
                                    
                                    if len(Bond_freq) >= 2:
                                        Bond_freq_avg, Bond_freq_stdev = Stats(Bond_freq)
                                        #print('Bond Frequency:')
                                        #print('Average: ', Bond_freq_avg, ', Standard deviation: ', Bond_freq_stdev)
                                    
                                    # Update period counter
                                    Bond_wave_counter += 0.5
                                    graph_update = True

                                #print('\nBond:\nHalf period: \n', Bond_half_period)
                                Bond_half_period = []

                        if amplitude_flag == 0:
                            Bond_half_period.append(Bond_height)

                        amplitude_flag += 1
                        # Make sure it stays bounded at 3
                        if amplitude_flag == 3:
                            amplitude_flag = 0
                            
                        #sine_counter += 1 # Only for simulations

                        # START GRAPH AND INTERFACE
                        Bond_measurements.append(Bond_height)
                        noBond_measurements.append(noBond_height)
                        time_csv.append(ttime)

                        if graph_update:
                            Bond_line.set_xdata(time_csv)
                            Bond_line.set_ydata(Bond_measurements)
                            
                            noBond_line.set_xdata(time_csv)
                            noBond_line.set_ydata(noBond_measurements)

                            Bond_axis.relim()
                            Bond_axis.autoscale_view()
                            Bond_canvas.draw()
                            Bond_canvas.flush_events() # Update data

                            noBond_axis.relim()
                            noBond_axis.autoscale_view()
                            noBond_canvas.draw()
                            noBond_canvas.flush_events() 

                            # Update pending tasks
                            window.update_idletasks()
                            window.update()

                            graph_update = False

                            if len(time_csv) >= graph_max:
                                Bond_axis.set_xlim(time_csv[-graph_max], time_csv[-1])
                                noBond_axis.set_xlim(time_csv[-graph_max], time_csv[-1])

                    # Store zero levelings and offsets
                    elif line.find('Zero levels') != -1:
                        data_zero = line.split(',')
                        noBond_zero_lvl = float(data_zero[1])
                        Bond_zero_lvl = float(data_zero[2])
                        noBond_offset = noBond_real_zero - noBond_zero_lvl
                        Bond_offset = Bond_real_zero - Bond_zero_lvl

                        print(line)
                        # print('No Bond offset: ', noBond_offset)
                        # print('Bond offset: ', Bond_offset)
                        winsound.Beep(350,500)
                        
                    # Ask for total number of crests
                    elif line.find('Ambient humidity') != -1 and crest_flag:
                        winsound.Beep(350,500)
                        crests = int(input('\nCrests between sensors: '))
                        crest_flag = False
                        print(line)
                        
                    else: print(line)
                    

        except KeyboardInterrupt:
            print("Deteniendo la lectura de datos.")
            
        ser.close()
        # Correct for wrong number of crests - ALWAYS MAKE CORR POSITIVE
        if input('\nWas the wavelength correct? (y/n): ') == 'n':
            corr = int(input('How many crests do you need to add?: '))
            for i in range(len(wavelength)):
                # Don't use for more than 2 crests
                if wavelength[i] != sensor_dist:
                    wavelength[i] = 1/(1/wavelength[i] + corr/sensor_dist)

            wavelength_avg, wavelength_stdev = Stats(wavelength)

    # Save results to csv file
    if (input('\nSave data? (y/n): ') == 'y'):
        results_file = open(csv_path + 'Results.csv', mode='a')
        # Name of the test
        results_file.write('\n' + crank_pos + ',' + motor_freq + ',')
        # Add the results
        results_file.write(str(noBond_pp_avg) + ',' + str(noBond_pp_stdev) + ',')
        results_file.write(str(Bond_pp_avg) + ',' + str(Bond_pp_stdev) + ',')
        results_file.write(str(noBond_freq_avg) + ',' + str(noBond_freq_stdev) + ',')
        results_file.write(str(Bond_freq_avg) + ',' + str(Bond_freq_stdev) + ',')
        results_file.write(str(wavelength_avg) + ',' + str(wavelength_stdev))
        results_file.close()




# RUN THE GUI
window.after(0, Update_graphs)
window.mainloop()
