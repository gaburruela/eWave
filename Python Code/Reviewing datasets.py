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
from matplotlib.widgets import Button, Slider
import pandas

# GENERAL VARIABLES

# Wave variables
max_waves = 100
graph_max = 50 # Data points, not waves
time_start_flag = True

# Variables for getting wave parameters

# Flags to ignore first wave
noBond_first_wave = True
Bond_first_wave = True

# Flags to avoid multiple zero crossings
noBond_anti_ripple = 0
Bond_anti_ripple = 0

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

# Quality of life for data viewing
temp_datapoints = 0
datapoints_per_wave = 0 # To adjust graph to some amount of waves on screen



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

# Visualizing data stuff

# DATA READING AND PREPROCESSING
# Read the actual data from csv file
csv_path = r'C:\Users\Daniel Q\Documents\TEC\2024 - II Semestre\eWave\eWave\Datasets\\' # Para Daniel
#csv_path = r'C:\Users\Lenovo\Documents\eWave\eWave\Datasets\\' # Para Andrés
#csv_path = r'C:\Users\garab\ewave Repo\eWave\Datasets\\' # Para Gabriel

# Make sure to change for each test
print('\nFile name information:')
crank_pos = input('Crank Position: ')
motor_freq = input('Motor Frequency (Hz): ')

csv_filename = csv_path + crank_pos + motor_freq + '.csv'


data = pandas.read_csv(csv_filename)

# Get time data
x_data = np.array(data['Time (s)'].tolist())
#x_data = x_data - x_data[0] # Phaseshift to start measurements at zero

# Get data from ultrasonic sensors
y_data1 = np.array(data['Bond_height 2 (mm)'].tolist())  # noBond 
y_data2 = np.array(data['noBond_height 1 (mm)'].tolist())  # Bond

pos_counter = 0


# ACTUAL CODE
# Abre el archivo CSV en modo de escritura

crests = int(input('\nCrests between sensors: '))
crest_flag = False

max_measurements = len(x_data)

try:
    while pos_counter < max_measurements: # Usar esta linea cuando se usa el archivo de datasets

        # Get time starting at zero
        if time_start_flag == True:
            time_start_flag = False
            time_start = float(x_data[pos_counter])

        # If at any point the current time is less than the starting time reset everything
        if float(x_data[pos_counter]) < time_start:
            time_start = float(x_data[pos_counter])
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

        # Datasets file info
        ttime = x_data[pos_counter] - time_start
        Bond_height = y_data1[pos_counter]
        noBond_height = y_data2[pos_counter]
        

        # NO BOND
        # Not an empty array
        if len(noBond_half_period) >= 1:
            if noBond_anti_ripple != 0:
                if noBond_anti_ripple > 2:
                    noBond_anti_ripple = 0
                else:
                    noBond_anti_ripple += 1
                
            # New zero crossing found
            if noBond_half_period[-1] * noBond_height < 0 and noBond_anti_ripple == 0:
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

                        # Add the second half of wave to obtain full # of datapoints/wave
                        datapoints_per_wave = temp_datapoints + len(noBond_half_period)
                        # print('Length of full period:', datapoints_per_wave)


                    else:
                        # Store the length of first half of wave
                        temp_datapoints = len(noBond_half_period)
                        # print('Length of first half period:', datapoints_per_wave)


                    # Update period counter
                    noBond_wave_counter += 0.5

                #print('\nNo Bond:\nHalf period: \n', noBond_half_period)
                noBond_half_period = []
        noBond_half_period.append(noBond_height)

        # BOND
        # Not an empty array
        if len(Bond_half_period) >= 1:
            if Bond_anti_ripple != 0:
                if Bond_anti_ripple > 2:
                    Bond_anti_ripple = 0
                else:
                    Bond_anti_ripple += 1
            
            # New zero crossing found
            if Bond_half_period[-1] * Bond_height < 0 and Bond_anti_ripple == 0:
                Bond_anti_ripple = 1
                # Ignore first wave
                if Bond_first_wave == True and noBond_first_wave == False:
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
                        # print('Bond Frequency:')
                        # print('Average: ', Bond_freq_avg, ', Standard deviation: ', Bond_freq_stdev)
                    
                    # Update period counter
                    Bond_wave_counter += 0.5

                # print('\nBond:\nHalf period: \n', Bond_half_period)
                Bond_half_period = []

        Bond_half_period.append(Bond_height)
        pos_counter += 1 # Only for dataset reviewing


except KeyboardInterrupt:
    print("Deteniendo la lectura de datos.")

# print('Length of data:', len(x_data))

# Plot variables
wave_graph = plt.figure()
Bond_ax = wave_graph.add_subplot(111)

# Slider
num_graph_waves = 4 # Amount of waves to plot
data_range = num_graph_waves*datapoints_per_wave # Resulting amount of data points to plot
max_data_ratio = data_range / len(x_data)
slider_width = 0.65
time_range = wave_graph.add_axes([0.15, 0.05, slider_width, 0.03]) # Slider visual dimensions


# Generate wave lines
Bond_line, = Bond_ax.plot(x_data[:data_range]-time_start, y_data2[:data_range], lw = 2, marker = '.', label = 'Bond')
noBond_line, = Bond_ax.plot(x_data[:data_range]-time_start, y_data1[:data_range], lw = 2, marker = '.', label = 'noBond', color = 'green')

# Generate x axis label
Bond_ax.set_xlabel('Time (s)')
Bond_ax.legend() # Add legend

# Plot readjustments to fit slider
wave_graph.subplots_adjust(left=0.15, bottom=0.2)

time_slider = Slider(
    ax = time_range,
    label = 'Time (s)',
    valmin = 0,
    valmax = slider_width*100,
    valinit = 0,
    valstep = 1
)

# Initial limits of the graph
plt.xlim([0, data_range])

# Update graph variables
def update_slider(val):
    # Slider mapping
    slider_pos = int(val*(len(x_data) - data_range -1)/(slider_width*100)) # Mapea el rango del slider a la posición del arreglo
    
    # Ajusta el lim del eje x para el nuevo valor del slider
    plt.subplot(111)
    plt.xlim([x_data[slider_pos]-time_start, x_data[slider_pos + data_range]-time_start]) 

    # Da nuevos datos por graficar
    Bond_line.set_ydata(y_data1[slider_pos:slider_pos + data_range])
    Bond_line.set_xdata(x_data[slider_pos:slider_pos + data_range]-time_start)
    noBond_line.set_ydata(y_data2[slider_pos:slider_pos + data_range])
    noBond_line.set_xdata(x_data[slider_pos:slider_pos + data_range]-time_start)

    wave_graph.canvas.draw_idle()
    
# Function to call on eahc slider change
time_slider.on_changed(update_slider)

plt.show()

# If there are wonky time measurements at the start the graph will break at the left most position
