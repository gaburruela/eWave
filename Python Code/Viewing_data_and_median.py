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
wavelength_median = 0

# Wavelength thingies
sensor_dist = 2.22
crest_flag = True
crests = 0

# Others
noBond_real_zero = 427
Bond_real_zero = 427
anti_ripple = 2

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


def find_position_in_array(array):
    # Get the position and frequency from the user
    position = input("Crank position: ")
    frequency = input("Frequency: ")
    
    # Search for the position and frequency in the array
    for index, (pos, freq) in enumerate(array):
        if pos == position and freq == frequency:
            print('Index position is:', index)
            return index  # Return the index if a match is found
    
    print("No match found in the array.")
    return -1  # Return -1 if no match is found

# Redo variables
file_pos = 13 # Pos inicial para iteracion de los datasets
filename = [['A','20b'],['A','22'],['A','24b'],['A','26'],['A','28'],['A','30'],['A','32c'],['A','34'],['A','36'],['A','38'],['A','40']
           ,['B','17'],['B','18'],['B','19'],['B','19.5'],['B','20'],['B','21'],['B','22'],['B','23'],['B','24'],['B','25'],['B','26'],['B','27'],['B','28'],['B','29b'],['B','30'],['B','31'],['B','32'],['B','33'],['B','34']
           ,['C','15'],['C','16'],['C','17'],['C','18'],['C','19'],['C','20'],['C','21'],['C','22'],['C','23'],['C','24'],['C','25'],['C','26b']
           ,['D','14'],['D','15'],['D','18'],['D','20'],['D','22'],['D','25']
           ,['E','15'],['E','18'],['E','19'],['E','20'],['E','23']]

changing_data = True
view_only_one = 0
if(input('Desea observar un unico archivo? (y/n):') == 'y'):
    file_pos = find_position_in_array(filename)
    changing_data = False


# ACTUAL CODE

try:
    while file_pos < len(filename):
        # Visualizing data stuff

        # DATA READING AND PREPROCESSING
        # Read the actual data from csv file
        # csv_path = r'C:\Users\Daniel Q\Documents\TEC\2024 - II Semestre\eWave\eWave\Datasets\\' # Para Daniel
        #csv_path = r'C:\Users\Lenovo\Documents\eWave\eWave\Datasets\\' # Para Andrés
        csv_path = r'C:\Users\garab\ewave Repo\eWave\Datasets\\' # Para Gabriel

        # # Make sure to change for each test
        # print('\nFile name to read information:')
        # crank_pos = input('Crank Position: ')
        # motor_freq = input('Motor Frequency (Hz): ')

        # Reset all parameters for new test
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
        wavelength_median = 0

        # Wavelength thingies
        sensor_dist = 2.22
        crest_flag = True
        crests = 0


        # To run through all dataset files
        dataset_filename = csv_path + filename[file_pos][0] + filename[file_pos][1]  + '.csv' # Chooses between different dataset files
        data = pandas.read_csv(dataset_filename) # Current file's dataset
        print('\nWorking on file: ', filename[file_pos][0] + filename[file_pos][1])

        # Get time data
        time_data = np.array(data['Time (s)'].tolist())
        #x_data = x_data - x_data[0] # Phaseshift to start measurements at zero

        # Get data from ultrasonic sensors
        noBond_height_data = np.array(data['noBond_height 1 (mm)'].tolist())  # Bond
        Bond_height_data = np.array(data['Bond_height 2 (mm)'].tolist())  # noBond 

        sim_counter = 0

        crests = int(input('Crests between sensors: '))
        crest_flag = False

        max_measurements = len(time_data)

        # Quality of life for data viewing
        temp_datapoints = 0
        datapoints_per_wave = 0 # To adjust graph to some amount of waves on screen
        while Bond_wave_counter < max_waves and sim_counter < len(time_data):

            # Get time starting at zero
            if time_start_flag == True:
                time_start_flag = False
                time_start = float(time_data[0])

            # If at any point the current time is less than the starting time reset everything
            if float(time_data[0]) < time_start:
                time_start = float(time_data[0])
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
            
            # # Get serial data into variables (offset made with real measurements)
            # ttime = float(data[0]) - time_start
            # noBond_height = float(data[9])
            # Bond_height = float(data[10])

            # Simutaled stuff
            ttime = time_data[sim_counter] - time_start
            Bond_height = Bond_height_data[sim_counter]
            noBond_height = noBond_height_data[sim_counter]
            
            
            #noBond_height = float(data[9]) + noBond_offset
            #Bond_height = float(data[10]) + Bond_offset
            

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

                    #print('\nBond:\nHalf period: \n', Bond_half_period)
                    Bond_half_period = []

            if amplitude_flag == 0:
                Bond_half_period.append(Bond_height)

            amplitude_flag += 1
            # Make sure it stays bounded at 3
            if amplitude_flag == 3:
                amplitude_flag = 0
                
            sim_counter += 1 # Only for simulations

            # START GRAPH AND INTERFACE
            Bond_measurements.append(Bond_height)
            noBond_measurements.append(noBond_height)
            time_csv.append(ttime)
        
        wavelength_median = np.median(wavelength)

        # Save results to csv file
        print('\nCheck results before saving: ')
        print('noBond pp average:', noBond_pp_avg, 'and std: ', noBond_pp_stdev)
        print('Bond pp average:', Bond_pp_avg, 'and std: ', Bond_pp_stdev)
        print('noBond freq average:', noBond_freq_avg, 'and std: ', noBond_freq_stdev)
        print('Bond freq average:', Bond_freq_avg, 'and std: ', Bond_freq_stdev)
        print('Wavelength average:', wavelength_avg, 'median:', wavelength_median, 'std: ', wavelength_stdev)

        if changing_data == True:
            results_file = open(csv_path + 'Results_with_median.csv', mode='a')
            # Name of the test
            crank_pos = filename[file_pos][0]
            motor_freq = filename[file_pos][1]

            results_file.write('\n' + crank_pos + ',' + motor_freq + ',')
            # Add the results
            results_file.write(str(noBond_pp_avg) + ',' + str(noBond_pp_stdev) + ',')
            results_file.write(str(Bond_pp_avg) + ',' + str(Bond_pp_stdev) + ',')
            results_file.write(str(noBond_freq_avg) + ',' + str(noBond_freq_stdev) + ',')
            results_file.write(str(Bond_freq_avg) + ',' + str(Bond_freq_stdev) + ',')
            results_file.write(str(wavelength_median) + ',' + str(wavelength_avg) + ',' + str(wavelength_stdev))
            results_file.close() 
        # print('\nDone with viewing data?') # Por alguna razon entra en panico si no
        
        if changing_data == True:
            file_pos += 1
        else: break
        
          

except KeyboardInterrupt:
    print("Deteniendo la lectura de datos.")


# # Correct for wrong number of crests - ALWAYS MAKE CORR POSITIVE
# if input('\nWas the wavelength correct? (y/n): ') == 'n':
#     corr = int(input('How many crests do you need to add?: '))
#     for i in range(len(wavelength)):
#         # Don't use for more than 2 crests
#         if wavelength[i] != sensor_dist:
#             wavelength[i] = 1/(1/wavelength[i] + corr/sensor_dist)

#     wavelength_avg, wavelength_stdev = Stats(wavelength)

# Uncomment if want to see graph

# Plot variables
wave_graph = plt.figure()
Bond_ax = wave_graph.add_subplot(111)

# Slider
num_graph_waves = 4 # Amount of waves to plot
data_range = num_graph_waves*datapoints_per_wave # Resulting amount of data points to plot
slider_width = 0.65
time_range = wave_graph.add_axes([0.15, 0.05, slider_width, 0.03]) # Slider visual dimensions


# Generate wave lines
Bond_line, = Bond_ax.plot(time_data[:data_range]-time_start, Bond_height_data[:data_range], lw = 2, marker = '.', label = 'Bond')
noBond_line, = Bond_ax.plot(time_data[:data_range]-time_start, noBond_height_data[:data_range], lw = 2, marker = '.', label = 'noBond', color = 'green')

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
    slider_pos = int(val*(len(time_data) - data_range -1)/(slider_width*100)) # Mapea el rango del slider a la posición del arreglo
    
    # Ajusta el lim del eje x para el nuevo valor del slider
    plt.subplot(111)
    plt.xlim([time_data[slider_pos]-time_start, time_data[slider_pos + data_range]-time_start]) 

    # Da nuevos datos por graficar
    Bond_line.set_ydata(Bond_height_data[slider_pos:slider_pos + data_range])
    Bond_line.set_xdata(time_data[slider_pos:slider_pos + data_range]-time_start)
    noBond_line.set_ydata(noBond_height_data[slider_pos:slider_pos + data_range])
    noBond_line.set_xdata(time_data[slider_pos:slider_pos + data_range]-time_start)

    wave_graph.canvas.draw_idle()
    
# Function to call on eahc slider change
time_slider.on_changed(update_slider)

if changing_data == False: plt.show()

