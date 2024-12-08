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
import pandas as pd


# Nombre del archivo CSV
crank_pos = str(input('Crank Position: '))
motor_freq = input('Motor Frequency (Hz): ')


csv_path = r'C:\Users\Daniel Q\Documents\TEC\2024 - II Semestre\eWave\eWave\Datasets\\' # Para Daniel
#csv_path = r'C:\Users\Lenovo\Documents\eWave\eWave\Datasets\\' # Para Andrés
#csv_path = r'C:\Users\garab\ewave Repo\eWave\Datasets\\' # Para Gabriel

csv_filename = csv_path + crank_pos + str(motor_freq) + '.csv'


# GENERAL VARIABLES

# Wave variables
max_waves = 99
graph_max = 50 # Data points, not waves
time_start_flag = True

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

    # Avoid extremely large results
    if abs(percentage) > 0.001:
        wavelength.append(sensor_dist/percentage)


def Stats(var):
    # Calculate new average and standard deviation
    avg = statistics.mean(var)
    stdev = statistics.stdev(var)

    return avg, stdev



# ACTUAL CODE
n = 0
data = pd.read_csv(csv_filename)
time_pd = np.array(data['Time (s)'].tolist())
noBond_pd = np.array(data['noBond_height 1 (mm)'].tolist())
Bond_pd = np.array(data['Bond_height 2 (mm)'].tolist())
time_start = 0
crests = int(input('Crests: '))

def Update_graphs(n):
    global time_csv, Bond_measurements, Bond_line, noBond_measurements, noBond_line
    global noBond_first_wave, Bond_first_wave, time_start_flag, noBond_anti_ripple, Bond_anti_ripple
    global noBond_half_period, noBond_wave_counter, Bond_half_period, Bond_wave_counter
    global noBond_max_height, noBond_min_height, Bond_max_height, Bond_min_height
    global noBond_prev_time, Bond_prev_time, noBond_sign_cross, Bond_sign_cross, time_diff
    global noBond_pp, Bond_pp, noBond_freq, Bond_freq, wavelength
    global noBond_pp_avg, noBond_pp_stdev, Bond_pp_avg, Bond_pp_stdev
    global noBond_freq_avg, noBond_freq_stdev, Bond_freq_avg, Bond_freq_stdev, wavelength_avg, wavelength_stdev, crest_flag, crests
    global time_start

    # Get time starting at zero
    if time_start_flag == True:
        time_start_flag = False
        time_start = float(time_pd[n])

    # If at any point the current time is less than the starting time reset everything
    if float(time_pd[n]) < time_start:
        time_start = float(time_pd[n])
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
    
    # Get serial data into variables
    ttime = float(time_pd[n]) - time_start
    noBond_height = float(noBond_pd[n])
    Bond_height = float(Bond_pd[n])
    

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

    Bond_half_period.append(Bond_height)



while Bond_wave_counter < max_waves:
    Update_graphs(n)
    n += 1

# Correct for wrong number of crests
if input('\nWas the wavelength correct? (y/n): ') == 'n':
    corr = int(input('How many crests do you need to add?: '))
    for i in range(len(wavelength)):
        wavelength[i] = 1/(1/wavelength[i] + corr/2.2)

    wavelength_avg, wavelength_stdev = Stats(wavelength)
