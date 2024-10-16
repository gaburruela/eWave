# Sine regression using SciPy - 2 sensors
import random
import pandas
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit


# Define function to find parameters for
def function(x, amp, freq, phase, offset):
    y = amp*np.sin(freq*x + phase) + offset
    return y


# DATA READING AND PREPROCESSING
# Read the actual data from csv file
data = pandas.read_excel('Double_data.xlsx','30 - 1 sensor')
x_full = np.array(data['Tiempo (ms)'].tolist())
y_full1 = np.array(data['Altura 1 (mm)'].tolist())
y_full2 = np.array(data['Altura 2 (mm)'].tolist())

# Use only some of the data in case of errors
max_data = 400 # Total data points to use (minus bad_data)
bad_data = np.where(x_full==60)[0][0] # First data points to ignore - Automatic

x_data = x_full[bad_data:max_data-bad_data]
y_data1 = y_full1[bad_data:max_data-bad_data]
y_data2 = y_full2[bad_data:max_data-bad_data]



# REGRESSION
# Define initial guesses
# Frequency is the one messing everything up - has to be within 0.0002 of real value
# Use excel as a lookup table to approximate

amp1 = 30 # [mm]
freq1 = 0.0045 # [krad/s]
phase1 = 0 # [rad]
offset1 = 390 # [mm]
initial1 = (amp1, freq1, phase1, offset1) # For the actual data

amp2 = 30 # [mm]
freq2 = 0.0045 # [krad/s]
phase2 = 0 # [rad]
offset2 = 390 # [mm]
initial2 = (amp2, freq2, phase2, offset2) # For the actual data


# Just use a function to get the regression, it's so awesome!
# popt is what matters, pcov is just about covariance so who cares

popt1, pcov1 = curve_fit(function, x_data, y_data1, initial1)
popt2, pcov2 = curve_fit(function, x_data, y_data2, initial2)



# PREDICTION
# Predict the regressed sine wave
amp1 = popt1[0]
freq1 = popt1[1]
phase1 = popt1[2]
offset1 = popt1[3]

# Avoid negative amplitudes
if popt1[0] < 0:
    amp1 = amp1 * -1
    phase1 = phase1 + np.pi

y_pred1 = amp1*np.sin(freq1*x_data + phase1) + offset1

amp2 = popt2[0]
freq2 = popt2[1]
phase2 = popt2[2]
offset2 = popt2[3]

if popt2[0] < 0:
    amp2 = amp2 * -1
    phase2 = phase2 + np.pi

y_pred2 = amp2*np.sin(freq2*x_data + phase2) + offset2


# Generate plots
plt.plot(x_data,y_data1,color='#00FFFF',label='Data')
plt.plot(x_data,y_pred1,color='#FF007F',label='Regression')
plt.legend()
plt.show()

plt.plot(x_data,y_data2,color='#00FFFF',label='Data')
plt.plot(x_data,y_pred2,color='#FF007F',label='Regression')
plt.legend()
plt.show()



# ACTUAL RESULTS
# Get averages
amp = (amp1 + amp1)/2
freq = (freq1 + freq2)/2 * 1000/(2*np.pi) # in Hz

# Get wavelength
distance_sensors = 100 # [mm]
time = abs(phase2 - phase1)/(freq*2*np.pi) # [s]
vel = distance_sensors/time # [mm/s]
wavelength = vel/freq # [mm]

# Display results
print("Amplitude:", amp, "mm")
print("Frequency:", freq, "Hz")
print("Wavelength:", wavelength, "mm")
