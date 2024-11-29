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
csv_path = r'C:\Users\Daniel Q\Documents\TEC\2024 - II Semestre\eWave\eWave\Datasets\\' # Para Daniel
#csv_path = r'C:\Users\Lenovo\Documents\eWave\eWave\Datasets\\' # Para Andrés
csv_filename = csv_path + 'Ultra_KKI_v2_20.csv'


data = pandas.read_csv(csv_filename)
x_full = np.array(data['Time (s)'].tolist())
y_full1 = np.array(data['Height 1 (mm)'].tolist())
y_full2 = np.array(data['Height 2 (mm)'].tolist())

# Use only some of the data in case of errors
max_data = 200 # Total data points to use (minus bad_data)
#bad_data = np.where(x_full==31.49)[0][0] # First data points to ignore - Automatic
bad_data = 0

x_data = x_full[bad_data:max_data-bad_data]
y_data1 = y_full1[bad_data:max_data-bad_data]
y_data2 = y_full2[bad_data:max_data-bad_data]



# REGRESSION
# Define initial guesses
# Frequency is the one messing everything up - has to be within 0.0002 of real value
# Use excel as a lookup table to approximate

amp1 = 25 # [mm]
freq1 = 3.11 # [rad/s]
phase1 = 0 # [rad]
offset1 = 0 # [mm]
initial1 = (amp1, freq1, phase1, offset1) # For the actual data

amp2 = 25 # [mm]
freq2 = 3.11 # [rad/s]
phase2 = 0 # [rad]
offset2 = 0 # [mm]
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

# Get phase between 0 and 2pi
if phase1 > 2*np.pi or phase1 < 0:
    phase1 = phase1 - phase1//(2*np.pi) * 2*np.pi

y_pred1 = amp1*np.sin(freq1*x_data + phase1) + offset1

amp2 = popt2[0]
freq2 = popt2[1]
phase2 = popt2[2]
offset2 = popt2[3]

if popt2[0] < 0:
    amp2 = amp2 * -1
    phase2 = phase2 + np.pi

if phase2 > 2*np.pi or phase2 < 0:
    phase2 = phase2 - phase2//(2*np.pi) * 2*np.pi

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
amp = (amp1 + amp2)/2
freq = (freq1 + freq2) / (4 * np.pi) # in Hz

# Get wavelength
distance_sensors = 2220 # [mm]
phase_diff = abs(phase2 - phase1) # [rad]

crests = 1 # Number of crests between sensors

# Correct for multiple wavelengths - Check on the actual wave or guess ranges?
for i in range(crests - 1):
    phase_diff += 2*np.pi

wavelength = distance_sensors * 2*np.pi/phase_diff # [mm]

# Display results
print("Amplitude:", amp, "mm")
print("Frequency:", freq, "Hz")
print("Wavelength:", wavelength, "mm")
