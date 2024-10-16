# Sine regression using SciPy
import random
import pandas
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit


# Define function to find parameters for
def function(x, amp, freq, phase, offset):
    y = amp*np.sin(freq*x + phase) + offset
    return y

# Read the actual data from csv file
data = pandas.read_excel('New_data.xlsx','data_35n 2')
x_full = np.array(data['Tiempo (ms)'].tolist())
y_full = np.array(data['Altura (mm)'].tolist())

# Use only some of the data in case of errors
max_data = 400 # Total data points to use (minus bad_data)
bad_data = np.where(x_full==60)[0][0] # First data points to ignore - Automatic

x_data = x_full[bad_data:max_data-bad_data]
y_data = y_full[bad_data:max_data-bad_data]

# Define initial guesses
# Frequency is the one messing everything up - has to be within 0.0002 of real value
# Use excel as a lookup table to approximate

#amp = (max(y_data)-min(y_data))/2
amp = 30 # [mm]
freq = 0.0055 # [krad/s]
phase = 0 # [rad]
offset = 390 # [mm]
initial = (amp, freq, phase, offset) # For the actual data


# Define bounds for the parameters
#lower_bounds = [-75,0.001,-10,385]
#upper_bounds = [75,0.01,10,400]

# Just use a function to get the regression, it's so awesome!
# popt is what matters, pcov is just about covariance so who cares

popt, pcov = curve_fit(function, x_data, y_data, initial) # Without bounds
#popt, pcov = curve_fit(function, x_data, y_data, initial, bounds=[lower_bounds,upper_bounds]) # With bounds


# Predict the regressed sine wave
amp = popt[0]
freq = popt[1]
phase = popt[2]
offset = popt[3]
y_pred = amp*np.sin(freq*x_data + phase) + offset


# Generate plots
plt.plot(x_data,y_data,color='#00FFFF',label='Data')
plt.plot(x_data,y_pred,color='#FF007F',label='Regression')
plt.legend()
plt.show()

# Display results
amp = abs(amp) # avoid negative values
freq = freq*1000/(2*np.pi) # convert to Hz


print("Amplitude:", abs(amp), "mm")
print("Frequency:", freq, "Hz")
