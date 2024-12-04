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

# Make sure to change for each test
print('\nFile name information:')
crank_pos = input('Crank Position: ')
motor_freq = input('Motor Frequency (Hz): ')

csv_filename = csv_path + crank_pos + motor_freq + '.csv'


data = pandas.read_csv(csv_filename)

# Get time data
x_data = np.array(data['Time (s)'].tolist())
x_data = x_data - x_data[0] # Phaseshift to start measurements at zero

# Get data from accelerometer
accel_x = np.array(data['Accel_x (m2/s)'].tolist())
accel_y = np.array(data['Accel_y (m2/s)'].tolist())
accel_z = np.array(data['Accel_z (m2/s)'].tolist())
accel_z -= 9.8

accel_total = np.sqrt(accel_x**2 + accel_y**2 + accel_z**2)

# Graph acceleration data
plt.subplot(211)
plt.plot(x_data, accel_x, color='r', label='Acceleration x')
plt.plot(x_data, accel_y, color='g', label='Acceleration y')
plt.plot(x_data, accel_z, color='b', label='Acceleration z')
plt.legend()
plt.ylabel('Acceleration (m²/s)')
plt.title('Wavemaker flap accelaration')

plt.subplot(212)
plt.plot(x_data, accel_total, 'c', label='Acceleration magnitude')
plt.legend()
plt.xlabel('Time (s)')
plt.ylabel('Acceleration (m²/s)')

plt.show()


# Get data from ultrasonic sensors
y_data1 = np.array(data['Height 1 (mm)'].tolist())
y_data2 = np.array(data['Height 2 (mm)'].tolist())

# Preliminary graphs to make initial guesses
plt.subplot(211) # For initial guesses
plt.plot(x_data, y_data1, color='m', label='No Bond')
plt.ylabel('Height (mm)')
plt.grid()
plt.legend()
plt.title('Heights for initial guesses')

plt.subplot(212) # For seeing trends in a lot more data
plt.plot(x_data[0:200], y_data2[0:200], color='c', label='Bond')
plt.xlabel('Time (s)')
plt.ylabel('Height (mm)')
plt.grid()
plt.legend()

plt.show()



# REGRESSION
# Define initial guesses
# Frequency is the one messing everything up - has to be within 0.0002 of real value

# Input guesses
print('\nInput educated guesses:')
amp1 = float(input('Peak-peak (mm): '))/2
freq1 = 2*np.pi/float(input('Period (s): '))

amp2 = amp1 # [mm]
freq2 = freq1 # [rad/s]


phase1 = 0 # [rad]
offset1 = 0 # [mm]
initial1 = (amp1, freq1, phase1, offset1) # For the actual data

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
if phase1 >= 2*np.pi or phase1 < 0:
    phase1 = phase1 - phase1//(2*np.pi) * 2*np.pi

y_pred1 = amp1*np.sin(freq1*x_data + phase1) + offset1

amp2 = popt2[0]
freq2 = popt2[1]
phase2 = popt2[2]
offset2 = popt2[3]

if popt2[0] < 0:
    amp2 = amp2 * -1
    phase2 = phase2 + np.pi

if phase2 >= 2*np.pi or phase2 < 0:
    phase2 = phase2 - phase2//(2*np.pi) * 2*np.pi

y_pred2 = amp2*np.sin(freq2*x_data + phase2) + offset2


# Generate plots - verify the regression was nice
plt.subplot(211)
plt.plot(x_data,y_data1,color='#00FFFF',label='Data - No Bond')
plt.plot(x_data,y_pred1,color='#FF007F',label='Regression - No Bond')
plt.ylabel('Height (mm)')
plt.legend()
plt.title('Regression vs measured data')

plt.subplot(212)
plt.plot(x_data,y_data2,color='#00FFFF',label='Data - Bond')
plt.plot(x_data,y_pred2,color='#FF007F',label='Regression - Bond')
plt.xlabel('Time (s)')
plt.ylabel('Height (mm)')
plt.legend()
plt.show()



# ACTUAL RESULTS
# Get averages
amp = (amp1 + amp2)/2
freq = (freq1 + freq2) / (4 * np.pi) # in Hz

# Get wavelength
distance_sensors = 2220 # [mm]
phase_diff = phase2 - phase1 # [rad] - Order does matter (Bond comes first)
phase_diff = phase_diff % (2*np.pi) # Get between 0 and 2pi

crests = int(input('\nNumber of crests between sensors: ')) # Number of crests between sensors

# Correct for multiple wavelengths - Check on the actual wave or guess ranges?
for i in range(crests - 1):
    phase_diff += 2*np.pi

wavelength = distance_sensors * 2*np.pi/phase_diff # [mm]
wavelength /= 1000 # [m]

# Round results
amp = round(amp, 2)
freq = round(freq, 3)
wavelength = round(wavelength, 3)

# Display results
print('\nRegression results:')
print('Amplitude:', amp, 'mm')
print('Frequency:', freq, 'Hz')
print('Wavelength:', wavelength, 'm')

# Get real measurements
print('\nReal measurements:')
real_amp = float(input('Real peak-peak (cm): '))/0.2
real_freq = 5/float(input('Real 5 periods (s): '))
real_wavelength = float(input('Real wavelength (m): '))

# Round results
real_amp = round(real_amp, 2)
real_freq = round(real_freq, 3)
real_wavelength = round(real_wavelength, 3)


# Abre el archivo CSV en modo de escritura
if (input('\nSave data? (y/n): ') == 'y'):
    results_file = open('results.csv', mode='a')
    # Name of the test
    results_file.write('\n' + crank_pos + ',' + motor_freq + ',')
    # Regression results vs Real measurements
    results_file.write(str(amp) + ',' + str(real_amp) + ',')
    results_file.write(str(freq) + ',' + str(real_freq) + ',')
    results_file.write(str(wavelength) + ',' + str(real_wavelength))
    results_file.close()
