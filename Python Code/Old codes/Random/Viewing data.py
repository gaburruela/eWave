# Sine regression using SciPy - 2 sensors
import random
import pandas
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

# DATA READING AND PREPROCESSING
# Read the actual data from csv file
csv_path = r'C:\Users\Daniel Q\Documents\TEC\2024 - II Semestre\eWave\eWave\Datasets\\' # Para Daniel
#csv_path = r'C:\Users\Lenovo\Documents\eWave\eWave\Datasets\\' # Para Andrés
#csv_path = r'C:\Users\garab\eWave Repo\eWave\Datasets\\' # Para Gabriel

# Make sure to change for each test
print('\nFile name information:')
crank_pos = input('Crank Position: ')
motor_freq = input('Motor Frequency (Hz): ')

csv_filename = csv_path + crank_pos + motor_freq + '.csv'

data = pandas.read_csv(csv_filename)

# Get data from ultrasonic sensors
x_data = np.array(data['Time (s)'].tolist())
y_data1 = np.array(data['Height 1 (mm)'].tolist())
y_data2 = np.array(data['Height 2 (mm)'].tolist())

x_data = x_data - x_data[0] # Phaseshift to start measurements at zero

# Preliminary graphs to make initial guesses
plt.subplot(211) # For initial guesses
# plt.plot(x_data, y_data1, marker = 'o', color='m', label='No Bond')
plt.plot(x_data, y_data1, color='m', label='No Bond')
plt.ylabel('Height (mm)')
plt.legend()
plt.title('Heights for initial guesses')

plt.subplot(212) # For seeing trends in a lot more data
# plt.plot(x_data[0:200], y_data2[0:200], marker = 'o', color='c', label='Bond')
plt.plot(x_data, y_data2, color='c', label='Bond')
plt.xlabel('Time (s)')
plt.ylabel('Height (mm)')
plt.legend()

plt.show()
