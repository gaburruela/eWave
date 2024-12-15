import csv
import numpy as np
import pandas


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

file_pos = 0 # Pos inicial para iteracion de los datasets
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
while file_pos < len(filename):
    # DATA READING AND PREPROCESSING
    # Read the actual data from csv file
    csv_path = r'C:\Users\Daniel Q\Documents\TEC\2024 - II Semestre\eWave\eWave\Datasets\\' # Para Daniel
    #csv_path = r'C:\Users\Lenovo\Documents\eWave\eWave\Datasets\\' # Para Andrés
    #csv_path = r'C:\Users\garab\ewave Repo\eWave\Datasets\\' # Para Gabriel


    # To run through all dataset files
    dataset_filename = csv_path + filename[file_pos][0] + filename[file_pos][1]  + '.csv' # Chooses between different dataset files
    data = pandas.read_csv(dataset_filename) # Current file's dataset
    print('\nWorking on file: ', filename[file_pos][0] + filename[file_pos][1])
    print('File pos:', file_pos)

    # Get time and temperature data
    time = np.array(data['Time (s)'].tolist())

    motor_temp = np.array(data['Motor_Temp (C)'].tolist())
    water_temp = np.array(data['Water_Temp (C)'].tolist())

    # Delete measurements if they have weird times
    prev_time = 0
    for i in range(len(time)):
        if time[i] - prev_time < -0.1:
            motor_temp = np.delete(motor_temp, np.arange(0,i))
            water_temp = np.delete(water_temp, np.arange(0,i))
            break
        prev_time = time[i]

    # Get statistic parameters
    motor_temp_median = np.median(motor_temp)
    motor_temp_avg = np.mean(motor_temp)
    motor_temp_stdev = np.std(motor_temp)

    water_temp_median = np.median(water_temp)
    water_temp_avg = np.mean(water_temp)
    water_temp_stdev = np.std(water_temp)


    # Save results to csv file
    print('\nCheck results before saving: ')
    print('Motor temperature - Median: ', motor_temp_median, ', Average: ', motor_temp_avg, ', Std: ', motor_temp_stdev)
    print('Water temperature - Median: ', water_temp_median, ', Average: ', water_temp_avg, ', Std: ', water_temp_stdev)


    if changing_data == True:
        results_file = open(csv_path + 'Temperature_synthesis.csv', mode='a')
        # Name of the test
        crank_pos = filename[file_pos][0]
        motor_freq = filename[file_pos][1]

        results_file.write('\n' + crank_pos + ',' + motor_freq + ',')
        # Add the results
        results_file.write(str(motor_temp_median) + ',' + str(motor_temp_avg) + ',' + str(motor_temp_stdev) + ',')
        results_file.write(str(water_temp_median) + ',' + str(water_temp_avg) + ',' + str(water_temp_stdev))
        results_file.close()

    if changing_data == True:
        file_pos += 1
        
    else: break
