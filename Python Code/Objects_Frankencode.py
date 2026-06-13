import time
import serial
import csv
import numpy as np
import argparse
import os
import statistics
from PIL import Image, ImageTk # Images
import tkinter as tk # Interface
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from matplotlib import style
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import winsound
# import pyautogui
import threading
import queue
from pymodbus.client import ModbusSerialClient

# Sensor class definition
class Sensor:

    # Sensor variables
    def __init__(self, name):
        self.name = name

        self.measurements = []  # Historic height meassurements
        self.rolling_array = []  # Temporary array for calculating rolling avg of height
                                 # the rolling avg is used as current height value

        self.first_wave = True  # Flag to ignore first wave
        self.anti_ripple = 0

        self.half_period = []  # Stores height data of half a period
        self.wave_counter = 0  # # of waves detected in this sensor

        self.max_height = 0
        self.min_height = 0

        self.prev_time = 0  # Holds time of last cero crossing
        self.sign_cross = 0  # Zero crossing sign positive or negative

        self.pp = []  # Historic pp calculations
        self.freq = [] # Historic freq calculations

        self.pp_avg = 0  # Peak to peak average from historic data
        self.pp_stdev = 0  # Peak to peak standard deviation from historic data

        self.freq_avg = 0  # Frequency average from historic data
        self.freq_stdev = 0  # Frequency standard deviation from historic data

        self.wavelength = [] # Historic wavelength calculations
        self.sensor_dist = 2.22  # Distance between sensors used for wavelength in meters
        self.wavelength_avg = 0  # Wavelength average from historic data
        self.wavelength_stdev = 0  # Wavelength standard deviation from historic data


    # Sensor methods
    def update_pp(self):

        # Update current max or min
        if self.half_period[-1] > 0:
            self.max_height = max(self.half_period)

        else:
            self.min_height = min(self.half_period)

        # Compute peak-to-peak
        if self.max_height != 0 and self.min_height != 0:
            self.pp.append(self.max_height - self.min_height)


    def update_freq(self, current_time):

        # Avoid division by zero
        if current_time - self.prev_time != 0:
            self.freq.append(1 / (current_time - self.prev_time))

    def update_wavelength(self, wavelength_companion_sensor):
        period = 1/wavelength_companion_sensor.freq[-1]

        # Add a full period per crest
        percentage = time_diff/period + (crests - 1)

        # Make sure phase is in sync - if not take away half a period
        if wavelength_companion_sensor.sign_cross * self.sign_cross < 0:
            percentage -= 0.5

        if abs(percentage) > 0.001:  # Check wether this makes any sense!!!!!!!!!!
            self.wavelength.append(self.sensor_dist/percentage)


    def compute_stats(self, values):

        avg = statistics.mean(values)
        stdev = statistics.stdev(values)

        return avg, stdev

# SERIAL COMMUNICATION

ARDPORT = 'COM6'  # COM3 para Andrés / COM4 para Daniel / COM6 para Gabriel
ARDBAUDRATE = 115200

winsound.Beep(350,500)

# Comment out when using simulated data
# Connect to serial port
# try:
#     ser = serial.Serial(port, baudrate)
#     print(f"Conectado al puerto {port} a {baudrate} baudios.")
# except serial.SerialException as e:
#     print(f"No se pudo abrir el puerto {port}: {e}")
#     exit()

# Data queues for threaded serial coms
ard_data_queue = queue.Queue()
vfd_data_queue = queue.Queue()

# Nombre del archivo CSV
motor_freq = input('Motor Frequency (Hz): ')
crank_pos = input('Crank Position (mm): ')

if input('Are you sure? (y/n): ') == 'n':
    motor_freq = input('Motor Frequency (Hz): ')
    crank_pos = input('Crank Position (mm): ')

print('\nReady to start measurements!')

# csv_path = r'C:\Users\Daniel Quesada\Documents\GitHub\eWave\Datasets\II Semester 2025\Raw_Data\\' # Para Daniel
#csv_path = r'C:\eWave\eWave\Datasets\II Semester 2025\Raw_Data\\' # Para Andrés
csv_path = r'C:\Users\Gabu\Documents\GitHub\eWave\Datasets\II Semester 2025\Raw_Data\\' # Para Gabriel

csv_filename = csv_path + motor_freq + ' Hz - ' + crank_pos + ' mm.csv'

# Waits a couple of seconds to establish a conection with the arduino
time.sleep(2)

# GENERAL VARIABLES

# Wave variables
max_waves = 5
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



# # Variables for getting wave parameters

# Other flags
time_start_flag = True

# Wavelength thingies
# sensor_dist = 2.22
crest_flag = True
crests = 0  # Check wether can be an input of GUI

# Rolling averages
rolling_window = 5 # number of points to average
# noBond_rolling_array = [] # store first 5 measurements
# Bond_rolling_array = []

# Others
anti_ripple = 7 # crests to ignore


# INTERFACE ANTESALA
window = tk.Tk() # The main Tkinter window
window.title('Plotting in Tkinter')
window.configure(bg="#80e0a7")

# Computer screen width and height
screen_width = 1390
screen_height = 800
# screen_width, screen_height = pyautogui.size()

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

# # Return max and min
# def PP(half_period, max_height, min_height, pp):
#     # PEAK-PEAK
#     # Get new maximum or new minimum
#     if half_period[-1] > 0: max_height = a(half_period)
#     else: min_height = min(half_period)
    
#     # Calculate peak-peak
#     if max_height != 0 and min_height != 0: # Both have been updated at least once
#         pp.append(max_height - min_height)

#     return max_height, min_height


# def Freq(wave_counter, ttime, prev_time, freq):
#     # FREQUENCY
#     if ttime - prev_time != 0:
#         freq.append(1/(ttime - prev_time))


# def Wavelength(wavelength):
#     period = 1/noBond.freq[-1]

#     # Add a full period per crest
#     percentage = time_diff/period + (crests - 1)

#     # Make sure phase is in sync - if not take away half a period
#     if noBond.sign_cross * Bond.sign_cross < 0:
#         percentage -= 0.5

#     if abs(percentage) > 0.001:
#         wavelength.append(sensor_dist/percentage)


# def Stats(var):
#     # Calculate new average and standard deviation
#     avg = statistics.mean(var)
#     stdev = statistics.stdev(var)

#     return avg, stdev

# Arduino polling function

ard_data_queue = queue.Queue()
VFD_data_queue = queue.Queue()

def Serial_coms_thread():
    print('Starting modbus client coms')
    client = ModbusSerialClient(
        port='COM8',  # Revisar puerto
        baudrate=115200,
        parity='N',
        stopbits=1,
        bytesize=8,
        timeout=0.02
    )
    if client.connect():
        print("Modbus client connected")

        # Flush any old data
        client.socket.reset_input_buffer()
        client.socket.reset_output_buffer()
    else:
        print("Failed to connect Modbus client")
        return  # or raise an exception

    SLAVE = 1

    # Connect to Arduino
    # Have to check how to put inputs outside of the function
    ard_port = 'COM6'  # COM3 para Andrés / COM4 para Daniel / COM6 para Gabriel
    baudrate = 115200

    # Connect to serial ard_port
    try:
        ser = serial.Serial(ard_port, baudrate)
        print(f"Conectado al puerto {ard_port} a {baudrate} baudios.")
    except serial.SerialException as e:
        print(f"No se pudo abrir el puerto {ard_port}: {e}")
        exit()

    print('Clearing serial buffers')
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    time.sleep(2)  # Give 2 seconds to let arduino boot up

    # Start automatic arduino meassurements process
    data = "1"
    data += "\r\n"
    ser.write(data.encode())

    while True:
        if ser.in_waiting > 0:
            data = ser.readline().decode('utf-8').strip()
            ard_data_queue.put(data)
            # print("Data: ", data)
            # time.sleep(0.02)

        if not VFD_data_queue.empty():
            cmd = VFD_data_queue.get()
            if cmd[0] == 'set_freq':
                freq = cmd[1]
                client.write_register(0x0002, freq, device_id=SLAVE)
                print('Frequency set')

            elif cmd[0] == 'start':
                client.write_register(0x0001, 1, device_id=SLAVE)
                print('Start drive')

            elif cmd[0] == 'stop':
                client.write_register(0x0001, 0, device_id=SLAVE)
                print('Stop drive')

            else:
                print('Codigo de VFD no soportado')

        else:
            time.sleep(0.01)


# ACTUAL CODE

def Update_graphs():

    global time_csv
    global time_start
    global time_start_flag
    global time_diff
    global crest_flag
    global crests
    global graph_update
    global graph_max
    global points_per_period_flag
    global writer
    global max_waves

    

    try:
        if Bond.wave_counter < max_waves and not ard_data_queue.empty():

            # Get data from ard thread queue
            line = ard_data_queue.get()
            # print('Line received inside the window logic', line)
            data = line.split(',')

            if len(data) == 11:

                writer.writerow(data)

                # Time setup

                if time_start_flag:
                    print('Took start time')
                    time_start_flag = False
                    time_start = float(data[0])

                # Reset if Arduino resets

                if float(data[0]) < time_start:

                    time_start = float(data[0])

                    noBond.first_wave = True
                    Bond.first_wave = True

                    noBond.half_period = []
                    Bond.half_period = []

                    noBond.wave_counter = 0
                    Bond.wave_counter = 0

                    noBond.pp = []
                    Bond.pp = []

                    noBond.freq = []
                    Bond.freq = []

                    wavelength = []

                # Read measurements

                ttime = float(data[0]) - time_start

                noBond_height = float(data[9])
                Bond_height = float(data[10])

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

                noBond_pp_avg_text.config(text = f"No Bond Peak-Peak Avg (mm): {noBond.pp_avg:.2f}")
                noBond_pp_stdev_text.config(text = f"No Bond Peak-Peak StDev (mm): {noBond.pp_stdev:.2f}")

                Bond_pp_avg_text.config(text = f"Bond Peak-Peak Avg (mm): {Bond.pp_avg:.2f}")
                Bond_pp_stdev_text.config(text = f"Bond Peak-Peak StDev (mm): {Bond.pp_stdev:.2f}")

                noBond_freq_avg_text.config(text = f"No Bond Frequency Avg (Hz): {noBond.freq_avg:.3f}")
                noBond_freq_stdev_text.config(text = f"No Bond Frequency StDev (Hz): {noBond.freq_stdev:.3f}")
                
                Bond_freq_avg_text.config(text = f"Bond Frequency Avg (Hz): {Bond.freq_avg:.3f}")
                Bond_freq_stdev_text.config(text = f"Bond Frequency StDev (Hz): {Bond.freq_stdev:.3f}")

                wavelength_avg_text.config(text = f"Wavelength Avg (m): {Bond.wavelength_avg:.3f}")
                wavelength_stdev_text.config(text = f"Wavelength StDev (m): {Bond.wavelength_stdev:.3f}")

                wave_number_text.config(text = f"Wave number: {int(Bond.wave_counter)}")

                # Rolling averages

                noBond.rolling_array.append(noBond_height)
                Bond.rolling_array.append(Bond_height)

                if len(noBond.rolling_array) == rolling_window:

                    noBond_height = statistics.mean(noBond.rolling_array)
                    Bond_height = statistics.mean(Bond.rolling_array)

                    # =================================================
                    # NO BOND
                    # =================================================

                    if len(noBond.half_period) >= 1:

                        if noBond.anti_ripple != 0:

                            if noBond.anti_ripple > anti_ripple:
                                noBond.anti_ripple = 0

                            else:
                                noBond.anti_ripple += 1

                        if (
                            noBond.half_period[-1] * noBond_height < 0
                            and noBond.anti_ripple == 0
                        ) or noBond_height == 0:

                            noBond.anti_ripple += 1

                            # Ignore first crossing

                            if noBond.first_wave:

                                noBond.first_wave = False
                                noBond.prev_time = ttime
                                noBond.sign_cross = noBond_height

                            else:

                                # Peak-to-peak

                                noBond.update_pp()

                                if len(noBond.pp) >= 2:

                                    (
                                        noBond.pp_avg,
                                        noBond.pp_stdev
                                    ) = noBond.compute_stats(noBond.pp)

                                # Frequency

                                if noBond.wave_counter % 1 == 0.5:

                                    noBond.update_freq(ttime)

                                    noBond.prev_time = ttime
                                    noBond.sign_cross = noBond_height

                                    if len(noBond.freq) >= 2:

                                        (
                                            noBond.freq_avg,
                                            noBond.freq_stdev
                                        ) = noBond.compute_stats(noBond.freq)

                                    # Wavelength

                                    Bond.update_wavelength(wavelength_companion_sensor=noBond)

                                    if len(Bond.wavelength) >= 2:

                                        (
                                            Bond.wavelength_avg,
                                            Bond.wavelength_stdev
                                        ) = Bond.compute_stats(Bond.wavelength)

                                    # Graph points

                                    if points_per_period_flag:

                                        graph_max = int(
                                            1.4 * len(noBond.measurements)
                                        )

                                        points_per_period_flag = False

                                noBond.wave_counter += 0.5

                            noBond.half_period = []

                    noBond.half_period.append(noBond_height)

                    noBond.rolling_array.pop(0)

                    # =================================================
                    # BOND
                    # =================================================

                    if len(Bond.half_period) >= 1:

                        if Bond.anti_ripple != 0:

                            if Bond.anti_ripple > anti_ripple:
                                Bond.anti_ripple = 0

                            else:
                                Bond.anti_ripple += 1

                        if (
                            Bond.measurements[-1] * Bond_height < 0
                            and Bond.anti_ripple == 0
                        ) or Bond_height == 0:

                            Bond.anti_ripple = 1

                            if Bond.first_wave:

                                if not noBond.first_wave:

                                    Bond.first_wave = False
                                    Bond.prev_time = ttime

                            else:

                                # Peak-to-peak

                                Bond.update_pp()

                                if len(Bond.pp) >= 2:

                                    (
                                        Bond.pp_avg,
                                        Bond.pp_stdev
                                    ) = Bond.compute_stats(Bond.pp)

                                # Frequency

                                if Bond.wave_counter % 1 == 0.5:

                                    Bond.update_freq(ttime)

                                    Bond.prev_time = ttime

                                else:

                                    time_diff = (
                                        ttime - noBond.prev_time
                                    )

                                    Bond.sign_cross = Bond_height

                                if len(Bond.freq) >= 2:

                                    (
                                        Bond.freq_avg,
                                        Bond.freq_stdev
                                    ) = Bond.compute_stats(Bond.freq)

                                Bond.wave_counter += 0.5

                                graph_update = True

                            Bond.half_period = []

                    Bond.half_period.append(Bond_height)

                    Bond.rolling_array.pop(0)

                    # =================================================
                    # STORE DATA
                    # =================================================

                    Bond.measurements.append(Bond_height)

                    noBond.measurements.append(noBond_height)

                    time_csv.append(ttime)

                    if graph_update:
                        Bond_line.set_xdata(time_csv)
                        Bond_line.set_ydata(Bond.measurements)
                        
                        noBond_line.set_xdata(time_csv)
                        noBond_line.set_ydata(noBond.measurements)

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

            elif line.find('Zero levels') != -1:

                data_zero = line.split(',')

                noBond_zero_lvl = float(data_zero[1])
                Bond_zero_lvl = float(data_zero[2])

                print(line)

                winsound.Beep(350, 500)

            # --- START MOTOR ---
            elif line.find("Zeros ready") != -1:
                VFD_data_queue.put(['start'])
                print('Starting motor, received zeros ready')

            elif line.find('Ambient humidity') != -1:

                if crest_flag:

                    crests = int(
                        input('\nCrests between sensors: ')
                    )

                    crest_flag = False

                else:

                    winsound.Beep(350, 500)

                    print(line)

            else:
                print(line)

            # Update window after running full logic sequence
            window.after(1,Update_graphs)  # Keep the window loop going

        # Stop motor if wave counter is over limit
        else:
            # print('Entering else condition')
            if Bond.wave_counter >= max_waves:
                VFD_data_queue.put(['stop'])
                print('Stopping motor')

                window.quit()
                return
            else: # Still missing waves but queue is empty
                window.after(10,Update_graphs)  # Keep the window loop going
            

    except KeyboardInterrupt:

        print("Deteniendo la lectura de datos.")




# Create sensor objects

Bond = Sensor('Bond')
Bond.anti_ripple = 7
Bond.sensor_dist = 2.22

noBond = Sensor('noBond')
noBond.anti_ripple = 7

# Open results file and write header
csv_file = open(csv_filename, mode='w', newline='')
writer = csv.writer(csv_file)

writer.writerow([
    "Time (s)",
    "Accel_x (m/s2)",
    "Accel_y (m/s2)",
    "Accel_z (m/s2)",
    "RPM",
    "Humidity (percentage)",
    "Amb_Temp (C)",
    "Water_Temp (C)",
    "Motor_Temp (C)",
    "noBond_height 1 (mm)",
    "Bond_height 2 (mm)"
])

# Start the serial reading thread
threading.Thread(target=Serial_coms_thread, daemon=True).start()
time.sleep(2)

# Set motor frequency to 2000 hz
VFD_data_queue.put(['set_freq',2000])

# RUN THE GUI
window.after(10, Update_graphs)
window.mainloop()

# Correct for wrong number of crests - ALWAYS MAKE CORR POSITIVE
# if input('\nWas the wavelength correct? (y/n): ') == 'n':
#     corr = int(input('How many crests do you need to add?: '))
#     for i in range(len(wavelength)):
#         # Don't use for more than 2 crests
#         if wavelength[i] != sensor_dist:
#             wavelength[i] = 1/(1/wavelength[i] + corr/sensor_dist)

#     wavelength_avg, wavelength_stdev = Stats(wavelength)

# Save results to csv file
if (input('\nSave data? (y/n): ') == 'y'):
    results_file = open(csv_path + 'Results.csv', mode='a')
    # Name of the test
    results_file.write('\n' + motor_freq + ',' + crank_pos + ',')
    # Add the results
    results_file.write(str(noBond.pp_avg) + ',' + str(noBond.pp_stdev) + ',')
    results_file.write(str(Bond.pp_avg) + ',' + str(Bond.pp_stdev) + ',')
    results_file.write(str(noBond.freq_avg) + ',' + str(noBond.freq_stdev) + ',')
    results_file.write(str(Bond.freq_avg) + ',' + str(Bond.freq_stdev) + ',')
    results_file.write(str(Bond.wavelength_avg) + ',' + str(Bond.wavelength_stdev))
    results_file.close()
