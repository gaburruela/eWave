import time
import serial
import csv
import os
import statistics
import winsound
# import pyautogui
import threading
import queue
from pymodbus.client import ModbusSerialClient

import sys
from eWave_automated_GUI import MainWindow
from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication

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

        self.crests = 0


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
        percentage = time_diff/period + (self.crests - 1)

        # Make sure phase is in sync - if not take away half a period
        if wavelength_companion_sensor.sign_cross * self.sign_cross < 0:
            percentage -= 0.5

        if abs(percentage) > 0.001:  # Check wether this makes any sense!!!!!!!!!!
            self.wavelength.append(self.sensor_dist/percentage)


    def compute_stats(self, values):

        avg = statistics.mean(values)
        stdev = statistics.stdev(values)

        return avg, stdev

class Comms:
    def __init__(self):
        self.ard_port = None
        self.VFD_port = None

        self.ser = None
        self.client = None

        self.ard_baudrate = 115200
        self.VFD_baudrate = 19200

    def set_ports(self, ard_port, VFD_port):
        self.ard_port = ard_port
        self.VFD_port = VFD_port

    def connect(self):
        self.client = ModbusSerialClient(
            port=self.VFD_port,
            baudrate=19200,
            parity="N",
            stopbits=1,
            bytesize=8,
            timeout=1
        )


        if not self.client.connect():
            raise RuntimeError(f"Failed to connect VFD on {self.VFD_port}")

        self.client.socket.reset_input_buffer()
        self.client.socket.reset_output_buffer()

        self.ser = serial.Serial(
            self.ard_port,
            115200,
            timeout=0.02
        )

        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

        time.sleep(2)


# csv_path = r'C:\Users\Daniel Quesada\Documents\GitHub\eWave\Datasets\II Semester 2025\Raw_Data\\' # Para Daniel
#csv_path = r'C:\eWave\eWave\Datasets\II Semester 2025\Raw_Data\\' # Para Andrés
#csv_path = r'C:\Users\Gabu\Documents\GitHub\eWave\Datasets\II Semester 2025\Raw_Data\\' # Para Gabriel
csv_path = r'C:\Users\Lourdes\Downloads\Andres\eWave\Datasets\II Semester 2025\Raw_Data\\'

# GENERAL VARIABLES

# Graph variables
time_csv = []
time_start = 0

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
crest_flag = True

# Rolling averages
rolling_window = 5 # number of points to average

# Others
anti_ripple = 7 # crests to ignore


# Arduino and VFD polling function

ard_data_queue = queue.Queue()
VFD_data_queue = queue.Queue()

# State machine control
state = 'IDLE'


def Serial_coms_thread():

    print('Clearing serial buffers')
    serial_comms.ser.reset_input_buffer()
    serial_comms.ser.reset_output_buffer()

    time.sleep(2)  # Give 2 seconds to let arduino boot up

    # Start automatic arduino meassurements process
    data = "Start\r\n"
    serial_comms.ser.write(data.encode())

    while True:
        if serial_comms.ser.in_waiting > 0:
            data = serial_comms.ser.readline().decode('utf-8').strip()
            ard_data_queue.put(data)
            print("Data: ", data)
            # time.sleep(0.02)

        if not VFD_data_queue.empty():
            cmd = VFD_data_queue.get()
            print('Command: ', cmd)
            if cmd[0] == 'set_freq':
                freq = int(cmd[1] * 100)
                serial_comms.client.write_register(0x0002, freq, device_id=1, no_response_expected=True)
                print('Frequency set')

            elif cmd[0] == 'start':
                serial_comms.client.write_register(0x0001, 1, device_id=1, no_response_expected=True)
                print('Start drive')

            elif cmd[0] == 'stop':
                serial_comms.client.write_register(0x0001, 0, device_id=1, no_response_expected=True)
                print('Stop drive')

            else:
                print('Codigo de VFD no soportado')

            time.sleep(2)

        else:
            time.sleep(0.01)


# WAVE LOGIC CONTROL FUNCTION

def Data_and_window_processing():

    global time_csv
    global time_start
    global time_start_flag
    global time_diff
    global crest_flag
    global writer
    global state

    

    try:
        if Bond.wave_counter < GUI.experiment_wave_limit and not ard_data_queue.empty():

            # Get data from ard thread queue
            line = ard_data_queue.get()
            # print('Line received inside the GUI logic', line)
            data = line.split(',')

            # if line.find('Wave data') != -1:
            if data[0] == "Wave data":

                writer.writerow(data)

                # Time setup

                if time_start_flag:
                    print('Took start time')
                    time_start_flag = False
                    time_start = float(data[1])

                # Reset if Arduino resets

                if float(data[1]) < time_start:

                    time_start = float(data[1])

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

                # Read measurements

                ttime = float(data[1]) - time_start

                noBond_height = float(data[10])
                Bond_height = float(data[11])

                # Update GUI values
                Humidity_value = (float(data[6]))
                
                AmbTemp_value = (float(data[7]))
                
                WaterTemp_value = (float(data[8]))

                MotorTemp_value = (float(data[9]))

                AngularVelocity_value = (float(data[5]))


                # Rolling averages

                noBond.rolling_array.append(noBond_height)
                Bond.rolling_array.append(Bond_height)

                if len(noBond.rolling_array) == rolling_window:

                    noBond_height = statistics.mean(noBond.rolling_array)
                    Bond_height = statistics.mean(Bond.rolling_array)

                    # ======================== NO BOND ========================

                    if len(noBond.half_period) >= 1:

                        if noBond.anti_ripple != 0:

                            if noBond.anti_ripple > anti_ripple:
                                noBond.anti_ripple = 0

                            else:
                                noBond.anti_ripple += 1

                        if (noBond.half_period[-1] * noBond_height < 0
                            and noBond.anti_ripple == 0) or noBond_height == 0:

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

                                    noBond.pp_avg, noBond.pp_stdev  = noBond.compute_stats(noBond.pp)

                                # Frequency

                                if noBond.wave_counter % 1 == 0.5:

                                    noBond.update_freq(ttime)

                                    noBond.prev_time = ttime
                                    noBond.sign_cross = noBond_height

                                    if len(noBond.freq) >= 2:

                                        noBond.freq_avg, noBond.freq_stdev = noBond.compute_stats(noBond.freq)

                                    # Wavelength

                                    Bond.update_wavelength(wavelength_companion_sensor=noBond)

                                    if len(Bond.wavelength) >= 2:

                                        Bond.wavelength_avg, Bond.wavelength_stdev  = Bond.compute_stats(Bond.wavelength)


                                noBond.wave_counter += 0.5

                            noBond.half_period = []

                    noBond.half_period.append(noBond_height)

                    noBond.rolling_array.pop(0)

                    # ======================== BOND ========================

                    if len(Bond.half_period) >= 1:

                        if Bond.anti_ripple != 0:

                            if Bond.anti_ripple > anti_ripple:
                                Bond.anti_ripple = 0

                            else:
                                Bond.anti_ripple += 1

                        if (Bond.measurements[-1] * Bond_height < 0
                            and Bond.anti_ripple == 0) or Bond_height == 0:

                            Bond.anti_ripple = 1

                            if Bond.first_wave:

                                if not noBond.first_wave:
                                    Bond.first_wave = False
                                    Bond.prev_time = ttime

                            else:

                                # Peak-to-peak

                                Bond.update_pp()

                                if len(Bond.pp) >= 2:

                                    Bond.pp_avg, Bond.pp_stdev = Bond.compute_stats(Bond.pp)

                                # Frequency

                                if Bond.wave_counter % 1 == 0.5:

                                    Bond.update_freq(ttime)

                                    Bond.prev_time = ttime

                                else:

                                    time_diff = ttime - noBond.prev_time

                                    Bond.sign_cross = Bond_height

                                if len(Bond.freq) >= 2:

                                    Bond.freq_avg, Bond.freq_stdev = Bond.compute_stats(Bond.freq)

                                Bond.wave_counter += 0.5


                            Bond.half_period = []

                    Bond.half_period.append(Bond_height)

                    Bond.rolling_array.pop(0)

                    # ============================== STORE DATA ==============================

                    Bond.measurements.append(Bond_height)

                    noBond.measurements.append(noBond_height)

                    time_csv.append(ttime)

                    GUI.update_wave_graphs(
                        t=ttime,
                        bond_height=Bond.measurements[-1],
                        nobond_height=noBond.measurements[-1]
                    )

                    GUI.update_experiment_values(
                        humidity=Humidity_value,
                        ambient_temp=AmbTemp_value,
                        water_temp=WaterTemp_value,
                        motor_temp=MotorTemp_value,
                        rpm=AngularVelocity_value,

                        bond_pp_avg=Bond.pp_avg,
                        bond_pp_stdev=Bond.pp_stdev,

                        nobond_pp_avg=noBond.pp_avg,
                        nobond_pp_stdev=noBond.pp_stdev,

                        bond_freq_avg=Bond.freq_avg,
                        bond_freq_stdev=Bond.freq_stdev,

                        nobond_freq_avg=noBond.freq_avg,
                        nobond_freq_stdev=noBond.freq_stdev,

                        wavelength_avg=Bond.wavelength_avg,
                        wavelength_stdev=Bond.wavelength_stdev,

                        wave_count=Bond.wave_counter
                    )

                    

            # elif line.find('Zero levels') != -1:
            elif data[0] == "Zero levels":

                data_zero = line.split(',')

                noBond_zero_lvl = float(data_zero[1])
                Bond_zero_lvl = float(data_zero[2])

                print(line)

                winsound.Beep(350, 500)

            else: # Put coms error code here for unexpected line
                print(line)

            # # Update GUI after running full logic sequence
            # GUI.after(1,Data_and_window_processing)  # Keep the GUI loop going

        # Stop motor if wave counter is over limit
        else:
            # print('Entering else condition')
            if Bond.wave_counter >= GUI.experiment_wave_limit:
                VFD_data_queue.put(['stop'])
                print('Stopping motor')

                state = 'IDLE'

                # GUI.quit()
                return
            # else: # Still missing waves but queue is empty
            #     GUI.after(10,Data_and_window_processing)  # Keep the GUI loop going
            

    except KeyboardInterrupt:

        print("Deteniendo la lectura de datos.")


def Wait_for_start():

    global csv_filename
    global csv_file
    global writer
    global state

    # Stay idle until the user has entered valid parameters
    if not GUI.params_ready or not GUI.start_requested:
        return

    # Stay idle until the user presses the start button
    if not GUI.start_requested:
        return


    # Transfer GUI values into control-side variables
    motor_freq = GUI.VFD_frequency
    crank_pos = GUI.crank_length
    max_waves = GUI.experiment_wave_limit
    ard_COM_port = GUI.ARD_port
    # ard_COM_port = "COM6"
    VFD_COM_port = GUI.VFD_port
    # VFD_COM_port = "COM8"

    print("Experiment starting with:")
    print("Motor frequency:", motor_freq)
    print("Crank position:", crank_pos)
    print("Wave limit:", max_waves)

    # Build CSV file name using GUI inputs
    csv_filename = os.path.join(
        csv_path,
        f"{motor_freq:g} Hz - {crank_pos:g} mm.csv"
    )

    # Open CSV file only after the user starts the experiment
    csv_file = open(csv_filename, mode="w", newline="")
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

    # Start serial coms
    serial_comms.set_ports(ard_COM_port,VFD_COM_port)
    try:
        serial_comms.connect()
    except Exception as error:
        print("Error connecting serial devices:", error)
        GUI.start_requested = False
        GUI.start_button.setEnabled(True)
        state = "IDLE"
        return

    # Start the serial reading thread

    threading.Thread(target=Serial_coms_thread, daemon=True).start()

    # Send selected frequency to VFD
    VFD_data_queue.put(["set_freq", int(motor_freq)])

    # Now leave idle mode and enter the control/data-processing stage
    # GUI.show_status("Starting preliminary readings.")
    state = "PRELIMINARY"


def preliminary_state():
    global state

    if ard_data_queue.empty():
        return

    line = ard_data_queue.get()
    data = line.split(",")

    if data[0] == "Zero levels":
        # Do nothing because we dont have anything used for zero levels
        print(line)
        # GUI.show_status("Zero levels received.")

    elif data[0] == "Zeros ready":
        # GUI.show_status("Zeros ready. Starting motors.")
        VFD_data_queue.put(['start'])

        time.sleep(3)
        GUI.open_crests_dialog() # Calls for GUI to display crests input

        state = "WAITING_FOR_CRESTS"

    # elif data[0] == "Zeros ready":
    #     GUI.show_status("Zeros ready. Starting motors.")
    #     VFD_data_queue.put(['start'])
    #     state = "WAITING_FOR_CRESTS"
    
    else:
        # GUI.show_alarm(f"Unexpected serial command: {line}")
        state = "ERROR"
    

def wait_for_crests():

    global state

    if not GUI.crests_ready:
        return
    
    # Start automatic arduino meassurements process
    data = "Crests_ready\r\n"
    serial_comms.ser.write(data.encode())

    Bond.crests = GUI.crests_between_sensors
    noBond.crests = GUI.crests_between_sensors

    state = "READING_WAVES"
    

def control_loop():
    global state

    if state == "IDLE":
        Wait_for_start()

    elif state == "PRELIMINARY":
        preliminary_state()

    elif state == "WAITING_FOR_CRESTS":
        wait_for_crests()

    elif state == "READING_WAVES":
        Data_and_window_processing()

    # elif state == "FINISHED":
    #     finished_state()

    # elif state == "ERROR":
    #     error_state()

    elif state == "STOP":
        print('Stoppa')

# Create sensor objects

Bond = Sensor('Bond')
Bond.anti_ripple = 7
Bond.sensor_dist = 2.22

noBond = Sensor('noBond')
noBond.anti_ripple = 7

# Create coms objects
serial_comms = Comms()


# Create window object

GUI_app = QApplication(sys.argv)

GUI = MainWindow()
GUI.showMaximized()



timer = QTimer()
timer.timeout.connect(control_loop)
timer.start(200)



GUI_app.exec()


# Save results to csv file
if (input('\nSave data? (y/n): ') == 'y'):
    results_file = open(csv_path + 'Results.csv', mode='a')
    # Name of the test
    results_file.write('\n' + GUI.VFD_frequency + ',' + GUI.crank_length + ',')
    # Add the results
    results_file.write(str(noBond.pp_avg) + ',' + str(noBond.pp_stdev) + ',')
    results_file.write(str(Bond.pp_avg) + ',' + str(Bond.pp_stdev) + ',')
    results_file.write(str(noBond.freq_avg) + ',' + str(noBond.freq_stdev) + ',')
    results_file.write(str(Bond.freq_avg) + ',' + str(Bond.freq_stdev) + ',')
    results_file.write(str(Bond.wavelength_avg) + ',' + str(Bond.wavelength_stdev))
    results_file.close()