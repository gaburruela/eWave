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
        self.time_diff = 0 # Time difference for wavelength calculation
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
        percentage = wavelength_companion_sensor.time_diff/period + (self.crests - 1)

        # Make sure phase is in sync - if not take away half a period
        if wavelength_companion_sensor.sign_cross * self.sign_cross < 0:
            percentage -= 0.5

        if abs(percentage) > 0.001:  # Check wether this makes any sense!!!!!!!!!!
            self.wavelength.append(self.sensor_dist/percentage)


    def compute_stats(self, values):

        avg = statistics.mean(values)
        stdev = statistics.stdev(values)

        return avg, stdev


    def reset_params(self):
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
        self.time_diff = 0 # Time difference for wavelength calculation
        self.wavelength_avg = 0  # Wavelength average from historic data
        self.wavelength_stdev = 0  # Wavelength standard deviation from historic data

        self.crests = 0

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
        # Connect VFD
        self.client = ModbusSerialClient(
            port=self.VFD_port,
            baudrate=self.VFD_baudrate,
            parity="N",
            stopbits=1,
            bytesize=8,
            timeout=1
        )

        if not self.client.connect():
            raise RuntimeError(f"Failed to connect VFD on {self.VFD_port}")

        # Connect Arduino
        self.ser = serial.Serial(
            self.ard_port,
            self.ard_baudrate,
            timeout=0.02
        )

        if not self.ser.is_open:
            raise RuntimeError(f"Failed to connect Arduino on {self.ard_port}")

        time.sleep(2)

        self.clear_buffers()

    def clear_buffers(self):
        # Clear Arduino buffers
        if self.ser is not None and self.ser.is_open:
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()

        # Clear VFD Modbus buffers
        if self.client is not None and self.client.connected:
            if self.client.socket is not None:
                self.client.socket.reset_input_buffer()
                self.client.socket.reset_output_buffer()

    def disconnect(self):
        # Close Arduino serial
        if self.ser is not None:
            if self.ser.is_open:
                self.ser.close()
            self.ser = None

        # Close VFD Modbus connection
        if self.client is not None:
            self.client.close()
            self.client = None

    def restart(self):
        self.disconnect()
        time.sleep(0.5)
        self.connect()
        self.clear_buffers()


# GENERAL VARIABLES

# Graph variables
time_csv = []
time_start = 0


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

serial_thread = None
stop_serial_thread = threading.Event()

# State machine control
state = 'IDLE'


def Serial_coms_thread(stop_event):
    try:
        print('Clearing serial buffers')

        if serial_comms.ser is None or not serial_comms.ser.is_open:
            print("Arduino serial port is not open")
            return

        serial_comms.ser.reset_input_buffer()
        serial_comms.ser.reset_output_buffer()

        # Wait for Arduino boot, but allow thread to stop cleanly
        if stop_event.wait(2):
            print("Serial thread stopped before Arduino start")
            return

        if serial_comms.ser is None or not serial_comms.ser.is_open:
            print("Arduino serial port closed before start command")
            return

        # Start automatic Arduino measurements process
        data = "Start\r\n"
        serial_comms.ser.write(data.encode())

        while not stop_event.is_set():

            if serial_comms.ser is None or not serial_comms.ser.is_open:
                print("Arduino serial port closed")
                break

            if serial_comms.ser.in_waiting > 0:
                data = serial_comms.ser.readline().decode(
                    'utf-8',
                    errors='ignore'
                ).strip()

                ard_data_queue.put(data)
                # print("Data: ", data)

            if not VFD_data_queue.empty():
                cmd = VFD_data_queue.get()
                print('Command: ', cmd)

                if serial_comms.client is None or not serial_comms.client.connected:
                    print("VFD client is not connected")
                    continue

                if cmd[0] == 'set_freq':
                    freq = int(cmd[1] * 100)
                    serial_comms.client.write_register(
                        0x0002,
                        freq,
                        device_id=1,
                        no_response_expected=True
                    )
                    print('Frequency set')

                elif cmd[0] == 'start':
                    serial_comms.client.write_register(
                        0x0001,
                        1,
                        device_id=1,
                        no_response_expected=True
                    )
                    print('Start drive')

                elif cmd[0] == 'stop':
                    serial_comms.client.write_register(
                        0x0001,
                        0,
                        device_id=1,
                        no_response_expected=True
                    )
                    print('Stop drive')

                else:
                    print('Codigo de VFD no soportado')

                # Wait, but allow thread stop during the wait
                stop_event.wait(2)

            else:
                stop_event.wait(0.01)

    except Exception as error:
        if not stop_event.is_set():
            print("Serial thread error:", error)

    print("Serial thread stopped")

def stop_serial_thread_safely():
    global serial_thread
    global stop_serial_thread

    stop_serial_thread.set()

    if serial_thread is not None and serial_thread.is_alive():
        serial_thread.join(timeout=3)

    serial_thread = None

# WAVE LOGIC CONTROL FUNCTION

def Data_and_window_processing():

    global time_csv
    global time_start
    global time_start_flag
    global time_diff
    global crest_flag
    global writer
    global state

    if GUI.stop_requested:
        state = "STOP"
        return

    try:
        if Bond.wave_counter < GUI.experiment_wave_limit and not ard_data_queue.empty():

            # Get data from ard thread queue
            line = ard_data_queue.get()
            # print('Line received inside the GUI logic', line)
            data = line.split(',')

            # if line.find('Wave data') != -1:
            if data[0] == "Wave data":

                writer.writerow(data[1:])

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

                        if (Bond.half_period[-1] * Bond_height < 0
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

                                    # Saves time diff for wavelength in both sensors since its shared
                                    noBond.time_diff = ttime - noBond.prev_time
                                    Bond.time_diff = ttime - noBond.prev_time

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


            else: # Put coms error code here for unexpected line

                state = 'ERROR'
                VFD_data_queue.put(['stop']) # Stop motor on error
                GUI.set_backend_status('Dato inesperado del arduino')

                return

        # Stop motor if wave counter is over limit
        else:
            # print('Entering else condition')
            if Bond.wave_counter >= GUI.experiment_wave_limit:
                VFD_data_queue.put(['stop'])
                print('Stopping motor')

                state = 'FINISHED'  # Switch to finished state

                # GUI.quit()
                return
            # else: # Still missing waves but queue is empty
            #     GUI.after(10,Data_and_window_processing)  # Keep the GUI loop going
            

    except KeyboardInterrupt:

        print("Deteniendo la lectura de datos.")

experiment_restart_requested = False

def Wait_for_start():

    global csv_filename
    global csv_file
    global writer
    global state
    global experiment_restart_requested
    global ard_data_queue
    global VFD_data_queue
    global time_csv, time_start, time_start_flag, crest_flag
    global serial_thread
    global stop_serial_thread


    if GUI.stop_requested:
        state = "STOP"
        VFD_data_queue.put(['stop']) # Stop motor on error
        return

    # Stay idle until the user has entered valid parameters
    if (not GUI.experiment_params_ready 
            or not GUI.ard_port_ready
            or not GUI.vfd_port_ready
            or not GUI.results_folder_ready):
        return

    # Stay idle until the user presses the start button
    if not GUI.start_requested:
        return

    if experiment_restart_requested:
        stop_serial_thread_safely()
        stop_serial_thread.clear()

        Bond.reset_params()
        noBond.reset_params()

        ard_data_queue = queue.Queue()
        VFD_data_queue = queue.Queue()

        time_csv = []
        time_start = 0
        time_start_flag = True
        crest_flag = True

        GUI.crests_ready = False

        # Reset graficas de la pantalla

        GUI.update_wave_graphs(
            t=0,
            bond_height=0,
            nobond_height=0
        )

        GUI.update_experiment_values(
            humidity=0,
            ambient_temp=0,
            water_temp=0,
            motor_temp=0,
            rpm=0,

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


    # Transfer GUI values into control-side variables
    motor_freq = GUI.VFD_frequency
    crank_pos = GUI.crank_length
    max_waves = GUI.experiment_wave_limit
    Bond.sensor_dist = GUI.sensor_distance
    noBond.sensor_dist = GUI.sensor_distance

    ard_COM_port = GUI.ARD_port
    VFD_COM_port = GUI.VFD_port

    csv_path = GUI.results_folder

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
    
    if not experiment_restart_requested:
        try:
            serial_comms.connect()
        except Exception as error:
            # print("Error connecting serial devices:", error)
            GUI.set_backend_status(f'Fallo en puertos COM {error}')
            GUI.start_requested = False
            GUI.start_button.setEnabled(False)
            state = "IDLE"
            return

    else:
        try:
            serial_comms.restart() 
        except Exception as error:
            # print("Error connecting serial devices:", error)
            GUI.set_backend_status(f'Fallo en puertos COM {error}')
            GUI.start_requested = False
            GUI.start_button.setEnabled(False)
            state = "IDLE"
            return

    # Start the serial reading thread

    stop_serial_thread.clear()

    serial_thread = threading.Thread(
        target=Serial_coms_thread,
        args=(stop_serial_thread,),
        daemon=True
    )

    serial_thread.start()

    experiment_restart_requested = False

    # Send selected frequency to VFD
    VFD_data_queue.put(["set_freq", motor_freq])

    # Now leave idle mode and enter the control/data-processing stage
    # GUI.show_status("Starting preliminary readings.")
    state = "PRELIMINARY"


def preliminary_state():
    global state

    if GUI.stop_requested:
        state = "STOP"
        VFD_data_queue.put(['stop']) # Stop motor on error
        return

    if ard_data_queue.empty():
        return

    line = ard_data_queue.get()
    data = line.split(",")

    if data[0] == "Zero levels":
        # Do nothing because we dont have anything used for zero levels
        # print(line)
        GUI.set_backend_status(f'Niveles cero: {data[1]}, {data[2]}')

    elif data[0] == "Zeros ready":
        VFD_data_queue.put(['start'])
        GUI.set_backend_status('Ceros listos')


    elif data[0] == "Average angular velocity: ":

        GUI.set_backend_status('Midiendo RPM y condiciones ambientales')


    elif data[0] == "Ambient temperature: ":

        GUI.set_backend_status('')
        GUI.open_crests_dialog() # Calls for GUI to display crests input

        state = "WAITING_FOR_CRESTS"

        
    elif data[0] == "ERROR":

        GUI.set_backend_status(data[1])

        state = "ERROR"

    
    # else:
    #     # GUI.show_alarm(f"Unexpected serial command: {line}")
    #     state = "ERROR"
    #     VFD_data_queue.put(['stop']) # Stop motor on error
    #     GUI.current_state = 'Comunicación erronea del arduino'

    #     return
    

def wait_for_crests():

    global state

    if GUI.stop_requested:
        state = "STOP"
        VFD_data_queue.put(['stop']) # Stop motor on error
        return

    if not GUI.crests_ready:
        return
    
    # Start automatic arduino meassurements process
    data = "Crests_ready\r\n"
    serial_comms.ser.write(data.encode())

    Bond.crests = GUI.crests_between_sensors
    noBond.crests = GUI.crests_between_sensors

    state = "READING_WAVES"
    

show_save_data_screen = True
data_saved = False

def finished_state():

    global state
    global show_save_data_screen
    global csv_file
    global data_saved
    global experiment_restart_requested

    if show_save_data_screen:
        GUI.ask_save_data()
        show_save_data_screen = False

    if GUI.save_data:
        save_results()
        GUI.save_data = False
        
        try:
            csv_file.close()
        except:
            pass
        
        data_saved = True

        return

    if GUI.stop_requested or data_saved:
    
        try:
            csv_file.close()
        except:
            pass

        state = "IDLE"
        experiment_restart_requested = True
        show_save_data_screen = True
        data_saved = False

        GUI.start_requested = False
        GUI.experiment_params_ready = False
        GUI.stop_requested = False
        GUI.save_data = False
        GUI.crests_ready = False

        stop_serial_thread_safely()

        return


def error_state():
    global state
    global experiment_restart_requested
    global csv_file
    global saved_data
    global show_save_data_screen

    if GUI.stop_requested:
        VFD_data_queue.put(['stop'])

        stop_serial_thread_safely()

        try:
            csv_file.close()
        except:
            pass

        state = "IDLE"
        GUI.start_requested = False
        GUI.experiment_params_ready = False
        GUI.stop_requested = False
        GUI.save_data = False
        GUI.crests_ready = False

        saved_data = False
        show_save_data_screen = True
        experiment_restart_requested = True

        return


def save_results():
    # Clean and normalize folder path from GUI
    results_folder = GUI.results_folder.strip().strip("'").strip('"')
    results_folder = os.path.normpath(results_folder)

    # Create folder if it does not exist
    os.makedirs(results_folder, exist_ok=True)

    # Full results file path
    results_path = os.path.join(results_folder, "Results.csv")

    # Check if file does not exist or is empty
    write_headers = (
        not os.path.isfile(results_path)
        or os.path.getsize(results_path) == 0
    )

    headers = [
        "VFD Frequency [Hz]",
        "Crank Length [mm]",
        "noBond PP Avg [mm]",
        "noBond PP Stdev [mm]",
        "Bond PP Avg [mm]",
        "Bond PP Stdev [mm]",
        "noBond Frequency Avg [Hz]",
        "noBond Frequency Stdev [Hz]",
        "Bond Frequency Avg [Hz]",
        "Bond Frequency Stdev [Hz]",
        "Wavelength Avg [m]",
        "Wavelength Stdev [m]"
    ]

    row = [
        GUI.VFD_frequency,
        GUI.crank_length,
        noBond.pp_avg,
        noBond.pp_stdev,
        Bond.pp_avg,
        Bond.pp_stdev,
        noBond.freq_avg,
        noBond.freq_stdev,
        Bond.freq_avg,
        Bond.freq_stdev,
        Bond.wavelength_avg,
        Bond.wavelength_stdev
    ]

    with open(results_path, mode="a", newline="") as results_file:
        writer = csv.writer(results_file)

        if write_headers:
            writer.writerow(headers)

        writer.writerow(row)


def control_loop():
    global state

    if state == "IDLE":
        GUI.set_backend_status('Configurando experimento')
        Wait_for_start()

    elif state == "PRELIMINARY":
        preliminary_state()

    elif state == "WAITING_FOR_CRESTS":
        GUI.set_backend_status('Esperando crestas')
        wait_for_crests()

    elif state == "READING_WAVES":
        GUI.set_backend_status('Leyendo altura olas')
        Data_and_window_processing()

    elif state == "FINISHED":
        GUI.set_backend_status('Experimento finalizado')
        finished_state()

    elif state == "ERROR":
        error_state()

    elif state == "STOP":
        GUI.set_backend_status('STOP solicitado')
        error_state()
        # print('Stoppa')

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
timer.start(20)



GUI_app.exec()

