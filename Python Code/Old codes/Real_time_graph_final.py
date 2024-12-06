import time
import serial
import time
import csv
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from matplotlib import style
import numpy as np
import argparse
import os
import statistics
import math
from PIL import Image, ImageTk  # Import Pillow for image handling
import tkinter as tk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg


port = 'COM9'  # COM9 para Andrés / COM10 para Daniel
baudrate = 115200

max_measurements = 100
graph_max = 20
time_csv = []
Bond_measurements = []
NotBond_measurements = []
AmbTemp_value = 0
WaterTemp_value = 0
Humidity_value = 0
MotorTemp_value = 0
AngularVelocity_value = 0


# Wave variables
max_waves = 1000
graph_max_waves = 4
wave_counter = 0

# Connect to serial port
try:
    ser = serial.Serial(port, baudrate)
    print(f"Conectado al puerto {port} a {baudrate} baudios.")
except serial.SerialException as e:
    print(f"No se pudo abrir el puerto {port}: {e}")
    exit()

# Nombre del archivo CSV
crank_pos = str(input('Crank Position: '))
motor_freq = input('Motor Frequency (Hz): ')


print('\nReady to start measurements!')

#csv_path = r'C:\Users\Daniel Q\Documents\TEC\2024 - II Semestre\eWave\eWave\Datasets\\' # Para Daniel
csv_path = r'C:\Users\Lenovo\Documents\eWave\eWave\Datasets\\' # Para Andrés

csv_filename = csv_path + crank_pos + str(motor_freq) + '.csv'

# Espera unos segundos para asegurarse de que la conexión esté establecida
time.sleep(2)






# Interface
window = tk.Tk() # The main Tkinter window
window.title('Plotting in Tkinter')
window.configure(bg="#80e0a7")

# Computer screen width and height
screen_width = window.winfo_screenwidth()
screen_height = window.winfo_screenheight()

window.geometry(f"{screen_width}x{screen_height}")

image = Image.open("images.png")
photo = ImageTk.PhotoImage(image)

image_label = tk.Label(window, image = photo, bg="#80e0a7")
image_label.place(relx = 0.983, rely = 0.92, anchor = 'se', x=-20, y=-20)



plt.ion() # turning interactive mode on


# Create graph and axis Bond
Bond_graph, Bond_axis = plt.subplots(figsize = (11, 4.1))
Bond_axis.set_ylabel('Amplitud (mm)')
Bond_axis.set_title("Mediciones de amplitud de la onda", fontsize = 16, fontweight = 'bold')
plt.close(Bond_graph) # Close the base graph
Bond_line, = Bond_axis.plot([],[]) # Graph line

# Insert the graph into tkinter window
Bond_canvas = FigureCanvasTkAgg(Bond_graph, master = window)
Bond_canvas.get_tk_widget().pack()

# Place the graph
Bond_canvas.get_tk_widget().place(x=20, y=20)

# Create graph and axis Not Bond
NotBond_graph, NotBond_axis = plt.subplots(figsize = (11, 4.1))
NotBond_axis.set_xlabel('Time (s)')
NotBond_axis.set_ylabel('Amplitud (mm)')
plt.close(NotBond_graph)
NotBond_line, = NotBond_axis.plot([],[])

# Insert the graph into tkinter window
NotBond_canvas = FigureCanvasTkAgg(NotBond_graph, master = window)
NotBond_canvas.get_tk_widget().pack()

# Place the graph
NotBond_canvas.get_tk_widget().place(x=20, y=429)




def update_graphs():
    global time_csv, Bond_measurements, Bond_line, NotBond_measurements, NotBond_line, WaterTemp_value
    #img = ax1.imshow(X, vmin=-1, vmax=1, interpolation="None", cmap="RdBu")

    with open(csv_filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        # Escribe los encabezados del archivo CSV
        writer.writerow(["Time (s)", "Accel_x (m/s2)", "Accel_y (m/s2)", "Accel_z (m/s2)", "RPM", "Humidity (percentage)", "Amb_Temp (C)", "Water_Temp (C)", "Motor_Temp (C)", "Height 1 (mm)", "Height 2 (mm)"])
        i=0
        try:
            while i < max_waves:
                if ser.in_waiting > 0:
                    # Read serial port string
                    line = ser.readline().decode('utf-8').strip()
                    # Split the whole serial string into values
                    data = line.split(',')
                    if len(data) == 11:
                        # Write data on csv file
                        writer.writerow(data)

                        # Update tkinter window
                        Bond_measurements.append(float(data[10]))
                        NotBond_measurements.append(float(data[9]))
                        time_csv.append(float(data[0]))

                        AmbTemp_value = (float(data[6]))
                        AmbTemp_valuetext.config(text = f"Temperatura del ambiente (°C): {AmbTemp_value:.2f}")
                        
                        WaterTemp_value = (float(data[7]))
                        WaterTemp_valuetext.config(text = f"Temperatura del agua (°C): {WaterTemp_value:.2f}")

                        Humidity_value = (float(data[5]))
                        Humidity_valuetext.config(text = f"Humedad (%): {Humidity_value:.2f}")

                        MotorTemp_value = (float(data[8]))
                        MotorTemp_valuetext.config(text = f"Temperatura del motor (°C): {MotorTemp_value:.2f}")

                        AngularVelocity_value = (float(data[4]))
                        AngularVelocity_valuetext.config(text = f"Velocidad angular (rpm): {AngularVelocity_value:.2f}")

                        
                       
                        
                        if i%3 == 0:
                            Bond_line.set_xdata(time_csv)
                            Bond_line.set_ydata(Bond_measurements)
                            NotBond_line.set_xdata(time_csv)
                            NotBond_line.set_ydata(NotBond_measurements)

                            Bond_axis.relim()
                            Bond_axis.autoscale_view()
                            Bond_canvas.draw()
                            Bond_canvas.flush_events() # Update data

                            NotBond_axis.relim()
                            NotBond_axis.autoscale_view()
                            NotBond_canvas.draw()
                            NotBond_canvas.flush_events() 

                            # Update pending tasks
                            window.update_idletasks()
                            window.update()


                            if len(time_csv)>=graph_max:
                                Bond_axis.set_xlim(time_csv[-graph_max], time_csv[-1])
                                NotBond_axis.set_xlim(time_csv[-graph_max], time_csv[-1])   


                        i+=1
               
        except KeyboardInterrupt:
            print("Deteniendo la lectura de datos.")
    # Cierra la conexión serie
            ser.close()
    
 #   while a<a_max:
    
        
# Environment condition box
T = tk.Text(window, height = 30, width = 45)
T.pack()
T.place(relx = 0.984, rely = 0.05, anchor = 'ne')


# Environment conditions
title = tk.Label(window, text = "Condiciones Ambientales")
title.config(font =("Courier", 14))
title.pack()
title.place(relx = 0.95, rely = 0.01, anchor = 'ne')

AmbTemp_valuetext = tk.Label(window)
AmbTemp_valuetext.config(font =("Courier", 12))
AmbTemp_valuetext.pack()
AmbTemp_valuetext.place(relx = 0.981, rely = 0.059, anchor = 'ne')

WaterTemp_valuetext = tk.Label(window)
WaterTemp_valuetext.config(font = ("Courier", 12))
WaterTemp_valuetext.pack()
WaterTemp_valuetext.place(relx = 0.971, rely = 0.097, anchor = 'ne')

Humid_valuetext = tk.Label(window, text = "Humedad: 100 %")
Humid_valuetext.config(font =("Courier", 12))
Humid_valuetext.pack()
Humid_valuetext.place(relx = 0.92, rely = 0.135, anchor = 'ne')

MotorTemp_valuetext = tk.Label(window)
MotorTemp_valuetext.config(font =("Courier", 12))
MotorTemp_valuetext.pack()
MotorTemp_valuetext.place(relx = 0.96, rely = 0.173, anchor = 'ne')

AngularVelocity_valuetext = tk.Label(window, text = 'Velocidad angular: ')
AngularVelocity_valuetext.config(font =("Courier", 12))
AngularVelocity_valuetext.pack()
AngularVelocity_valuetext.place(relx = 0.94, rely = 0.211, anchor = 'ne')

# run the gui
window.after(0, update_graphs)
window.mainloop() 
