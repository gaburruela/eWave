import pandas
import numpy as np
import matplotlib.pyplot as plt


def goda_suzuki_method(eta1, eta2, dx, dt, depth):
    """
    Implements the Goda and Suzuki method for separating incident and reflected waves.
    
    Parameters:
    - eta1: Time series of wave elevation at the first point (numpy array)
    - eta2: Time series of wave elevation at the second point (numpy array)
    - dx: Distance between the two measurement points (meters)
    - dt: Time step between samples (seconds)
    - depth: Water depth (meters)
    
    Returns:
    - eta_incident: Incident wave time series
    - eta_reflected: Reflected wave time series
    """
    # Constants
    g = 9.81  # gravitational acceleration (m/s^2)
    N = len(eta1)  # Number of data points
    
    # Fourier transform of signals
    freq = np.fft.fftfreq(N, dt)  # Frequency array
    omega = 2 * np.pi * freq      # Angular frequency array
    k = np.sqrt(omega**2 / g)     # Wavenumber array (linear theory)
    
    # Ensure k and omega are positive for valid wave numbers
    k[np.where(omega < 0)] = -k[np.where(omega < 0)]
    
    # Fourier transform of signals
    eta1_fft = np.fft.fft(eta1)
    eta2_fft = np.fft.fft(eta2)
    
    # Transfer function
    H = np.exp(-1j * k * dx)  # Phase difference due to spacing
    
    # Incident and reflected wave components in frequency domain
    eta_incident_fft = (eta1_fft + eta2_fft * H) / (1 + H)
    eta_reflected_fft = (eta1_fft - eta2_fft * H) / (1 - H)
    
    # Transform back to time domain
    eta_incident = np.fft.ifft(eta_incident_fft).real
    eta_reflected = np.fft.ifft(eta_reflected_fft).real
    
    return eta_incident, eta_reflected

# Example usage
if __name__ == "__main__":
    
    csv_path = r'C:\Users\garab\eWave Repo\eWave\Datasets\\' # Para Gabriel

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

    dx = 2.22                      # Distance between measurement points (meters)
    dt = 0.040              # Time step (seconds)
    depth = 0.79                   # Water depth (meters)
    
    # Apply Goda and Suzuki method
    eta_incident, eta_reflected = goda_suzuki_method(y_data1, y_data2, dx, dt, depth)
    
    # Plot results
    plt.figure(figsize=(10, 6))
    plt.plot(x_data, y_data1, label="Measured Eta1")
    plt.plot(x_data, y_data2, label="Measured Eta2")
    plt.plot(x_data, eta_incident, label="Incident Wave", linestyle="--")
    plt.plot(x_data, eta_reflected, label="Reflected Wave", linestyle="--")
    plt.xlabel("Time (s)")
    plt.ylabel("Wave Elevation (m)")
    plt.legend()
    plt.grid()
    plt.title("Goda and Suzuki Wave Separation")
    plt.show()
