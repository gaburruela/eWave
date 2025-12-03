import pandas as pd
import numpy as np
import csv

datos = pd.read_csv(r'C:\Users\Daniel Quesada\Documents\GitHub\eWave\Datasets\II Semester 2025\Raw_Data\74 cm altura\Results.csv',
                    names=['freq','brazo',3,4,'bond_pp_avg',6,7,8,9,10,'wavelength',12]
                    )

datos_15hz = datos[datos['freq']==15]



print(type(datos_15hz['brazo'].iloc[2]))

h = 0.74
for i in range(len(datos_15hz)):
    b = float(datos_15hz['brazo'].iloc[i])/1000
    k = 2*np.pi/float(datos['wavelength'].iloc[i])
    S = 1.48*np.tan(np.arcsin(b/1.61))
    termA = 4*(np.sinh(k*h)/(k*h))
    termB = (1 + k*h*np.sinh(k*h) - np.cosh(k*h))/(2*k*h+np.sinh(2*k*h))
    H = S * termA * termB
    #H =0.5*k*h
    print('Brazo: ', b, ' Altura formula: ', H*1000, ' Altura olas: ', datos_15hz['bond_pp_avg'].iloc[i])



#print(datos[1])
