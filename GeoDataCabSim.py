import gpxpy
import matplotlib.pyplot as plt
import datetime
from geopy import distance
from geopy.distance import geodesic
from math import sqrt, floor, pi, sin
import numpy as np
import pandas as pd
import plotly.plotly as py
import plotly.graph_objs as go
import haversine
import math as mth
#########################################
g = 9.81
Weight = 380 + 170
Tire_Radius = 0.318
Cd = 0.825
Frontal_Area_Cab = 1.84*1.14
rho = 1.225
Vmax = 50
Vs = Vmax/3.6
effmotor = 0.785
effgearbox = 0.98
effelectrical = 0.9
Battery_Cap = 7.2
Pressure_Tire = 2.2
Average_Trip_Distance = 2
Change_In_Altitude = 20
Cab_Acceleration = 0.76
########################################

gpx_file = open('Test track.gpx', 'r')
gpx = gpxpy.parse(gpx_file)
data = gpx.tracks[0].segments[0].points
start = data[0]
finish = data[-1]

df = pd.DataFrame(columns=['lon', 'lat', 'alt', 'time'])
for point in data:
    df = df.append({'lon': point.longitude, 'lat' : point.latitude, 'alt' : point.elevation, 'time' : point.time}, ignore_index=True)

Time_Seconds = np.linspace(0,len(df['alt'])-1,len(df['alt']))
Altitude = df['alt']
Longatude = df['lon']
Latitude = df['lat']

x = np.zeros(len(df)) 
i = 0

while i < len(df)-1:
    Data_Now = (Longatude[i],Latitude[i])
    Data_Next = (Longatude[i+1],Latitude[i+1])
    x[i] = geodesic(Data_Now, Data_Next).meters
    i += 1
    
Speed = x
Degree = np.zeros(len(df))
n = 0

while n < len(df)-1:
    Ratio = (Altitude[n+1] - Altitude[n]) / x[n]
    Degree[n] = ((mth.asin(Ratio))/((mth.pi)*2))*360
    n += 1


Acceleration = np.zeros(len(df))
k = 0

while k < len(df)-1:
    Difference = Speed[k+1] - Speed[k]
    if Difference < 0:
        Acceleration[k] = 0
    elif Difference > 0.76:
        Acceleration[k] = 0.76
    else:
        Acceleration[k] = Difference
    
    k += 1

    
Crr = 0.005 + (1 / Pressure_Tire)*(0.01 + 0.0095*((Speed*3.6) / 100)**2)
theta = (Degree*pi)/180


    
FORCE_Rolling_Resistance = Crr*Weight*g

FORCE_Drag = 0.5*Cd*Frontal_Area_Cab*(rho*(Speed**2))

FORCE_Incline = Weight*g*theta

FORCE_Acceleration = Weight*Acceleration

FORCE_Resultant = FORCE_Rolling_Resistance + FORCE_Drag + FORCE_Incline + FORCE_Acceleration

TORQUE_Wheel = FORCE_Resultant*Tire_Radius

Radial_Wheel_Speed = Speed/Tire_Radius

eff = effmotor*effgearbox*effelectrical

Pout = TORQUE_Wheel*Radial_Wheel_Speed
Pin = (Pout/(eff))
Energy = np.sum(Pin)

plt.plot(Time_Seconds,Speed)
plt.show()


###############################################################################
















    



        
        
        