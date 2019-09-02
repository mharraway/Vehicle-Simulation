############################### Imports #######################################
import numpy as np
import scipy as sci
from matplotlib import pyplot as plt
from math import pi,degrees,sin,cos,sqrt,asin

############################ Kown Parameters ##################################

g = 9.81                        # Gravity
Vehicle_Weight = 380 + 80       # Weight + Driver
Tire_Radius = 0.318             # Rear Tire Radius
Cd = 0.825                      # Drag Coefficient
Frontal_Area_Cab = 1.84*1.14    # Frontal Area (Drag Calculations)
rho = 1.225                     # Desnsity
Vmax = 50                       # Max Speed of Cab
Vs = Vmax/3.6                   # Max Speed in m/s (Roaling Resistance Calcs)
effelectrical = 0.8             # Expected Electrical Eff
Battery_Cap = 7.2               # Battery Capacity kWh
Pressure_Tire = 2.2             # Tire Pressure (Bar)
Cab_Acceleration = 0.76         # Real World Tested Acceleration (m/s^2)

############################## Calculations ###################################

############################### Functions #####################################
   
def Acceleration_Distance(Top_End_Speed, Acceleration):
    # Calculate the distance required to get up to cruising speed
    t = Top_End_Speed / Acceleration # Time
    Acceleration_Distance = 0.5 * Acceleration * t**2 # Distance
    return(Acceleration_Distance)
    
    
def Congestion_Converter(Congestion_Factor):
    # Function to determine stop - go frequency (m)
    Congestion = 1000 / Congestion_Factor # Max min and medium
    return(Congestion)
    

def Constant_Speed_Power (Velocity, Weight, Altitude, Distance):
    # Function to determine load under constant speed
    # From test data create polynomial function to determine losses
    No_Load_Data = [0, 200, 400, 600]
    No_Load_Test_Speed = [0,  5.8, 10.3, 14.5]     
    No_Load_Poly = np.polyfit(No_Load_Test_Speed, No_Load_Data, 2)
    No_Load_Speed = np.poly1d(No_Load_Poly)
    No_Load_Losses = No_Load_Speed(Velocity) # Gives the losses as a function of speed (Drivetrain)
    
    theta = asin(Altitude/(Distance*1000)) # Average incline over trip distance (rads)
    
    Crr = 0.005 + (1 / Pressure_Tire)*(0.01 + 0.0095*(Vs / 100)**2) # Roaling resistance Coefficient
        
    FORCE_Rolling_Resistance = Crr*Weight*g # Force contributed by Roaling Resistance
    FORCE_Drag = 0.5*Cd*Frontal_Area_Cab*(rho*(Velocity**2)) # Force contributed by Drag
    FORCE_Incline = Weight*g*sin(theta) # Force contributed by Incline
    # Resultant Force    
    FORCE_Resultant = FORCE_Rolling_Resistance + FORCE_Drag + FORCE_Incline
        
    TORQUE_Wheel = FORCE_Resultant*Tire_Radius # Torque Required
    Radial_Wheel_Speed = Velocity/Tire_Radius # Radial wheel speed (Rad/s)
        
    eff = effelectrical # Electrical contribution to eff
        
    Pout = TORQUE_Wheel*Radial_Wheel_Speed # Power output 
    Pin = (Pout/(eff)) + No_Load_Losses # Required Power Input
    
    return(Pin)
 
    
def Acceleration_Energy_Required (Velocity, Acceleration, WeightA, Altitude, Distance):
    
    # Function to calculate energy required for acceleration
    V_to_sec = Velocity/3.6 # km/h to m/s
    Acceleration_Time = int(np.round(V_to_sec/Acceleration)) # Time to Accelerate rounded to sec
    Top_V = Acceleration_Time*Acceleration # Top speed from rounded time
    Va = np.linspace(Acceleration, Top_V, Acceleration_Time) # Array with speed value at increments of seconds
    # From test data create polynomial function to determine eff at different speeds
    Motor_Eff = [10, 69.8, 75.1, 71.8, 74.9, 72, 78, 78, 78, 78, 78, 78, 78, 78, 78, 78, 60]
    Speed = [0, 9.09, 9.23, 9.364, 9.5, 9.636, 9.7, 10, 10.5, 11, 11.5, 12, 12.5, 13, 13.5, 14, 15]
    Eff_Poly = np.polyfit(Speed, Motor_Eff,2)
    Eff_Eq = np.poly1d(Eff_Poly)

    theta = asin(Altitude/(Distance*1000)) # Average incline over trip distance (rads)
    
    Crr = 0.005 + (1 / Pressure_Tire)*(0.01 + 0.0095*(Va / 100)**2) # Roaling resistance Coefficient
    # Forces acting on cab at each increment of speed    
    FORCE_Rolling_Resistance = Crr*WeightA*g
    FORCE_Drag = 0.5*Cd*Frontal_Area_Cab*(rho*(Va**2))
    FORCE_Incline = WeightA*g*sin(theta)
    FORCE_Acceleration = Cab_Acceleration*WeightA*np.ones(len(Va))

    FORCE_Resultant = FORCE_Rolling_Resistance + FORCE_Drag + FORCE_Incline + FORCE_Acceleration

    TORQUE_Wheel = FORCE_Resultant*Tire_Radius
    Radial_Wheel_Speed = Va/Tire_Radius
       
    Pout = TORQUE_Wheel*Radial_Wheel_Speed
    # Devide Pout array by calculated eff at each increment of speed
    Pin = np.divide( Pout, (Eff_Eq(Va)/100)*effelectrical) 
    Total = np.sum(Pin) # Sum al the Power in array (W -> J/s and array is per sec so for each part of array the power is used over that second)
    Total_kWh = Total/3600000
    
    return(Total_kWh)
    
    
    
#################################### Main #####################################
# Required parameters from user
Speed_Input = float(input(f"Please provide average trip speed [km/h]. (This may not exceed vehicle maximum speed, thus between 0km/h and {Vmax}km/h): "))
Congestion_Input = float(input("Please provide opperating area relative congestion. (1 = Open Roads -> 10 = City Center): "))
Weight_Input = float(input(f"Please provide amount of passengers. (1 - 2 passengers): "))
Trip_Input = float(input(f"Please provide average trip distance [km]: "))
Altitude_Input = float(input(f"Please Provide Expected Altitude Gains [m]: "))

Total_Weight = Vehicle_Weight + Weight_Input*75 # Calculate Total weight

# Power Usage for up down and flat Constant Speed
PowerUp = Constant_Speed_Power((Speed_Input/3.6), Total_Weight, Altitude_Input,Trip_Input)
PowerFlat = Constant_Speed_Power((Speed_Input/3.6), Total_Weight, 0, Trip_Input)
PowerDown = Constant_Speed_Power((Speed_Input/3.6), Total_Weight, -Altitude_Input,Trip_Input)
# Power Usage for up down and flat Acceleration
AccelerationUp = Acceleration_Energy_Required(Speed_Input, Cab_Acceleration, Total_Weight, Altitude_Input,Trip_Input)
AccelerationFlat = Acceleration_Energy_Required(Speed_Input, Cab_Acceleration, Total_Weight, 0,Trip_Input)
AccelerationDown = Acceleration_Energy_Required(Speed_Input, Cab_Acceleration, Total_Weight, -Altitude_Input,Trip_Input)
# Intervals between stop go
Distance_Intervals = Congestion_Converter(Congestion_Input)
# Distance acceleration takes place
Acceleration_Interval = Acceleration_Distance(Speed_Input/3.6,Cab_Acceleration)
# Amount of times stop go
Stop_Go_Frequency = (Trip_Input*1000)/Distance_Intervals
# Distance acceleration takes place
Acceleration_Span = Stop_Go_Frequency * Acceleration_Interval
# Distance traveled at cruising speed
Distance_Delta = Trip_Input*1000 - Acceleration_Span
# Time of trip
Distance_Time = Distance_Delta/(Speed_Input/3.6)
# Cruising speed energy
Distance_Energy_Up = PowerUp*0.5*Distance_Time
Distance_Energy_Down = PowerDown*0.5*Distance_Time
Distance_Energy = Distance_Energy_Up + Distance_Energy_Down
# Acceleration energy
Acceleration_Energy = 0.5*Stop_Go_Frequency*AccelerationUp + 0.5*Stop_Go_Frequency*AccelerationDown
# Total energy for the trip
Trip_Energy_kWh = (Distance_Energy / 3600000) + Acceleration_Energy
# Range is how many times the battery can complete the simulated trip times the trip distance.
Range = (Battery_Cap/Trip_Energy_kWh)*Trip_Input
print(Range)
