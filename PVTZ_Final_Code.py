"""
This program is meant to control the various functions of our final microfluidic pump circuit system.
    The processes controlled in this code include:
    
    ---Control of the liquid flow using a Bartels MP6 Microfluidic Pump.
    ---Measurement of liquid temperature using a Resistive Temperature Detector (RTD
    ---Calibration of temperature using a Peltier thermoelectric and a fan

    
    
ESP32 PIN Choices
2: H-Bridge Control 'Y' for Heating
4 = Heating element ON/OFF
5 = Fan on/off
12 = H_Bridge current sensor ADC
13 = Prssure Sensor ADC
14 = Wheatstone 1 ADC
15 = H Bridge Control 'X' for Cooling

18 = LED RED
19 = LED Green
21 = LED Blue
22 = MP6 driver shutdown
23 = MP6 Driver Clock
25 = MP6 Driver Amplitude
27 = Wheatstone 2 ADC
33 = Wheatstone 3 ADC
34 = Peristaltic Pump Control
"""

#----------------------------------Imports----------------------------------
from machine import ADC,Pin,PWM, Timer
from time import sleep
from utime import sleep_ms
import time

import math
import esp
import sys
import _thread
from pid_controller import pid


#----------------------------------General Initialization----------------------------------


#Initialize frequencies for the peltier module and Bartels Microfluidic Pump.
f_peltier = 500 #Go to about 500 Hz
f_pump = 200 #Bartels pump that is around 200 Hz is pretty sufficient for liquid



#Initialize RTD ADC's
RTD1 = ADC(Pin(14));
RTD1.atten(ADC.ATTN_2_5DB) 
#RTD1.width(ADC.WIDTH_12BIT) #not entirely necessary



"""
These are potential extra RTD's that we are not making use of
RTD2 = ADC(Pin(27));
RTD2.atten(ADC.ATTN_11DB)
"""
#ground pin RTD
RTD3 = ADC(Pin(33));
RTD3.atten(ADC.ATTN_2_5DB)

#Shutdown for bartels pump driver which we wish to drive low before pumping occurs to prevent it from turning on
Shutdown = Pin(22,Pin.OUT)
Shutdown.off()

#controls AMP pin of bartels (keeping it floating ensures functionality each program)
#Amp = Pin(25,Pin.OUT)

#Turn the X and Y PWM Pins OFF to ensure no voltage is sent to thermoelectric
X = Pin(15,Pin.OUT,Pin.PULL_DOWN);
X.off()

Y = Pin(2,Pin.OUT)
Y.off()


#GPIO Fan Control
Fan = Pin(5,Pin.OUT);
Fan.off() #use .on and .off for control

#LED Pins for color control of LED These are essentially extra GPIO Pins
RED = Pin(18,Pin.OUT);
BLUE = Pin(21,Pin.OUT);
GREEN = Pin(19,Pin.OUT);



#----------------------------------Reading The PT100 Resistance and Temperature Code----------------------------------

#Definition to calculate the resistance of the PT100 using our wheatstone bridge circuit.
def PT100_Resistance(V_Diff,Rk,V_Init):
    #Pt100 Equation
    R_pt100 = ((Rk*Rk)+ Rk*(2*Rk)*(V_Diff/V_Init))/(Rk-(2*Rk)*(V_Diff/V_Init))
      
    return R_pt100

#Definition to calculate the temperature of the PT100 using resistance equation is from
#https://rdccontrol.com/thermocouples/rtds-101/temperature-calculations/
def PT100_Temp(Pt_Resistance):
    #a_coeff = (138.51 - 105)/(105*100)
    #Coefficients taken from standard PT100 derivation
    A= 3.9083*pow(10,-3)
    B = -5.77*pow(10,-7)
    R0 = 105 #This value may be slightly off depending on temperature calibration of your PT100
    #In this case R0 was calculated to be around 105 Ohms
    PT_Temp = (-A+math.sqrt(pow(A,2)-4*B*(1-Pt_Resistance/R0)))/(2*B)
    return PT_Temp


#Definition to acquire current temperature of PT100
def Get_Temp(RTD1):
    #Read Analog value from ADC necessary for determining RTD Resistance and temperature later on
    Temp1 = RTD1.read_uv()
    #Not Necessary
    GND = RTD3.read_uv()
    #print("GROUND VOLTAGE", GND/1000000)
    
    #Initialize a list to store adc samples so that we can average them
    TempOut1 = []
    
    #This loop is used to control the sampling rate of the ADC in order to account for noise.
    #The ADC is still pretty shaky
    for i in range(1,4):
        TempOut1.insert(i,Temp1)
        sleep_ms(50) 
    
    #Take mean of voltage reading from RTD sensor and add 0.004 to it
    V_Diff = (sum(TempOut1)/len(TempOut1))/1000000+0.004 #Voltage difference.
    print("Voltage",V_Diff)

    V_Init = 3.3 #3.3V is put into the PT100 Circuit
    Rk = 150 #150 Ohm resistors are put in all known Wheatstone Bridge resistors.
    
    #Calculate Resistance
    PT_Resistance = PT100_Resistance(-V_Diff,Rk,V_Init)
    #print (PT_Resistance)
    #Calculate Temperature
    PT_Temperature = PT100_Temp(PT_Resistance)
    #print("Resistance",PT_Resistance)
    #print("Temperature",PT_Temperature)
    return PT_Temperature

#Array to store data into csv file
#Iterable to check if goal has been reached at least 5 times (helps account for inconsistent ADC readings
goal_count = 0
Temp_List = []
time_list = []
total_time = 0
#Interrupt to capture data
def capture_data(timer):
    PT100_Current_Temp = Get_Temp(RTD1)
    Temp_List.append(PT100_Current_Temp)
    time_list.append(total_time)
    



#----------------------------------User Input----------------------------------

#Prompt user input
print("What temperature do you want to heat your solution to? (Enter between 4 and 40 Degrees C)")

goal_temp = sys.stdin.readline() #Take in goal temperature

#Ensure that user has input the appropriate temperature.
if(goal_temp.isdigit()==1):
    print("Invalid temperature! Please restart and enter an integer value")
    sys.exit()
    
#Check if temperature range is feasible.
elif (int(goal_temp) <= 4) or (int(goal_temp) >= 40):
    print("Invalid temperature! Please restart and put a value between 4C and 40C")
    sys.exit()
    
else:
    #---------------------------------- Bartels Pump Init----------------------------------
    #Begin Pumping by initializing PWM on Pin 18
    Pump_Control = PWM(Pin(23), f_pump, duty = 300) #max duty cycle is 403 for bartels pump
    Shutdown.on() #Keep the shutdown pin high to prevent shutdown from occuring
    #Pump_Control.deinit() #TURNS OFF PWM
    #---------------------------------- PID Init----------------------------------
    goal_temp = int(goal_temp) # Turn Goal temp taken in by user input into an integer
    Kp=1.0 #Proportional
    Ki=0.1 #Integral
    Kd=0.05 #Derivative
    #Initialize PID controller
    pid_sys = pid.PID(Kp,Ki,Kd)
    #Set System Target
    pid_sys.target = goal_temp 
    

    #Initialize Timer For Interrupt
    timer = Timer(-1)
    timer.init(period = 1000,mode = Timer.PERIODIC, callback = capture_data)


    start_time = time.ticks_ms()
    #Data Recording
    #setpoint,Capture_Time,Temperature,Current_Error = [],[],[],[]
    #----------------------------------Main Program Loop----------------------------------
    while True:
        current_time = time.ticks_ms()
        

        #----------------------------------Temperature Reading Main Code----------------------------------        
        #Capture current temperature of liquid
        PT100_Current_Temp = Get_Temp(RTD1)
        
        #Append that temperature to a list each iteration of the loop
        # Show the PID error in the system to see how far it is from the goal temp 
        pid_error = pid_sys.target - PT100_Current_Temp
        print("Error",pid_error)
        
        #Goal Temperature Check
        #Check if PID error is low enough to count as a goal
        if abs(pid_error) <= goal_temp * 0.05:
            #Indicate goal temp is
            print("Goal Temperature In Range")
            #Increment Goal Count
            goal_count += 1
            #If Goal Temperature is reached 12 times then shut down all the pins to record data
            #This is done to capture temperature data without causing large memory issues.
            if goal_count == 20:
                Y_PIN = Pin(2,Pin.OUT)
                Y_PIN.off()
                X_PIN = Pin(15,Pin.OUT)
                X_PIN.off()
                Fan.off()
                filename = "captured_data.csv"
                x = open(filename, "w")
                x.write("Temperature,Time\n")  # Write header
                for Temperature,Time in zip(Temp_List,time_list):
                    x.write(str(Temperature)+","+str(Time)+'\n')
                x.close()
                        
                print("Data written to", filename)
                break
            
        #Show user current temperature
        pid_output = pid_sys(PT100_Current_Temp)
        print("PID Val",pid_output)
        print("Current Temperature is:",PT100_Current_Temp,"Degrees Celsius")
        
        #----------------------------------Thermoelectric PWM Initialization----------------------------------
        #PWM_Y is for Heating
        #If PID error is between a value of -5 and -2, set duty cycle of PWM slightly lower
        #So that the heating is slower
        if -5 <= pid_error < -2:
            #Turn PWM_X Off
            X_PIN = Pin(15,Pin.OUT)
            X_PIN.off()
            Peltier_PWM_Y = PWM(Pin(2),f_peltier,duty = 700)
            Fan.on()
            timer.init(period=5000,mode=Timer.ONE_SHOT)
            #sleep_ms(10)
            
        #If PID error is less than -5 set duty cycle of PWM higher
        #So that the thermoelectric heats faster.
        elif pid_error < -5:
            X_PIN = Pin(15,Pin.OUT)
            X_PIN.off()
            Peltier_PWM_Y = PWM(Pin(2),f_peltier,duty = 900)
            Fan.on()
            #sleep_ms(10)    
            
            
        #PWM_X is for Cooling
        #If PID error is between 5 and 2, begin cooling the system at a lower duty cycle
        elif 5>= pid_error > 2:
            Y_PIN = Pin(2,Pin.OUT)
            Y_PIN.off()
            Peltier_PWM_X = PWM(Pin(15),f_peltier,duty = 700) # you can set frequency in constructor but it's not necessary.
            Fan.on()
        #If PID error greater than 5 , Cool system faster using larger duty cycle     
        elif pid_error > 5:
            Y_PIN = Pin(2,Pin.OUT)
            Y_PIN.off()
            Peltier_PWM_X = PWM(Pin(15),f_peltier,duty = 900) # you can set frequency in constructor but it's not necessary.
            Fan.on()
            
        total_time = time.ticks_diff(current_time,start_time)
        #Sleep for 1 second to input data
        sleep(0.1)

        