import RPi.GPIO as GPIO            # import RPi.GPIO module  
from time import sleep             # lets us have a delay
import sys
from sys import stdout
from daqhats import hat_list, HatIDs, mcc118, mcc134, HatError, TcTypes
import pigpio
import PID
import PID2
import PID3
import os.path

GPIO.setmode(GPIO.BOARD)             # choose BCM or BOARD
GPIO.setwarnings(False)

#pins:
Enable = 11
Direction = 15
Step = 13

Servo = 35
pigpioServo = 19 # both same pin, bottom is BCM format and top is Board format

proportionalValve = 31
ExVPosition = 0

#pin setup
GPIO.setup(Enable, GPIO.OUT)
GPIO.setup(Step, GPIO.OUT)
GPIO.setup(Direction, GPIO.OUT)

pi = pigpio.pi()
pi.set_servo_pulsewidth(pigpioServo, 1000)

#PWM setup
GPIO.setup(proportionalValve, GPIO.OUT)
p = GPIO.PWM(proportionalValve, 200)
p.start(0)



###
#PID setup
targetT = 25
P = 5
I = 3
D = 1

pid = PID.PID(P, I, D)
pid.SetPoint = targetT
pid.setSampleTime(0.1)
pid.setWindup(5)

targetF = 0.2
P2 = 50
I2 = 45
D2 = 2

pid2 = PID2.PID2(P2, I2, D2)
pid2.SetPoint = targetF
pid2.setSampleTime(0.1)
pid.setWindup(10)

targetP = 90
P3 = 4
I3 = 3
D3 = 0

pid3 = PID3.PID3(P3, I3, D3)
pid3.SetPoint = targetP
pid3.setSampleTime(0.1)
####





#thermocouple setup
hat = mcc134(1)
tc_type = TcTypes.TYPE_K

hat.tc_type_write(0, tc_type)
hat.tc_type_write(1, tc_type)
hat.tc_type_write(2, tc_type)
hat.tc_type_write(3, tc_type)

hat2 = mcc134(2)
hat2.tc_type_write(0, tc_type)
hat2.tc_type_write(1, tc_type)


#method to read mcc118 voltage
def getVoltage(port):
    totalVoltage = 0
    for x in range(0, 20):
        voltage = mcc118(0).a_in_read(int(port))
        totalVoltage += voltage
        sleep(0.00000000001)
    voltage = totalVoltage / 20
    return voltage

#method to rotate EXV stepper
def stepperRotate(steps):
    GPIO.output(Enable, 0)
    steps = int(steps)
    global stepperCurrentPosition
    if (steps > stepperCurrentPosition):
        GPIO.output(Direction, 0)
        while (steps > stepperCurrentPosition):
            GPIO.output(Step, 1)
            sleep(0.0001)
            GPIO.output(Step, 0)
            sleep(0.0001)
            stepperCurrentPosition += 1
    elif (steps < stepperCurrentPosition):
        GPIO.output(Direction, 1)
        while (steps < stepperCurrentPosition):
            GPIO.output(Step, 1)
            sleep(0.0001)
            GPIO.output(Step, 0)
            sleep(0.0001)
            stepperCurrentPosition -= 1
    GPIO.output(Enable, 1)

#method to set compressor speed, accepts values from 1000 to 1900 (1 to 1.9ms pulses)
def Compressor_Throttle(pulseWidth):
    pi.set_servo_pulsewidth(pigpioServo, pulseWidth)
  
#method to convert flow voltage reading to flow rate
def getFlowRate(port):
    flowVoltage = getVoltage(int(port))
    flow = (0.3125 * flowVoltage) - 0.125
    if (flow < 0):
        flow = 0
    return flow

#method to convert pressure voltage reading to pressure value
def getPressure(port):
    pressureVoltage = getVoltage(int(port))
    pressure = (40.016 * pressureVoltage) + 14.38
    return pressure

#method to set proportional valve position
def propValve(duty):
    p.ChangeDutyCycle(duty)


#method to read temperature from thermocouples
def thermocoupleTemp(channel):
    temp = hat.t_in_read(int(channel))
    return temp

def thermocoupleTemp2(channel):
    temp = hat2.t_in_read(int(channel))
    return temp


#method to read config file to change setpoint, P, I, D values
def readConfig ():
    global targetT
    global targetF
    with open ('/tmp/pid.conf', 'r') as f:
        config = f.readline().split(',')
        pid.SetPoint = (float(config[0]))
        targetT = pid.SetPoint
        pid.setKp (float(config[1]))
        pid.setKi (float(config[2]))
        pid.setKd (float(config[3]))
        
        pid2.SetPoint = float(config[4])
        targetF = pid2.SetPoint
        pid2.setKp (float(config[5]))
        pid2.setKi (float(config[6]))
        pid2.setKd (float(config[7]))
        
        pid3.SetPoint = float(config[8])
        targetP = pid3.SetPoint
        pid3.setKp (float(config[9]))
        pid3.setKi (float(config[10]))
        pid3.setKd (float(config[11]))

#method to create the config file
def createConfig ():
    if not os.path.isfile('/tmp/pid.conf'):
        with open ('/tmp/pid.conf', 'w') as f:
            f.write('%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s'%(targetT, P, I, D, targetF, P2, I2, D2, targetP, P3, I3, D3))
            
createConfig()
    
#runs EXV closed all the way and resets position
GPIO.output(Enable, 0)
GPIO.output(Direction, 1)

#"""
for x in range(0,  25000):
    GPIO.output(Step, 1)
    sleep(0.0001)
    GPIO.output(Step, 0)
    sleep(0.0001)
#"""


stepperCurrentPosition = 0
oldPos = 0

Compressor_Throttle(1000)
servoPosition = 1000

duty = 0
propValve(duty)

samples = 0
#def Labview_Data():
#data = array(data, [flowRate, CompInletP, CompOutletP, CondenserP, ExVP, HighSideRefT, LowSideRefT, CompInletT, ComOutletT, InletWaterT, OutletWaterT])
#return data


#main loop, displays sensor values and positions
while True:
    #display headers
    print('\nSample', end='')
    print('  ExVPos    CompThrottle    WaterValve    RefFlow  InletPress   OutPress   CondPress    ExVPress    CompInT   CompOutT   HighSideT    LowSideT ')

    while True:
        readConfig()
          
        #sensor variables
        flowRate = getFlowRate(7)
        CompInletP = getPressure(6)
        CompOutletP = getPressure(5)
        CondenserP = getPressure(3)
        ExVP = getPressure(2)
        HighSideRefT = thermocoupleTemp(0)
        LowSideRefT = thermocoupleTemp(1)
        CompInletT = thermocoupleTemp(2)
        CompOutletT = thermocoupleTemp(3)
        InletWaterT = thermocoupleTemp2(0)
        OutletWaterT = thermocoupleTemp2(1)
        
        #Calculate Enthalpy removed
        WaterDeltaT = OutletWaterT - InletWaterT

        #Get duty cycle using PID function
        pid.update(HighSideRefT)
        targetPwm = pid.output
        targetPwm = max(min(int(targetPwm), 100), 0)
        targetPwm = abs(100-targetPwm)
            
        # Set PWM expansion channel 0 to the target setting
        propValve(targetPwm)
            
        #Get compressor speed using PID function
        pid3.update(ExVP)
        ThrottlePIDOutput = pid3.output
        ThrottlePIDOutput = max(min(int(ThrottlePIDOutput), 70), 0)
        targetPwm32 = abs(70-ThrottlePIDOutput)
        targetPwm33 = (targetPwm32 * 10) + 1300
        #targetPwm3 = abs(1800-targetPwm3)
            
        Compressor_Throttle(targetPwm33)
        CompressorThrottle = (targetPwm33 - 1000)/10
            
        #Get steps using PID function
        pid2.update(flowRate)
        targetPwm2 = pid2.output
        targetPwm2 = max(min(int(targetPwm2), 100), 0)
        targetPwm2 = targetPwm2 * 200
        #targetPwm2 = abs(20000-targetPwm2)
            
        stepperRotate(targetPwm2)
        stepperCurrentPosition = targetPwm2
        ExVPosition = stepperCurrentPosition/ 20000 * 100
                  
        # Display the updated samples per channel count
        samples += 1
        print('\r{:6d}'.format(samples), end='')

        # Read a single value from each variable and format to fit display
        print('{:12.0f}'.format(ExVPosition), end='')
        print('{:12.0f}'.format(ThrottlePIDOutput), end='')
        print('{:12d}'.format(targetPwm33), end='')
        print('{:12.3f}'.format(flowRate), end='')
        print('{:12.1f}'.format(CompInletP), end='')
        print('{:12.1f}'.format(CompOutletP), end='')
        print('{:12.1f}'.format(CondenserP), end='')
        print('{:12.1f}'.format(ExVP), end='')
        print('{:12.1f}'.format(CompInletT), end='')
        print('{:12.1f}'.format(CompOutletT), end='')
        print('{:12.1f}'.format(HighSideRefT), end='')
        print('{:12.1f}'.format(LowSideRefT), end='')

        stdout.flush()