import RPi.GPIO as GPIO            # import RPi.GPIO module  
from time import sleep             # lets us have a delay
import sys, time, pigpio, PID, PID2, PID3
from sys import stdout
from daqhats import hat_list, HatIDs, mcc118, mcc134, HatError, TcTypes, mcc152, OptionFlags
import os.path, socket, threading, os, sys, select
import numpy as arr
import struct
import json  



HOST = '169.254.98.13' # The remote host - windows machine running the LabVIEW Server
PORT = 2055 # The same port as used by the server - defined in LabVIEW

buffer_size = 8
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, buffer_size)
s.bind((HOST, PORT))
s.listen(1)

GPIO.setmode(GPIO.BOARD)             # choose BCM or BOARD
GPIO.setwarnings(False)

#pins:
Enable = 7
Direction = 15
Step = 18

Servo = 35
pigpioServo = 19 # both same pin, bottom is BCM format and top is Board format

proportionalValve = 23
ExVPosition = 0

#pin setup
GPIO.setup(Enable, GPIO.OUT)
GPIO.setup(Step, GPIO.OUT)
GPIO.setup(Direction, GPIO.OUT)

Comp = pigpio.pi()
Comp.set_servo_pulsewidth(pigpioServo, 1000)

#Proportional Valve setup
Valve = pigpio.pi()
Valve.set_PWM_frequency(proportionalValve, 50)
Valve.set_PWM_dutycycle(proportionalValve, 0)
Valve.set_PWM_range(proportionalValve, 100)

GPIO.output(Enable, 1)


###

targetT = 25
P = 5
I = 3
D = 1

Valvepid = PID.PID(P, I, D)
Valvepid.SetPoint = targetT
Valvepid.setSampleTime(0.125)
Valvepid.setWindup(5)

targetF = 0.05
P2 = 300
I2 = 45
D2 = 2

RefFlowpid = PID2.PID2(P2, I2, D2)
RefFlowpid.SetPoint = targetF
RefFlowpid.setSampleTime(0.125)
RefFlowpid.setWindup(5)

targetP = 90
P3 = 4
I3 = 3
D3 = 0

LowPressurepid = PID3.PID3(P3, I3, D3)
LowPressurepid.SetPoint = targetP
LowPressurepid.setSampleTime(0.12515620)
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
ValvePercentage = 0
ManualMode = 0
CompressorThrottle = 0
hat4 = mcc152(4)

#method to read mcc118 voltage
def getVoltage(port):
    totalVoltage = 0
    for x in range(0, 10):
        voltage = mcc118(0).a_in_read(int(port))
        totalVoltage += voltage
       #sleep(0.000000000001)
    voltage = totalVoltage / 10
    return voltage

#method to rotate EXV stepper
def MoveExV(steps):
    GPIO.output(Enable, 0)
    steps = int(steps)
    global ExVCurrentPosition
    if (steps > ExVCurrentPosition):
        GPIO.output(Direction, 0)
        while (steps > ExVCurrentPosition):
            GPIO.output(Step, 1)
            sleep(0.00000005)
            GPIO.output(Step, 0)
            sleep(0.000005)
            ExVCurrentPosition += 1
    elif (steps < ExVCurrentPosition):
        GPIO.output(Direction, 1)
        while (steps < ExVCurrentPosition):
            GPIO.output(Step, 1)
            sleep(0.00000005)
            GPIO.output(Step, 0)
            sleep(0.000005)
            ExVCurrentPosition -= 1
    GPIO.output(Enable, 1)

#method to set compressor speed, accepts values from 1000 to 1900 (1 to 1.9ms pulses)
def Compressor_Throttle(pulseWidth):
    Volts = ((pulseWidth - 1000)/1000) * 5
   # hat4.a_out_write(0, Volts, OptionFlags.DEFAULT )
    Comp.set_servo_pulsewidth(pigpioServo, pulseWidth)
  
#method to convert flow voltage reading to flow rate
def getFlowRate(port):
    #global FlowRate
    flowVoltage = getVoltage(int(port))
    flow = (170.95 * flowVoltage) - 113.5
    FlowRate = flow
    flow = flow/1000
    FlowRate = int(FlowRate)
    if (flow < 0):
        flow = 0
    return flow

#method to convert pressure voltage reading to pressure value
def getPressure(port):
    pressureVoltage = getVoltage(int(port))
    return pressureVoltage

#method to set proportional valve position
def propValve(duty):
    Valve.set_PWM_dutycycle(proportionalValve, duty)


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
    global targetP
    with open ('/tmp/pid.conf', 'r') as f:
        config = f.readline().split(',')
        Valvepid.SetPoint = (float(config[0]))
        targetT = Valvepid.SetPoint
        Valvepid.setKp (float(config[1]))
        Valvepid.setKi (float(config[2]))
        Valvepid.setKd (float(config[3]))
        
        RefFlowpid.SetPoint = float(config[4])
        targetF = RefFlowpid.SetPoint
        RefFlowpid.setKp (float(config[5]))
        RefFlowpid.setKi (float(config[6]))
        RefFlowpid.setKd (float(config[7]))
        
        LowPressurepid.SetPoint = float(config[8])
        targetP = LowPressurepid.SetPoint
        LowPressurepid.setKp (float(config[9]))
        LowPressurepid.setKi (float(config[10]))
        LowPressurepid.setKd (float(config[11]))

#method to create the config file
def createConfig ():
    if not os.path.isfile('/tmp/pid.conf'):
        with open ('/tmp/pid.conf', 'w') as f:
            f.write('%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s'%(targetT, P, I, D, targetF, P2, I2, D2, targetP, P3, I3, D3))
    
                
def truncate(f, n):
    '''Truncates/pads a float f to n decimal places without rounding'''
    s = '{}'.format(f)
    if 'e' in s or 'E' in s:
        return '{0:.{1}f}'.format(f, n)
    i, p, d = s.partition('.')
    return '.'.join([i, (d+'0'*n)[:n]])

def TruncAndFillP(variable):
    if(variable < 0):
        variable = 0
    variable = truncate(variable, 2)
    variable = variable.zfill(6)
    variable = variable.encode()
    return variable

def TruncAndFillT(variable):
    variable = truncate(variable, 2)
    variable = variable.zfill(6)
    variable = variable.encode()
    return variable

def TruncAndFillExV(variable):
    variable = truncate(variable, 0)
    variable = variable.zfill(6)
    variable = variable.encode()
    return variable

def TruncAndFillV(variable):
    variable = truncate(variable, 3)
    variable = variable.zfill(4)
    variable = variable.encode()
    return variable

def CalP0(P0):
    P0_Cal = 39.938*P0 + 0.1032 + ATM
    return P0_Cal

def CalP1(P1):
     P1_Cal = 39.935*P1 + 0.1994 + ATM
     return P1_Cal

def CalP2(P2):
    P2_Cal = 39.860*P2 + 0.209 + ATM
    return P2_Cal

def CalP3(P3):
    P3_Cal = 39.979*P3 + 0.3136 + ATM 
    return P3_Cal

def Calculate_Prop_Valve():
    global ValvePercentage
    if (ManualMode == 0):
      #Get duty cycle using PID function
        Valvepid.update(OutletWaterT)
        ValvePercentage = Valvepid.output
        ValvePercentage = max(min(int(ValvePercentage), 100), 0)
        ValvePercentage = abs(100-ValvePercentage)
    elif (ManualMode == 1):
        ValvePercentage  = PropValvePercent
        
    # Set PWM expansion channel 0 to the target setting
    propValve(ValvePercentage)

def Calculate_Compressor_Throttle():
    global ThrottleCommand
    global CompressorThrottle
    global ThrottlePIDOutput
    if (ManualMode == 0):
        LowPressurepid.update(ExVP)
        ThrottlePIDOutput = LowPressurepid.output
        ThrottlePIDOutput = max(min(int(ThrottlePIDOutput), 70), 0)
        ThrottlePIDOutput = abs(70-ThrottlePIDOutput)
        ThrottleCommand = (ThrottlePIDOutput * 10) + 1300
    elif (ManualMode == 1):
        ThrottleCommand = 1000 + 10*CompSpeedPercent
        
    Compressor_Throttle(ThrottleCommand)
    CompressorThrottle = (ThrottleCommand - 1000)/10
    
def Calculate_ExV_Position():
    global ExVCurrentPosition
    global ExVCommand
    global ExVPosition
    global AutoFlag
    if (ManualMode == 0):
        #Get steps using PID function
        RefFlowpid.update(flowRate)
        ExVCommand = RefFlowpid.output
        ExVCommand = max(min(int(ExVCommand), 100), 0)
        ExVCommand = ExVCommand * 200
        AutoFlag = 1
    elif (ManualMode == 1):
        ExVCommand = 20*ExVPositionPercent
        AutoFlag = 0
    MoveExV(ExVCommand)
    ExVCurrentPosition = ExVCommand
    ExVPosition = ExVCurrentPosition/ 20000 * 100

def Get_Sensor_Variables():
    global flowRate
    global CompInletP
    global CompOutletP
    global CondenserP
    global ExVP
    global HighSideRefT
    global LowSideRefT
    global CompInletT
    global CompOutletT
    global InletWaterT
    global OutletWaterT
    
    flowRate = getFlowRate(7)
    CompInletP_r = getPressure(6)
    CompInletP = CalP0(CompInletP_r)
    CompOutletP_r = getPressure(5)
    CompOutletP = CalP1(CompOutletP_r)
    CondenserP_r = getPressure(3)
    CondenserP = CalP3(CondenserP_r)
    ExVP_r = getPressure(2)
    ExVP = CalP3(ExVP_r)
    
    HighSideRefT = thermocoupleTemp(0)
    LowSideRefT = thermocoupleTemp(1)
    CompInletT = thermocoupleTemp(2)
    CompOutletT = thermocoupleTemp(3)
    InletWaterT = thermocoupleTemp2(0)
    OutletWaterT = thermocoupleTemp2(1)

def INITIALIZE():
    global ExVCurrentPosition
    global samples
    #runs EXV closed all the way and resets position
    GPIO.output(Enable, 0)
    GPIO.output(Direction, 1)
    #"""
    for x in range(0,  25000):
        GPIO.output(Step, 1)
        sleep(0.0000051)
        GPIO.output(Step, 0)
        sleep(0.0000051)
    #"""
    GPIO.output(Enable, 1)
    ExVCurrentPosition = 0
    oldPos = 0
    #sets the throttle to zero percent 
    Compressor_Throttle(1000)
    #closes the water valve
    propValve(0)
    #resets the iteration count
    samples = 0
    #makes the config file for the initial startup conditions
    createConfig()
    
def RecieveSettings():
    
    global CompSpeedPercent 
    global ExVPositionPercent
    global PropValvePercent
    global LowSidePressureSP
    global targetP
    global FlowrateSP
    global flowRate
    global targetF
    global CondensingTempSP
    global targetT
    global ValvePIDKp
    global ValvePIDKi
    global ValvePIDKd 
    global RefFlowPIDKp
    global RefFlowPIDKi 
    global RefFlowPIDKd 
    global PressurePIDKp 
    global PressurePIDKi 
    global PressurePIDKd
    global ATM
    CompSpeedPercent_b= conn.recv(3)
    CompSpeedPercent = int(CompSpeedPercent_b)
    ExVPositionPercent_b = conn.recv(4)
    ExVPositionPercent = float(ExVPositionPercent_b)
    ExVPositionPercent = ExVPositionPercent
    PropValvePercent_b = conn.recv(3)
    PropValvePercent = int(PropValvePercent_b)
    LowSidePressureSP_b = conn.recv(4)
    LowSidePressureSP = float(LowSidePressureSP_b)
    LowSidePressureSP = LowSidePressureSP/10
    targetP = LowSidePressureSP
    FlowrateSP_b = conn.recv(4)
    FlowrateSP =int(FlowrateSP_b)
    FlowrateSP = float(FlowrateSP)
    FlowrateSP = FlowrateSP/1000
    targetF = FlowrateSP
    CondensingTempSP_b = conn.recv(3)
    CondensingTempSP = int(CondensingTempSP_b)
    targetT = CondensingTempSP
    ValvePIDKp_b = conn.recv(3)
    ValvePIDKp = int(ValvePIDKp_b)
    ValvePIDKi_b = conn.recv(3)
    ValvePIDKi = int(ValvePIDKi_b)
    ValvePIDKd_b = conn.recv(3)
    ValvePIDKd = int(ValvePIDKd_b)
    RefFlowPIDKp_b = conn.recv(3)
    RefFlowPIDKp = int(RefFlowPIDKp_b)
    RefFlowPIDKi_b = conn.recv(3)
    RefFlowPIDKi = int(RefFlowPIDKi_b)
    RefFlowPIDKd_b = conn.recv(3)
    RefFlowPIDKd = int(RefFlowPIDKd_b)
    PressurePIDKp_b = conn.recv(3)
    PressurePIDKp = int(PressurePIDKp_b)
    PressurePIDKi_b = conn.recv(3)
    PressurePIDKi = int(PressurePIDKi_b)
    PressurePIDKd_b = conn.recv(3)
    PressurePIDKd = int(PressurePIDKd_b)
    Valvepid.SetPoint = (float(targetT))
    RefFlowpid.SetPoint = (float(targetF))
    LowPressurepid.SetPoint = (float(targetP))
    ATM_b = conn.recv(3)
    ATM = (float(ATM_b))
     
    Valvepid.setKp (float(ValvePIDKp))
    Valvepid.setKi (float(ValvePIDKi))
    Valvepid.setKd (float(ValvePIDKd))
   
    RefFlowpid.setKp (float(RefFlowPIDKp))
    RefFlowpid.setKi (float(RefFlowPIDKi))
    RefFlowpid.setKd (float(RefFlowPIDKd))
    
    LowPressurepid.setKp (float(PressurePIDKp))
    LowPressurepid.setKi (float(PressurePIDKi))
    LowPressurepid.setKd (float(PressurePIDKd))
    
def Send_Data():
    T0_Data = CompInletT
    T1_Data = CompOutletT
    T2_Data = HighSideRefT
    T3_Data = LowSideRefT
    P0_Data = CompInletP
    P1_Data = CompOutletP
    P2_Data = CondenserP
    P3_Data = ExVP
    T0_Data = TruncAndFillT(T0_Data)
    T1_Data = TruncAndFillT(T1_Data)
    T2_Data = TruncAndFillT(T2_Data)
    T3_Data = TruncAndFillT(T3_Data)
    P0_Data = TruncAndFillP(P0_Data)
    P1_Data = TruncAndFillP(P1_Data)
    P2_Data = TruncAndFillP(P2_Data)
    P3_Data = TruncAndFillP(P3_Data)
    T4_Data = TruncAndFillT(InletWaterT)
    T5_Data = TruncAndFillT(OutletWaterT)
    RefFlow_Data = TruncAndFillT(flowRate)
    ExVPos_Data = TruncAndFillExV(ExVCurrentPosition)
    PropValve_Data = TruncAndFillT(ValvePercentage)
    CompThrottle_Data = TruncAndFillExV(CompSpeedPercent)
   
    
    conn.send(b'P0')
    conn.send(P0_Data)
    conn.send(b'P1')
    conn.send(P1_Data)
    conn.send(b'P2')
    conn.send(P2_Data)
    conn.send(b'P3')
    conn.send(P3_Data)
    conn.send(b'T0')
    conn.send(T0_Data)
    conn.send(b'T1')
    conn.send(T1_Data)
    conn.send(b'T2')
    conn.send(T2_Data)
    conn.send(b'T3')
    conn.send(T3_Data)
    
    conn.send(b'T4')
    conn.send(T4_Data)
    conn.send(b'T5')
    conn.send(T5_Data)
    conn.send(b'Fl')
    conn.send(RefFlow_Data)
    conn.send(b'EV')
    conn.send(ExVPos_Data)
    conn.send(b'PV')
    conn.send(PropValve_Data)
    conn.send(b'EN')
    conn.send(b'UNUSE')
    conn.send(CurrentState)
    conn.send(b'CP')
    conn.send(CompThrottle_Data)
    
   # conn.send(FlowV_Data)
  
conn, addr = s.accept()
samples = 0
ExVCurrentPosition = 0
AutoSamples = 0
ManualMode = 0
createConfig()
iterationStart = 0
#main loop, displays sensor values and positions
while 1:
    iterationEnd = time.time()
    iterationTime = iterationEnd - iterationStart
   #Take a command to know what to do
    cmnd = conn.recv(4)  # The default size of the command packet is 4 bytes
    iterationStart = time.time() 
    if 'INIT' in str(cmnd):
        RecieveSettings()
        print('INITIALIZING')
        INITIALIZE()
        Get_Sensor_Variables()
        CompOutletT = thermocoupleTemp(2)
        CurrentState = b'INIT-DONE'
        Send_Data()
        print('INIT -Done')
        print('')
        
    elif 'AUTO' in str(cmnd):
        
        ManualMode = 0
        manualSamples = 0
        start = time.time()
        RecieveSettings()
        end = time.time()
        RecievingTime = end - start
       # if (AutoSamples == 0): #only need to do this if first run or switching from manual
        #    readConfig()
        #    print('Getting Ready...')
        #    print('') 
        #    print('Config Read')
        start = time.time()
        Get_Sensor_Variables()
        end = time.time()
        GetVariablesTime = end - start
        start = time.time()
        Calculate_Prop_Valve()
        end = time.time()
        PropValveTime = end - start
        start = time.time()
        Calculate_Compressor_Throttle()
        end = time.time()
        CompressorThrottleTime = end - start
        start = time.time()
        Calculate_ExV_Position()
        end = time.time()
        ExVPositionTime = end - start
        CurrentState = b'AUTO-MODE'
        start = time.time()
        Send_Data()
        end = time.time()
        SendDataTime = end - start
           
        # Display the updated samples per channel count
        AutoSamples += 1
        PV_Valve, ExV_Pos, Comp_Speed = ValvePercentage, ExVPosition, CompSpeedPercent
        t = f"""
        {'-'*40}
        #Recieving Time:     {RecievingTime}
        #Get Variables Time:    {GetVariablesTime}
        #Prop Valve Time:     {PropValveTime}
        #Compressor Throttle Time:   {CompressorThrottleTime}
        #ExV Positioning:     {ExVPositionTime}
        #Sending Data Time:    {SendDataTime}
        {'-'*40}
        """
        
        s = f"""
        {'-'*40}
        # Auto Mode
        # Target Condesing Temp:    {targetT}  C    - Actual Temp    :{'{:12.1f}'.format(OutletWaterT)} C
        # Target Flowrate: {'{:12.3f}'.format(targetF)} Lpm  - Actual Flowrate:{'{:12.3f}'.format(flowRate)} Lpm
        # Target Pressure:         {targetP} psia - Actual Pressure:{'{:12.1f}'.format(ExVP)} psia
        # PV Position:         {PV_Valve}%
        # ExVPosition:         {'{:12.1f}'.format(ExV_Pos)}%
        # Compressor Throttle: {Comp_Speed}%
        # Iteration Time:      {'{:12.4f}'.format(iterationTime)} 
        # Iterations : {AutoSamples}
        {'-'*40}
        """
        #print(t)
        print(s)
 
    elif 'MANU' in str(cmnd):
        samples = 0
        ManualMode = 1
        AutoSamples = 0
        start = time.time()
        RecieveSettings()
        end = time.time()
        RecievingTime = end - start
        start = time.time()
        Get_Sensor_Variables()
        end = time.time()
        GetVariablesTime = end - start
        start = time.time()
        Calculate_Prop_Valve()
        end = time.time()
        PropValveTime = end - start
        start = time.time()
        Calculate_Compressor_Throttle()
        end = time.time()
        CompressorThrottleTime = end - start
        start = time.time()
        Calculate_ExV_Position()
        end = time.time()
        ExVPositionTime = end - start
        CurrentState = b'MANU-MODE'
        start = time.time()
        Send_Data()
        end = time.time()
        SendDataTime = end - start
        manualSamples += 1
        PV_Valve, ExV_Pos, Comp_Speed = ValvePercentage, ExVPosition, CompSpeedPercent
        s = f"""
        {'-'*40}
        # Manual Mode
        # PV Position:         {PV_Valve}%
        # ExVPosition:         {'{:12.1f}'.format(ExV_Pos)}%
        # Compressor Throttle: {Comp_Speed}%
        # Iteration Time:      {'{:12.4f}'.format(iterationTime)}
        # Iterations: {manualSamples}
        {'-'*40}
        """
        t = f"""
        {'-'*40}
        #Recieving Time:     {RecievingTime}
        #Get Variables Time:    {GetVariablesTime}
        #Prop Valve Time:     {PropValveTime}
        #Compressor Throttle Time:   {CompressorThrottleTime}
        #ExV Positioning:     {ExVPositionTime}
        #Sending Data Time:    {SendDataTime}
        {'-'*40}
        """
        #print(t)
        print(s)
     
    elif 'IDLE' in str(cmnd):
        ManualMode = 0
        RecieveSettings()
        manualSamples = 0
        Get_Sensor_Variables()
        Compressor_Throttle(1000)
        CurrentState = b'IDLE-MODE'
        print(CurrentState)
        Send_Data()          
        # Display the updated samples per channel count
        samples += 1
        s = f"""
        {'-'*40}
        # Idle Mode
        # Compressor InetP:         {'{:12.1f}'.format(CompInletP)} psia
        # Compressor OutletP:       {'{:12.1f}'.format(CompOutletP)} psia
        # Condenser InletP:         {'{:12.1f}'.format(CondenserP)} psia
        # ExV P:                    {'{:12.1f}'.format(ExVP)} psia
        # Iteration Time:      {'{:12.4f}'.format(iterationTime)}
        # Iterations: {samples}
        {'-'*40}
        """

        print(s)
 
stdout.flush()
