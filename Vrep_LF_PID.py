# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""

import vrep
import sys
import time

vrep.simxFinish(-1) # just in case, close all opened connections

clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to V-REP

if clientID!=-1:
    print ('Connected to remote API server')

else:
    print("connection not successful")
    sys.exit('Could not connect')
    
errorCode,left_motor_handle=vrep.simxGetObjectHandle(clientID,'left_joint',vrep.simx_opmode_oneshot_wait)#初始化
errorCode,right_motor_handle=vrep.simxGetObjectHandle(clientID,'right_joint',vrep.simx_opmode_oneshot_wait)

sensor_h=[] #empty list for handles

#for loop to retrieve sensor arrays and initiate sensors
for x in range(0,5+1):
    errorCode,sensor_handle=vrep.simxGetObjectHandle(clientID,'line_sensor'+str(x),vrep.simx_opmode_oneshot_wait)
    sensor_h.append(sensor_handle) #keep list of handles 
    errorCode,detectionState,auxPackets = vrep.simxReadVisionSensor(clientID,sensor_handle,vrep.simx_opmode_streaming)

Kp = 0.22
Ki = -0.0025
Kd = 0.00078
error = 0
previous_error = 0
I = 0
V_based = 2.3
SampleTime = 0.1
PID_value = 0

t=time.time()
t_cal = time.time()

while (time.time()-t)<200:    #运行200s
    #Loop Execution
    sensor_val=[]
    for x in range(0,5+1):
        errorCode,sensor_handle=vrep.simxGetObjectHandle(clientID,'line_sensor'+str(x),vrep.simx_opmode_oneshot_wait)
        errorCode,detectionState,auxPackets = vrep.simxReadVisionSensor(clientID,sensor_h[x],vrep.simx_opmode_buffer)
        
        sensor_val.insert(x,auxPackets[0][10])
        
    if sensor_val[2]<0.2 or sensor_val[3]<0.2:
        error = 0
       
    if sensor_val[1]<0.2:
        error = -2
        
    if sensor_val[4]<0.2:
        error = 2
        
    while(time.time()-t_cal)>=SampleTime:    
        PID_value = Kp*error + Kd*(error-previous_error)/SampleTime + Ki*(I + error)/SampleTime
        previous_error = error
        t_cal = time.time()
        
    V_left = V_based - PID_value
    V_right = V_based + PID_value
    
    errorCode = vrep.simxSetJointTargetVelocity(clientID,left_motor_handle,V_left,vrep.simx_opmode_streaming)
    errorCode = vrep.simxSetJointTargetVelocity(clientID,right_motor_handle,V_right,vrep.simx_opmode_streaming)
    
        
   # time.sleep(0.2)
    
errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle,0,vrep.simx_opmode_streaming)
errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle,0,vrep.simx_opmode_streaming)
    #loop executes once every 0.2 seconds (= 5 Hz)


