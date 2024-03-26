"""line_following_behavior controller."""
# Distributed Line-following (LF) + Obstacle avoidance (OA) behavior 
# for the e-puck robot colony . 

# The original LF code was tested on Webots R2020a, revision 1, on Windows 10 running
# Python 3.7.7 64-bit

# Original author: Felipe N. Martins
# Extended for a LF+OA colony by SB for Webots R2021b + Python 3.9.7 64-bit

from controller import Robot, DistanceSensor, Motor
import numpy as np

#-------------------------------------------------------
# Initialize variables

NO_SIDE = -1
LEFT = 0
RIGHT = 1
FORWARD= 2

TIME_STEP = 64
MAX_SPEED = 6.28

# These defined values have been acquired EMPIRICALLY
LFM_DIST_THRESHOLD = 350
LFM_LINE_THRESHOLD = 600

OAM_OBST_THRESHOLD = 80 
OAM_FORWARD_SPEED = 5

OAM_COUNTER_LIMIT = 1000
OFM_DELTA = 40

# 8 proximity sensors
PS_RIGHT_00 = 0
PS_RIGHT_45 = 1
PS_RIGHT_90 = 2
PS_BACK_RIGHT = 3
PS_BACK_LEFT = 4
PS_LEFT_90 = 5
PS_LEFT_45 = 6
PS_LEFT_00 = 7

speed = 1 * MAX_SPEED

# create the Robot instance.
robot = Robot()
rname = robot.getName();

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())   # [ms]

# states
states = ['forward', 'turn_right', 'turn_left']
current_state = states[0]

# counter: used to maintain an active state for a number of cycles
counter = 0
counter_max = 5

# speed weights
w_speed = 1.0
distance = 0.0

#-------------------------------------------------------
# Initialize devices

# distance sensors
ps = []
psNames = ['ps0', 'ps1', 'ps2', 'ps3', 'ps4', 'ps5', 'ps6', 'ps7']
for i in range(8):
    ps.append(robot.getDistanceSensor(psNames[i]))
    ps[i].enable(timestep)

# ground sensors
gs = []
gsNames = ['gs0', 'gs1', 'gs2']
for i in range(3):
    gs.append(robot.getDistanceSensor(gsNames[i]))
    gs[i].enable(timestep)

# motors    
leftMotor = robot.getMotor('left wheel motor')
rightMotor = robot.getMotor('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

#-------------------------------------------------------
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Update sensor readings
    psValues = []
    for i in range(8):
        psValues.append(ps[i].getValue())

    gsValues = []
    for i in range(3):
        gsValues.append(gs[i].getValue())

    # Process sensor data
    line_right = gsValues[0] > LFM_LINE_THRESHOLD
    line_left = gsValues[2] > LFM_LINE_THRESHOLD
    
#---------------------------------------------------------------------------------------------------------------          
#///////////////////////////////////////////////////////// 
# LFM - Line Following Module

    # 1. distance obtained from frontal distance sensors
    distance = psValues[0] + psValues[1] + psValues[6] + psValues[7]

    print(str(rname) + '.Distance ' + "%.2f" % distance )

    # 2. w_speed obtained from distance    
    if distance > 300 and distance < LFM_DIST_THRESHOLD:
        w_speed *= 0.5; # Reduce velocity in order to do not crash with other robots
    
    elif distance > LFM_DIST_THRESHOLD: 
        w_speed = 0; # Has crashed or is about to crash
    
    else:
        w_speed = 1.0; # No robots in front of
    
    DELTA = 0.03
    turn = (gsValues[0] - gsValues[2])*DELTA  
    
    print(str(rname) + '.gsValues[1] ' + "%.2f" % gsValues[1] )
    
#---------------------------------------------------------------------------------------------------------------          
#///////////////////////////////////////////////////////// 
# OAM - Obstacle Avoidance Module

# The OAM routine first detects obstacles in front of the robot 
# (the obstacle will be avoided by its right side) and avoid the detected obstacle 
# by turning away according to frontal distance sensors and the motors. "oam_active" 
# becomes "True" when an object in front of the robot is detected and "oam_reset" inactivates 
# the module and set "oam_side" to NO_SIDE. Output speeds are in oam_left_speed and 
# oam_right_speed.

    oam_reset = False
    oam_active = False
    oam_side = NO_SIDE
          
    # Module RESET
    if oam_reset: 
        oam_active = False
        oam_side = NO_SIDE
                
    oam_reset = False
    
    # Average between frontal sensors
    psValuesFront = (psValues[0] + psValues[7])/2
    print(str(rname) + '.FRONT ' + "%.2f" % psValuesFront )
    
    if !oam_active and max_ps_value > OAM_OBST_THRESHOLD:    # when front above a th, there is an obstacle
        oam_active = True
        oam_side=LEFT 
        oam_counter=0
        
    if oam_active:
        oam_left_speed = OAM_FORWARD_SPEED
        oam_right_speed = OAM_FORWARD_SPEED
        
        if oam_counter>OAM_COUNTER_LIMIT:
            if oam_side==LEFT:
              oam_side=FORWARD
            if oam_side==FORWARD:
              oam_side=RIGHT
            if oam_side==RIGHT:
              oam_side=NO_SIDE
              oam_reset=1
            oam_counter=0;
                               
        if oam_side == LEFT:
          oam_right_speed+= OAM_DELTA 
        
        if oam_side == RIGHT:
          oam_left_speed+= OAM_DELTA 
        
        if oam_side == FORWARD:
          # do nothing here, already done in L172-3
        
        oam_counter+=1   
           
#---------------------------------------------------------------------------------------------------------------          
#///////////////////////////////////////////////////////// 
# CONTROLLER 

    # Implement control
    if oam_active:
        leftSpeed = oam_left_speed * speed    #CHANGED (original: + turn)  
        if leftSpeed < 0:
            leftSpeed = 0
        rightSpeed = oam_right_speed * speed  ##CHANGED (original:- turn) 
        if rightSpeed < 0:
            rightSpeed = 0 
        
    else:
        leftSpeed  =  w_speed * speed + turn 
        rightSpeed =  w_speed * speed - turn 
        if turn < 0:
            current_state = 'turn_right'
            counter = 0 
        elif turn > 0: 
            current_state = 'turn_left'
            counter = 0
        else:
            current_state = 'forward'
    
    if leftSpeed>MAX_SPEED:
        leftSpeed=MAX_SPEED
    if leftSpeed<0:
        leftSpeed=0
    if rightSpeed>MAX_SPEED:
        rightSpeed=MAX_SPEED
    if rightSpeed<0:
        rightSpeed=0
    
    

    # increment counter
    counter += 1
    
    # Update reference velocities for the motors
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)
     
    # increment counter
    counter += 1
    
    # current state 
    print( str(rname) + '. Current state: ' + current_state)

    # Update reference velocities for the motors
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)

clear 