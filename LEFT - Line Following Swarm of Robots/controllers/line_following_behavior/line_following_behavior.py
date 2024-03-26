"""
Name: Vladimir Frunza

This is my code for the Line Following, Obstacle Avoiding swarm of
5 robots programmed with a PID controller project. 

General suggestion:

The code itself is far from solid. My suggestion is to run the
simulated world I have created first (hopefully you see exactly what
I see on my PC) which will show that, in concept, the code works to 
a certain level. If you want to change the world, I can already tell 
you that the robots might start behaving unexpectedly pretty fast.

Problems I have faced:

1. The sensors are completely unreliable for some reason.
Their values can sometimes vary as much as 8-10% for no reason
while standing still. That made this project exceedingly difficult.

2. The way this enviroment is made, from what I understood, is that
every individual robot is a member of the same "class", and every 
time the main loop runs each robot is individually simulated 
in order. For this reason, I didn't find any easy way to communicate
between robots, since all variables are local to the robot (object).
If I were to find a way to easily communicate between robots,
the solution to make them stop crashing with each other would have
been way easier.

"""
from controller import Robot, DistanceSensor, Motor
import numpy as np 

#----------------Initialize variables-------------------------
# 
kp = 0.01            # the three parameters for PID
ki = 0.0000001       # NOTE: Even if ki and kd may appear
kd = 0.0000001       # very little, their impact is very visible.
error = 0            # error and last_error, parameters used in PID
last_error = 0       #
turn = 0             # turning speed
integral = 0         # integral and derivative used in PID
derivative = 0       #
expected_value = 300 # expected value for error calculation in PID
approx_left = 0      # values used for gps variable
approx_right = 0     #
gps = 0              # gps variables for locating robot relative to surface
gps_aux = 0          #
gps_alert = 0        #
previous_state = 'forward'  # variable used for debugging
yes_oam = 0          # variable used to check if the robot is currently in an oam process
wait_time = 0        # variable used in formation maintaining
aux_dist = 0         # aux_dist used in formation maintaining
maintaining_formation = 0 # parameter to tell if robot is in the formation maintaining process
leftSpeed = 6.28     # left and right speed used to set the speed of the robot's wheels
rightSpeed = 6.28    #
oam_left_speed = 0   # oam left and right speed used to set the speed of the robot's wheels while in oam
oam_right_speed = 0  #
# ----------------------------- OA Module ----------------------------------
#
#  Finite-state automaton 
#  states 0+1: avoid collide to the obstacle, 
#  state 2:    leave behind the obstacle 
#  state 3:    come back to the circuit
NO_SIDE = -1  # stand-by state
RIGHT = 0     # state 0: turn right
RIGHT_LEFT=1  # state 1: turn left (after turn right)
FORWARD= 2    # state 2: go forward
LEFT = 3      # state 3. turn left

OAM_OBST_THRESHOLD = 350  # obstacle detection threshold 
OAM_RBTDIST_THRESHOLD = 295 # other robots detection threshold
OAM_FORWARD_SPEED = 3     # default speed while avoiding obstacles
OAM_COUNTER_LIMIT = 45    # permanence time in each state 
OAM_DELTA = 0.35          # weighting factor applied in turning left/right

oam_counter= 0            # state permanence counter 
oam_reset = False         # reset for going to the stand-by state
oam_active = False        # indicator of OA task ON/OFF
oam_side = NO_SIDE        # OA state
#
# ----------------------------- OA Module ----------------------------------

TIME_STEP = 64
MAX_SPEED = 6.28

# These defined values have been acquired EMPIRICALLY
LFM_DIST_THRESHOLD = 400
LFM_LINE_THRESHOLD = 600

speed = 0.6 * MAX_SPEED

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
    # make sure the base speed is set
    speed = 0.6 * MAX_SPEED 
    
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
    on_the_track = gsValues[1] < LFM_LINE_THRESHOLD
    
    
#---------------------------------------------------------------------------------------------------------------          
#///////////////////////////////////////////////////////// 
# LFM - Line Following Module

    # distance obtained from frontal distance sensors
    distance = psValues[0] + psValues[1] + psValues[6] + psValues[7]

    # updating approx_right and approx_left in order to tell 
    # if the robot's sensors are both on the same surface or not
    # values of the sensors are divided by 50 in order to see only
    # the big differences in values, since the reflectiveness of 
    # the 3 colors present on the track are distinctly different
    approx_right = int(gsValues[0] / 50)
    approx_left = int(gsValues[2] / 50)
    
    # calculate gps value
    # this value is 0 if both sensors are on the same material
    # and not 0 if they are not
    gps = approx_right - approx_left
    
    # calculating gps_alert
    # this variable is used for the fringe case in which both sensors
    # change surface at the exact same time, meaning "gps" wouldn't 
    # notice the change in surface
    if gsValues[0] > 600 and gsValues[0] > 600 and gps_alert == 0:
        if gps_aux == 0:
            gps_alert = 1
        else:
            gps_alert = 0
        gps_aux = 1
    else:
        if gps_aux == 1:
            gps_alert = -1
        else:
            gps_alert = 0
        gps_aux = 0
        
    # PID control calculation
    last_error = error;
    error = max(gsValues[0],gsValues[2]) - expected_value;
    
    integral = integral + error;
    derivative = error - last_error;
    turn = (kp * error + ki * integral + kd * derivative);
    
#---------------------------------------------------------------------------------------------------------------          
#///////////////////////////////////////////////////////// 
# FORMATION MAINTAINING

    # this is the formation maintaining algorithm 
    # it only affects the 2nd -> 5th robots and makes sure
    # they keep stop and wait for a period of time if they see
    # an object, as to allow the robot in front to go past 
    # the obstacle in front without pushing it
    if str(rname) != "e-puck" and yes_oam == 0:
        if distance >= OAM_RBTDIST_THRESHOLD:
            if aux_dist == 0:
                aux_dist = distance
            speed = 0
            wait_time += 1
            maintaining_formation = 1
        if wait_time >= 40:
            speed = 0.7 * MAX_SPEED
            aux_dist == 0
            wait_time = 0
            maintaining_formation = 0
        elif wait_time > 0:
            speed = 0
            wait_time += 1

    
                
#---------------------------------------------------------------------------------------------------------------          
#///////////////////////////////////////////////////////// 
# OAM - Obstacle Avoidance Module
    
    # I have only made one change to the OAM, which I marked 
    # I also added yes_oam which tells if the robot is currently
    # in an object avoidance maneuver    
    # Module RESET
    if oam_reset: 
        oam_active = False
        oam_side = NO_SIDE
                
    oam_reset = False
    oam_active = False
     
    if (oam_active==False and distance > OAM_OBST_THRESHOLD) or yes_oam == 1:    # when front: above a th, there is an obstacle
        oam_active = True
            
        if yes_oam == 0:
            oam_side=RIGHT 
            oam_counter=0
        
    if oam_active and maintaining_formation == 0:
        # go forward is the default maneuver
        yes_oam = 1 
        oam_left_speed = OAM_FORWARD_SPEED
        oam_right_speed = OAM_FORWARD_SPEED
        
        # if state permanence has ended, go to the next state
        if oam_counter>OAM_COUNTER_LIMIT:
            if oam_side==RIGHT_LEFT:
              oam_side=FORWARD
            elif oam_side==FORWARD:
              oam_side=LEFT
            elif oam_side==LEFT:
              oam_side=NO_SIDE
              yes_oam = 0
              oam_reset=1
            oam_counter=0;
        
        # This is the only change I've made
        # This makes sure the first turn is done at a faster rate
        # since I have also changed the speed of the right wheel
        # This was done because I saw that the robot keeps getting
        # stuck in the obstacle
        if oam_counter > 25 and oam_side == RIGHT:
            oam_side = RIGHT_LEFT
                               
        # turning left maneuver 
        if oam_side == LEFT or oam_side == RIGHT_LEFT:
          oam_left_speed*= OAM_DELTA 
        
        # when robot is coming back to the circuit, end if it is already in the circuit
        if oam_side == LEFT and on_the_track == True: 
            oam_side=NO_SIDE
            oam_reset = 1
            yes_oam = 0

        # turning right maneuver 
        if oam_side == RIGHT :
          oam_right_speed*= 0.03
             
        #if oam_side == FORWARD:
          # do nothing here, already done in L150-1
        
        oam_counter+=1
       
#---------------------------------------------------------------------------------------------------------------          
#///////////////////////////////////////////////////////// 
# CONTROLLER 

    # Changes to the controller are when the robot is not
    # in an oam process
    # Implement control
    if oam_active:
        leftSpeed = oam_left_speed  
        if leftSpeed < 0:
            leftSpeed = 0
        rightSpeed = oam_right_speed
        if rightSpeed < 0:
            rightSpeed = 0   
    else:
        # When gps is positive, that means the robot should turn 
        # to the right, and when it is positive, it should turn
        # to the left
        if gps > 0:
            previous_state = 'turn_right'
            counter = 0 
            gps_alert = 0
            leftSpeed  =  w_speed * speed + turn 
            rightSpeed =  w_speed * speed - turn 
        elif gps < 0: 
            previous_state = 'turn_left'
            counter = 0
            gps_alert = 0
            leftSpeed  =  w_speed * speed - turn 
            rightSpeed =  w_speed * speed + turn 
        # Also, when gps == 0, we need to check the gps_alert variable,
        # to see if the robot has maybe switched surfaces with
        # both sensors at the same time
        elif gps_alert == 0:
            previous_state = 'forward'
            counter = 0
            leftSpeed  =  w_speed * speed  
            rightSpeed =  w_speed * speed 
        elif gps_alert == 1:
            previous_state = 'emergency_turn_right'
            counter = 0 
            leftSpeed  =  w_speed * speed + turn 
            rightSpeed =  w_speed * speed - turn
        elif gps_alert == -1:
            previous_state = 'emergency_turn_left'
            counter = 0 
            leftSpeed  =  w_speed * speed - turn 
            rightSpeed =  w_speed * speed + turn
            
    
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
  
clear 