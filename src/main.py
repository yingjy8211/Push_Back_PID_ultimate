# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       Margaret Liu                                                 #
# 	Created:      1/13/2025, 10:24:50 PM                                      #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports
import time
from vex import *
# import math

# Parameters Definition and Robots Configuration
brain = Brain()
controller = Controller()
tl = 7.382
tr = 7.382
pi = 3.14159
# #Time_wait=0
wheelFactor = pi * 3.25 / (3/2)

#motor gear teeth 24. wheel gear teeth 36
# Motor Configuration
motorFL = Motor(Ports.PORT12, GearSetting.RATIO_6_1, True)
motorFR = Motor(Ports.PORT15, GearSetting.RATIO_6_1, False)
motorBL = Motor(Ports.PORT14, GearSetting.RATIO_6_1, True)
motorBR = Motor(Ports.PORT2, GearSetting.RATIO_6_1, False)
motorML = Motor(Ports.PORT13, GearSetting.RATIO_6_1, False)
motorMR = Motor(Ports.PORT1, GearSetting.RATIO_6_1, True)

# Intake and Roller Motors
intakeMotor_1 = Motor(Ports.PORT4, GearSetting.RATIO_6_1, True)
TopRoller = Motor(Ports.PORT10, GearSetting.RATIO_18_1, True)

# Sensors and Pneumatics
inertialSensor = Inertial(Ports.PORT3)
arm = DigitalOut(brain.three_wire_port.a)
cap = DigitalOut(brain.three_wire_port.h)
controller_1 = Controller(PRIMARY)
# AI Classification Competition Element IDs
class GameElements:
    MOBILE_GOAL = 0
    RED_RING = 1
    BLUE_RING = 2
    SKIP=-1
# AI Vision Color Descriptions
# AI Vision Code Descriptions
AI_clamp = AiVision(Ports.PORT15, AiVision.ALL_AIOBJS)

# hookPneumatic = DigitalOut(brain.three_wire_port.a)
# rightArm = Motor(Ports.PORT8, GearSetting.RATIO_18_1, False)
# leftArm = Motor(Ports.PORT7, GearSetting.RATIO_18_1, True)

def motor_Stop():  
    motorFL.stop()
    motorFR.stop()
    motorML.stop()
    motorMR.stop()
    motorBL.stop()
    motorBR.stop()

def motor_hold():  
    motorFL.stop(HOLD)
    motorFR.stop(HOLD)
    motorML.stop(HOLD)
    motorMR.stop(HOLD)
    motorBL.stop(HOLD)
    motorBR.stop(HOLD)

def motor_brake():  
    motorFL.stop(BRAKE)
    motorFR.stop(BRAKE)
    motorML.stop(BRAKE)
    motorMR.stop(BRAKE)
    motorBL.stop(BRAKE)
    motorBR.stop(BRAKE)

# Commanding the motors based on velocity percentage
def motor_Motion(motorFLSpeed, motorFRSpeed, motorBLSpeed, motorBRSpeed, motorMLSpeed, motorMRSpeed,):
    motorFL.spin(DirectionType.FORWARD, motorFLSpeed, VelocityUnits.PERCENT)
    motorFR.spin(DirectionType.FORWARD, motorFRSpeed, VelocityUnits.PERCENT)
    motorML.spin(DirectionType.FORWARD, motorBLSpeed, VelocityUnits.PERCENT)
    motorMR.spin(DirectionType.FORWARD, motorMRSpeed, VelocityUnits.PERCENT)
    motorBL.spin(DirectionType.FORWARD, motorMLSpeed, VelocityUnits.PERCENT)
    motorBR.spin(DirectionType.FORWARD, motorBRSpeed, VelocityUnits.PERCENT)

# Reset all sensors
def Reset_all():
    motorFL.set_position(0, DEGREES)
    motorFR.set_position(0, DEGREES)
    motorML.set_position(0, DEGREES)
    motorMR.set_position(0, DEGREES)
    motorBL.set_position(0, DEGREES)
    motorBR.set_position(0, DEGREES)
    # inertialSensor.set_rotation(0, DEGREES)

def get_Rotation_Sensor_Position():
    L_current_coor_pre=(motorFL.position(TURNS))
    R_current_coor_pre=(motorBR.position(TURNS))
    return L_current_coor_pre,R_current_coor_pre

def calculate_Rotation_From_Wheels(xLeft, yRight, wheelFactor, tl, tr):
    rotationInRadians = (xLeft * wheelFactor - yRight * wheelFactor) / (tl + tr)
    return rotationInRadians

def autonomousPID(target, initialMaxSpeedLimit, maxSpeedLimit, time_out, export_flag, Kp_l, Kp_r, previousError=[0.0,0.0]):
    """
    Drive a straight or turning motion using simple PID:
      - Encoders for distance
      - Inertial sensor for heading
    Simplified: no IMU acceleration, no drift PID.

    target: [target_distance_inches, target_heading_radians]
    """

    global hook_flag
    brain.screen.clear_screen()
    brain.screen.print("autonomous code")

    Reset_all()

    # --- PID constants ---
    Kp = Kp_l      # linear P
    Ki = 0.0

    KpRotation = Kp_r  # heading P
    KiRotation = 0.0

    # Derivative terms (optionally controlled by globals for testing)
    try:
        Kd = Kd_linear_global
    except NameError:
        Kd = 0.0

    try:
        KdRotation = Kd_rotation_global
    except NameError:
        KdRotation = 0.0

    # currentPosition[0] = distance (inches)
    # currentPosition[1] = heading (radians)
    currentPosition = [0.0, inertialSensor.rotation() * pi / 180.0]

    error = [0.0, 0.0]          # [distance_error, heading_error]
    integral = 0.0              # distance integral
    headingIntegral = 0.0       # heading integral

    dt = 0.005                  # 200 Hz loop
    counter = 0
    leftRotation = 0.0
    rightRotation = 0.0

    # ---------- Heading guard settings ----------
    # When heading error is larger than this angle, scale down forward output.
    HEADING_GUARD_DEG = 2.0
    # -------------------------------------------

    while True:
        # --- 1. Compute errors ---
        error[0] = target[0] - currentPosition[0]   # distance error (inches)
        error[1] = target[1] - currentPosition[1]   # heading error (radians)

        # --- 2. Integrals & derivatives ---
        integral += error[0] * dt
        headingIntegral += error[1] * dt

        derivative = (error[0] - previousError[0]) / dt
        headingDerivative = (error[1] - previousError[1]) / dt

        # --- 3. Raw PID outputs (before guards/limits) ---
        xOutput = (Kp * error[0]) + (Ki * integral) + (Kd * derivative)

        turnSpeed = (KpRotation * error[1] +
                     KiRotation * headingIntegral +
                     KdRotation * headingDerivative)

        # ---------- HEADING GUARD ON FORWARD ----------
        # Convert heading error to degrees for intuitive thresholding
        heading_error_deg = error[1] * 180.0 / pi
        abs_head_err = abs(heading_error_deg)

        if abs_head_err > HEADING_GUARD_DEG:
            # Scale forward down when heading is off.
            scale = HEADING_GUARD_DEG / abs_head_err
            if scale < 0.0:
                scale = 0.0
            elif scale > 1.0:
                scale = 1.0
            xOutput *= scale
        # -------------------------------------------------

        # --- 4. Mix into individual motor speeds ---
        motorFLSpeed = xOutput + turnSpeed
        motorFRSpeed = xOutput - turnSpeed
        motorMLSpeed = xOutput + turnSpeed
        motorMRSpeed = xOutput - turnSpeed
        motorBLSpeed = xOutput + turnSpeed
        motorBRSpeed = xOutput - turnSpeed

        # --- 5. Speed limiting (existing ramp logic) ---
        if counter <= 100:
            maxSpeed = max(abs(motorFLSpeed), abs(motorFRSpeed),
                           abs(motorMLSpeed), abs(motorMRSpeed),
                           abs(motorBLSpeed), abs(motorBRSpeed),
                           initialMaxSpeedLimit)
            if maxSpeed > initialMaxSpeedLimit:
                motorFLSpeed = (motorFLSpeed / maxSpeed) * initialMaxSpeedLimit
                motorFRSpeed = (motorFRSpeed / maxSpeed) * initialMaxSpeedLimit
                motorMLSpeed = (motorMLSpeed / maxSpeed) * initialMaxSpeedLimit
                motorMRSpeed = (motorMRSpeed / maxSpeed) * initialMaxSpeedLimit
                motorBLSpeed = (motorBLSpeed / maxSpeed) * initialMaxSpeedLimit
                motorBRSpeed = (motorBRSpeed / maxSpeed) * initialMaxSpeedLimit
        else:
            maxSpeed = max(abs(motorFLSpeed), abs(motorFRSpeed),
                           abs(motorMLSpeed), abs(motorMRSpeed),
                           abs(motorBLSpeed), abs(motorBRSpeed),
                           maxSpeedLimit)
            if maxSpeed > maxSpeedLimit:
                motorFLSpeed = (motorFLSpeed / maxSpeed) * maxSpeedLimit
                motorFRSpeed = (motorFRSpeed / maxSpeed) * maxSpeedLimit
                motorMLSpeed = (motorMLSpeed / maxSpeed) * maxSpeedLimit
                motorMRSpeed = (motorMRSpeed / maxSpeed) * maxSpeedLimit
                motorBLSpeed = (motorBLSpeed / maxSpeed) * maxSpeedLimit
                motorBRSpeed = (motorBRSpeed / maxSpeed) * maxSpeedLimit

        # --- 6. Command the motors ---
        motor_Motion(motorFLSpeed, motorFRSpeed,
                     motorBLSpeed, motorBRSpeed,
                     motorMLSpeed, motorMRSpeed)

        # --- 7. Save previous errors ---
        previousError[0] = error[0]
        previousError[1] = error[1]

        # --- 8. Update position from encoders & inertial heading ---
        leftRotation, rightRotation = get_Rotation_Sensor_Position()

        robotHeadingFromWheels = calculate_Rotation_From_Wheels(
            leftRotation, rightRotation, wheelFactor, tl, tr
        )
        robotHeadingFromInertial = inertialSensor.rotation() * pi / 180.0

        if leftRotation == rightRotation:
            currentPosition[0] = leftRotation * wheelFactor
        else:
            currentPosition[0] = ((leftRotation + rightRotation)) / 2.0 * wheelFactor

        currentPosition[1] = robotHeadingFromInertial
        # currentPosition[1] = robotHeadingFromWheels  # optional alternative

        # --- 9. Optional debug printout ---
        if counter % 10 == 0 and export_flag == 1:
            print(counter, end="\t")
            print('{:.5f}'.format(error[0]), end="\t")
            print('{:.5f}'.format(error[1]), end="\t")
            print('{:.5f}'.format(xOutput), end="\t")
            print('{:.5f}'.format(turnSpeed), end="\t")
            print('{:.5f}'.format(motorFLSpeed), end="\n")

        # --- 10. Exit conditions ---
        if abs(error[0]) < 0.01 and abs(error[1]) < 0.015:
            print("COMPLETE!!!")
            motor_Stop()
            print(error)
            print(counter)
            return error
        elif counter > time_out:
            print("COMPLETE!!! Timed Out!!!")
            motor_Stop()
            print(error)
            print(counter)
            return error

        time.sleep(dt)
        counter += 1

# def set_and_run_rollers(v_low,v_mid,v_top):
#     intakeMotor_1.set_velocity(v_low,PERCENT)
#     intakeMotor_2.set_velocity(v_low,PERCENT)
#     MidRoller.set_velocity(v_mid,PERCENT)
#     TopRoller.set_velocity(v_top,PERCENT)
#     intakeMotor_1.spin(FORWARD)
#     intakeMotor_2.spin(FORWARD)
#     MidRoller.spin(FORWARD)
#     TopRoller.spin(FORWARD)

# def all_intake_stop():
#     set_and_run_rollers(0,0,0)

# def intake_ground_to_mid(velocity):
#     set_and_run_rollers(-1*velocity,-1*velocity,1*velocity)

# def intake_ground_to_top(velocity):
#     set_and_run_rollers(-0.7*velocity,-1*velocity,-1*velocity)

# def intake_mid_to_top(velocity):
#     set_and_run_rollers(0*velocity,-1*velocity,-1*velocity)

# def intake_ground_to_basket(velocity):
#     set_and_run_rollers(1*velocity,1*velocity,1*velocity)    

# def intake_ground_cycle_at_top(velocity):
#     #topv orgininaly -0.6
#     set_and_run_rollers(-0.8*velocity,-1*velocity,-0.8*velocity)

# def intake_ground_hold_in_basket(velocity):
#     #topv orgininaly -0.6
#     set_and_run_rollers(-0.7*velocity,1*velocity,0*velocity)

# def intake_ground_hold_in_basket_top(velocity):
#     #topv orgininaly -0.6
#     set_and_run_rollers(-1*velocity,0*velocity,-0.5*velocity)

# def outake_top_to_ground(velocity):
#     set_and_run_rollers(1*velocity,-0.5*velocity,-1*velocity)

# def outake_basket_to_ground(velocity):
#     set_and_run_rollers(-1*velocity,-1*velocity,0*velocity)



# def user_control():
#     brain.screen.clear_screen()
#     brain.screen.print("driver control")
#     # place driver control in this while loop
#     while True:
#         wait(20, MSEC)
#         forward_speed = controller.axis3.position()  # Forward/Backward (vertical axis)
#         turn_speed = controller.axis4.position()  # Turning (horizontal axis)

#         # Calculate motor speeds
#         left_speed = forward_speed + turn_speed
#         right_speed = forward_speed - turn_speed

#         # Apply joystick values to motors
#         motorFL.spin(DirectionType.FORWARD, left_speed, VelocityUnits.PERCENT)
#         motorFR.spin(DirectionType.FORWARD, right_speed, VelocityUnits.PERCENT)
#         motorBL.spin(DirectionType.FORWARD, left_speed, VelocityUnits.PERCENT)
#         motorBR.spin(DirectionType.FORWARD, right_speed, VelocityUnits.PERCENT)

#         if controller.buttonA.pressing():
#             intakeMotor.spin(DirectionType.FORWARD, 100, VelocityUnits.PERCENT)
#         else:
#             intakeMotor.stop()
            
        

# create competition instance
# comp = Competition(user_control, autonomous)
vexcode_brain_precision = 0
vexcode_console_precision = 0
myVariable = 0




# create a function for handling the starting and stopping of all autonomous tasks
def vexcode_auton_function():
    # Start the autonomous control tasks
    auton_task_0 = Thread( onauton_autonomous_0 )
    # wait for the driver control period to end
    while( competition.is_autonomous() and competition.is_enabled() ):
        # wait 10 milliseconds before checking again
        wait( 10, MSEC )
    # Stop the autonomous control tasks
    auton_task_0.stop()
def when_started1():
    global myVariable
    pass
def onauton_autonomous_0():

    t_1=brain.timer.time(MSEC)
    inertialSensor.calibrate()
        
    # global #Time_wait
    # Wait until calibration is complete
    while inertialSensor.is_calibrating():
        wait(100, TimeUnits.MSEC)  # Check every 100 ms
    t_2=brain.timer.time(MSEC)

    if t_2-t_1<1800:
        inertialSensor.calibrate()
        while inertialSensor.is_calibrating():
            wait(100, TimeUnits.MSEC)  # Check every 100 ms


    # defult_time=200

    # intake_ground_hold_in_basket_top(100)
    
    # Time_wait=defult_time
    # extra_buffer=-11
    # intake_v=90
    # print("calibration finished")
    # v_min=20
    # v_max=60
    # export_flag=0
    # r_offset=math.radians(-180)
    # Time_wait=350
    # cap.set(False)
    # arm.set(True)


    # cap.set(True)
    # intake_ground_hold_in_basket(100)
    # return
    # v_min=20
    # v_max=20
    # f=20
    # r=0
    # Kp_linear=4
    # Kp_rotation=55
    # autonomousPID([f, math.radians(r)], v_min, v_max, Time_wait+extra_buffer,export_flag,
    #               Kp_linear,Kp_rotation)
    # return
    # intake_ground_to_top(100)
    # cap.set(True)
    # arm.set(False)
    # return


    # f=35.25
    # r=0
    # Kp_linear=4
    # Kp_rotation=30
    # autonomousPID([f, math.radians(r)], v_min, v_max, Time_wait+extra_buffer,export_flag,
    #               Kp_linear,Kp_rotation)
    # Time_wait=250
    # f=0
    # r=-90
    # Kp_linear=4
    # Kp_rotation=30
    # autonomousPID([f, math.radians(r)], v_min, v_max, Time_wait+extra_buffer,export_flag,
    #               Kp_linear,Kp_rotation)
    # export_flag=1
    # Time_wait=900
    # intake_ground_hold_in_basket(100)
    # v_min=30
    # v_max=40
    # f=12
    # r=90
    # Kp_linear=4
    # Kp_rotation=30
    # autonomousPID([f, math.radians(r)+r_offset], v_min, v_max, Time_wait+extra_buffer,export_flag,
    #               Kp_linear,Kp_rotation)
    # # for i in range (4):
    # #     f=1
    # #     r=90
    # #     Kp_linear=4
    # #     Kp_rotation=30
    # #     autonomousPID([f, math.radians(r)+r_offset], v_min, v_max, Time_wait+extra_buffer,export_flag,
    # #                 Kp_linear,Kp_rotation)
    # #     wait(10,MSEC)
    # motor_brake()
    
    # #motor_brake()

    # # v_min=20
    # # v_max=20
    # # f=-0.2
    # # r=90
    # # Kp_linear=2
    # # Kp_rotation=30
    # # autonomousPID([f, math.radians(r)+r_offset], v_min, v_max, Time_wait+extra_buffer,export_flag,
    # #               Kp_linear,Kp_rotation)

    # Time_wait=250
    # f=-5
    # r=90
    # Kp_linear=6
    # Kp_rotation=30
    # autonomousPID([f, math.radians(r)+r_offset], v_min, v_max, Time_wait+extra_buffer,export_flag,
    #               Kp_linear,Kp_rotation)
    
    # v_max=40
    # f=0
    # r=-86.5
    # Kp_linear=4
    # Kp_rotation=30
    # autonomousPID([f, math.radians(r)+r_offset], v_min, v_max, Time_wait+extra_buffer,export_flag,
    #               Kp_linear,Kp_rotation)
    # arm.set(False)
    
    # all_intake_stop()
    # cap.set(True)
    # v_min=20
    # v_max=40
    # Time_wait=300
    # f=17.75
    # r=-86.5
    # Kp_linear=4
    # Kp_rotation=25
    # autonomousPID([f, math.radians(r)+r_offset], v_min, v_max, Time_wait+extra_buffer,export_flag,
    #               Kp_linear,Kp_rotation)
    
    # v_min=20
    # v_max=40
    # Time_wait=100
    # f=-1
    # r=-86.5
    # Kp_linear=4
    # Kp_rotation=25
    # autonomousPID([f, math.radians(r)+r_offset], v_min, v_max, Time_wait+extra_buffer,export_flag,
    #               Kp_linear,Kp_rotation)
    # intake_ground_to_top(100)

    # wait(4, SECONDS)
    # all_intake_stop()
    # Time_wait=1200
    # v_min=20
    # v_max=40
    # f=-90.5
    # r=-180
    # Kp_linear=1.5
    # Kp_rotation=60
    # autonomousPID([f, math.radians(r)+r_offset], v_min, v_max, Time_wait+extra_buffer,export_flag,
    #               Kp_linear,Kp_rotation)
    # arm.set(True)
    # Time_wait=400
    # v_min=20
    # v_max=40
    # f=0
    # r=-266
    # Kp_linear=2.3
    # Kp_rotation=30
    # autonomousPID([f, math.radians(r)+r_offset], v_min, v_max, Time_wait+extra_buffer,export_flag,
    #               Kp_linear,Kp_rotation)
    # cap.set(False)    
    
    # intake_ground_hold_in_basket(100)
    # Time_wait=1600
    # v_min=40
    # v_max=50
    # #previous v_max=40
    # f=20
    # r=-266
    # Kp_linear=3
    # Kp_rotation=40
    # autonomousPID([f, math.radians(r)+r_offset], v_min, v_max, Time_wait+extra_buffer,export_flag,
    #               Kp_linear,Kp_rotation)
    # #motor_brake()
    
    # all_intake_stop()
    
    # Time_wait=300
    # v_min=20
    # v_max=40
    # f=-5
    # r=-266
    # Kp_linear=3
    # Kp_rotation=40
    # autonomousPID([f, math.radians(r)+r_offset], v_min, v_max, Time_wait+extra_buffer,export_flag,
    #               Kp_linear,Kp_rotation)
    # arm.set(False)
    # cap.set(True)  
    # Time_wait=300
    # v_min=20
    # v_max=40
    # f=0
    # r=-77
    # Kp_linear=3
    # Kp_rotation=40
    # autonomousPID([f, math.radians(r)+r_offset], v_min, v_max, Time_wait+extra_buffer,export_flag,
    #               Kp_linear,Kp_rotation)
    
    # Time_wait=400
    # v_min=20
    # v_max=40
    # f=20
    # r=-79
    # Kp_linear=3
    # Kp_rotation=30
    # autonomousPID([f, math.radians(r)+r_offset], v_min, v_max, Time_wait+extra_buffer,export_flag,
    #               Kp_linear,Kp_rotation)
    # intake_ground_to_top(100)

    # wait(4,SECONDS)
    # intake_ground_hold_in_basket(100)
    # Time_wait=1000
    # v_min=20
    # v_max=40
    # f=-46
    # r=0
    # Kp_linear=1.3
    # Kp_rotation=52
    # autonomousPID([f, math.radians(r)+r_offset], v_min, v_max, Time_wait+extra_buffer,export_flag,
    #               Kp_linear,Kp_rotation)
    
    # Time_wait=400
    # v_min=20
    # v_max=40
    # f=0
    # r=90
    # Kp_linear=1.8
    # Kp_rotation=30
    # autonomousPID([f, math.radians(r)+r_offset], v_min, v_max, Time_wait+extra_buffer,export_flag,
    #               Kp_linear,Kp_rotation)
    
    # Time_wait=400
    # v_min=20
    # v_max=40
    # f=-6
    # r=90
    # Kp_linear=3
    # Kp_rotation=30
    # autonomousPID([f, math.radians(r)+r_offset], v_min, v_max, Time_wait+extra_buffer,export_flag,
    #               Kp_linear,Kp_rotation)
    
    # Time_wait=400
    # v_min=50
    # v_max=60
    # f=50
    # r=90
    # Kp_linear=4
    # Kp_rotation=30
    # autonomousPID([f, math.radians(r)+r_offset], v_min, v_max, Time_wait+extra_buffer,export_flag,
    #               Kp_linear,Kp_rotation)
    
    # return
    # intake_ground_hold_in_basket(100)
    # Time_wait=500
    # f=0
    # r=-265
    # Kp_linear=2.5
    # Kp_rotation=30
    # autonomousPID([f, math.radians(r)+r_offset], v_min, v_max, Time_wait+extra_buffer,export_flag,
    #               Kp_linear,Kp_rotation)
    # v_min=10
    # v_max=20
    # f=-6
    # r=-265
    # Kp_linear=4
    # Kp_rotation=30
    # autonomousPID([f, math.radians(r)+r_offset], v_min, v_max, Time_wait+extra_buffer,export_flag,
    #               Kp_linear,Kp_rotation)
    # v_min=40
    # v_max=70
    # f=45
    # r=-265
    # Kp_linear=4
    # Kp_rotation=30
    # autonomousPID([f, math.radians(r)+r_offset], v_min, v_max, Time_wait+extra_buffer,export_flag,
    #               Kp_linear,Kp_rotation)
    
    
    

    # return
    # Time_wait=500
    # f=-25
    # r=-180
    # Kp_linear=4
    # Kp_rotation=20
    # autonomousPID([f, math.radians(r)+r_offset], v_min, v_max, Time_wait+extra_buffer,export_flag,
    #               Kp_linear,Kp_rotation)
    # arm.set(True)
    # Time_wait=1000
    # v_max=70
    # f=130
    # r=17-90
    # Kp_linear=2.7
    # Kp_rotation=60
    # autonomousPID([f, math.radians(r)+r_offset], v_min, v_max, Time_wait+extra_buffer,export_flag,
    #               Kp_linear,Kp_rotation)
    
    # intake_ground_hold_in_basket(100)
    # return
    # return
    
    # intake_ground_to_top(90)
    # wait(5,SECONDS)
    # Time_wait=500
    # f=-20
    # r=-150
    # Kp_linear=3.7
    # Kp_rotation=40
    # autonomousPID([f, math.radians(r)+r_offset], v_min, v_max, Time_wait+extra_buffer,export_flag,
    #               Kp_linear,Kp_rotation)
    # Time_wait=1000
    # v_max=70
    # f=100
    # r=-110
    # Kp_linear=3
    # Kp_rotation=50
    # autonomousPID([f, math.radians(r)+r_offset], v_min, v_max, Time_wait+extra_buffer,export_flag,
    #               Kp_linear,Kp_rotation)
    # return
    # intake_ground_hold_in_basket_top(100)
    # f=-15
    # r=0
    # Kp_linear=4
    # Kp_rotation=30
    # autonomousPID([f, math.radians(r)+r_offset], v_min, v_max, Time_wait+extra_buffer,export_flag,
    #               Kp_linear,Kp_rotation)


    # v_max=100
    # f=80
    # r=5
    # Kp_linear=6
    # Kp_rotation=40
    # autonomousPID([f, math.radians(r)+r_offset], v_min, v_max, Time_wait+extra_buffer,export_flag,
    #               Kp_linear,Kp_rotation)
    # arm.set(False)
    
   
def vexcode_driver_function():
    # Start the driver control tasks

    # wait for the driver control period to end
    while( competition.is_driver_control() and competition.is_enabled() ):
        # wait 10 milliseconds before checking again
        wait( 10, MSEC )
    # Stop the driver control tasks
competition = Competition(vexcode_driver_function, vexcode_auton_function )
when_started1()

# ------------------------------------------------------------
# PID tuning helper functions
# Each test moves the robot forward 60 inches ONCE.
# Physically reset the robot between running different tests.
# ------------------------------------------------------------

# Global derivative gains used by autonomousPID (optional)
Kd_linear_global = 0.0
Kd_rotation_global = 0.0

def test_move_60in(Kp_l, Kp_r, initialMaxSpeed=50, maxSpeed=90, label=""):
    """Run a single 60-inch move with given Kp values.
    Place the robot at the starting line, facing forward,
    then call this function from main() or the VEX console.
    """
    print("=== 60-inch test:", label, "Kp_l=", Kp_l, "Kp_r=", Kp_r, "===")
    # Use current inertial heading as the target heading
    target_heading_rad = inertialSensor.rotation() * pi / 180.0
    # time_out is in loop iterations (200 Hz * seconds)
    # 2000 iterations at 0.005s ~ 10 seconds max
    autonomousPID([60.0, target_heading_rad],
                  initialMaxSpeed, maxSpeed,
                  time_out=2000,
                  export_flag=1,
                  Kp_l=Kp_l,
                  Kp_r=Kp_r)

def test_kp_default():
    """Test with your typical Kp values (e.g. 4 for linear, 30 for rotation)."""
    global Kd_linear_global, Kd_rotation_global
    Kd_linear_global = 0.0
    Kd_rotation_global = 0.0
    test_move_60in(4.0, 30.0, label="Kp default")

def test_kp_more_heading():
    """Increase heading Kp slightly to see if drift improves."""
    global Kd_linear_global, Kd_rotation_global
    Kd_linear_global = 0.0
    Kd_rotation_global = 0.0
    test_move_60in(4.0, 36.0, label="Kp_rotation +20%")

def test_kd_linear():
    """Add a small linear derivative term to see if overshoot improves."""
    global Kd_linear_global, Kd_rotation_global
    Kd_linear_global = 0.1   # example starting value
    Kd_rotation_global = 0.0
    test_move_60in(4.0, 30.0, label="Kd_linear=0.1")

def test_kd_rotation():
    """Add a small rotational derivative term to damp heading oscillations."""
    global Kd_linear_global, Kd_rotation_global
    Kd_linear_global = 0.0
    Kd_rotation_global = 1.0   # example starting value
    test_move_60in(4.0, 30.0, label="Kd_rotation=1.0")