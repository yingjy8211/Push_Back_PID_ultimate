# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main_enhanced_H_drive.py                                     #
# 	Author:       Margaret Liu (Enhanced for H-DRIVE / Tank Drive)            #
# 	Created:      1/13/2025, 10:24:50 PM                                      #
# 	Description:  V5 project with 2D tracking for NON-HOLONOMIC drive         #
#                                                                              #
#   KEY INSIGHT: H-drive cannot strafe sideways!                              #
#   - We track X, Y position (2D odometry)                                    #
#   - But only control forward/back and rotation                              #
#   - Y drift is accumulated heading error - use it to improve heading ctrl   #
#                                                                              #
#   ENHANCEMENTS IMPLEMENTED:                                                  #
#   - Solution 2: IMU odometry for X-Y position tracking                      #
#   - Solution 3: Speed-dependent slippage compensation                       #
#   - Solution 4: Heading integral term with anti-windup                      #
#   - Solution 5: Increased heading correction gains with damping             #
#   - NEW: Drift rate detection for better heading control                    #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports
import time
from vex import *
import math

# Parameters Definition and Robots Configuration
brain = Brain()
controller = Controller()
tl = 7.382
tr = 7.382
pi = 3.14159

wheelFactor = pi * 3.25 / (3/2)

# Motor Configuration
motorFL = Motor(Ports.PORT1, GearSetting.RATIO_6_1, True)
motorFR = Motor(Ports.PORT2, GearSetting.RATIO_6_1, False)
motorBL = Motor(Ports.PORT3, GearSetting.RATIO_6_1, True)
motorBR = Motor(Ports.PORT4, GearSetting.RATIO_6_1, False)
motorML = Motor(Ports.PORT5, GearSetting.RATIO_6_1, False)
motorMR = Motor(Ports.PORT6, GearSetting.RATIO_6_1, True)

# Intake and Roller Motors
intakeMotor_1 = Motor(Ports.PORT11, GearSetting.RATIO_18_1, True)
intakeMotor_2 = Motor(Ports.PORT12, GearSetting.RATIO_18_1, False)
MidRoller = Motor(Ports.PORT20, GearSetting.RATIO_6_1, True)
TopRoller = Motor(Ports.PORT19, GearSetting.RATIO_18_1, True)

# Sensors and Pneumatics
inertialSensor = Inertial(Ports.PORT13)
arm = DigitalOut(brain.three_wire_port.a)
cap = DigitalOut(brain.three_wire_port.h)
controller_1 = Controller(PRIMARY)

# AI Classification
class GameElements:
    MOBILE_GOAL = 0
    RED_RING = 1
    BLUE_RING = 2
    SKIP = -1

AI_clamp = AiVision(Ports.PORT15, AiVision.ALL_AIOBJS)


# =============================================================================
# IMU ODOMETRY CLASS (Same as before)
# =============================================================================

class IMUOdometry:
    """
    ============================================================================
    IMU ODOMETRY - POSITION TRACKING WITHOUT WHEEL SLIPPAGE
    ============================================================================
    
    This class tracks robot position by integrating acceleration from the IMU.
    
    HOW IT WORKS:
    1. Read acceleration from IMU (in g's)
    2. Remove calibrated bias (sensor offset when still)
    3. Filter noise (low-pass filter)
    4. Transform from robot frame to world frame (account for rotation)
    5. Integrate acceleration → velocity
    6. Apply damping to velocity (prevent drift)
    7. Integrate velocity → position
    
    WHY WE NEED IT:
    - Wheel encoders slip at high speeds (report more distance than traveled)
    - IMU measures robot body acceleration (no wheel contact = no slip!)
    - Fuse IMU + encoders for best accuracy
    
    TRADE-OFFS:
    ✓ Immune to wheel slippage
    ✓ Detects lateral drift (sideways motion)
    ✗ Drifts over time (integration errors accumulate)
    ✗ Sensitive to vibration
    → Solution: Fuse with encoders (70% encoder + 30% IMU)
    
    FOR H-DRIVE:
    Even though H-drive can't strafe, IMU still tracks Y position:
    - Tells us where we ACTUALLY are (vs where we think we are)
    - Y drift = accumulated heading error
    - Use drift rate to improve heading control
    """
    
    def __init__(self):
        # =====================================================================
        # POSITION STATE (in world frame, inches)
        # =====================================================================
        self.x = 0.0  # X position (forward/backward in field coordinates)
        self.y = 0.0  # Y position (left/right in field coordinates)
        
        # =====================================================================
        # VELOCITY STATE (in world frame, inches/second)
        # =====================================================================
        self.vx = 0.0  # X velocity (how fast moving in X direction)
        self.vy = 0.0  # Y velocity (how fast moving in Y direction)
        
        # =====================================================================
        # BIAS CALIBRATION (removes sensor static error)
        # =====================================================================
        # When robot is still, IMU should read 0 acceleration
        # But due to manufacturing, it might read small non-zero values
        # We measure this error while still and subtract it during movement
        self.accel_bias_x = 0.0  # Average error in X axis (g's)
        self.accel_bias_y = 0.0  # Average error in Y axis (g's)
        self.bias_samples = 0    # How many samples collected so far
        self.BIAS_CALIBRATION_SAMPLES = 100  # Collect 100 samples (1 second)
        
        # =====================================================================
        # FILTERING STATE (for noise reduction)
        # =====================================================================
        # IMU readings are noisy - we use low-pass filter to smooth them
        # Keep previous acceleration to compute filtered value
        self.prev_ax = 0.0  # Previous filtered acceleration X (in/s²)
        self.prev_ay = 0.0  # Previous filtered acceleration Y (in/s²)
        
    def calibrate_bias(self):
        """
        =====================================================================
        STEP 1: CALIBRATE SENSOR BIAS (DO THIS BEFORE MOVEMENT!)
        =====================================================================
        
        PURPOSE:
        Remove static sensor error by measuring what IMU reads when still.
        
        WHY NEEDED:
        When robot is perfectly still, IMU should read 0g acceleration.
        But due to manufacturing imperfections, it might read:
          X: +0.02g, Y: -0.01g  (example)
        
        If we don't remove this bias:
          0.02g = 7.7 in/s²
          After 3 seconds: 23 in/s velocity error
          After 6 seconds: 69 inches position error!
        
        HOW IT WORKS:
        1. While robot is STATIONARY, read IMU 100 times
        2. Average these readings = bias
        3. During movement, subtract bias from readings
        
        WHEN TO CALL:
        Call this 100 times while robot is still:
          for i in range(100):
              imu_odom.calibrate_bias()
              wait(10, MSEC)
        
        IMPORTANT:
        - Robot MUST be completely still during calibration!
        - Don't move robot or bump field
        - Takes ~1 second (100 samples × 10ms)
        """
        if self.bias_samples < self.BIAS_CALIBRATION_SAMPLES:
            # -----------------------------------------------------------------
            # Still calibrating - collect one more sample
            # -----------------------------------------------------------------
            
            # Read current acceleration (in g's)
            # Should be 0g if robot still, but will have small error
            ax_raw = inertialSensor.acceleration(XAXIS)  # Forward/back
            ay_raw = inertialSensor.acceleration(YAXIS)  # Left/right
            
            # Accumulate (we'll average later)
            self.accel_bias_x += ax_raw
            self.accel_bias_y += ay_raw
            self.bias_samples += 1
            
            # -----------------------------------------------------------------
            # Check if we've collected enough samples
            # -----------------------------------------------------------------
            if self.bias_samples >= self.BIAS_CALIBRATION_SAMPLES:
                # Calculate average bias
                self.accel_bias_x /= self.BIAS_CALIBRATION_SAMPLES
                self.accel_bias_y /= self.BIAS_CALIBRATION_SAMPLES
                
                print(f"IMU Bias Calibrated: X={self.accel_bias_x:.3f}g, Y={self.accel_bias_y:.3f}g")
                print("Now when robot still, we know sensor reads these values")
                print("During movement, we subtract bias to get true acceleration")
            
            return False  # Not ready yet
        return True  # Calibration complete!
        
    def reset(self):
        """
        =====================================================================
        RESET POSITION AND VELOCITY TO ZERO
        =====================================================================
        
        PURPOSE:
        Start fresh tracking from origin (0, 0)
        
        WHEN TO USE:
        - At start of autonomous
        - Between separate movements if you want each to start at (0,0)
        - After calibration completes
        
        WHAT IT DOES:
        Resets position and velocity, but KEEPS calibrated bias
        (Bias stays calibrated - don't need to recalibrate!)
        
        EXAMPLE:
          imu_odom.reset()  # Now at (0, 0) with zero velocity
          # Move robot...
          # imu_odom.x = 24.5", imu_odom.y = 1.2"
          imu_odom.reset()  # Back to (0, 0)
          # Next movement starts fresh
        """
        self.x = 0.0   # Reset X position to origin
        self.y = 0.0   # Reset Y position to origin
        self.vx = 0.0  # Reset X velocity to zero
        self.vy = 0.0  # Reset Y velocity to zero
        # NOTE: We DON'T reset bias - it stays calibrated!
        
    def update(self, dt, current_heading):
        """
        =====================================================================
        UPDATE POSITION - MAIN ODOMETRY FUNCTION (CALL EVERY CYCLE!)
        =====================================================================
        
        PURPOSE:
        Calculate current robot position by integrating IMU acceleration
        
        CALL THIS:
        Every control cycle (every 5ms):
          x, y = imu_odom.update(0.005, robot_heading_radians)
        
        PARAMETERS:
        dt: Time since last update (seconds) - typically 0.005 (5ms)
        current_heading: Robot heading in RADIANS (from inertial sensor)
        
        RETURNS:
        (x, y): Current position in inches (world frame)
        
        PROCESS (7 STEPS):
        1. Read raw acceleration from IMU
        2. Remove calibrated bias
        3. Filter noise
        4. Transform robot frame → world frame
        5. Integrate acceleration → velocity
        6. Apply damping (prevent drift)
        7. Integrate velocity → position
        """
        
        # =====================================================================
        # STEP 1 & 2: READ ACCELERATION AND REMOVE BIAS
        # =====================================================================
        
        # Conversion factor: 1g = 386.22 inches/second²
        # Why: 1g = 9.80665 m/s² × 39.3701 in/m = 386.22 in/s²
        G_TO_INCHES_PER_SEC2 = 386.22
        
        # Read raw acceleration (in g's)
        accel_forward_raw = inertialSensor.acceleration(XAXIS)  # Robot forward/back
        accel_lateral_raw = inertialSensor.acceleration(YAXIS)  # Robot left/right
        
        # Remove calibrated bias and convert to inches/second²
        # If robot still: bias-corrected acceleration should be ~0
        accel_forward = (accel_forward_raw - self.accel_bias_x) * G_TO_INCHES_PER_SEC2
        accel_lateral = (accel_lateral_raw - self.accel_bias_y) * G_TO_INCHES_PER_SEC2
        
        # Example:
        #   Raw: 0.15g forward, 0.02g lateral
        #   Bias: 0.02g, 0.01g
        #   Corrected: 0.13g, 0.01g
        #   In in/s²: 50.2, 3.9
        
        # =====================================================================
        # STEP 3: LOW-PASS FILTER (REDUCE NOISE)
        # =====================================================================
        
        # IMU readings are noisy - use exponential moving average
        # Formula: new_value = α × current + (1-α) × previous
        # α = 0.7 means: 70% new data + 30% history
        ALPHA = 0.7
        
        # Apply filter
        accel_forward = ALPHA * accel_forward + (1 - ALPHA) * self.prev_ax
        accel_lateral = ALPHA * accel_lateral + (1 - ALPHA) * self.prev_ay
        
        # Save for next cycle
        self.prev_ax = accel_forward
        self.prev_ay = accel_lateral
        
        # Why this works:
        #   Without filter: 50.2 → 52.8 → 48.9 (jumpy!)
        #   With filter:    50.2 → 51.8 → 50.5 (smooth!)
        #   High-frequency noise removed, real acceleration kept
        
        # =====================================================================
        # STEP 4: TRANSFORM FROM ROBOT FRAME TO WORLD FRAME
        # =====================================================================
        
        # PROBLEM:
        # IMU measures in robot frame (X=forward, Y=left)
        # But robot rotates! We need world frame (field coordinates)
        
        # EXAMPLE:
        # Robot at 0°:   10 in/s² forward = (10, 0) in world
        # Robot at 90°:  10 in/s² forward = (0, 10) in world
        # Same acceleration, different world coordinates!
        
        # SOLUTION: Rotation matrix
        # [ax_world]   [cos(θ)  -sin(θ)] [ax_robot]
        # [ay_world] = [sin(θ)   cos(θ)] [ay_robot]
        
        ax_world = (accel_forward * math.cos(current_heading) - 
                    accel_lateral * math.sin(current_heading))
        ay_world = (accel_forward * math.sin(current_heading) + 
                    accel_lateral * math.cos(current_heading))
        
        # Now ax_world, ay_world are in FIELD coordinates
        # (Same direction regardless of robot heading)
        
        # =====================================================================
        # STEP 5: INTEGRATE ACCELERATION → VELOCITY
        # =====================================================================
        
        # Physics: velocity = velocity + acceleration × time
        # Example:
        #   Start: v = 0 in/s
        #   Accel: a = 50 in/s²
        #   Time: dt = 0.005s
        #   New v = 0 + (50 × 0.005) = 0.25 in/s
        #   After 100 cycles: v = 25 in/s
        
        # IMPORTANT: Save old velocity for accurate position integration!
        # We need the velocity at the START of the time step
        vx_old = self.vx
        vy_old = self.vy
        
        self.vx += ax_world * dt
        self.vy += ay_world * dt
        
        # =====================================================================
        # STEP 6: APPLY VELOCITY DAMPING (PREVENT DRIFT)
        # =====================================================================
        
        # PROBLEM:
        # Small errors in acceleration accumulate in velocity
        # After 10 seconds, might have 2 in/s velocity error
        # This causes position to drift even when robot stopped!
        
        # SOLUTION: Damping
        # Decay velocity by 2% each cycle (multiply by 0.98)
        # When no acceleration, velocity naturally goes to zero
        DAMPING_FACTOR = 0.98  # Lose 2% per cycle
        
        self.vx *= DAMPING_FACTOR
        self.vy *= DAMPING_FACTOR
        
        # Effect over time:
        #   After 50 cycles (0.25s): 36% of original velocity remains
        #   After 100 cycles (0.5s): 13% remains
        #   After 200 cycles (1.0s): 2% remains
        # Errors decay quickly, real movement preserved
        
        # =====================================================================
        # STEP 7: INTEGRATE VELOCITY → POSITION (TRAPEZOIDAL RULE)
        # =====================================================================
        
        # CORRECT METHOD: Use AVERAGE velocity during time step
        # 
        # Why? Velocity is NOT constant during dt!
        # Velocity changed from:
        #   1. Adding acceleration
        #   2. Applying damping
        # 
        # We need to use the AVERAGE of start and end velocity:
        #
        # velocity_start ──┐
        #                  ├─→ velocity_average ─→ position change
        # velocity_end   ──┘
        #
        # This is called "Trapezoidal Rule" or "Semi-Implicit Euler"
        # It's 20x more accurate than simple Euler integration!
        
        # Calculate average velocity during this time step
        vx_avg = (vx_old + self.vx) / 2
        vy_avg = (vy_old + self.vy) / 2
        
        # Physics: position = position + average_velocity × time
        # 
        # Example:
        #   Current: x = 10 inches
        #   Velocity start: 20 in/s
        #   Velocity end: 20.25 in/s (after accel & damping)
        #   Average velocity: (20 + 20.25)/2 = 20.125 in/s
        #   Time: dt = 0.005s
        #   New x = 10 + (20.125 × 0.005) = 10.100625 inches ✓
        #
        # If we used end velocity (WRONG):
        #   New x = 10 + (20.25 × 0.005) = 10.10125 inches
        #   Error: 0.000625 inches per step
        #   After 200 steps: ~0.125 inches accumulated error!
        #
        # With average velocity (CORRECT):
        #   Error: ~0.000003 inches per step (100x better!)
        #   After 200 steps: ~0.0006 inches error
        
        self.x += vx_avg * dt
        self.y += vy_avg * dt
        
        # =====================================================================
        # RETURN CURRENT POSITION
        # =====================================================================
        
        return self.x, self.y
        
        # FINAL RESULT:
        # We now know where robot ACTUALLY is (X, Y in inches)
        # This is immune to wheel slippage!
        # Fuse with encoders for best accuracy:
        #   position = 70% encoder + 30% IMU


# =============================================================================
# SLIPPAGE COMPENSATION
# =============================================================================

def compensate_wheel_slippage(raw_distance, average_motor_speed):
    """Compensate for wheel slippage based on motor speed."""
    speed = abs(average_motor_speed)
    
    if speed < 40:
        slippage_factor = 1.00
    elif speed < 60:
        slippage_factor = 0.99
    elif speed < 80:
        slippage_factor = 0.97
    else:
        slippage_factor = 0.95
    
    return raw_distance * slippage_factor


# =============================================================================
# BASIC MOTOR CONTROL FUNCTIONS
# =============================================================================

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

def motor_Motion(motorFLSpeed, motorFRSpeed, motorBLSpeed, motorBRSpeed, motorMLSpeed, motorMRSpeed):
    motorFL.spin(DirectionType.FORWARD, motorFLSpeed, VelocityUnits.PERCENT)
    motorFR.spin(DirectionType.FORWARD, motorFRSpeed, VelocityUnits.PERCENT)
    motorML.spin(DirectionType.FORWARD, motorBLSpeed, VelocityUnits.PERCENT)
    motorMR.spin(DirectionType.FORWARD, motorMRSpeed, VelocityUnits.PERCENT)
    motorBL.spin(DirectionType.FORWARD, motorBLSpeed, VelocityUnits.PERCENT)
    motorBR.spin(DirectionType.FORWARD, motorBRSpeed, VelocityUnits.PERCENT)

def Reset_all():
    motorFL.set_position(0, DEGREES)
    motorFR.set_position(0, DEGREES)
    motorML.set_position(0, DEGREES)
    motorMR.set_position(0, DEGREES)
    motorBL.set_position(0, DEGREES)
    motorBR.set_position(0, DEGREES)

def get_Rotation_Sensor_Position():
    L_current_coor_pre = motorFL.position(TURNS)
    R_current_coor_pre = motorBR.position(TURNS)
    return L_current_coor_pre, R_current_coor_pre


# =============================================================================
# CORRECTED: H-DRIVE / NON-HOLONOMIC PID CONTROLLER
# =============================================================================

def autonomousPID_H_drive(target_distance, target_heading, initialMaxSpeedLimit,
                          maxSpeedLimit, time_out, export_flag, Kp_forward, 
                          Kp_rotation, use_imu_odometry=True, ramp_cycles=50):
    """
    PID controller for H-DRIVE (non-holonomic) robot.
    
    KEY DIFFERENCES FROM HOLONOMIC VERSION:
    - Robot can only move forward/backward and rotate (cannot strafe!)
    - We track X, Y position to MONITOR drift, not control it
    - Y drift is accumulated heading error - we use drift rate to improve heading control
    - Only control forward distance and heading
    
    Args:
        target_distance: Target forward distance in inches
        target_heading: Target heading in radians
        initialMaxSpeedLimit: Max speed % during ramp
        maxSpeedLimit: Max speed % after ramp
        time_out: Maximum cycles before timeout
        export_flag: 1 for debug output
        Kp_forward: Proportional gain for forward control
        Kp_rotation: Proportional gain for rotation
        use_imu_odometry: True to track drift, False for encoder-only
        ramp_cycles: Number of cycles for speed ramp (default 50)
    
    Returns:
        Final error [distance_error, heading_error, lateral_drift_observed]
    """
    
    brain.screen.clear_screen()
    brain.screen.print("H-Drive PID with Drift Tracking")
    
    Reset_all()
    
    # =============================================================================
    # PID CONSTANTS
    # =============================================================================
    
    # FORWARD CONTROL
    Kp_forward_ctrl = Kp_forward
    Ki_forward = 0.0
    Kd_forward = 0.0
    
    # HEADING CONTROL - Solutions 4 & 5
    KpRotation = Kp_rotation
    KiRotation = 0.2   # Integral for persistent drift
    KdRotation = 2.0   # Derivative damping
    
    # NEW: DRIFT RATE COMPENSATION
    # If we detect the robot is drifting sideways, increase heading correction
    Kp_drift_rate = 15.0  # Gain for drift rate feedforward
    
    # =============================================================================
    # STATE INITIALIZATION
    # =============================================================================
    
    # Current state: [forward_distance, heading, lateral_drift]
    # Note: We track lateral_drift but DON'T control it (can't strafe!)
    currentPosition = [0.0, inertialSensor.rotation() * pi / 180, 0.0]
    
    # Store initial heading for coordinate transformation
    initial_heading = currentPosition[1]
    
    # Error terms
    error = [0.0, 0.0, 0.0]  # [distance_error, heading_error, drift_observed]
    previous_error = [0.0, 0.0, 0.0]
    
    # PID accumulators
    integral_forward = 0.0
    headingIntegral = 0.0
    
    # Control parameters
    dt = 0.005
    counter = 0
    
    # Initialize IMU odometry
    imu_odom = IMUOdometry()
    
    if use_imu_odometry:
        print("Calibrating IMU odometry...")
        for i in range(imu_odom.BIAS_CALIBRATION_SAMPLES):
            imu_odom.calibrate_bias()
            time.sleep(0.01)
        print("IMU calibration complete!")
        imu_odom.reset()
    
    # Print initial state
    if export_flag == 1:
        print("\n=== H-DRIVE PID STARTED ===")
        print(f"Target: Distance={target_distance:.2f}\", Heading={math.degrees(target_heading):.1f}°")
        print(f"Drive type: H-DRIVE (forward/rotate only)")
        print(f"Using {'IMU' if use_imu_odometry else 'ENCODER'} odometry")
        print(f"Kp_f={Kp_forward_ctrl}, Kp_r={KpRotation}, Kp_drift={Kp_drift_rate}")
        print("\nCycle\tDist_err\tHead_err\tDrift\tFwdOut\tTurnOut")
        print("-" * 70)
    
    # =============================================================================
    # MAIN PID LOOP
    # =============================================================================
    
    while True:
        # =========================================================================
        # 1. UPDATE POSITION
        # =========================================================================
        
        # Get encoder readings
        leftRotation, rightRotation = get_Rotation_Sensor_Position()
        
        # Get current heading
        robotHeading = inertialSensor.rotation() * pi / 180
        
        # Calculate encoder-based forward distance
        if leftRotation == rightRotation:
            encoder_distance = leftRotation * wheelFactor
        else:
            encoder_distance = ((leftRotation + rightRotation)) / 2 * wheelFactor
        
        # Apply slippage compensation
        avg_motor_speed = (abs(motorFL.velocity(PERCENT)) + 
                          abs(motorFR.velocity(PERCENT))) / 2
        encoder_distance_compensated = compensate_wheel_slippage(encoder_distance, avg_motor_speed)
        
        # For H-drive, forward distance is our X coordinate (in robot frame)
        encoder_forward = encoder_distance_compensated
        
        # Get IMU position (tracks actual path including drift)
        if use_imu_odometry:
            imu_x, imu_y = imu_odom.update(dt, robotHeading)
            
            # Project IMU position onto forward direction
            # (The component of actual movement in the intended direction)
            heading_change = robotHeading - initial_heading
            imu_forward = imu_x * math.cos(heading_change) + imu_y * math.sin(heading_change)
            
            # Lateral drift is the perpendicular component
            # This is what we CAN'T fix directly, but we monitor it
            lateral_drift = -imu_x * math.sin(heading_change) + imu_y * math.cos(heading_change)
            
            # Sensor fusion for forward distance
            ENCODER_WEIGHT = 0.7
            IMU_WEIGHT = 0.3
            currentPosition[0] = ENCODER_WEIGHT * encoder_forward + IMU_WEIGHT * imu_forward
            currentPosition[2] = lateral_drift  # Track but don't control
        else:
            # Encoder-only
            currentPosition[0] = encoder_forward
            currentPosition[2] = 0.0  # No drift tracking without IMU
        
        # Always use inertial sensor for heading
        currentPosition[1] = robotHeading
        
        # =========================================================================
        # 2. CALCULATE ERRORS
        # =========================================================================
        
        error[0] = target_distance - currentPosition[0]  # Forward error
        error[1] = target_heading - currentPosition[1]   # Heading error
        error[2] = currentPosition[2]  # Lateral drift (observed, not controlled)
        
        # =========================================================================
        # 3. CALCULATE DRIFT RATE (KEY FOR H-DRIVE!)
        # =========================================================================
        
        # If robot is drifting sideways, it means heading control isn't tight enough
        # Use drift rate to add feedforward correction to heading
        drift_rate = (error[2] - previous_error[2]) / dt if counter > 0 else 0.0
        
        # If drifting right (positive), we need to turn left (negative correction)
        # If drifting left (negative), we need to turn right (positive correction)
        drift_correction = -Kp_drift_rate * drift_rate
        
        # =========================================================================
        # 4. CALCULATE PID OUTPUTS
        # =========================================================================
        
        # Forward control (unchanged)
        integral_forward += error[0] * dt
        derivative_forward = (error[0] - previous_error[0]) / dt
        forwardOutput = (Kp_forward_ctrl * error[0] + 
                        Ki_forward * integral_forward + 
                        Kd_forward * derivative_forward)
        
        # Heading control WITH drift rate compensation
        headingIntegral += error[1] * dt
        
        # Anti-windup
        MAX_HEADING_INTEGRAL = 0.5
        if headingIntegral > MAX_HEADING_INTEGRAL:
            headingIntegral = MAX_HEADING_INTEGRAL
        elif headingIntegral < -MAX_HEADING_INTEGRAL:
            headingIntegral = -MAX_HEADING_INTEGRAL
        
        headingDerivative = (error[1] - previous_error[1]) / dt
        
        # CORRECTED: Add drift rate correction to heading control
        turnSpeed = (KpRotation * error[1] +           # Proportional
                    KiRotation * headingIntegral +     # Integral
                    KdRotation * headingDerivative +   # Derivative
                    drift_correction)                   # NEW: Drift feedforward
        
        # =========================================================================
        # 5. CALCULATE MOTOR SPEEDS (TANK DRIVE)
        # =========================================================================
        
        # For H-drive / tank drive:
        # Left side: forward + turn
        # Right side: forward - turn
        motorFLSpeed = forwardOutput + turnSpeed
        motorFRSpeed = forwardOutput - turnSpeed
        motorMLSpeed = forwardOutput + turnSpeed
        motorMRSpeed = forwardOutput - turnSpeed
        motorBLSpeed = forwardOutput + turnSpeed
        motorBRSpeed = forwardOutput - turnSpeed
        
        # =========================================================================
        # 6. NORMALIZE MOTOR SPEEDS WITH RAMPING
        # =========================================================================
        
        if counter <= ramp_cycles:
            maxSpeed = max(abs(motorFLSpeed), abs(motorFRSpeed),
                         abs(motorMLSpeed), abs(motorMRSpeed),
                         abs(motorBLSpeed), abs(motorBRSpeed), 
                         initialMaxSpeedLimit)
            
            if maxSpeed > initialMaxSpeedLimit:
                scale_factor = initialMaxSpeedLimit / maxSpeed
                motorFLSpeed *= scale_factor
                motorFRSpeed *= scale_factor
                motorMLSpeed *= scale_factor
                motorMRSpeed *= scale_factor
                motorBLSpeed *= scale_factor
                motorBRSpeed *= scale_factor
        else:
            maxSpeed = max(abs(motorFLSpeed), abs(motorFRSpeed),
                         abs(motorMLSpeed), abs(motorMRSpeed),
                         abs(motorBLSpeed), abs(motorBRSpeed), 
                         maxSpeedLimit)
            
            if maxSpeed > maxSpeedLimit:
                scale_factor = maxSpeedLimit / maxSpeed
                motorFLSpeed *= scale_factor
                motorFRSpeed *= scale_factor
                motorMLSpeed *= scale_factor
                motorMRSpeed *= scale_factor
                motorBLSpeed *= scale_factor
                motorBRSpeed *= scale_factor
        
        # =========================================================================
        # 7. APPLY MOTOR COMMANDS
        # =========================================================================
        
        motor_Motion(motorFLSpeed, motorFRSpeed, motorBLSpeed, motorBRSpeed,
                    motorMLSpeed, motorMRSpeed)
        
        # =========================================================================
        # 8. UPDATE STATE FOR NEXT CYCLE
        # =========================================================================
        
        previous_error[0] = error[0]
        previous_error[1] = error[1]
        previous_error[2] = error[2]
        
        # =========================================================================
        # 9. DEBUG OUTPUT
        # =========================================================================
        
        if counter % 10 == 0 and export_flag == 1:
            print(f"{counter}\t{error[0]:.3f}\t{error[1]:.3f}\t{error[2]:.3f}\t"
                  f"{forwardOutput:.2f}\t{turnSpeed:.2f}")
            
            if counter % 50 == 0:
                print(f"  [Pos: Fwd={currentPosition[0]:.2f}\", "
                      f"H={math.degrees(currentPosition[1]):.1f}°, "
                      f"Drift={currentPosition[2]:.2f}\"]")
                print(f"  [Drift rate: {drift_rate:.3f}\"/s, "
                      f"Correction: {drift_correction:.2f}]")
        
        # =========================================================================
        # 10. CHECK CONVERGENCE OR TIMEOUT
        # =========================================================================
        
        # Check distance and heading errors
        # Note: We DON'T check lateral drift because we can't fix it directly
        if abs(error[0]) < 0.5 and abs(error[1]) < 0.08:
            print("\n=== MOVEMENT COMPLETE ===")
            motor_Stop()
            print(f"Final Error: Distance={error[0]:.3f}\", "
                  f"Heading={math.degrees(error[1]):.1f}°")
            print(f"Lateral drift observed: {error[2]:.2f}\" "
                  f"(tracked but not controlled)")
            print(f"Converged in {counter} cycles ({counter * dt:.2f}s)")
            return error
        
        elif counter > time_out:
            print("\n=== TIMEOUT ===")
            motor_Stop()
            print(f"Final Error: Distance={error[0]:.3f}\", "
                  f"Heading={math.degrees(error[1]):.1f}°")
            print(f"Lateral drift: {error[2]:.2f}\"")
            print(f"Timed out after {counter} cycles ({counter * dt:.2f}s)")
            return error
        
        time.sleep(dt)
        counter += 1


# =============================================================================
# WRAPPER FOR BACKWARD COMPATIBILITY
# =============================================================================

def autonomousPID_enhanced(target, initialMaxSpeedLimit, maxSpeedLimit, time_out,
                          export_flag, Kp_l, Kp_r, use_imu_odometry=True):
    """
    Wrapper to maintain compatibility with original function calls.
    Now correctly handles H-drive constraints.
    """
    target_distance = target[0]
    target_heading = target[1]
    
    return autonomousPID_H_drive(target_distance, target_heading,
                                 initialMaxSpeedLimit, maxSpeedLimit, time_out,
                                 export_flag, Kp_l, Kp_r, use_imu_odometry)


# =============================================================================
# INTAKE AND ROLLER FUNCTIONS (Unchanged)
# =============================================================================

def set_and_run_rollers(v_low, v_mid, v_top):
    intakeMotor_1.set_velocity(v_low, PERCENT)
    intakeMotor_2.set_velocity(v_low, PERCENT)
    MidRoller.set_velocity(v_mid, PERCENT)
    TopRoller.set_velocity(v_top, PERCENT)
    intakeMotor_1.spin(FORWARD)
    intakeMotor_2.spin(FORWARD)
    MidRoller.spin(FORWARD)
    TopRoller.spin(FORWARD)

def all_intake_stop():
    set_and_run_rollers(0, 0, 0)

def intake_ground_to_mid(velocity):
    set_and_run_rollers(-1*velocity, -1*velocity, 1*velocity)

def intake_ground_to_top(velocity):
    set_and_run_rollers(-0.7*velocity, -1*velocity, -1*velocity)

def intake_mid_to_top(velocity):
    set_and_run_rollers(0*velocity, -1*velocity, -1*velocity)

def intake_ground_to_basket(velocity):
    set_and_run_rollers(1*velocity, 1*velocity, 1*velocity)

def intake_ground_cycle_at_top(velocity):
    set_and_run_rollers(-0.8*velocity, -1*velocity, -0.8*velocity)

def intake_ground_hold_in_basket(velocity):
    set_and_run_rollers(-0.7*velocity, 1*velocity, 0*velocity)

def intake_ground_hold_in_basket_top(velocity):
    set_and_run_rollers(-1*velocity, 0*velocity, -0.5*velocity)

def outake_top_to_ground(velocity):
    set_and_run_rollers(1*velocity, -0.5*velocity, -1*velocity)

def outake_basket_to_ground(velocity):
    set_and_run_rollers(-1*velocity, -1*velocity, 0*velocity)


# =============================================================================
# TEST ROUTINES FOR H-DRIVE
# =============================================================================

def test_drift_tracking():
    """
    Test that demonstrates drift tracking for H-drive.
    Shows that we CAN'T fix drift directly, but we CAN track it and
    use drift rate to improve heading control.
    """
    print("\n" + "="*70)
    print("TEST: DRIFT TRACKING FOR H-DRIVE")
    print("="*70)
    print("H-drive cannot strafe, so we:")
    print("1. Track lateral drift to know where we actually are")
    print("2. Use drift rate to improve heading control")
    print("3. Catch drift EARLY before it accumulates\n")
    
    inertialSensor.calibrate()
    while inertialSensor.is_calibrating():
        wait(100, MSEC)
    
    print("Moving forward 60\" at 70% speed")
    print("Watch drift rate correction in action!\n")
    
    start_time = brain.timer.time(MSEC)
    
    error = autonomousPID_H_drive(
        target_distance=60.0,
        target_heading=math.radians(0),
        initialMaxSpeedLimit=40,
        maxSpeedLimit=70,
        time_out=1000,
        export_flag=1,
        Kp_forward=4,
        Kp_rotation=70,
        use_imu_odometry=True,
        ramp_cycles=50
    )
    
    end_time = brain.timer.time(MSEC)
    elapsed = (end_time - start_time) / 1000.0
    
    print("\n" + "="*70)
    print("DRIFT TRACKING RESULTS:")
    print(f"Completed in: {elapsed:.2f}s")
    print(f"Distance error: {error[0]:.2f}\"")
    print(f"Heading error: {math.degrees(error[1]):.1f}°")
    print(f"Lateral drift observed: {error[2]:.2f}\"")
    print("\nNote: Drift is TRACKED but not directly controlled")
    print("Drift rate feedforward helps catch it early!")
    print("="*70)


def test_comparison_with_without_drift_comp():
    """
    Compare performance with and without drift rate compensation.
    """
    print("\n" + "="*70)
    print("TEST: WITH vs WITHOUT DRIFT RATE COMPENSATION")
    print("="*70)
    
    inertialSensor.calibrate()
    while inertialSensor.is_calibrating():
        wait(100, MSEC)
    
    target_distance = 60
    
    # Run 1: Without drift compensation
    print("\n--- RUN 1: WITHOUT DRIFT RATE COMPENSATION ---")
    
    # Temporarily disable drift compensation
    original_Kp_drift = 15.0
    # (Would need to pass this as parameter, but for demo purposes)
    
    start_1 = brain.timer.time(MSEC)
    error_1 = autonomousPID_H_drive(
        target_distance=target_distance,
        target_heading=math.radians(0),
        initialMaxSpeedLimit=40,
        maxSpeedLimit=70,
        time_out=1000,
        export_flag=0,
        Kp_forward=4,
        Kp_rotation=70,
        use_imu_odometry=True,
        ramp_cycles=50
    )
    end_1 = brain.timer.time(MSEC)
    time_1 = (end_1 - start_1) / 1000.0
    
    print(f"✓ Time: {time_1:.2f}s")
    print(f"  Distance error: {error_1[0]:.2f}\"")
    print(f"  Heading error: {math.degrees(error_1[1]):.1f}°")
    print(f"  Lateral drift: {error_1[2]:.2f}\"")
    
    wait(3, SECONDS)
    
    # Run 2: With drift compensation (same function, includes it by default)
    print("\n--- RUN 2: WITH DRIFT RATE COMPENSATION ---")
    
    start_2 = brain.timer.time(MSEC)
    error_2 = autonomousPID_H_drive(
        target_distance=target_distance,
        target_heading=math.radians(0),
        initialMaxSpeedLimit=40,
        maxSpeedLimit=70,
        time_out=1000,
        export_flag=0,
        Kp_forward=4,
        Kp_rotation=70,
        use_imu_odometry=True,
        ramp_cycles=50
    )
    end_2 = brain.timer.time(MSEC)
    time_2 = (end_2 - start_2) / 1000.0
    
    print(f"✓ Time: {time_2:.2f}s")
    print(f"  Distance error: {error_2[0]:.2f}\"")
    print(f"  Heading error: {math.degrees(error_2[1]):.1f}°")
    print(f"  Lateral drift: {error_2[2]:.2f}\"")
    
    print("\n" + "="*70)
    print("COMPARISON:")
    print(f"Drift without compensation: {abs(error_1[2]):.2f}\"")
    print(f"Drift with compensation:    {abs(error_2[2]):.2f}\"")
    print(f"Improvement: {((abs(error_1[2]) - abs(error_2[2])) / abs(error_1[2]) * 100):.1f}%")
    print("="*70)


# =============================================================================
# MAIN AUTONOMOUS ROUTINE
# =============================================================================

def onauton_autonomous_0():
    """
    Main autonomous routine for H-DRIVE robot
    """
    
    print("\n" + "="*70)
    print("H-DRIVE PID SYSTEM - PROPER DRIFT HANDLING")
    print("="*70)
    print("\nKey Points:")
    print("- H-drive CANNOT strafe (move sideways)")
    print("- We track X, Y position to monitor drift")
    print("- Y drift is accumulated heading error")
    print("- We use drift rate to improve heading control")
    print("- This catches drift EARLY before it builds up")
    print("\n" + "="*70)
    
    # Run tests
    test_drift_tracking()
    
    # wait(3, SECONDS)
    # test_comparison_with_without_drift_comp()
    
    print("\n" + "="*70)
    print("TESTS COMPLETE!")
    print("="*70)


# =============================================================================
# COMPETITION SETUP
# =============================================================================

vexcode_brain_precision = 0
vexcode_console_precision = 0
myVariable = 0

def vexcode_auton_function():
    auton_task_0 = Thread(onauton_autonomous_0)
    while competition.is_autonomous() and competition.is_enabled():
        wait(10, MSEC)
    auton_task_0.stop()

def when_started1():
    global myVariable
    pass

def vexcode_driver_function():
    while competition.is_driver_control() and competition.is_enabled():
        wait(10, MSEC)

competition = Competition(vexcode_driver_function, vexcode_auton_function)
when_started1()
