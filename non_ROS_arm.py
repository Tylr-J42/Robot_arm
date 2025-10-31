
import time
import math
import RPi.GPIO as GPIO
import numpy as np

# Config
JOINT_NAMES = ['1st', '2nd', '3rd', '4th', '5th', '6th']  # URDF virtual joints
DIR_PINS = [31, 36, 38, 33, 23, 22]
STEP_PINS = [32, 37, 40, 35, 21, 29]
EN_PIN = 11
STEPS_PER_REV = 200 * 8 # steppers at 1/8 microstepping
RAD_PER_REV = 2 * math.pi
GEAR_RATIOS = [150.0/15.0, 33.0/13.0*19.0, -24.0/16.0*19.0, 100.0/14.0, 80.0/12.0*25.0/13.0, -80.0/12.0*25.0/13.0]  # For motor1-4, then motor5,6 (wrist shared GR)
WRIST_DIFF_FACTOR = 2 * (25.0 / 13.0)  # E.g., pitch = (m5 + m6)/2, yaw = (m5 - m6)/2
STEP_FREQUENCY = 1000

def convert_virtual_to_motor(virtual_positions):
    """Convert [j1..j4, pitch, yaw] to motor1..6 positions (radians)"""
    j1, j2, j3, j4, pitch, yaw = virtual_positions
    m1 = j1 * GEAR_RATIOS[0]
    m2 = j2 * GEAR_RATIOS[1]
    m3 = j3 * GEAR_RATIOS[2]
    m4 = j4 * GEAR_RATIOS[3]
    m5 = ((pitch + yaw*2) * GEAR_RATIOS[4] / 2) # motor5 = (pitch + yaw)/2 * GR
    m6 = ((pitch - yaw*2) * GEAR_RATIOS[5] / 2)  # motor6 = (pitch - yaw)/2 * GR

    return [m1, m2, m3, m4, m5, m6]

def convert_motor_to_virtual(motor_positions):
    """Convert motor1..6 to [j1..j4, pitch, yaw] for /joint_states"""
    m1, m2, m3, m4, m5, m6 = motor_positions
    j1 = m1 / GEAR_RATIOS[0]
    j2 = m2 / GEAR_RATIOS[1]
    j3 = m3 / GEAR_RATIOS[2]
    j4 = m4 / GEAR_RATIOS[3]
    pitch = (m5 + m6) * WRIST_DIFF_FACTOR / GEAR_RATIOS[4]  # pitch = (m5 + m6)/2
    yaw = (m5 - m6) * WRIST_DIFF_FACTOR / GEAR_RATIOS[5]   # yaw = (m5 - m6)/2
    return [j1, j2, j3, j4, pitch, yaw]

# starting down position
# current_motor_positions = convert_virtual_to_motor([0.0,
#                                                     -46.0*math.pi/180.0,
#                                                     -93.0*math.pi/180.0,
#                                                     -360.0*math.pi/180.0,
#                                                     97.0*math.pi/180.0,
#                                                     0.0*math.pi/180.0])  # In motor radians

# "neutral" position
neutral_position = convert_virtual_to_motor([0.0,
                                                    0.0*math.pi/180.0,
                                                    0.0*math.pi/180.0,
                                                    0.0*math.pi/180.0,
                                                    0.0*math.pi/180.0,
                                                    0.0*math.pi/180.0])  # In motor radians

current_motor_positions = neutral_position

def setup_gpio():
    GPIO.setmode(GPIO.BOARD)

    GPIO.setup(DIR_PINS[0], GPIO.OUT)
    GPIO.setup(DIR_PINS[1], GPIO.OUT)
    GPIO.setup(DIR_PINS[2], GPIO.OUT)
    GPIO.setup(DIR_PINS[3], GPIO.OUT)
    GPIO.setup(DIR_PINS[4], GPIO.OUT)
    GPIO.setup(DIR_PINS[5], GPIO.OUT)
    GPIO.setup(STEP_PINS[0], GPIO.OUT)
    GPIO.setup(STEP_PINS[1], GPIO.OUT)
    GPIO.setup(STEP_PINS[2], GPIO.OUT)
    GPIO.setup(STEP_PINS[3], GPIO.OUT)
    GPIO.setup(STEP_PINS[4], GPIO.OUT)
    GPIO.setup(STEP_PINS[5], GPIO.OUT)

    GPIO.setup(EN_PIN, GPIO.OUT)

def cleanup_gpio():
    GPIO.cleanup()

def move_steppers(target_motor_positions):
    """Move steppers to target motor positions."""
    global current_motor_positions
    deltas = [t - c for t, c in zip(target_motor_positions, current_motor_positions)]
    steps_list = [int(abs(d * STEPS_PER_REV / RAD_PER_REV)) for d in deltas]
    directions = [1 if d >= 0 else -1 for d in deltas]

    # Calculate pulse timing
    # period = 1.0 / STEP_FREQUENCY  # Time per step
    print(f"Moving to steps: {steps_list}, Directions: {directions}")

    count = 0
    prevtime = 0

    for i in range(0,len(DIR_PINS)):
            if(directions[i]>0):
                GPIO.output(DIR_PINS[i], GPIO.HIGH)
            else:
                GPIO.output(DIR_PINS[i], GPIO.LOW)
            print(directions[i])

    while count<=max(steps_list)*2:
        current_time = time.perf_counter()

        if(current_time-prevtime >= 1/(STEP_FREQUENCY*2)):
            if(count%2 == 1):
                for i in range(0, len(STEP_PINS)):
                    if(steps_list[i]*2>count):
                        GPIO.output(STEP_PINS[i], GPIO.HIGH)

            else:
                for i in STEP_PINS:
                    GPIO.output(i, GPIO.LOW)

            count = count + 1
            prevtime = current_time

    current_motor_positions = target_motor_positions


def main():
    setup_gpio()
    print("moving now")
    move_steppers(
        convert_virtual_to_motor([0.0,
                                5.0*math.pi/180.0,
                                -72.0*math.pi/180.0,
                                180.0*math.pi/180.0,
                                78.0*math.pi/180.0,
                                90.0*math.pi/180.0])
    )
    print(current_motor_positions)
    print("done moving")
    cleanup_gpio()

if __name__ == "__main__":
    main()