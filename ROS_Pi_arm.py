# pi_side.py - Run this on the Raspberry Pi for stepper execution
import rclpy
from rclpy.logging import get_logger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import time
import math
import RPi.GPIO as GPIO

# Stepper config (same as before)
JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
DIR_PINS = [20, 21, 22, 23, 24, 25]
STEP_PINS = [26, 27, 16, 17, 18, 19]
STEPS_PER_REV = 200
RAD_PER_REV = 2 * math.pi
STEPS_PER_RAD = STEPS_PER_REV / RAD_PER_REV
DEFAULT_DELAY = 0.001

current_positions = [0.0] * 6

def setup_gpio():
    GPIO.setmode(GPIO.BCM)
    for dir_pin, step_pin in zip(DIR_PINS, STEP_PINS):
        GPIO.setup(dir_pin, GPIO.OUT)
        GPIO.setup(step_pin, GPIO.OUT)

def cleanup_gpio():
    GPIO.cleanup()

def move_steppers(target_positions, delta_time=0.0):
    global current_positions
    deltas = [target - current for target, current in zip(target_positions, current_positions)]
    steps_list = [int(abs(delta * STEPS_PER_RAD)) for delta in deltas]
    directions = [1 if delta > 0 else 0 for delta in deltas]

    for i, dir_pin in enumerate(DIR_PINS):
        GPIO.output(dir_pin, directions[i])

    max_steps = max(steps_list) if steps_list else 0
    if max_steps == 0:
        current_positions = target_positions[:]
        return

    if delta_time > 0:
        cycle_delay = delta_time / max_steps
    else:
        cycle_delay = DEFAULT_DELAY * max_steps

    errors = [0.0] * 6
    increments = [steps / max_steps if max_steps > 0 else 0 for steps in steps_list]

    for macro_step in range(max_steps):
        start_cycle = time.time()
        for i in range(6):
            errors[i] += increments[i]
            if errors[i] >= 1.0:
                GPIO.output(STEP_PINS[i], 1)
                time.sleep(0.0001)
                GPIO.output(STEP_PINS[i], 0)
                errors[i] -= 1.0

        elapsed = time.time() - start_cycle
        if cycle_delay > elapsed:
            time.sleep(cycle_delay - elapsed)

    actual_steps = [round(inc * max_steps) for inc in increments]
    for i in range(6):
        delta_rad = (actual_steps[i] * (1 if directions[i] else -1)) / STEPS_PER_RAD
        current_positions[i] += delta_rad

def trajectory_callback(msg: JointTrajectory):
    logger = get_logger("pi_stepper_executor")
    if msg.joint_names != JOINT_NAMES:
        logger.error("Joint names mismatch!")
        return

    setup_gpio()
    prev_time = 0.0
    for point in msg.points:
        target_pos = point.positions  # Assume order matches JOINT_NAMES
        delta_time = (point.time_from_start.sec + point.time_from_start.nanosec * 1e-9) - prev_time
        move_steppers(target_pos, delta_time)
        # Publish current state for RViz
        js = JointState()
        js.header.stamp = rclpy.clock.Clock().now().to_msg()
        js.name = JOINT_NAMES
        js.position = current_positions
        joint_state_pub.publish(js)
        prev_time += delta_time

    cleanup_gpio()
    logger.info("Trajectory executed on steppers")

def main():
    global joint_state_pub
    rclpy.init()
    node = rclpy.create_node('pi_stepper_executor')
    joint_state_pub = node.create_publisher(JointState, '/joint_states', 10)
    node.create_subscription(JointTrajectory, '/joint_trajectory', trajectory_callback, 10)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()