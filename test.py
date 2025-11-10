import pybullet as p, time, numpy as np
p.connect(p.DIRECT)
robot = p.loadURDF("/home/tyler/Desktop/Robot_URDF/robot.urdf", useFixedBase=True)
time.sleep(0.1)

ee = p.getNumJoints(robot) - 1
pos = [0.30, 0.0, 0.45]
orn = p.getQuaternionFromEuler([0, np.pi, 0])

q = p.calculateInverseKinematics(robot, ee, pos, orn)
print("IK success -> first three joints (deg):",
      [round(np.degrees(q[i]),2) for i in range(3)])