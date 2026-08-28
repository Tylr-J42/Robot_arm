import numpy as np

# for the RPi v3 wide camera
'''
FOCAL_LEN_PIXELS = 621.5827338
camera_matrix = np.array([[976.16482142,   0.,         771.05155174],
                [  0.,         974.47104393, 408.52081949],
                [  0.,           0.,           1.        ]])
'''

#for the Arducam OV9872 Camera orange
# camera_matrix = np.array([
#     [912.91589296,   0.,         705.22436474],
#     [  0.,         914.05652473, 364.61560638],
#     [  0. ,          0.,           1.,        ]
#     ])

# for rocketfish HD webcam
camera_matrix = np.array([
    [1007.02419665, 0.00000000, 575.93319816],
    [0.00000000, 1007.07176499, 357.68908087],
    [0.00000000, 0.00000000, 1.00000000],
    ])

CAMERA_SETTINGS = {
    "fourcc": "YUYV",   # exposure is a no-op in MJPG on the Rocketfish
    "fps": 10.0,
    # At 10 fps the frame period is 100 ms, i.e. 1000 in this control's 100us
    # units, so anything above that is clamped. Values from ~35 up to 3000
    # measure identically -- exposure is NOT the brightness lever on this
    # camera, gamma is.
    "exposure": 800,    # tune with tune_camera.py
    "contrast": None,    # under 60 clamps to 60, this camera's worst setting
    # Below ~350 this camera returns a blown-out first frame (mean ~250) and
    # then settles far too dark (mean ~56) -- the "sometimes overexposed,
    # usually dark" failure. 450 measures a stable mean ~151 across reopens.
    # NB: open_camera did not write gamma at all until this was found, so any
    # value set here before then was silently ignored.
    "gamma": 450,
    "sharpness": 0,
}


# for rocketfish HD webcam dist
dist = np.array([[0.01040809, -0.13017203, 0.00133320, -0.00613676, 0.14209619]])

# for arducam OV9872 camrea
# dist = np.array([[ 0.02808095,  0.01822137, -0.00086566,  0.00161964, -0.19848367]])

# from Camera_Calibration.py for RPi v3 wide camera
#dist = np.array([-0.04790604,  0.08489533, -0.00387366,  0.00616192, -0.03875398])

camera_res = (1280, 720)

# for RPi camera v3 wide
#camera_res = (1536, 864)


# for RPi Camera v3 Wide
#VERTICAL_FOV = 67
#HORIZONTAL_FOV = 102

#X = forward/back, Y = left/right, Z = up/down
# roll, pitch, yaw (CW positive)
# x,y,z,rx,ry,rz
