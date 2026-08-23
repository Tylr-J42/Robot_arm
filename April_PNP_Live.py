#!/usr/bin/env python

import time
import cv2
import dt_apriltags
import numpy as np
from math import sqrt
from math import pi
import math
import argparse
from pathlib import Path

import constants
from tag_map import load_tag_map, build_correspondences

from wpimath.geometry import Translation3d
from wpimath.geometry import Transform3d
from wpimath.geometry import Pose3d
from wpimath.geometry import Rotation3d

RAD2DEG = 180*pi

# To show display of camera feed add --display in terminal when running script. To set IP address use --ip_add.
parser = argparse.ArgumentParser(description="Select display")
parser.add_argument("--display", action='store_true', help="enable a display of the camera")
#parser.add_argument("--high_res", action='store_true', help="high resolution camera capture")
parser.add_argument("--pose_estimation", action='store_true', help="estimate pose based on detected tags")
args = parser.parse_args()

'''
if args.high_res:
    FOCAL_LEN_PIXELS = 991.5391539
    constants.camera_matrix = np.array([[FOCAL_LEN_PIXELS, 0.00000000, 528.420369],
    [0.00000000, FOCAL_LEN_PIXELS, 342.737594],
    [0.00000000, 0.00000000, 1.00000000]])
    constants.dist = np.array([[ 2.52081760e-01, -1.34794418e+00,  1.24975695e-03, -7.77510823e-04,
    2.29608398e+00]])
    camera_res = (2304, 1296)
'''
# x_size = size of the target tags in cm
x_size=0.03

# table_size = size of the tags on the table making the coordinate system relative to the robot.
table_tag_size=0.03

# 3d object array. The points of the 3d april tag that coresponds to tag_points which we detect
objp = np.array([[0,0,0], [-x_size/2, -x_size/2, 0], [x_size/2, -x_size/2, 0], [x_size/2, x_size/2, 0], [-x_size/2, x_size/2, 0]], dtype=np.float32)
# 2d axis array points for drawing cube overlay
axis = np.array([
    [x_size/2, x_size/2, 0], 
    [-x_size/2, x_size/2, 0], 
    [-x_size/2, -x_size/2, 0], 
    [x_size/2, -x_size/2, 0], 
    [x_size/2, x_size/2, x_size], 
    [-x_size/2, x_size/2, x_size], 
    [-x_size/2, -x_size/2, x_size], 
    [x_size/2, -x_size/2, x_size]],
    dtype=np.float32)

# FPS = 0

# set camera parameters
cam = cv2.VideoCapture(0)

cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
cam.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

def tag_corners(tag_coords):
    corners = []

    for i in range(len(constants.tag_coords)):
        x = tag_coords[i][1]
        y = tag_coords[i][2]
        z = tag_coords[i][3]
        z_rotation = tag_coords[i][4]

        coordinates = [[], [], [] ,[], []]

        x_offset = (x_size/2)*math.cos(math.radians(z_rotation))
        y_offset = (x_size/2)*math.sin(math.radians(z_rotation))
        coordinates[0] = tag_coords[i][0]
        coordinates[1] = [x-x_offset, y+y_offset, z+x_size/2]
        coordinates[2] = [x+x_offset, y+y_offset, z+x_size/2]
        coordinates[3] = [x+x_offset, y+y_offset, z-x_size/2]
        coordinates[4] = [x-x_offset, y+y_offset, z-x_size/2]

        corners.append(coordinates)

    return corners

# def getTXTY(tvecX, tvecY, tvecZ):
    
#     robotToCamera = Pose3d(Translation3d(12.95, 2.19, 0), Rotation3d(np.deg2rad(180), np.deg2rad(-30), np.deg2rad(-35)))

#     tagPose = Translation3d(tvecZ, tvecX, tvecY)

#     robotPose = robotToCamera.transformBy(Transform3d(tagPose, Rotation3d()))
    
#     tx = np.rad2deg(math.atan(robotPose.Y()/robotPose.X()))
#     ty = np.rad2deg(math.atan(robotPose.Z()/robotPose.X()))

#     return tx, ty

# constants.tag_coords no longer exists; tag world geometry now comes from
# tags_base_assy.yaml via tag_map.py, which is the single place the corner
# ordering convention is defined.
# field_tag_coords = tag_corners(constants.tag_coords)

TAG_MAP_PATH = Path(__file__).resolve().parent / "tags_base_assy.yaml"
TAG_MAP = load_tag_map(TAG_MAP_PATH)

# The tags whose corners are pooled into one multi-tag PnP solve.
SOLVE_TAG_IDS = frozenset({0, 1, 2, 3})

# create overlay on camera feed
def display_features(det, image, imgpts, totalDist):
    # making red lines around fiducial
    for i in range(0,4):
        f = i+1
        if f>3: f=0
        cv2.line(image, (int(det.corners[i][0]), int(det.corners[i][1])), (int(det.corners[f][0]), int(det.corners[f][1])), (0,0,255), 3)

    imgpts = np.int32(imgpts).reshape(-1,2)
    # draw ground floor in green
    #image = cv2.drawContours(image, [imgpts[:4]],-1,(0,255,0),-3)
    # draw pillars in blue color
    for i,j in zip(range(4),range(4,8)):
        image = cv2.line(image, tuple(imgpts[i]), tuple(imgpts[j]),(255),3)
    # draw top layer in red color
    image = cv2.drawContours(image, [imgpts[4:]],-1,(0,0,255),3)
    image = cv2.putText(image, "#"+str(det.tag_id)+", "+str(round(totalDist, 4))+"in", (int(det.center[0]),int(det.center[1])+25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 2, cv2.LINE_AA)
    return image

def tag_by_id(dets, *, min_margin=30.0, max_hamming=0):
    """{tag_id: Detection}, dropping low-confidence and duplicate-id hits."""
    good = [d for d in dets
            if d.decision_margin >= min_margin and d.hamming <= max_hamming]
    seen, dupes = {}, set()
    for d in good:
        if d.tag_id in seen:
            dupes.add(d.tag_id)
        seen[d.tag_id] = d
    for tag_id in dupes:
        seen.pop(tag_id, None)
    return seen

class detection_pipeline:

    def __init__(self):
        # setting up apriltag detection. Make sure this is OUTSIDE the loop next time
        self.detector = dt_apriltags.Detector(
                            searchpath=['apriltags'],
                            families='tag36h11',
                            nthreads=6,
                            quad_decimate=2,
                            quad_sigma=0,
                            refine_edges=1,
                            decode_sharpening=0.0,
                            debug=0
                            )

        # self.counter = 0

    def PNP_detection(self):
        self.detector
        # self.counter = counter

        # main vision processing code
        time.sleep(0.1)
        while True:
            # frame_start = time.time()
            ret, image = cam.read()
            data_array = []
            tags_detected = []

            #detecting april tags
            tagFrame = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            
            output = self.detector.detect(tagFrame)

            #print(output)

            tags = tag_by_id(output)

            # Multi-tag PnP. Pool the corners of tags 0-3 into ONE solve rather
            # than solving each tag independently: 16 point pairs rigidly tied
            # to a single frame is far better conditioned than four 4-point
            # solves, and tag 3 sits out of the table plane, which is what
            # removes the planar two-fold ambiguity.
            if SOLVE_TAG_IDS <= tags.keys():
                observations = {tid: tags[tid].corners for tid in SOLVE_TAG_IDS}
                corr = build_correspondences(observations, TAG_MAP, warn=False)

                ok, rvecs, tvecs = cv2.solvePnP(
                    corr.obj_pts,
                    corr.img_pts,
                    constants.camera_matrix,
                    constants.dist,
                    flags=cv2.SOLVEPNP_SQPNP,
                )

                if ok:
                    # solvePnP returns the base_assy -> camera transform.
                    # Invert it for the camera's pose in the robot frame.
                    R, _ = cv2.Rodrigues(rvecs)
                    cam_in_base = (-R.T @ tvecs).ravel()

                    projected, _ = cv2.projectPoints(
                        corr.obj_pts, rvecs, tvecs,
                        constants.camera_matrix, constants.dist,
                    )
                    residuals = projected.reshape(-1, 2) - corr.img_pts
                    rms = float(np.sqrt(np.mean(np.sum(residuals**2, axis=1))))

                    print(
                        f"cam xyz (m): {cam_in_base.round(4)}  "
                        f"tags: {corr.n_tags}  pts: {corr.n_points}  rms: {rms:.2f} px"
                    )

            


            # for det in output:

                # if the confidence is less than 30% exclude the tag from being processed.
                # if det.decision_margin>30:
                #     # points of the tag to be tracked
                #     tag_points = np.array([[det.corners[0][0], det.corners[0][1]], [det.corners[1][0], det.corners[1][1]], [det.corners[2][0], det.corners[2][1]], [det.corners[3][0], det.corners[3][1]]], dtype=np.float32)

                #     ret,rvecs, tvecs = cv2.solvePnP(objp, tag_points, constants.camera_matrix, constants.dist, flags=0)

                #     # making translation and rotation vectors into a format good for networktables
                #     tvecDist = tvecs.tolist()
                #     rvecRads = rvecs.tolist()

                #     rvecDeg = (rvecs*RAD2DEG).tolist()
                #     for i in range(0,len(tvecDist)):
                #         tvecDist[i] = float(tvecDist[i][0])
                #     for i in range(0,len(rvecDeg)):
                #         rvecDeg[i] = float(rvecDeg[i][0])

                #     totalDist = sqrt((tvecDist[0]**2)+(tvecDist[1]**2)+(tvecDist[2]**2))

                #     # only show display if you use --display for argparse
                #     if args.display:
                #         imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, constants.camera_matrix, constants.dist)
                #         image = display_features(det, image, imgpts, totalDist)

                #     data_array.append((det.tag_id, tvecDist, rvecDeg, totalDist))
                #     tags_detected.append(det.tag_id)
            
            # for i in range(len(data_array)):
                # tx, ty = getTXTY(data_array[i][1][0], data_array[i][1][1], data_array[i][1][2])
                
            #Showing image. use --display to show image
            # if args.display:
            #     # image = cv2.putText(image, "FPS: "+str(round(FPS, 4)), (25,440), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2, cv2.LINE_AA)
            #     cv2.imshow("Frame", image)

            #     key = cv2.waitKey(1) & 0xFF
            #     if key ==ord("q"):
            #         break

            # counter = counter+1
            # if(counter==25):
            #     # frame rate for performance
            #     FPS = (1/(time.time()-frame_start))
            #     counter = 0
            #     print(FPS)