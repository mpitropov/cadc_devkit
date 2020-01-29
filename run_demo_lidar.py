#!/usr/bin/env python

import numpy as np
import cv2
import load_calibration
from lidar_utils import lidar_utils

frame = 90
cam = '0'
seq = '0027'
DISTORTED = False
MOVE_FORWARD = True
BASE = "/media/matthew/WAVELAB_2TB/winter/"

if DISTORTED:
  path_type = 'raw'
else:
  path_type = 'processed'

lidar_path = BASE + "data/" + seq + "/" + path_type + "/lidar_points/data/" + format(frame, '010') + ".bin";
calib_path = BASE + "calib/";
img_path = BASE + "data/" + seq + "/" + path_type + "/image_0" + cam + "/data/" + format(frame, '010') + ".png";

# load calibration dictionary
calib = load_calibration.load_calibration(calib_path);

# Projection matrix from camera to image frame
T_IMG_CAM = np.eye(4);
T_IMG_CAM[0:3,0:3] = np.array(calib['CAM0' + cam]['camera_matrix']['data']).reshape(-1, 3);
T_IMG_CAM = T_IMG_CAM[0:3,0:4]; # remove last row

T_CAM_LIDAR = np.linalg.inv(np.array(calib['extrinsics']['T_LIDAR_CAM0' + cam]));

dist_coeffs = np.array(calib['CAM0' + cam]['distortion_coefficients']['data'])

lidar_utils_obj = lidar_utils(T_CAM_LIDAR);

while True:
  print(frame)
  # read image
  img = cv2.imread(img_path)

  # Project points onto image
  img = lidar_utils_obj.project_points(img, lidar_path, T_IMG_CAM, T_CAM_LIDAR, dist_coeffs, DISTORTED);
  # cv2.imwrite("test.png", img)

  cv2.imshow('image',img)
  cv2.waitKey(1000)

  if MOVE_FORWARD:
    frame += 1;
    lidar_path = BASE + "data/" + seq + "/" + path_type + "/lidar_points/data/" + format(frame, '010') + ".bin"
    img_path = BASE + "data/" + seq + "/" + path_type + "/image_0" + cam + "/data/" + format(frame, '010') + ".png"
    img = cv2.imread(img_path)
