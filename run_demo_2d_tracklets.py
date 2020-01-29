#!/usr/bin/env python

import json
import numpy as np
import cv2
import load_calibration
from scipy.spatial.transform import Rotation as R

frame = 0
cam = '0'
seq = '0069'
DISTORTED = False
MOVE_FORWARD = True
base_path = "/media/matthew/WAVELAB_2TB/winter"
calib_path = "/media/matthew/WAVELAB_2TB/winter/calib/";

if DISTORTED:
  path_type = 'raw'
else:
  path_type = 'processed'

img_path = base_path + "/data/" + seq + "/" + path_type + "/image_0" + cam + "/data/" + format(frame, '010') + ".png";
annotations_file = base_path + "/data/" + seq + "/" + '/2d_annotations.json';

# load calibration dictionary
calib = load_calibration.load_calibration(calib_path);

# Projection matrix from camera to image frame
T_IMG_CAM = np.eye(4);
T_IMG_CAM[0:3,0:3] = np.array(calib['CAM0' + cam]['camera_matrix']['data']).reshape(-1, 3);
T_IMG_CAM = T_IMG_CAM[0:3,0:4]; # remove last row

dist_coeffs = np.array(calib['CAM0' + cam]['distortion_coefficients']['data'])

# Load 2d annotations
annotations_data = None
with open(annotations_file) as f:
    annotations_data = json.load(f)

img = cv2.imread(img_path)
img_h, img_w = img.shape[:2]

# Add each box to image
for camera_response in annotations_data[frame]['camera_responses']:
  if camera_response['camera_used'] != int(cam):
    continue;

  for annotation in camera_response['annotations']:
    left = int(annotation['left'])
    top = int(annotation['top'])
    width = int(annotation['width'])
    height = int(annotation['height'])

    if DISTORTED:
      cv2.rectangle(img,(left,top),(left + width,top + height),(0,255,0),thickness=3)
    else:
      pts_uv = np.array([[[left,top]],[[left + width,top + height]]], dtype=np.float32)
      new_pts = cv2.undistortPoints(pts_uv, T_IMG_CAM[0:3,0:3], dist_coeffs, P=T_IMG_CAM[0:3,0:3])
      cv2.rectangle(img,(new_pts[0][0][0],new_pts[0][0][1]),(new_pts[1][0][0],new_pts[1][0][1]),(0,255,0),thickness=3)

cv2.imshow('image',img)
cv2.waitKey(10000)
