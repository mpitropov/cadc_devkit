#!/usr/bin/env python

import json
import numpy as np
import cv2
import load_calibration
from scipy.spatial.transform import Rotation as R

frame = 26
cam = '0'
seq = '0010'
DISTORTED = False
MOVE_FORWARD = False
# BASE = "/media/matthew/WAVELAB_2TB/winter/data/"
BASE = "/media/matthew/MOOSE-4TB/2019_02_27/"
CALIB_BASE = "/media/matthew/WAVELAB_2TB/winter/"

if DISTORTED:
  path_type = 'raw'
else:
  path_type = 'processed'

lidar_path = BASE + seq + "/" + path_type + "/lidar_points/data/" + format(frame, '010') + ".bin";
calib_path = CALIB_BASE + "calib/";
img_path = BASE + seq + "/" + path_type + "/image_0" + cam + "/data/" + format(frame, '010') + ".png";

annotations_file = BASE + seq + "/3d_ann.json";

# Load 3d annotations
annotations_data = None
with open(annotations_file) as f:
    annotations_data = json.load(f)

calib = load_calibration.load_calibration(calib_path);

# Projection matrix from camera to image frame
T_IMG_CAM = np.eye(4);
T_IMG_CAM[0:3,0:3] = np.array(calib['CAM0' + cam]['camera_matrix']['data']).reshape(-1, 3);
T_IMG_CAM = T_IMG_CAM[0:3,0:4]; # remove last row

T_CAM_LIDAR = np.linalg.inv(np.array(calib['extrinsics']['T_LIDAR_CAM0' + cam]));

T_IMG_LIDAR = np.matmul(T_IMG_CAM, T_CAM_LIDAR);

img = cv2.imread(img_path)
img_h, img_w = img.shape[:2]

# Add each cuboid to image
for cuboid in annotations_data[frame]['cuboids']:
  T_Lidar_Cuboid = np.eye(4);
  T_Lidar_Cuboid[0:3,0:3] = R.from_euler('z', cuboid['yaw'], degrees=False).as_dcm();
  T_Lidar_Cuboid[0][3] = cuboid['position']['x'];
  T_Lidar_Cuboid[1][3] = cuboid['position']['y'];
  T_Lidar_Cuboid[2][3] = cuboid['position']['z'];

  #T_Lidar_Cuboid[0][3] = -T_Lidar_Cuboid[0][3];

  # if (cuboid['label'] != 'Truck'):
  #   continue;
  # if (cuboid['attributes']['truck_type'] != 'Semi_Truck'):
  #   continue;
  # print(cuboid['yaw'])
  # print(cuboid)
  # print(T_Lidar_Cuboid)
  width = cuboid['dimensions']['x'];
  length = cuboid['dimensions']['y'];
  height = cuboid['dimensions']['z'];
  radius = 3

  # Create circle in middle of the cuboid
  tmp = np.matmul(T_CAM_LIDAR, T_Lidar_Cuboid);
  if tmp[2][3] < 0: # Behind camera
    continue;
  test = np.matmul(T_IMG_CAM, tmp);
  x = int(test[0][3]/test[2][3]);
  y = int(test[1][3]/test[2][3]);
  cv2.circle(img, (x,y), radius, [0, 0, 255], thickness=2, lineType=8, shift=0);

  front_right_bottom = np.array([[1,0,0,length/2],[0,1,0,-width/2],[0,0,1,-height/2],[0,0,0,1]]);
  front_right_top = np.array([[1,0,0,length/2],[0,1,0,-width/2],[0,0,1,height/2],[0,0,0,1]]);
  front_left_bottom = np.array([[1,0,0,length/2],[0,1,0,width/2],[0,0,1,-height/2],[0,0,0,1]]);
  front_left_top = np.array([[1,0,0,length/2],[0,1,0,width/2],[0,0,1,height/2],[0,0,0,1]]);

  back_right_bottom = np.array([[1,0,0,-length/2],[0,1,0,-width/2],[0,0,1,-height/2],[0,0,0,1]]);
  back_right_top = np.array([[1,0,0,-length/2],[0,1,0,-width/2],[0,0,1,height/2],[0,0,0,1]]);
  back_left_bottom = np.array([[1,0,0,-length/2],[0,1,0,width/2],[0,0,1,-height/2],[0,0,0,1]]);
  back_left_top = np.array([[1,0,0,-length/2],[0,1,0,width/2],[0,0,1,height/2],[0,0,0,1]]);

  # Project to image
  tmp = np.matmul(T_CAM_LIDAR, np.matmul(T_Lidar_Cuboid, front_right_bottom));
  if tmp[2][3] < 0:
    continue;
  f_r_b = np.matmul(T_IMG_CAM, tmp);
  tmp = np.matmul(T_CAM_LIDAR, np.matmul(T_Lidar_Cuboid, front_right_top));
  if tmp[2][3] < 0:
    continue;
  f_r_t = np.matmul(T_IMG_CAM, tmp);
  tmp = np.matmul(T_CAM_LIDAR, np.matmul(T_Lidar_Cuboid, front_left_bottom));
  if tmp[2][3] < 0:
    continue;
  f_l_b = np.matmul(T_IMG_CAM, tmp);
  tmp = np.matmul(T_CAM_LIDAR, np.matmul(T_Lidar_Cuboid, front_left_top));
  if tmp[2][3] < 0:
    continue;
  f_l_t = np.matmul(T_IMG_CAM, tmp);

  tmp = np.matmul(T_CAM_LIDAR, np.matmul(T_Lidar_Cuboid, back_right_bottom));
  if tmp[2][3] < 0:
    continue;
  b_r_b = np.matmul(T_IMG_CAM, tmp);
  tmp = np.matmul(T_CAM_LIDAR, np.matmul(T_Lidar_Cuboid, back_right_top));
  if tmp[2][3] < 0:
    continue;
  b_r_t = np.matmul(T_IMG_CAM, tmp);
  tmp = np.matmul(T_CAM_LIDAR, np.matmul(T_Lidar_Cuboid, back_left_bottom));
  if tmp[2][3] < 0:
    continue;
  b_l_b = np.matmul(T_IMG_CAM, tmp);
  tmp = np.matmul(T_CAM_LIDAR, np.matmul(T_Lidar_Cuboid, back_left_top));
  if tmp[2][3] < 0:
    continue;
  b_l_t = np.matmul(T_IMG_CAM, tmp);

  # Make sure the 
  # Remove z
  f_r_b_coord = (int(f_r_b[0][3]/f_r_b[2][3]), int(f_r_b[1][3]/f_r_b[2][3]));
  f_r_t_coord = (int(f_r_t[0][3]/f_r_t[2][3]), int(f_r_t[1][3]/f_r_t[2][3]));
  f_l_b_coord = (int(f_l_b[0][3]/f_l_b[2][3]), int(f_l_b[1][3]/f_l_b[2][3]));
  f_l_t_coord = (int(f_l_t[0][3]/f_l_t[2][3]), int(f_l_t[1][3]/f_l_t[2][3]));
  if f_r_b_coord[0] < 0 or f_r_b_coord[0] > img_w or f_r_b_coord[1] < 0 or f_r_b_coord[1] > img_h:
    continue;
  if f_r_t_coord[0] < 0 or f_r_t_coord[0] > img_w or f_r_t_coord[1] < 0 or f_r_t_coord[1] > img_h:
    continue;
  if f_l_b_coord[0] < 0 or f_l_b_coord[0] > img_w or f_l_b_coord[1] < 0 or f_l_b_coord[1] > img_h:
    continue;
  if f_l_t_coord[0] < 0 or f_l_t_coord[0] > img_w or f_l_t_coord[1] < 0 or f_l_t_coord[1] > img_h:
    continue;

  b_r_b_coord = (int(b_r_b[0][3]/b_r_b[2][3]), int(b_r_b[1][3]/b_r_b[2][3]));
  b_r_t_coord = (int(b_r_t[0][3]/b_r_t[2][3]), int(b_r_t[1][3]/b_r_t[2][3]));
  b_l_b_coord = (int(b_l_b[0][3]/b_l_b[2][3]), int(b_l_b[1][3]/b_l_b[2][3]));
  b_l_t_coord = (int(b_l_t[0][3]/b_l_t[2][3]), int(b_l_t[1][3]/b_l_t[2][3]));
  if b_r_b_coord[0] < 0 or b_r_b_coord[0] > img_w or b_r_b_coord[1] < 0 or b_r_b_coord[1] > img_h:
    continue;
  if b_r_t_coord[0] < 0 or b_r_t_coord[0] > img_w or b_r_t_coord[1] < 0 or b_r_t_coord[1] > img_h:
    continue;
  if b_l_b_coord[0] < 0 or b_l_b_coord[0] > img_w or b_l_b_coord[1] < 0 or b_l_b_coord[1] > img_h:
    continue;
  if b_l_t_coord[0] < 0 or b_l_t_coord[0] > img_w or b_l_t_coord[1] < 0 or b_l_t_coord[1] > img_h:
    continue;

  # Draw  12 lines
  # Front
  cv2.line(img, f_r_b_coord, f_r_t_coord, [0, 0, 255], thickness=2, lineType=8, shift=0);
  cv2.line(img, f_r_b_coord, f_l_b_coord, [0, 0, 255], thickness=2, lineType=8, shift=0);
  cv2.line(img, f_l_b_coord, f_l_t_coord, [0, 0, 255], thickness=2, lineType=8, shift=0);
  cv2.line(img, f_l_t_coord, f_r_t_coord, [0, 0, 255], thickness=2, lineType=8, shift=0);
  # back
  cv2.line(img, b_r_b_coord, b_r_t_coord, [0, 0, 100], thickness=2, lineType=8, shift=0);
  cv2.line(img, b_r_b_coord, b_l_b_coord, [0, 0, 100], thickness=2, lineType=8, shift=0);
  cv2.line(img, b_l_b_coord, b_l_t_coord, [0, 0, 100], thickness=2, lineType=8, shift=0);
  cv2.line(img, b_l_t_coord, b_r_t_coord, [0, 0, 100], thickness=2, lineType=8, shift=0);
  # connect front to back
  cv2.line(img, f_r_b_coord, b_r_b_coord, [0, 0, 150], thickness=2, lineType=8, shift=0);
  cv2.line(img, f_r_t_coord, b_r_t_coord, [0, 0, 150], thickness=2, lineType=8, shift=0);
  cv2.line(img, f_l_b_coord, b_l_b_coord, [0, 0, 150], thickness=2, lineType=8, shift=0);
  cv2.line(img, f_l_t_coord, b_l_t_coord, [0, 0, 150], thickness=2, lineType=8, shift=0);

  print(cuboid)
  print(f_r_b_coord)
  print(f_r_t_coord)
  print(f_l_b_coord)
  print(f_l_t_coord)

  print(b_r_b_coord)
  print(b_r_t_coord)
  print(b_l_b_coord)
  print(b_l_t_coord)

  #break;

cv2.imshow('image',img)
# cv2.imwrite("test.png", img)
cv2.waitKey(10000)
