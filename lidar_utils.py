#!/usr/bin/env python

import numpy as np
import cv2

class lidar_utils:
  def __init__(self, T_CAM_LIDAR):
    self.T_CAM_LIDAR = T_CAM_LIDAR;
    print("init lidar utils")

  def project_points(self, img, lidar_path, T_IMG_CAM, T_CAM_LIDAR, dist_coeffs, DISTORTED):
    self.T_CAM_LIDAR = T_CAM_LIDAR;

    scan_data = np.fromfile(lidar_path, dtype=np.float32);

    # 2D array where each row contains a point [x, y, z, intensity]
    lidar = scan_data.reshape((-1, 4));

    # Get height and width of the image
    h, w = img.shape[:2]

    projected_points = [];

    [rows, cols] = lidar.shape;
    # print(lidar[0,:])
    # print(lidar[1,:])
    # print(lidar[1,0:3])

    for i in range(rows):
      # print(lidar[i,:])
      p = np.array([0.0, 0.0, 0.0, 1.0]);
      p[0:3] = lidar[i,0:3];
      # print("p",p);
      projected_p =  np.matmul(self.T_CAM_LIDAR, p.transpose());
      if projected_p[2] < 2: # arbitrary cut off
        continue;
      projected_points.append([projected_p[0], projected_p[1], projected_p[2]]);

    #print("projected_points", projected_points)

    # Send [x, y, z] and Transform
    projected_points_np = np.array(projected_points)
    image_points = self.project(projected_points_np, T_IMG_CAM, dist_coeffs, DISTORTED);
    # print("image_points")
    # print(image_points)

    radius = 0

    [rows, cols] = projected_points_np.shape;

    NUM_COLOURS = 7;
    rainbow = [
      [0, 0, 255], # Red
      [0, 127, 255], # Orange
      [0, 255, 255], # Yellow
      [0, 255, 0], # Green
      [255, 0, 0], # Blue
      [130, 0, 75], # Indigo
      [211, 0, 148] # Violet
    ];

    for i in range(rows):
      colour = int(NUM_COLOURS*(projected_points_np[i][2]/70));
      x = int(image_points[i][0])
      y = int(image_points[i][1])
      if x < 0 or x > w - 1 or y < 0 or y > h - 1:
        continue;
      if colour > NUM_COLOURS-1:
        continue;

      cv2.circle(img, (x,y), radius, rainbow[colour], thickness=2, lineType=8, shift=0);

    return img;

  def project(self, p_in, T_IMG_CAM, dist_coeffs, DISTORTED):
    p_out = []
    [rows, cols] = p_in.shape;

    for i in range(rows):
      # print("p_in[i]", p_in[i])
      point = np.array([0.0, 0.0, 0.0, 1.0]);
      # print("p_in[i][0]",p_in[i][0])
      point[0:3] = p_in[i];
      # print("p[0]",p[0])
      if DISTORTED:
        rvec = tvec = np.zeros(3)
        # print(p[0:3])
        # print(T_IMG_CAM[0:3,0:3])
        # print(np.array(calib['CAM02']['distortion_coefficients']['data']))
        image_points, jac = cv2.projectPoints(np.array([point[0:3]]), rvec, tvec, T_IMG_CAM[0:3,0:3], dist_coeffs)
        p_out.append([image_points[0,0,0], image_points[0,0,1]]);
        # print("image_points", image_points[0,0])
      else:
        curr = np.matmul(T_IMG_CAM, point.transpose()).transpose();
        # print("curr",curr);
        done = [curr[0] / curr[2], curr[1] / curr[2]]
        p_out.append(done);
        # print("p_out append", done)

    return p_out;
