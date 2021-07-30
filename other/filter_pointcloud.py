
# Implementation of DROR in python
# https://github.com/nickcharron/lidar_snow_removal/blob/master/src/DROR.cpp
# Slightly modified due to radiusSearch not being implemented in python-pcl

import numpy as np
import pcl
import pcl.pcl_visualization
import sys, math
# seq = sys.argv[1] #'0006'
# print(seq)

def dror_filter(input_cloud):
    radius_multiplier_ = 3
    azimuth_angle_ = 0.16 # 0.04
    min_neighbors_ = 3
    k_neighbors_ = min_neighbors_ + 1
    min_search_radius_ = 0.04

    filtered_cloud_list = []

    # init. kd search tree
    kd_tree = input_cloud.make_kdtree_flann()

    # Go over all the points and check which doesn't have enough neighbors
    # perform filtering
    for p_id in range(input_cloud.size):
        x_i = input_cloud[p_id][0];
        y_i = input_cloud[p_id][1];
        range_i = math.sqrt(pow(x_i, 2) + pow(y_i, 2));
        search_radius_dynamic = \
            radius_multiplier_ * azimuth_angle_ * 3.14159265359 / 180 * range_i;

        if (search_radius_dynamic < min_search_radius_):
            search_radius_dynamic = min_search_radius_

        [ind, sqdist] = kd_tree.nearest_k_search_for_point(input_cloud, p_id, k_neighbors_)

        # Count all neighbours
        neighbors = -1 # Start at -1 since it will always be its own neighbour
        for val in sqdist:
            if math.sqrt(val) < search_radius_dynamic:
                neighbors += 1;

        # This point is not snow, add it to the filtered_cloud
        if (neighbors >= min_neighbors_):
            filtered_cloud_list.append(input_cloud[p_id]);
            # print(filtered_cloud_list)
    
    return pcl.PointCloud(np.array(filtered_cloud_list, dtype=np.float32))

def crop_cloud(input_cloud):
    clipper = input_cloud.make_cropbox()
    clipper.set_Translation(0,0,0) # tx,ty,tz
    clipper.set_Rotation(0,0,0) # rx,ry,rz
    min_vals = [-4,-4,-3,0] # x,y,z,s
    max_vals = [4,4,10,0] # x,y,z,s
    clipper.set_MinMax(min_vals[0], min_vals[1], min_vals[2], min_vals[3], \
        max_vals[0], max_vals[1], max_vals[2], max_vals[3])
    return clipper.filter()

def print_snow_points(frame):
    # print(frame)
    seq = format(frame, '04')
    # print(seq)

    annotations_file = BASE + seq + "/3d_ann.json"
    export_annotations_file = BASE + seq + "/3d_ann_p.json"
    path_type = "processed"

    # Load lidar msg for this frame
    lidar_path = BASE + seq + "/" + path_type + "/lidar_points/data/" + format(0, '010') + ".bin"
    scan_data = np.fromfile(lidar_path, dtype=np.float32);
    # 2D array where each row contains a point [x, y, z, intensity]
    lidar = scan_data.reshape((-1, 4));
    # Convert lidar 2d array to pcl cloud
    point_cloud = pcl.PointCloud()
    point_cloud.from_array(lidar[:,0:3])
    # Crop the pointcloud to around autnomoose
    cropped_cloud = crop_cloud(point_cloud)
    # Run DROR
    filtered_cloud = dror_filter(cropped_cloud)
    # Print number of snow points
    number_snow_points = cropped_cloud.size - filtered_cloud.size
    print(number_snow_points)

    visual = pcl.pcl_visualization.CloudViewing()

    # PointXYZ
    visual.ShowMonochromeCloud(point_cloud, b'cloud')
    # visual.ShowMonochromeCloud(cropped_cloud, b'cloud')
    # visual.ShowMonochromeCloud(filtered_cloud, b'cloud')
    # visual.ShowGrayCloud(ptcloud_centred, b'cloud')
    # visual.ShowColorCloud(ptcloud_centred, b'cloud')
    # visual.ShowColorACloud(ptcloud_centred, b'cloud')

    v = True
    while v:
        v = not(visual.WasStopped())

BASE = '/media/matthew/WAVELAB_2TB/winter/data/'
# BASE = '/media/matthew/MOOSE-4TB/2019_02_27/'
# BASE = '/media/matthew/MOOSE-4TB/2018_03_06/data/'
# BASE = '/media/matthew/MOOSE-4TB/2018_03_07/data/'

LOOP = False

# for frame in range(21, 83):
print_snow_points(68)
# Low 74
# Medium 81
# High 80
# High example 68
