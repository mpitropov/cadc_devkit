import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation as R
import json
import matplotlib.patches as patches


frame = 12
cam = '0'
seq = '0033'
DISTORTED = False
MOVE_FORWARD = True
DISPLAY_LIDAR = False
DISPLAY_CUBOID_CENTER = False
MIN_CUBOID_DIST = 40.0

BASE = '/media/matthew/WAVELAB_2TB/winter/data/'
# BASE = '/media/matthew/MOOSE-4TB/2019_02_27/'
# BASE = '/media/matthew/MOOSE-4TB/2018_03_06/data/'
# BASE = '/media/matthew/MOOSE-4TB/2018_03_07/data/'

if DISTORTED:
  path_type = 'raw'
else:
  path_type = 'processed'

lidar_path = BASE + seq + "/" + path_type + "/lidar_points/data/" + format(frame, '010') + ".bin";
calib_path = "/media/matthew/WAVELAB_2TB/winter/calib/";
img_path =  BASE + seq + "/" + path_type + "/image_0" + cam + "/data/" + format(frame, '010') + ".png";
annotations_path =  BASE + seq + "/3d_ann.json";

def bev(s1,s2,f1,f2,frame,lidar_path,annotations_path):
    '''

    :param s1: example 15 (15 meter to the left of the car)
    :param s2: s2 meters from the right of the car
    :param f1: f1 meters from the front of the car
    :param f2: f2 meters from the back of the car
    :param frame: the frame number
    :return:
    '''

    #limit the viewing range
    side_range = [-s1,s2] #15 meters from either side of the car
    fwd_range = [-f1,f2] # 15 m infront of the car

    scan_data = np.fromfile(lidar_path, dtype= np.float32) #numpy from file reads binary file
    #scan_data is a single row of all the lidar values
    # 2D array where each row contains a point [x, y, z, intensity]
    #we covert scan_data to format said above
    lidar = scan_data.reshape((-1, 4));

    lidar_x = lidar[:,0]
    lidar_y = lidar[:,1]
    lidar_z = lidar [:,2]



    lidar_x_trunc = []
    lidar_y_trunc = []
    lidar_z_trunc = []

    for i in range(len(lidar_x)):
        if lidar_x[i] > fwd_range[0] and lidar_x[i] < fwd_range[1]: #get the lidar coordinates
            if lidar_y[i] > side_range[0] and lidar_y[i] < side_range[1]:

                lidar_x_trunc.append(lidar_x[i])
                lidar_y_trunc.append(lidar_y[i])
                lidar_z_trunc.append(lidar_z[i])

    # to use for the plot
    x_img = [i* -1 for i in lidar_y_trunc] #in the image plot, the negative lidar y axis is the img x axis
    y_img = lidar_x_trunc #the lidar x axis is the img y axis
    pixel_values = lidar_z_trunc


    #shift values such that 0,0 is the minimum
    x_img = [i -side_range[0] for i in x_img]
    y_img = [i -fwd_range[0] for i in y_img]




    '''
    tracklets 
    '''

    # Load 3d annotations
    annotations_data = None
    with open(annotations_path) as f:
        annotations_data = json.load(f)

    # Add each cuboid to image
    '''
    Rotations in 3 dimensions can be represented by a sequece of 3 rotations around a sequence of axes. 
    In theory, any three axes spanning the 3D Euclidean space are enough. In practice the axes of rotation are chosen to be the basis vectors.
    The three rotations can either be in a global frame of reference (extrinsic) or in a body centred frame of refernce (intrinsic),
    which is attached to, and moves with, the object under rotation
    '''


    # PLOT THE IMAGE
    cmap = "jet"    # Color map to use
    dpi = 100       # Image resolution
    x_max = side_range[1] - side_range[0]
    y_max = fwd_range[1] - fwd_range[0]
    fig, ax = plt.subplots(figsize=(2000/dpi, 2000/dpi), dpi=dpi)

    # the coordinates in the tracklet json are lidar coords
    x_trunc = []
    y_trunc = []
    x_1 = []
    x_2 =[]
    x_3 = []
    x_4 =[]
    y_1 =[]
    y_2 =[]
    y_3 = []
    y_4 =[]

    for cuboid in annotations_data[frame]['cuboids']:
        T_Lidar_Cuboid = np.eye(4);  # identify matrix
        T_Lidar_Cuboid[0:3, 0:3] = R.from_euler('z', cuboid['yaw'],
                                                degrees=False).as_dcm();  # rotate the identity matrix
        T_Lidar_Cuboid[0][3] = cuboid['position']['x'];  # center of the tracklet, from cuboid to lidar
        T_Lidar_Cuboid[1][3] = cuboid['position']['y'];
        T_Lidar_Cuboid[2][3] = cuboid['position']['z'];



        if cuboid['position']['x']> fwd_range[0] and cuboid['position']['x'] < fwd_range[1]: #make sure the cuboid is within the range we want to see
            if cuboid['position']['y'] > side_range[0] and cuboid['position']['y'] < side_range[1]:
                x_trunc.append(cuboid['position']['x'])
                y_trunc.append(cuboid['position']['y'])

                width = cuboid['dimensions']['x'];
                length = cuboid['dimensions']['y'];
                height = cuboid['dimensions']['z'];
                radius = 3


                #the top view of the tracklet in the "cuboid frame". The cuboid frame is a cuboid with origin (0,0,0)
                #we are making a cuboid that has the dimensions of the tracklet but is located at the origin
                front_right_top = np.array(
                    [[1, 0, 0, length / 2], [0, 1, 0, width / 2], [0, 0, 1, height / 2], [0, 0, 0, 1]]);

                front_left_top = np.array(
                    [[1, 0, 0, length / 2], [0, 1, 0, -width / 2], [0, 0, 1, height / 2], [0, 0, 0, 1]]);


                back_right_top = np.array(
                    [[1, 0, 0, -length / 2], [0, 1, 0, width / 2], [0, 0, 1, height / 2], [0, 0, 0, 1]]);

                back_left_top = np.array(
                    [[1, 0, 0, -length / 2], [0, 1, 0, -width / 2], [0, 0, 1, height / 2], [0, 0, 0, 1]]);

                # Project to lidar


                f_r_t =  np.matmul(T_Lidar_Cuboid, front_right_top)
                f_l_t  = np.matmul(T_Lidar_Cuboid, front_left_top)
                b_r_t  = np.matmul(T_Lidar_Cuboid, back_right_top)
                b_l_t = np.matmul(T_Lidar_Cuboid, back_left_top)


                x_1.append(f_r_t[0][3])
                y_1.append(f_r_t[1][3])

                x_2.append(f_l_t[0][3])
                y_2.append(f_l_t[1][3])

                x_3.append(b_r_t[0][3])
                y_3.append(b_r_t[1][3])

                x_4.append(b_l_t[0][3])
                y_4.append(b_l_t[1][3])






    # to use for the plot

    x_img_tracklets = [i * -1 for i in y_trunc]  # in the image to plot, the negative lidar y axis is the img x axis
    y_img_tracklets = x_trunc  # the lidar x axis is the img y axis

    x_img_1 = [i * -1 for i in y_1]
    y_img_1 = x_1

    x_img_2 = [i * -1 for i in y_2]
    y_img_2 = x_2

    x_img_3 = [i * -1 for i in y_3]
    y_img_3 = x_3

    x_img_4 = [i * -1 for i in y_4]
    y_img_4 = x_4



    # shift values such that 0,0 is the minimum
    x_img_tracklets = [i -side_range[0] for i in x_img_tracklets]
    y_img_tracklets = [i -fwd_range[0] for i in y_img_tracklets]

    x_img_1 = [i -side_range[0] for i in x_img_1]
    y_img_1 = [i - fwd_range[0] for i in y_img_1]

    x_img_2 = [i -side_range[0] for i in x_img_2]
    y_img_2 = [i - fwd_range[0] for i in y_img_2]

    x_img_3 = [i -side_range[0] for i in x_img_3]
    y_img_3 = [i - fwd_range[0] for i in y_img_3]

    x_img_4 = [i -side_range[0] for i in x_img_4]
    y_img_4 = [i - fwd_range[0] for i in y_img_4]

    for i in range(len(x_img_1)): #plot the tracklets
        poly = np.array([[x_img_1[i], y_img_1[i]], [x_img_2[i], y_img_2[i]], [x_img_4[i], y_img_4[i]], [x_img_3[i], y_img_3[i]]])
        polys = patches.Polygon(poly,closed=True,fill=False, edgecolor ='r', linewidth=1)
        ax.add_patch(polys)



    # ax.scatter(x_img_tracklets,y_img_tracklets, marker ='o', color='red', linewidths=1) #center of the tracklets
    ax.scatter(x_img, y_img, s=1, c=pixel_values, alpha=1.0, cmap=cmap)
    ax.set_facecolor((0, 0, 0))  # backgrounD is black
    ax.axis('scaled')  # {equal, scaled}
    ax.xaxis.set_visible(False)  # Do not draw axis tick marks
    ax.yaxis.set_visible(False)  # Do not draw axis tick marks
    plt.xlim([0, x_max])
    plt.ylim([0, y_max])  
    fig.savefig("/home/matthew/Desktop/bev_" + str(frame) + ".png", dpi=dpi, bbox_inches='tight', pad_inches=0.0)

bev(50,50,50,50,frame,lidar_path,annotations_path)
