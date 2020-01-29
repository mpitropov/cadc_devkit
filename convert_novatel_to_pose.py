
# Convert LL to UTM
import utm
import numpy as np
import math

# Converts GPS data to poses in the ENU frame
def convert_novatel_to_pose(novatel):
  poses = [];
  FIRST_RUN = True;
  origin = [];

  for gps_msg in novatel:
    # utm_data[0] = East (m), utm_data[1] = North (m)
    utm_data = utm.from_latlon(float(gps_msg[0]), float(gps_msg[1]));
    # Ellipsoidal height = MSL (orthometric) + undulation
    ellipsoidal_height = float(gps_msg[2]) + float(gps_msg[3]);

    roll = np.deg2rad(float(gps_msg[7]));
    pitch = np.deg2rad(float(gps_msg[8]));

    # Azimuth = north at 0 degrees, east at 90 degrees, south at 180 degrees and west at 270 degrees
    azimuth = float(gps_msg[9]);
    # yaw = north at 0 deg, 90 at west and 180 at south, east at 270 deg
    yaw = np.deg2rad(-1.0 * azimuth); 

    c_phi = math.cos(roll);
    s_phi = math.sin(roll);
    c_theta = math.cos(pitch);
    s_theta = math.sin(pitch);
    c_psi = math.cos(yaw);
    s_psi = math.sin(yaw);

    if FIRST_RUN:
      origin = [utm_data[0], utm_data[1], ellipsoidal_height];
      FIRST_RUN = False;

    # This is the T_locallevel_body transform where ENU is the local level frame
    # and the imu is the body frame
    # https://www.novatel.com/assets/Documents/Bulletins/apn037.pdf
    poses.append(np.matrix([
      [c_psi * c_phi - s_psi * s_theta * s_phi, -s_psi * c_theta, c_psi * s_phi + s_psi * s_theta * c_phi, utm_data[0] - origin[0]],
      [s_psi * c_phi + c_psi * s_theta * s_phi, c_psi * c_theta, s_psi * s_phi - c_psi * s_theta * c_phi, utm_data[1] - origin[1]],
      [-c_theta * s_phi, s_theta, c_theta * c_phi, ellipsoidal_height - origin[2]],
      [0.0, 0.0, 0.0, 1.0]]));

  return poses;
