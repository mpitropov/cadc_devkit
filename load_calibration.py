
import yaml
import numpy as np

def load_calibration(calib_path):
  calib = {}

  # Get calibrations
  calib['extrinsics'] = yaml.load(open(calib_path + '/extrinsics.yaml'), yaml.SafeLoader)
  calib['CAM00'] = yaml.load(open(calib_path + '/00.yaml'), yaml.SafeLoader)
  calib['CAM01'] = yaml.load(open(calib_path + '/01.yaml'), yaml.SafeLoader)
  calib['CAM02'] = yaml.load(open(calib_path + '/02.yaml'), yaml.SafeLoader)
  calib['CAM03'] = yaml.load(open(calib_path + '/03.yaml'), yaml.SafeLoader)
  calib['CAM04'] = yaml.load(open(calib_path + '/04.yaml'), yaml.SafeLoader)
  calib['CAM05'] = yaml.load(open(calib_path + '/05.yaml'), yaml.SafeLoader)
  calib['CAM06'] = yaml.load(open(calib_path + '/06.yaml'), yaml.SafeLoader)
  calib['CAM07'] = yaml.load(open(calib_path + '/07.yaml'), yaml.SafeLoader)

  return calib
