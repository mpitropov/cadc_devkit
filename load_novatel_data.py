
import os

def load_novatel_data(novatel_path):
  files = os.listdir(novatel_path);
  novatel = [];

  for file in sorted(files):
    with open(novatel_path + file) as fp:
      novatel.append(fp.readline().split(' '));

  return novatel;