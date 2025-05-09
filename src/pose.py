"""
pip install mujoco_ar numpy tf_transformations

python pose.py

Download MuJoCo AR app (https://github.com/omarrayyann/MujocoAR?tab=readme-ov-file#installation), and on the same network, connect to the IP address and port.

Latest pose will be saved to ~/pose.csv
"""

from mujoco_ar import MujocoARConnector
from tf_transformations import quaternion_from_matrix
import csv
import numpy as np
import os
import time

connector = MujocoARConnector()
connector.start()

while True:
    data = connector.get_latest_data()
    if data["position"] is None:
        continue

    T = np.eye(4)
    T[:3, :3] = np.asarray(data["rotation"])
    T[:3, 3] = np.asarray(data["position"])
    pos = T[:3, 3]
    quat = quaternion_from_matrix(T)

    with open(os.path.expanduser("~/pose.csv"), "w") as f:
        writer = csv.writer(f)
        writer.writerow(
            ["x", "y", "z", "qx", "qy", "qz", "qw", "button", "toggle"])
        row = [*pos, *quat, int(data["button"]), int(data["toggle"])]
        writer.writerow(row)
        print(row)

    time.sleep(0.05)
