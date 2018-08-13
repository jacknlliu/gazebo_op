#!/usr/bin/env python3

import rospy
import time

from gazebo_op.gazebo_op import GazeboOp
import pandas as pd

def main():
    gz_com = GazeboOp()
    # model_name = "textured_shapes"
    model_name = "person_walking"
    gz_com.pause_physics()
    gz_com.delete_model(model_name)

    data_file_path = "../config/orientation.csv"
    df = pd.read_csv(data_file_path, header=None)
    orientation_data = df.values

    gz_com.spawn_model(model_name, [0,0,1.5] + orientation_data[0].tolist())
    time.sleep(2.0)

    for i in range(orientation_data.shape[0]):
        gz_com.set_model_pose(model_name, position=[0, 0, 1.5], orientation=orientation_data[i].tolist())

        if i == 0:
            time.sleep(2.0)
        else:
            time.sleep(0.1)


if __name__ == '__main__':
    main()