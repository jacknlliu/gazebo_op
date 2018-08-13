#!/usr/bin/env python3

import rospy
import time

from gazebo_op.gazebo_op import GazeboOp

def main():
    gz_com = GazeboOp()
    gz_com.reset_world()
    model_name = "textured_shapes"
    gz_com.pause_physics()
    gz_com.delete_model(model_name)
    time.sleep(1.0)
    gz_com.spawn_model(model_name)
    time.sleep(3.0)
    # gz_com.delete_model(model_name)
    gz_com.set_model_pose(model_name, position=[0, 0, 1.5], orientation=[1,0,0,0])

if __name__ == '__main__':
    main()