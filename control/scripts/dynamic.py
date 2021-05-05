#!/usr/bin/python3

import rosparam
import rospy
import yaml

class update_params:

    def __init__(self):

        rospy.init_node("update_params")
        self.filename = "/home/pkvk/sptp/src/lemons/config/target.yaml" 

    def read(self):
        with open(self.filename, 'r') as yaml_file:
            yaml_data = yaml.load(yaml_file, Loader=yaml.FullLoader)
            rospy.set_param('x', yaml_data['x'])
            rospy.set_param('y', yaml_data['y'])
            rospy.set_param('z', yaml_data['z'])
   
            rospy.set_param('r', yaml_data['r'])
            rospy.set_param('yaw', yaml_data['yaw'])
            rospy.set_param('p', yaml_data['p'])


if __name__ == "__main__":
    
    up = update_params()

    while not rospy.is_shutdown():
        up.read()
