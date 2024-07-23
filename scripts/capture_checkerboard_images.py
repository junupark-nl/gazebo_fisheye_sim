#!/usr/bin/env python3

import rospy
import rospkg
import gazebo_msgs.srv
import geometry_msgs.msg
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import math

class CheckerboardCapture:
    def __init__(self):
        rospy.init_node('checkerboard_image_capture')
        
        self.bridge = CvBridge()
        self.image_count = 0
        self.num_images = rospy.get_param('~num_images', 20)
        self.output_dir = rospy.get_param('~output_dir', 'fisheye_images')
        
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)
        
        self.image_sub = rospy.Subscriber('/fisheye_camera/image_raw', Image, self.image_callback)
        
        rospy.wait_for_service('/gazebo/set_model_state')
        self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', gazebo_msgs.srv.SetModelState)
        
        # Wait for models to spawn
        rospy.sleep(15)
        
        self.capture_images()

    def image_callback(self, msg):
        if self.image_count < self.num_images:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            filename = os.path.join(self.output_dir, f'checkerboard_{self.image_count:03d}.png')
            cv2.imwrite(filename, cv_image)
            rospy.loginfo(f"Saved image {self.image_count + 1}/{self.num_images}")
            self.image_count += 1

    def set_checkerboard_pose(self, x, y, z, roll, pitch, yaw):
        state_msg = gazebo_msgs.msg.ModelState()
        state_msg.model_name = 'checkerboard'
        state_msg.pose.position.x = x
        state_msg.pose.position.y = y
        state_msg.pose.position.z = z
        
        q = self.quaternion_from_euler(roll, pitch, yaw)
        state_msg.pose.orientation.x = q[0]
        state_msg.pose.orientation.y = q[1]
        state_msg.pose.orientation.z = q[2]
        state_msg.pose.orientation.w = q[3]
        
        self.set_model_state(state_msg)

    def quaternion_from_euler(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = [0] * 4
        q[0] = sr * cp * cy - cr * sp * sy
        q[1] = cr * sp * cy + sr * cp * sy
        q[2] = cr * cp * sy - sr * sp * cy
        q[3] = cr * cp * cy + sr * sp * sy

        return q

    def capture_images(self):
        for i in range(self.num_images):
            angle = -135 + (270 / (self.num_images - 1)) * i
            roll = math.radians(angle)
            pitch = math.radians(angle / 2)  # Adjust as needed
            yaw = 0
            
            self.set_checkerboard_pose(1, 0, 0.5, roll, pitch, yaw)
            rospy.sleep(1)  # Wait for the checkerboard to move and the image to update

        rospy.signal_shutdown("Finished capturing images")

if __name__ == '__main__':
    try:
        CheckerboardCapture()
    except rospy.ROSInterruptException:
        pass