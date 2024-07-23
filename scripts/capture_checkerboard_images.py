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
        self.num_grid = rospy.get_param('~num_grid', 7)
        self.num_skew = rospy.get_param('~num_skew', 5)
        self.num_images = self.num_grid * self.num_grid * self.num_skew
        self.output_dir = rospy.get_param('~output_dir', 'images')
        
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)
        
        self.image_sub = rospy.Subscriber('/fisheye_camera/image_raw', Image, self.image_callback)
        
        rospy.wait_for_service('/gazebo/set_model_state')
        self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', gazebo_msgs.srv.SetModelState)
        
        self.ready_to_capture = False
        self.latest_image = None
        
        # Wait for models to spawn
        rospy.sleep(5)
        
        self.capture_images()

    def image_callback(self, msg):
        if self.ready_to_capture:
            self.latest_image = msg
            self.save_image()

    def save_image(self):
        if self.latest_image is not None and self.image_count < self.num_images:
            cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, "bgr8")
            filename = os.path.join(self.output_dir, f'checkerboard_{self.image_count:03d}.png')
            cv2.imwrite(filename, cv_image)
            rospy.loginfo(f"Saved image {self.image_count + 1}/{self.num_images}")
            self.image_count += 1
            self.ready_to_capture = False
            self.latest_image = None

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
        import numpy as np
        angle_max = math.radians(15)
        grid_max = 0.25
        board_horizontal_range = np.linspace(-grid_max, grid_max, self.num_grid)
        board_vertical_range = np.linspace(-grid_max, grid_max, self.num_grid)
        angle_range = np.linspace(-angle_max, angle_max, self.num_skew)
        roll_range = angle_range
        yaw_range = angle_range / 10
        for i in range(self.num_grid):
            for j in range(self.num_grid):
                for k in range(self.num_skew):
                    roll = roll_range[k]
                    pitch = math.pi/2
                    yaw = 0
                    x = 0.6 + 0.05*np.random.randint(1, 100)/100
                    y = board_horizontal_range[i]
                    z = board_vertical_range[j] + 1
                    self.set_checkerboard_pose(x, y, z, roll, pitch, yaw)
                    rospy.sleep(0.1)  # Wait for the checkerboard to move
                    self.ready_to_capture = True
                    rospy.sleep(0.1)  # Wait for the image to update

        rospy.signal_shutdown("Finished capturing images")

if __name__ == '__main__':
    try:
        CheckerboardCapture()
    except rospy.ROSInterruptException:
        pass