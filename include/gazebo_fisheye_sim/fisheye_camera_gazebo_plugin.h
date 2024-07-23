#ifndef FISHEYE_CAMERA_GAZEBO_PLUGIN_H
#define FISHEYE_CAMERA_GAZEBO_PLUGIN_H

#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/common/common.hh>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>

namespace gazebo
{
  class FisheyeCameraPlugin : public SensorPlugin
  {
    public:
      FisheyeCameraPlugin();
      virtual ~FisheyeCameraPlugin();
      void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);
    
    private:
      void OnNewFrame(const unsigned char *_image,
                      unsigned int _width, unsigned int _height,
                      unsigned int _depth, const std::string &_format);
      
      cv::Mat distortImage(const cv::Mat& src);
    
      sensors::CameraSensorPtr parentSensor;
      rendering::CameraPtr camera;
      
      event::ConnectionPtr newFrameConnection;
      
      // ROS NodeHandle
      ros::NodeHandle* rosNode;
      
      // ROS Publisher
      ros::Publisher imagePub;
      
      // Fisheye camera parameters
      double k1, k2, k3, k4;
      double fx, fy, cx, cy;
      int width, height;
  };
}

#endif