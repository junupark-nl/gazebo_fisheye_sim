#include "gazebo_fisheye_sim/fisheye_camera_gazebo_plugin.h"
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

namespace gazebo
{
  GZ_REGISTER_SENSOR_PLUGIN(FisheyeCameraPlugin)

  FisheyeCameraPlugin::FisheyeCameraPlugin() : SensorPlugin(), rosNode(NULL) {}

  FisheyeCameraPlugin::~FisheyeCameraPlugin()
  {
    if (rosNode)
    {
      rosNode->shutdown();
      delete rosNode;
    }
  }
  void FisheyeCameraPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
  {
    // Load camera parameters
    this->parentSensor = std::dynamic_pointer_cast<sensors::CameraSensor>(_sensor);
    this->camera = this->parentSensor->Camera();

    this->width = this->camera->ImageWidth();
    this->height = this->camera->ImageHeight();

    // Load distortion coefficients from SDF
    if (_sdf->HasElement("k1"))
      this->k1 = _sdf->Get<double>("k1");
    if (_sdf->HasElement("k2"))
      this->k2 = _sdf->Get<double>("k2");
    if (_sdf->HasElement("k3"))
      this->k3 = _sdf->Get<double>("k3");
    if (_sdf->HasElement("k4"))
      this->k4 = _sdf->Get<double>("k4");

    // Load camera intrinsics from SDF
    if (_sdf->HasElement("fx"))
      this->fx = _sdf->Get<double>("fx");
    if (_sdf->HasElement("fy"))
      this->fy = _sdf->Get<double>("fy");
    if (_sdf->HasElement("cx"))
      this->cx = _sdf->Get<double>("cx");
    if (_sdf->HasElement("cy"))
      this->cy = _sdf->Get<double>("cy");

    // Initialize ROS
    if (!ros::isInitialized())
    {
      int argc = 0;
      char **argv = NULL;
      ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
    }
    this->rosNode = new ros::NodeHandle();

    // Create ROS publisher
    this->imagePub = this->rosNode->advertise<sensor_msgs::Image>("fisheye_camera/image_raw", 1);

    // Connect new frame event
    this->newFrameConnection = this->camera->ConnectNewImageFrame(
      std::bind(&FisheyeCameraPlugin::OnNewFrame, this,
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
        std::placeholders::_4, std::placeholders::_5));

    // Make sure the parent sensor is active
    this->parentSensor->SetActive(true);
  }

  void FisheyeCameraPlugin::OnNewFrame(const unsigned char *_image,
                                       unsigned int _width, unsigned int _height,
                                       unsigned int _depth, const std::string &_format)
  {
    // Convert raw image data to OpenCV Mat
    cv::Mat original(_height, _width, CV_8UC3, (void*)_image);
    
    // Apply fisheye distortion
    cv::Mat distorted = distortImage(original);
    
    // Convert to ROS image message and publish
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", distorted).toImageMsg();
    this->imagePub.publish(msg);
  }

  cv::Mat FisheyeCameraPlugin::distortImage(const cv::Mat& src)
  {
    cv::Mat dst = src.clone();
    
    for (int y = 0; y < height; y++)
    {
      for (int x = 0; x < width; x++)
      {
        // Normalize coordinates
        double xn = (x - cx) / fx;
        double yn = (y - cy) / fy;
        
        // Calculate radius
        double r = std::sqrt(xn*xn + yn*yn);
        
        // Apply distortion
        double theta = std::atan(r);
        double theta2 = theta * theta;
        double theta4 = theta2 * theta2;
        double theta6 = theta4 * theta2;
        double theta8 = theta4 * theta4;
        double thetad = theta * (1 + k1 * theta2 + k2 * theta4 + k3 * theta6 + k4 * theta8);
        
        // Calculate distorted coordinates
        double xd = 0, yd = 0;
        if (r > 0)
        {
          xd = thetad * xn / r;
          yd = thetad * yn / r;
        }
        
        // Map back to pixel coordinates
        int x_distorted = static_cast<int>(xd * fx + cx);
        int y_distorted = static_cast<int>(yd * fy + cy);
        
        // Copy pixel if within bounds
        if (x_distorted >= 0 && x_distorted < width && y_distorted >= 0 && y_distorted < height)
        {
          dst.at<cv::Vec3b>(y, x) = src.at<cv::Vec3b>(y_distorted, x_distorted);
        }
      }
    }
    
    return dst;
  }
}