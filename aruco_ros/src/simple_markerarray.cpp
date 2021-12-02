/**
 * @file simple_markerakkay.cpp
 * @author Matteo Murtaa
 * @date December 2021
 * @version 0.1
 * @brief Modified version of simple_single publishing all markers as MarkerArray
 */

#include <iostream>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <aruco_ros/aruco_ros_utils.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <dynamic_reconfigure/server.h>
#include <aruco_ros/ArucoThresholdConfig.h>

class ArucoSimple
{
private:
  cv::Mat inImage;
  aruco::CameraParameters camParam;
  bool useRectifiedImages;
  aruco::MarkerDetector mDetector;
  std::vector<aruco::Marker> markers;
  ros::Subscriber cam_info_sub;
  bool cam_info_received;
  image_transport::Publisher image_pub;
  image_transport::Publisher debug_pub;
  ros::Publisher pose_pub;
  ros::Publisher marker_pub; // rviz visualization marker

  double marker_size;

  ros::NodeHandle nh;
  image_transport::ImageTransport it;
  image_transport::Subscriber image_sub;

  dynamic_reconfigure::Server<aruco_ros::ArucoThresholdConfig> dyn_rec_server;

public:
  ArucoSimple() :
      cam_info_received(false), nh("~"), it(nh)
  {

    if (nh.hasParam("corner_refinement"))
      ROS_WARN(
          "Corner refinement options have been removed in ArUco 3.0.0, corner_refinement ROS parameter is deprecated");

    aruco::MarkerDetector::Params params = mDetector.getParameters();
    std::string thresh_method;
    switch (params._thresMethod)
    {
      case aruco::MarkerDetector::ThresMethod::THRES_ADAPTIVE:
        thresh_method = "THRESH_ADAPTIVE";
        break;
      case aruco::MarkerDetector::ThresMethod::THRES_AUTO_FIXED:
        thresh_method = "THRESH_AUTO_FIXED";
        break;
      default:
        thresh_method = "UNKNOWN";
        break;
    }

    // Print parameters of ArUco marker detector:
    ROS_INFO_STREAM("Threshold method: " << thresh_method);

    float min_marker_size; // percentage of image area
    nh.param<float>("min_marker_size", min_marker_size, 0.02);

    std::string detection_mode;
    nh.param<std::string>("detection_mode", detection_mode, "DM_FAST");
    if (detection_mode == "DM_FAST")
      mDetector.setDetectionMode(aruco::DM_FAST, min_marker_size);
    else if (detection_mode == "DM_VIDEO_FAST")
      mDetector.setDetectionMode(aruco::DM_VIDEO_FAST, min_marker_size);
    else
      // Aruco version 2 mode
      mDetector.setDetectionMode(aruco::DM_NORMAL, min_marker_size);

    ROS_INFO_STREAM("Marker size min: " << min_marker_size << "% of image area");
    ROS_INFO_STREAM("Detection mode: " << detection_mode);

    image_sub = it.subscribe("/image", 1, &ArucoSimple::image_callback, this);
    cam_info_sub = nh.subscribe("/camera_info", 1, &ArucoSimple::cam_info_callback, this);

    image_pub = it.advertise("result", 1);
    debug_pub = it.advertise("debug", 1);
    marker_pub = nh.advertise<visualization_msgs::MarkerArray>("markers", 10);

    nh.param<double>("marker_size", marker_size, 0.05);
    nh.param<bool>("image_is_rectified", useRectifiedImages, true);

    ROS_INFO("ArUco node started with marker size of %f m", marker_size);

    dyn_rec_server.setCallback(boost::bind(&ArucoSimple::reconf_callback, this, _1, _2));
  }

  void image_callback(const sensor_msgs::ImageConstPtr& msg)
  {
    if ((image_pub.getNumSubscribers() == 0) && (debug_pub.getNumSubscribers() == 0)
        && (marker_pub.getNumSubscribers() == 0))
    {
      ROS_DEBUG("No subscribers, not looking for ArUco markers");
      return;
    }

    if (cam_info_received)
    {
      ros::Time curr_stamp = msg->header.stamp;
      std::string camera_frame = msg->header.frame_id;
      cv_bridge::CvImagePtr cv_ptr;
      try
      {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
        inImage = cv_ptr->image;

        // detection results will go into "markers"
        markers.clear();
        // ok, let's detect
        mDetector.detect(inImage, markers, camParam, marker_size, false);

        visualization_msgs::MarkerArray visMarkerArray;

        // for each marker, draw info and its boundaries in the image
        for (std::size_t i = 0; i < markers.size(); ++i)
        {
            tf::Transform transform = aruco_ros::arucoMarker2Tf(markers[i]);
            geometry_msgs::PoseStamped poseMsg;
            tf::poseTFToMsg(transform, poseMsg.pose);
            poseMsg.header.frame_id = camera_frame;
            poseMsg.header.stamp = curr_stamp;
            
            // publish rviz marker representing the ArUco marker patch
            visualization_msgs::Marker visMarker;
            visMarker.header = poseMsg.header;
            visMarker.id = 1;
            visMarker.type = visualization_msgs::Marker::CUBE;
            visMarker.action = visualization_msgs::Marker::ADD;
            visMarker.pose = poseMsg.pose;
            visMarker.scale.x = marker_size;
            visMarker.scale.y = marker_size;
            visMarker.scale.z = 0.001;
            visMarker.color.r = 1.0;
            visMarker.color.g = 0;
            visMarker.color.b = 0;
            visMarker.color.a = 1.0;
            visMarker.lifetime = ros::Duration(3.0);
            visMarkerArray.markers.push_back(visMarker);

            // draw the detected marker
            markers[i].draw(inImage, cv::Scalar(0, 0, 255), 2);
        }

        marker_pub.publish(visMarkerArray);

        // draw a 3d cube in each marker if there is 3d info
        if (camParam.isValid() && marker_size != -1)
        {
          for (std::size_t i = 0; i < markers.size(); ++i)
          {
            aruco::CvDrawingUtils::draw3dAxis(inImage, markers[i], camParam);
          }
        }

        if (image_pub.getNumSubscribers() > 0)
        {
          // show input with augmented information
          cv_bridge::CvImage out_msg;
          out_msg.header.stamp = curr_stamp;
          out_msg.encoding = sensor_msgs::image_encodings::RGB8;
          out_msg.image = inImage;
          image_pub.publish(out_msg.toImageMsg());
        }

        if (debug_pub.getNumSubscribers() > 0)
        {
          // show also the internal image resulting from the threshold operation
          cv_bridge::CvImage debug_msg;
          debug_msg.header.stamp = curr_stamp;
          debug_msg.encoding = sensor_msgs::image_encodings::MONO8;
          debug_msg.image = mDetector.getThresholdedImage();
          debug_pub.publish(debug_msg.toImageMsg());
        }
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }
    }
  }

  // wait for one camerainfo, then shut down that subscriber
  void cam_info_callback(const sensor_msgs::CameraInfo &msg)
  {
    camParam = aruco_ros::rosCameraInfo2ArucoCamParams(msg, useRectifiedImages);

    cam_info_received = true;
    cam_info_sub.shutdown();
  }

  void reconf_callback(aruco_ros::ArucoThresholdConfig &config, uint32_t level)
  {
    mDetector.setDetectionMode(aruco::DetectionMode(config.detection_mode), config.min_image_size);
    if (config.normalizeImage)
    {
      ROS_WARN("normalizeImageIllumination is unimplemented!");
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "aruco_simple");

  ArucoSimple node;

  ros::spin();
}
