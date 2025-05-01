#pragma once

#include <set>

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/centroid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/angles.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "common/utils.hpp"

struct Semantic
{
  Semantic()
  {
    cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
  }
  int id;  // ID of the semantic in this system
  int label;
  int local_id;  // ID provided by the detector
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
  Eigen::Vector6d state;
  Eigen::Vector3d principal_direction;
  bool seen = false;

  int vertex_id;
};