#pragma once

#include "bwt_ssg_builder/base_structures.hpp"

#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include "common/communicator.hpp"
#include "graph/ssg_manager.hpp"

#include "std_srvs/Empty.h"

#include "planner_msgs/MultipleOpeningDetections.h"
#include "planner_msgs/OpeningDetection.h"

class BWTSSGReal
{
public:
  BWTSSGReal(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private, std::shared_ptr<Communicator> comm, std::shared_ptr<Config> config);

  void stitchedCloudCallback(const sensor_msgs::PointCloud2 &lidar_cloud);
  void stitchedCloudCallbackRedone(const sensor_msgs::PointCloud2 &lidar_cloud_msg);
  void manholeDetectionsCallback(const planner_msgs::MultipleOpeningDetections &detections);
  void manholeDetectionsPACallback(const geometry_msgs::PoseArray &detections);
  void odomCallback(const nav_msgs::Odometry &odom);

  bool getLongsCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
  
  bool isInsideCompartment(int compartment_id, Eigen::Vector3d point);
  std::vector<Plane> extractPlanes(pcl::PointCloud<pcl::PointXYZ>::Ptr stitched_cloud_sparse);
  geometry_msgs::PoseArray getCompartmentLongs(int compartment_vertex_id);
  geometry_msgs::PoseArray getFirstCompartmentLongs();
  geometry_msgs::PoseArray getAllLongs();
  int getCurrentCompartmentID() { return current_compartment_vertex_id_; }
  double compareOverlap(std::shared_ptr<SSGManager> candidate_graph);
  Eigen::Vector6d getFirstCompartmentState();

  void visualizeGraph();
  void visualizeSegmentedCloud();
  void visualizeLines(std::vector<Line> lines);

private:
	ros::NodeHandle nh_;
	ros::NodeHandle nh_private_;

	std::shared_ptr<Communicator> comm_;
	std::shared_ptr<Config> config_;

  ros::Subscriber stitched_cloud_sub_;
  ros::Subscriber manhole_det_sub_;
  ros::Subscriber odom_sub_;

  ros::Publisher graph_vis_pub_;
  ros::Publisher combined_segmented_cloud_pub_;
  ros::Publisher extracted_lines_cloud_pub_;
  ros::Publisher close_to_wall_cloud_pub_;
  ros::Publisher wall_cloud_pub_;
  ros::Publisher tracked_wall_cloud_pub_;
  ros::Publisher line_pub_;
  ros::Publisher current_compartment_pub_;
  ros::Publisher current_compartment_longs_pub_;
  ros::Publisher filtered_cloud_pub_;

  tf2_ros::Buffer tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_ptr_;

  Eigen::Vector6d current_robot_state_;

  /// Maps from semantics' local id to custom datatypes;
  std::map<int, WallInstance> tracked_walls_;
  std::map<int, CompartmentInstance> tracked_compartments_;
  std::map<int, LongInstance> tracked_longs_;
  std::map<int, ManholeInstance> tracked_manholes_;
  
  int low_certainty_longs_count = 999;
  int init_low_certainty_longs_count = low_certainty_longs_count;
  int low_certainty_wall_count_ = 1999;
  int low_certainty_compartment_count_ = 2999;

  int current_compartment_vertex_id_ = -1;
  int first_compartment_vertex_id_ = -1;
  
  std::map<int, int> semantic_to_vertex_map_;
  std::map<int, int> vertex_to_semantic_map_;
  std::vector<int> known_labels_;

  std::map<int, std::map<int, int>> class_wise_id_to_vertex_;

  std::map<std::string, int> label_string_to_int_;
  int substruct_label_int = 9000;
};