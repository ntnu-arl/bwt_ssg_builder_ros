#pragma once

#include "bwt_ssg_builder/bwt_ssg_builder.hpp"

class BWTSSGFrontEnd
{
public:
  BWTSSGFrontEnd(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private, std::shared_ptr<Communicator> comm, std::shared_ptr<Config> config);

  geometry_msgs::PoseArray getCompartmentLongs(int compartment_vertex_id);
  geometry_msgs::PoseArray getFirstCompartmentLongs();
  geometry_msgs::PoseArray getAllLongs();
  Eigen::Vector6d getFirstCompartmentState();
  double compareOverlap(std::shared_ptr<SSGManager> candidate_graph);

  int getCurrentCompartmentID();

private:
  ros::NodeHandle nh_;
	ros::NodeHandle nh_private_;

	std::shared_ptr<Communicator> comm_;
	std::shared_ptr<Config> config_;

  std::shared_ptr<BWTSSGReal> bwt_ssg_real_;
};