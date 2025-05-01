#include "bwt_ssg_builder/bwt_ssg_manager.hpp"

BWTSSGFrontEnd::BWTSSGFrontEnd(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private, std::shared_ptr<Communicator> comm, std::shared_ptr<Config> config)
  :nh_(nh), nh_private_(nh_private), comm_(comm), config_(config)
{
  // bwt_ssg_sim_ = std::make_shared<BWTSSGSim>(nh_, nh_private_, comm_, config_);
  bwt_ssg_real_ = std::make_shared<BWTSSGReal>(nh_, nh_private_, comm_, config_);
}

geometry_msgs::PoseArray BWTSSGFrontEnd::getCompartmentLongs(int compartment_vertex_id)
{
  return bwt_ssg_real_->getCompartmentLongs(compartment_vertex_id);
}

geometry_msgs::PoseArray BWTSSGFrontEnd::getFirstCompartmentLongs()
{
  return bwt_ssg_real_->getFirstCompartmentLongs();
}

Eigen::Vector6d BWTSSGFrontEnd::getFirstCompartmentState()
{
  return bwt_ssg_real_->getFirstCompartmentState();
}

geometry_msgs::PoseArray BWTSSGFrontEnd::getAllLongs()
{
  return bwt_ssg_real_->getAllLongs();
}

int BWTSSGFrontEnd::getCurrentCompartmentID()
{
  return bwt_ssg_real_->getCurrentCompartmentID();
}

double BWTSSGFrontEnd::compareOverlap(std::shared_ptr<SSGManager> candidate_graph)
{
  return bwt_ssg_real_->compareOverlap(candidate_graph);
}