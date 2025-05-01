#include "bwt_ssg_builder/bwt_ssg_builder.hpp"

BWTSSGReal::BWTSSGReal(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private, std::shared_ptr<Communicator> comm, std::shared_ptr<Config> config)
  :nh_(nh), nh_private_(nh_private), comm_(comm), config_(config)
{
  stitched_cloud_sub_ = nh_.subscribe("lidar_cloud", 1, &BWTSSGReal::stitchedCloudCallbackRedone, this);
  if(config_->ssg_params.external_mh_detections)
    manhole_det_sub_ = nh_.subscribe("/stable_detections_vis", 1, &BWTSSGReal::manholeDetectionsPACallback, this);
  else
    manhole_det_sub_ = nh_.subscribe("manhole_detections", 1, &BWTSSGReal::manholeDetectionsCallback, this);
  odom_sub_ = nh_.subscribe("odometry", 10, &BWTSSGReal::odomCallback, this);

  graph_vis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("ssg_vis", 1);
  current_compartment_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("current_compartment", 1);
  current_compartment_longs_pub_ = nh_.advertise<geometry_msgs::PoseArray>("current_compartment_longs", 1);
  combined_segmented_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("combined_segmented_cloud", 1);
  extracted_lines_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("extracted_lines_cloud", 1);
  close_to_wall_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("close_to_wall_cloud", 1);
  wall_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("wall_cloud", 1);
  tracked_wall_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("tracked_wall_cloud", 1);
  line_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("extracted_lines", 1);
  filtered_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("filtered_lidar_cloud", 1);

  tf_listener_ptr_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);

  pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
}

void BWTSSGReal::odomCallback(const nav_msgs::Odometry &odom)
{
  current_robot_state_.x() = odom.pose.pose.position.x;
  current_robot_state_.y() = odom.pose.pose.position.y;
  current_robot_state_.z() = odom.pose.pose.position.z;
  Eigen::Quaterniond quat(odom.pose.pose.orientation.w,
                          odom.pose.pose.orientation.x,
                          odom.pose.pose.orientation.y,
                          odom.pose.pose.orientation.z);
  current_robot_state_.tail(3) = quat.toRotationMatrix().eulerAngles(0,1,2);
}

void BWTSSGReal::manholeDetectionsCallback(const planner_msgs::MultipleOpeningDetections &detections)
{
  if(current_compartment_vertex_id_ < 0) return;

  for(auto det : detections.multiple_detections)
  {
    auto it = std::find_if(comm_->ssg_manager()->vertices_map_.begin(), comm_->ssg_manager()->vertices_map_.end(), 
                        [det] (const std::pair<int, std::shared_ptr<SemanticVertex>> &a) {
                          return ((a.second->local_id == det.id) && (a.second->label == L_MANHOLE));
                        });
    if(it == comm_->ssg_manager()->vertices_map_.end())  // New manhole
    {
      std::shared_ptr<SemanticVertex> new_vertex = comm_->ssg_manager()->initializeNewVertex();
      new_vertex->label = L_MANHOLE;
      new_vertex->local_id = det.id;
      new_vertex->state.x() = det.pose.position.x;
      new_vertex->state.y() = det.pose.position.y;
      new_vertex->state.z() = det.pose.position.z;
      new_vertex->state(3) = 0.0;
      new_vertex->state(4) = 0.0;
      new_vertex->state(5) = tf::getYaw(det.pose.orientation);
      comm_->ssg_manager()->addVertex(new_vertex);
      ROS_WARN("Addition new vertex %d, label: %d", new_vertex->id, new_vertex->label);
      new_vertex->seen = true;
      if((comm_->ssg_manager()->getVertex(current_compartment_vertex_id_)->state.head(3) - new_vertex->state.head(3)).norm() < config_->ssg_params.compartment_distance_thr)
      {
        std::shared_ptr<EdgeRelation> new_edge = comm_->ssg_manager()->initializeNewEdge();
        new_edge->source_vertex = new_vertex;
        new_edge->target_vertex = comm_->ssg_manager()->getVertex(current_compartment_vertex_id_);
        new_edge->weight = (new_edge->target_vertex->state.head(3) - new_edge->source_vertex->state.head(3)).norm();
        new_edge->label = 1;
        comm_->ssg_manager()->addEdge(new_edge);
      }
    }
    else
    {
      it->second->state.x() = det.pose.position.x;
      it->second->state.y() = det.pose.position.y;
      it->second->state.z() = det.pose.position.z;
      it->second->state(3) = 0.0;
      it->second->state(4) = 0.0;
      it->second->state(5) = tf::getYaw(det.pose.orientation);
      it->second->seen = true;
      auto it_n = it->second->neighbor_map.find(current_compartment_vertex_id_);
      if(it_n == it->second->neighbor_map.end())  // Need to add an edge between the manhole and the current compartment
      {
        if((comm_->ssg_manager()->getVertex(current_compartment_vertex_id_)->state.head(3) - it->second->state.head(3)).norm() < config_->ssg_params.compartment_distance_thr)
        {
          std::shared_ptr<EdgeRelation> new_edge = comm_->ssg_manager()->initializeNewEdge();
          new_edge->source_vertex = it->second;
          new_edge->target_vertex = comm_->ssg_manager()->getVertex(current_compartment_vertex_id_);
          new_edge->weight = (new_edge->target_vertex->state.head(3) - new_edge->source_vertex->state.head(3)).norm();
          new_edge->label = 1;
          comm_->ssg_manager()->addEdge(new_edge);
        }
      }
    }
  }
}

void BWTSSGReal::manholeDetectionsPACallback(const geometry_msgs::PoseArray &detections)
{
  if(current_compartment_vertex_id_ < 0) return;

  // ROS_WARN("MH callback");

  for(geometry_msgs::Pose det : detections.poses)
  {
    Eigen::Vector6d mh_state;
    convert(det, mh_state);
    std::vector<std::shared_ptr<SemanticVertex>> nearest_nbs;
    comm_->ssg_manager()->getNearestVerticesInRange(mh_state, nearest_nbs, 1.0);
    bool mh_found = false;
    double closest_dist = 1.0;
    std::shared_ptr<SemanticVertex> closest_mh;
    for(auto v : nearest_nbs)
    {
      if(v->label == L_MANHOLE)
      {
        double dist = (v->state.head(3) - mh_state.head(3)).norm();
        if(dist < closest_dist)
        {
          mh_found = true;
          closest_dist = dist;
          closest_mh = v;
        }
      }
    }
    
    if(!mh_found || closest_mh == nullptr)  // New manhole
    {
      std::shared_ptr<SemanticVertex> new_vertex = comm_->ssg_manager()->initializeNewVertex();
      new_vertex->label = L_MANHOLE;
      // new_vertex->local_id = (int)det.header.seq;
      new_vertex->state.x() = det.position.x;
      new_vertex->state.y() = det.position.y;
      new_vertex->state.z() = det.position.z;
      new_vertex->state(3) = 0.0;
      new_vertex->state(4) = 0.0;
      new_vertex->state(5) = tf::getYaw(det.orientation);
      // new_vertex->metadata = std::make_shared<ManholeInstance>(&)
      comm_->ssg_manager()->addVertex(new_vertex);
      ROS_WARN("Addition new vertex %d, label: %d", new_vertex->id, new_vertex->label);
      new_vertex->seen = true;
      if((comm_->ssg_manager()->getVertex(current_compartment_vertex_id_)->state.head(3) - new_vertex->state.head(3)).norm() < config_->ssg_params.compartment_distance_thr)
      {
        std::shared_ptr<EdgeRelation> new_edge = comm_->ssg_manager()->initializeNewEdge();
        new_edge->source_vertex = new_vertex;
        new_edge->target_vertex = comm_->ssg_manager()->getVertex(current_compartment_vertex_id_);
        new_edge->weight = (new_edge->target_vertex->state.head(3) - new_edge->source_vertex->state.head(3)).norm();
        new_edge->label = 1;
        comm_->ssg_manager()->addEdge(new_edge);
      }
    }
    else
    {
      closest_mh->state.x() = det.position.x;
      closest_mh->state.y() = det.position.y;
      closest_mh->state.z() = det.position.z;
      closest_mh->state(3) = 0.0;
      closest_mh->state(4) = 0.0;
      closest_mh->state(5) = tf::getYaw(det.orientation);
      closest_mh->seen = true;
      auto it_n = closest_mh->neighbor_map.find(current_compartment_vertex_id_);
      if(it_n == closest_mh->neighbor_map.end())  // Need to add an edge between the manhole and the current compartment
      {
        if((comm_->ssg_manager()->getVertex(current_compartment_vertex_id_)->state.head(3) - closest_mh->state.head(3)).norm() < config_->ssg_params.compartment_distance_thr)
        {
          std::shared_ptr<EdgeRelation> new_edge = comm_->ssg_manager()->initializeNewEdge();
          new_edge->source_vertex = closest_mh;
          new_edge->target_vertex = comm_->ssg_manager()->getVertex(current_compartment_vertex_id_);
          new_edge->weight = (new_edge->target_vertex->state.head(3) - new_edge->source_vertex->state.head(3)).norm();
          new_edge->label = 1;
          comm_->ssg_manager()->addEdge(new_edge);
        }
      }
    }
  }
}


std::vector<Plane> BWTSSGReal::extractPlanes(pcl::PointCloud<pcl::PointXYZ>::Ptr stitched_cloud_sparse)
{
  std::vector<Plane> extracted_planes;
  int num_walls = 4;
  for(int k=0; k<num_walls; ++k)
  {
    if(stitched_cloud_sparse->points.size() <= 3) break;
    // Estimate plane
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
    // seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.05);
    seg.setInputCloud(stitched_cloud_sparse);
    seg.setAxis(Eigen::Vector3f(0.0, 0.0, 1.0));
    seg.setEpsAngle(pcl::deg2rad(15.0));
    seg.setMaxIterations(1000);
    try 
    {
      auto t1_s = std::chrono::high_resolution_clock::now();
      auto t2_s = t1_s;
      seg.segment(*inliers, *coefficients);
      t2_s = std::chrono::high_resolution_clock::now();
    }
    catch(...)
    {
      ROS_WARN("Plane extraction failed");
      break;
    }

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(stitched_cloud_sparse);
    extract.setIndices(inliers);
    extract.setNegative(true);
    pcl::PointCloud<pcl::PointXYZ> cloud_F;
    extract.filter(cloud_F);

    Plane current_plane;

    for(int j=0; j<inliers->indices.size(); ++j) 
    {
      pcl::PointXYZ pt = stitched_cloud_sparse->points[inliers->indices[j]];
      pcl::PointXYZRGB pt_rgb;
      pt_rgb.x = pt.x;
      pt_rgb.y = pt.y;
      pt_rgb.z = pt.z;
      current_plane.points->points.push_back(pt);
    }
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*current_plane.points, centroid);

    // Eigen::Vector3f plane_normal(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
    if(Eigen::Vector3f(coefficients->values[0], coefficients->values[1], coefficients->values[2]).dot(current_robot_state_.head(3).cast<float>() - centroid.head(3)) < 0.0)
    {
      for(int i=0; i<3; ++i) coefficients->values[i] = -coefficients->values[i];
    }

    current_plane.id = k;
    current_plane.coefficients = *coefficients;
    current_plane.centroid = centroid.head(3).cast<double>();
    current_plane.normal = Eigen::Vector3d(coefficients->values[0], coefficients->values[1], coefficients->values[2]).normalized();

    extracted_planes.push_back(current_plane);

    stitched_cloud_sparse->swap(cloud_F);
  }

  return extracted_planes;
}

void BWTSSGReal::stitchedCloudCallbackRedone(const sensor_msgs::PointCloud2 &lidar_cloud_msg)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr stitched_cloud_sensor (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PointCloud<pcl::PointXYZ>::Ptr stitched_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PointCloud<pcl::PointXYZ>::Ptr stitched_cloud_sparse (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PointCloud<pcl::PointXYZ>::Ptr stitched_cloud_small_fov (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PointCloud<pcl::PointXYZ>::Ptr stitched_cloud_small_fov_sensor (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr wall_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());

  pcl::fromROSMsg(lidar_cloud_msg, *stitched_cloud_sensor);

  /* TEMP */
  geometry_msgs::TransformStamped T_W_sensor;
  geometry_msgs::TransformStamped R_W_sensor;
	bool found = true;
	try
	{
		T_W_sensor = tf_buffer_.lookupTransform("world", lidar_cloud_msg.header.frame_id,
												lidar_cloud_msg.header.stamp, ros::Duration(config_->ssg_params.lidar_tf_lookup_delay));
    R_W_sensor.transform.rotation = T_W_sensor.transform.rotation;
  }
	catch (tf2::TransformException &ex)
	{
		found = false;
    ROS_WARN("LiDAR -> World TF Failed. %s", ex.what());
    return;
	}
  Eigen::Matrix4f T_W_sensor_eigen;
  Eigen::Quaternionf quat_f(T_W_sensor.transform.rotation.w, T_W_sensor.transform.rotation.x, T_W_sensor.transform.rotation.y, T_W_sensor.transform.rotation.z);
  // Eigen::Quaternionf quat_f(1.0, 0.0, 0.0, 0.0);
  T_W_sensor_eigen.block<3,3>(0,0) = quat_f.toRotationMatrix();
  T_W_sensor_eigen.block<3,1>(0,3) << T_W_sensor.transform.translation.x, T_W_sensor.transform.translation.y, T_W_sensor.transform.translation.z;
  T_W_sensor_eigen.block<1,3>(3,0) = Eigen::Vector3f::Zero();
  T_W_sensor_eigen(3,3) = 1.0;
  pcl::PointCloud<pcl::PointXYZ> tfed_pointcloud;
  pcl::transformPointCloud(*stitched_cloud_sensor, *stitched_cloud, T_W_sensor_eigen);
  /*******/
  pcl::CropBox<pcl::PointXYZ> crop_box_filter (true);
  crop_box_filter.setInputCloud (stitched_cloud);
  Eigen::Vector4f min_pt (-100.0f, -100.0f, config_->ssg_params.min_long_height-0.5, 1.0f);
  Eigen::Vector4f max_pt (100.0f, 100.0f, config_->ssg_params.max_long_height+1.0, 1.0f);
  crop_box_filter.setMin (min_pt);
  crop_box_filter.setMax (max_pt);
  // crop_box_filter.
  // crop_box_filter.setNegative(true);
  crop_box_filter.filter(*stitched_cloud);

  auto t1 = std::chrono::high_resolution_clock::now();
  auto t2 = t1;

  pcl::VoxelGrid<pcl::PointXYZ> vox_filter;
  vox_filter.setInputCloud (stitched_cloud);
  vox_filter.setLeafSize (0.1f, 0.1f, 0.1f);
  vox_filter.filter (*stitched_cloud_sparse);

  for(int i=0; i<stitched_cloud->points.size(); ++i)
  {
    if(std::abs(std::atan2(stitched_cloud_sensor->points[i].y, stitched_cloud_sensor->points[i].x)) <= config_->ssg_params.detection_fov*M_PI/180.0)
    {
      stitched_cloud_small_fov->points.push_back(stitched_cloud->points[i]);
      stitched_cloud_small_fov_sensor->points.push_back(stitched_cloud_sensor->points[i]);
    }
    // stitched_cloud_small_fov->points.push_back(stitched_cloud->points[i]);
  }

  sensor_msgs::PointCloud2 filtered_cloud_msg;
  pcl::toROSMsg(*stitched_cloud_small_fov_sensor, filtered_cloud_msg);
  // filtered_cloud_msg.header.frame_id = "world";
  filtered_cloud_msg.header = lidar_cloud_msg.header;
  filtered_cloud_pub_.publish(filtered_cloud_msg);

  //// EXTRACT PLANES ////
  std::vector<Plane> extracted_planes = extractPlanes(stitched_cloud_sparse);
  /// Trim the Point cloud ///
  std::vector<Plane> extracted_planes_copy;
  for(int i=0; i<extracted_planes.size(); ++i)
  {
    Eigen::Vector3d plane_normal(extracted_planes[i].coefficients.values[0],
                                 extracted_planes[i].coefficients.values[1],
                                 extracted_planes[i].coefficients.values[2]);

    for(int j=0; j<extracted_planes.size(); ++j)
    {
      if(j == i) continue;
      std::vector<pcl::PointXYZ> points_side1, points_side2;
      Eigen::Vector3d pt_q(extracted_planes[j].points->points[0].x, extracted_planes[j].points->points[0].y, extracted_planes[j].points->points[0].z);
      for(auto pt : extracted_planes[i].points->points)
      {
        Eigen::Vector3d pt_p(pt.x, pt.y, pt.z);
        double dist = extracted_planes[j].normal.dot(pt_p - pt_q);
        if(dist >= 0.0)
          points_side1.push_back(pt);
        else
          points_side2.push_back(pt);
      }
      extracted_planes[i].points->points.clear();
      if(points_side1.size() > points_side2.size())
      {
        extracted_planes[i].points->points.insert(extracted_planes[i].points->points.begin(), points_side1.begin(), points_side1.end());
      }
      else
      {
        extracted_planes[i].points->points.insert(extracted_planes[i].points->points.begin(), points_side2.begin(), points_side2.end());
      }
    }

    for(auto w_it : tracked_walls_)
    {
      /// Radius filter:
      if((current_robot_state_.head(3) - w_it.second.plane.centroid).norm() > config_->ssg_params.furthest_wall_to_consider)
        continue;
      Eigen::Vector3d wall_normal(w_it.second.plane.coefficients.values[0],
                                  w_it.second.plane.coefficients.values[1],
                                  w_it.second.plane.coefficients.values[2]);
      if(std::abs(wall_normal.normalized().dot(extracted_planes[i].normal)) >= std::cos(config_->ssg_params.wall_normal_thr))  // No need to check for parallel walls
        continue;
      
      if(w_it.second.num_detections <= 0)
        continue;
      
      Eigen::Vector3d pt_q(w_it.second.plane.points->points[0].x, w_it.second.plane.points->points[0].y, w_it.second.plane.points->points[0].z);
      std::vector<pcl::PointXYZ> points_side1, points_side2;
      for(auto pt : extracted_planes[i].points->points)
      {
        Eigen::Vector3d pt_p(pt.x, pt.y, pt.z);
        double dist = w_it.second.plane.normal.dot(pt_p - pt_q);
        if(dist >= 0.0)
          points_side1.push_back(pt);
        else
          points_side2.push_back(pt);
      }
      extracted_planes[i].points->points.clear();
      if(points_side1.size() > points_side2.size())
      {
        extracted_planes[i].points->points.insert(extracted_planes[i].points->points.begin(), points_side1.begin(), points_side1.end());
      }
      else
      {
        extracted_planes[i].points->points.insert(extracted_planes[i].points->points.begin(), points_side2.begin(), points_side2.end());
      }
    }

    if(extracted_planes[i].points->points.size() > 0)
    {
      Eigen::Vector4f centroid;
      pcl::compute3DCentroid(*extracted_planes[i].points, centroid);
      extracted_planes[i].centroid = centroid.head(3).cast<double>();

      extracted_planes[i].coefficients.values[3] = -extracted_planes[i].centroid.dot(extracted_planes[i].normal);

      extracted_planes_copy.push_back(extracted_planes[i]);
    }
  }

  extracted_planes = extracted_planes_copy;
  extracted_planes_copy.clear();

  for(int i=0; i<extracted_planes.size(); ++i)
  {
    bool same_side_check_passed = true;
    for(int j=0; j<extracted_planes.size(); ++j)
    {
      if(i==j) continue;
      Eigen::Vector3d pt_q(extracted_planes[j].points->points[0].x, extracted_planes[j].points->points[0].y, extracted_planes[j].points->points[0].z);
      double dist_1 = extracted_planes[j].normal.dot(extracted_planes[i].centroid - pt_q);
      double dist_2 = extracted_planes[j].normal.dot(current_robot_state_.head(3) - pt_q);
      if(dist_2 * dist_1 < 0)
      {
        same_side_check_passed = false;
        break;
      }
    }
    if(same_side_check_passed && extracted_planes[i].points->points.size() >= config_->ssg_params.min_wall_points) 
      extracted_planes_copy.push_back(extracted_planes[i]);
  }
  extracted_planes = extracted_planes_copy;

  for(auto plane : extracted_planes)
  {
    for(auto pt : plane.points->points) 
    {
      pcl::PointXYZRGB pt_rgb;
      pt_rgb.x = pt.x;
      pt_rgb.y = pt.y;
      pt_rgb.z = pt.z;
      Eigen::Vector3i rgb = getColor(plane.id*30+10);
      pt_rgb.r = rgb(0);
      pt_rgb.g = rgb(1);
      pt_rgb.b = rgb(2);
      wall_cloud->points.push_back(pt_rgb);
    }
  }

  sensor_msgs::PointCloud2 wall_cloud_msg;
  pcl::toROSMsg(*wall_cloud, wall_cloud_msg);
  wall_cloud_msg.header.frame_id = "world";
  wall_cloud_msg.header.stamp = ros::Time::now();
  wall_cloud_pub_.publish(wall_cloud_msg);


  //// MATCH AGAINST WALL INSTANCES //// 
  std::vector<std::pair<int, int>> wall_instances_to_update;
  std::vector<int> matched_wall_instances;
  std::vector<int> wall_instances_updated_this_iter;
  for(Plane &current_plane : extracted_planes)
  {
    Eigen::Vector3d plane_normal(current_plane.coefficients.values[0], current_plane.coefficients.values[1], current_plane.coefficients.values[2]);
    bool wall_found = false;
    for(auto &w_it : tracked_walls_)
    {
      /// Radius filter:
      if((current_robot_state_.head(3) - w_it.second.plane.centroid).norm() > config_->ssg_params.furthest_wall_to_consider)
        continue;
      WallInstance &wall_instance = w_it.second;
      Eigen::Vector3d wall_normal(wall_instance.plane.coefficients.values[0], wall_instance.plane.coefficients.values[1], wall_instance.plane.coefficients.values[2]);
      
      bool same_side_check_passed = true;
      for(auto w_it2 : tracked_walls_)
      {
        /// Radius filter:
        if((current_robot_state_.head(3) - w_it.second.plane.centroid).norm() > config_->ssg_params.furthest_wall_to_consider)
          continue;
        Eigen::Vector3d wall_normal2(w_it2.second.plane.coefficients.values[0],
                                    w_it2.second.plane.coefficients.values[1],
                                    w_it2.second.plane.coefficients.values[2]);
        if(std::abs(wall_normal2.normalized().dot(current_plane.normal)) >= std::cos(config_->ssg_params.wall_normal_thr))  // No need to check for parallel walls
          continue;

        if(w_it2.second.num_detections <= 0)
          continue;
        
        Eigen::Vector3d pt_q(w_it2.second.plane.points->points[0].x,
                             w_it2.second.plane.points->points[0].y,
                             w_it2.second.plane.points->points[0].z);
        double dist_w = w_it2.second.plane.normal.dot(w_it.second.plane.centroid - pt_q);
        double dist_p = w_it2.second.plane.normal.dot(current_plane.centroid - pt_q);
        if(dist_p * dist_w < 0)
        {
          same_side_check_passed = false;
          break;
        }
      }
      for(auto other_plane : extracted_planes)
      {
        Eigen::Vector3d wall_normal2(other_plane.coefficients.values[0],
                                    other_plane.coefficients.values[1],
                                    other_plane.coefficients.values[2]);
        if(std::abs(wall_normal2.normalized().dot(current_plane.normal)) >= std::cos(config_->ssg_params.wall_normal_thr))  // No need to check for parallel walls
          continue;
        
        Eigen::Vector3d pt_q(other_plane.points->points[0].x,
                             other_plane.points->points[0].y,
                             other_plane.points->points[0].z);
        double dist_w = other_plane.normal.dot(w_it.second.plane.centroid - pt_q);
        double dist_p = other_plane.normal.dot(current_plane.centroid - pt_q);
        
        if(dist_p * dist_w < 0)
        {
          same_side_check_passed = false;
          break;
        }
      }
      
      bool distance_check_passed = (wall_instance.plane.centroid - current_plane.centroid).norm() < 2.0;
      if(std::acos(wall_normal.normalized().dot(current_plane.normal)) <= config_->ssg_params.wall_normal_thr
          && same_side_check_passed
        )
      {
        if(std::abs((wall_instance.plane.centroid - current_plane.centroid).dot(current_plane.normal)) <= config_->ssg_params.wall_distance_thr)
        {
          wall_normal = (wall_normal * wall_instance.num_detections + current_plane.normal) / (wall_instance.num_detections + 1);
          wall_instance.plane.coefficients.values[0] = wall_normal(0);
          wall_instance.plane.coefficients.values[1] = wall_normal(1);
          wall_instance.plane.coefficients.values[2] = wall_normal(2);
          wall_instance.plane.normal = wall_normal;

          wall_instance.plane.coefficients.values[3] = 
            (wall_instance.plane.coefficients.values[3] * wall_instance.num_detections + current_plane.coefficients.values[3]) / (wall_instance.num_detections + 1);

          ++wall_instance.num_detections;
          wall_found = true;
          wall_instance.seen = true;
          
          wall_instance.plane.points->points.insert(wall_instance.plane.points->points.begin()+wall_instance.plane.points->points.size(), current_plane.points->points.begin(), current_plane.points->points.end());
          pcl::VoxelGrid<pcl::PointXYZ> vox_filter;
          vox_filter.setInputCloud (wall_instance.plane.points);
          vox_filter.setLeafSize (0.1f, 0.1f, 0.1f);
          vox_filter.filter (*wall_instance.plane.points);

          // wall_vertices_this_iter.push_back(wall_instance.id);

          Eigen::Vector4f centroid;
          pcl::compute3DCentroid(*wall_instance.plane.points, centroid);
          wall_instance.plane.centroid = centroid.head(3).cast<double>();
          wall_instance.plane.coefficients.values[3] = -wall_instance.plane.centroid.dot(wall_normal);

          if(wall_instance.num_detections > config_->ssg_params.min_wall_detections)
          {
            if(wall_instance.just_verified)
            {
              wall_instance.just_verified = false;
              std::shared_ptr<SemanticVertex> new_vertex = comm_->ssg_manager()->initializeNewVertex();
              new_vertex->label = L_WALL;
              new_vertex->state.head(3) = wall_instance.plane.centroid;
              Eigen::Quaterniond quat = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d(1.0, 0.0, 0.0), wall_normal);
              geometry_msgs::Quaternion q;
              q.x = quat.x();
              q.y = quat.y();
              q.z = quat.z();
              q.w = quat.w();
              new_vertex->state(5) = tf::getYaw(q);
              new_vertex->state(3) = 0.0;
              new_vertex->state(4) = 0.0;
              new_vertex->seen = true;
              new_vertex->metadata = &wall_instance;
              comm_->ssg_manager()->addVertex(new_vertex);

              wall_instances_to_update.push_back(std::make_pair(wall_instance.id, new_vertex->id));

              matched_wall_instances.push_back(new_vertex->id);
            }
            else
            {
              matched_wall_instances.push_back(wall_instance.id);
              comm_->ssg_manager()->getVertex(wall_instance.id)->state.head(3) = wall_instance.plane.centroid;
            }
          }
          else
          {
            matched_wall_instances.push_back(wall_instance.id);
          }

          break;
        }
        
      }
    }

    // Add new vertex if no vertex found
    if(!wall_found)
    {
      WallInstance new_wall;
      new_wall.plane = current_plane;
      new_wall.seen = true;
      new_wall.num_detections = 1;
      new_wall.id = low_certainty_wall_count_;
      // new_wall.associated_compartment_vertex_id = current_compartment_vertex_id_;
      tracked_walls_[new_wall.id] = new_wall;
      low_certainty_wall_count_++;
    }
  }


  for(auto w_it : tracked_walls_)
  {
    if(!w_it.second.seen )
    {
      if(w_it.second.num_detections < config_->ssg_params.min_wall_detections)
      {
        --w_it.second.num_detections;
        if(w_it.second.num_detections < 0) w_it.second.num_detections = 0;
      }
    }
    else
    {
      w_it.second.seen = false;
    }
  }

  for(auto it : wall_instances_to_update)
  {
    WallInstance temp = tracked_walls_[it.first];
    tracked_walls_.erase(it.first);
    temp.id = it.second;
    tracked_walls_[it.second] = temp;
    comm_->ssg_manager()->getVertex(it.second)->metadata = &tracked_walls_[it.second];
    for(int l_id : tracked_walls_[it.second].associated_long_vertex_ids)
    {
      tracked_longs_[l_id].associated_wall_vertex_id = it.second;
    }
  }

  //// Merge Walls
  std::vector<int> all_walls, filtered_walls;
  for(auto &w_it : tracked_walls_) 
  {
    all_walls.push_back(w_it.first);
}

  while(!all_walls.empty())
  {
    std::vector<int> group_instances;
    group_instances.push_back(all_walls.front());
    WallInstance inst_to_follow = tracked_walls_[all_walls.front()];
    all_walls.erase(all_walls.begin());

    std::vector<int> remaining_walls;
    for(int i=0; i<all_walls.size(); ++i)
    {
      if(inst_to_follow.plane.normal.dot(tracked_walls_[all_walls[i]].plane.normal) >= std::cos(config_->ssg_params.wall_normal_thr))
      {
        Eigen::Vector3d c1_c2 = tracked_walls_[all_walls[i]].plane.centroid - inst_to_follow.plane.centroid;
        double dist_perp_wall = std::abs(c1_c2.dot(inst_to_follow.plane.normal));
        double dist_along_wall = c1_c2.cross(inst_to_follow.plane.normal).norm();
        if(dist_perp_wall <= config_->ssg_params.wall_distance_thr &&
           dist_along_wall <= config_->ssg_params.along_wall_dist_thr)
        {
          group_instances.push_back(all_walls[i]);
        }
        else
        {
          remaining_walls.push_back(all_walls[i]);
        }
      }
      else
      {
        remaining_walls.push_back(all_walls[i]);
      }
    }
    all_walls = remaining_walls;

    for(int w_id : group_instances)
    {
      if(w_id < low_certainty_wall_count_)
      {
        inst_to_follow = tracked_walls_[w_id];
        break;
      }
    }
    filtered_walls.push_back(inst_to_follow.id); 
  }

  std::map<int, WallInstance> filtered_wall_map;
  for(int w_id : filtered_walls)
  {
    filtered_wall_map[w_id] = tracked_walls_[w_id];
  }
  tracked_walls_ = filtered_wall_map;

  pcl::PointCloud<pcl::PointXYZRGB> tracked_wall_cloud;
  for(auto w_it : tracked_walls_)
  {
    for(auto pt : w_it.second.plane.points->points)
    {
      pcl::PointXYZRGB pt_rgb;
      pt_rgb.x = pt.x;
      pt_rgb.y = pt.y;
      pt_rgb.z = pt.z;
      Eigen::Vector3i rgb = getColor(w_it.first*30+10);
      pt_rgb.r = rgb(0);
      pt_rgb.g = rgb(1);
      pt_rgb.b = rgb(2);
      tracked_wall_cloud.points.push_back(pt_rgb);
    }
  }
  sensor_msgs::PointCloud2 tracked_wall_cloud_msg;
  pcl::toROSMsg(tracked_wall_cloud, tracked_wall_cloud_msg);
  tracked_wall_cloud_msg.header.frame_id = "world";
  tracked_wall_cloud_msg.header.stamp = ros::Time::now();
  tracked_wall_cloud_pub_.publish(tracked_wall_cloud_msg);

  t2 = std::chrono::high_resolution_clock::now();

  //// EXTRACT COMPARTMENTS ////
  if(matched_wall_instances.size() == 4)
  {
    bool proper_detection = true;
    
    std::vector<int> matched_wall_instances_copy = matched_wall_instances;
    std::vector<int> matched_wall_instances_remaining;
    std::map<int, std::vector<int>> parallel_planes;
    while(!matched_wall_instances_copy.empty())
    {
      matched_wall_instances_remaining.clear();
      int id = matched_wall_instances_copy[0];
      matched_wall_instances_copy.erase(matched_wall_instances_copy.begin()+0);
      Eigen::Vector3d plane_normal(tracked_walls_[id].plane.coefficients.values[0],
                                  tracked_walls_[id].plane.coefficients.values[1],
                                  tracked_walls_[id].plane.coefficients.values[2]);
      parallel_planes[id].push_back(id);
      for(int i : matched_wall_instances_copy)
      {
        Eigen::Vector3d candidate_normal(tracked_walls_[i].plane.coefficients.values[0],
                                    tracked_walls_[i].plane.coefficients.values[1],
                                    tracked_walls_[i].plane.coefficients.values[2]);
        if(std::abs(candidate_normal.normalized().dot(plane_normal.normalized())) >= std::cos(config_->ssg_params.wall_normal_thr))
          parallel_planes[id].push_back(i);
        else
          matched_wall_instances_remaining.push_back(i);
      }

      matched_wall_instances_copy = matched_wall_instances_remaining;
    }

    if(parallel_planes.size() != 2) 
    {
      proper_detection = false;
      std::cout << "Not 2 sets of parallel planes" << std::endl;
    }
    else
    {
      for(auto it : parallel_planes)
      {
        if(it.second.size() != 2)
        {
          proper_detection = false;
          std::cout << "Parallel planes set not of length 2" << std::endl;
          break;
        }
      }
    }
    for(int id : matched_wall_instances)
    {
      bool same_side_check_passed = true;
      Eigen::Vector3d plane_normal(tracked_walls_[id].plane.coefficients.values[0],
                                   tracked_walls_[id].plane.coefficients.values[1],
                                   tracked_walls_[id].plane.coefficients.values[2]);
      for(auto w_it : tracked_walls_)
      {
        /// Radius filter:
        if((current_robot_state_.head(3) - w_it.second.plane.centroid).norm() > config_->ssg_params.furthest_wall_to_consider)
          continue;
        Eigen::Vector3d wall_normal(w_it.second.plane.coefficients.values[0],
                                    w_it.second.plane.coefficients.values[1],
                                    w_it.second.plane.coefficients.values[2]);
        if(std::abs(w_it.second.plane.normal.dot(tracked_walls_[id].plane.normal)) >= std::cos(config_->ssg_params.wall_normal_thr)) 
          if(std::abs((w_it.second.plane.centroid - tracked_walls_[id].plane.centroid).dot(tracked_walls_[id].plane.normal)) <= config_->ssg_params.wall_distance_thr)  
            continue;
        
        Eigen::Vector3d pt_q(w_it.second.plane.points->points[0].x,
                             w_it.second.plane.points->points[0].y,
                             w_it.second.plane.points->points[0].z);
        double dist_w = w_it.second.plane.normal.dot(tracked_walls_[id].plane.centroid - pt_q);
        double dist_p = w_it.second.plane.normal.dot(current_robot_state_.head(3) - pt_q);
        if(dist_p >= 0.5 && dist_w >= 0.5)
        {
          if(dist_p * dist_w < 0)
          {
            same_side_check_passed = false;
            break;
          }
        }
      }
      if(!same_side_check_passed)
      {
        std::cout << "Same side check failed" << std::endl;
        proper_detection = false;
        break;
      }
    }

    if(proper_detection)
    {
      Eigen::Vector3d current_compartment_center(0.0, 0.0, 0.0);
      for(int i : matched_wall_instances)
      {
        current_compartment_center += tracked_walls_[i].plane.centroid;
      }

      current_compartment_center /= 4;

      geometry_msgs::PoseStamped ps;
      ps.pose.position.x = current_compartment_center.x();
      ps.pose.position.y = current_compartment_center.y();
      ps.pose.position.z = current_compartment_center.z();
      ps.pose.orientation.w = 1.0;
      ps.header.frame_id = "world";
      current_compartment_pub_.publish(ps);


      /// Update the tracked compartments
      bool new_compartment = true;
      double closest_compartment_dist = config_->ssg_params.compartment_center_thr * 2.0;
      int current_compartment_vertex_id = -1;
      for(auto c_it : tracked_compartments_)
      {
        double dist = (current_compartment_center.head(2) - c_it.second.state.head(2)).norm();
        if(dist <= config_->ssg_params.compartment_center_thr && dist <= closest_compartment_dist)
        {
          closest_compartment_dist = dist;
          current_compartment_vertex_id = c_it.second.id;
          new_compartment = false;
        }
      }

      // If found
      if(new_compartment)
      {
        current_compartment_vertex_id_ = -1;
        
        CompartmentInstance new_compartment;
        new_compartment.id = low_certainty_compartment_count_++;
        new_compartment.state.head(3) = current_compartment_center;
        new_compartment.state.tail(3) << 0.0, 0.0, 0.0;
        tracked_compartments_[new_compartment.id] = new_compartment;
        new_compartment.seen = true;
      }
      // else create a new compartment instance
      else
      {
        tracked_compartments_[current_compartment_vertex_id].state.head(3) *= tracked_compartments_[current_compartment_vertex_id].num_detections;
        tracked_compartments_[current_compartment_vertex_id].state.head(3) += current_compartment_center;
        tracked_compartments_[current_compartment_vertex_id].state.head(3) /= (tracked_compartments_[current_compartment_vertex_id].num_detections + 1);


        tracked_compartments_[current_compartment_vertex_id].seen = true;

        bool update_tracked_compartment = false;
        if(tracked_compartments_[current_compartment_vertex_id].num_detections > config_->ssg_params.min_compartment_detections)
        {
          if(tracked_compartments_[current_compartment_vertex_id].num_detections <= (config_->ssg_params.min_compartment_detections * 5.0))
          {
            ++tracked_compartments_[current_compartment_vertex_id].num_detections;
          }

          if(tracked_compartments_[current_compartment_vertex_id].just_verified)
          {
            tracked_compartments_[current_compartment_vertex_id].just_verified = false;
            std::shared_ptr<SemanticVertex> new_vertex = comm_->ssg_manager()->initializeNewVertex();
            new_vertex->label = L_COMPARTMENT;
            new_vertex->state = tracked_compartments_[current_compartment_vertex_id].state;
            new_vertex->seen = true;
            comm_->ssg_manager()->addVertex(new_vertex);
            current_compartment_vertex_id_ = new_vertex->id;
            if(first_compartment_vertex_id_ < 0) first_compartment_vertex_id_ = current_compartment_vertex_id_;

            update_tracked_compartment = true;

            // Add edges
            for(int id : matched_wall_instances)
            {
              if(id >= low_certainty_wall_count_)
                continue;
              

              Eigen::Vector3d wall_normal(tracked_walls_[id].plane.coefficients.values[0],
                                          tracked_walls_[id].plane.coefficients.values[1],
                                          tracked_walls_[id].plane.coefficients.values[2]);
              // Not pointing towards the compartment center
              if(tracked_walls_[id].plane.normal.dot((new_vertex->state.head(3) - tracked_walls_[id].plane.centroid).normalized()) < 0)  
                continue;
              
              
              // Same side check
              bool same_side_check_passed = true;

              int num_wall_neighbors = 0;
              for(auto n_it : new_vertex->neighbor_map)
              {
                if(comm_->ssg_manager()->getVertex(n_it.first)->label == L_WALL)
                  ++num_wall_neighbors;
              }

              if(num_wall_neighbors < 4)
              {
                std::shared_ptr<EdgeRelation> new_edge = comm_->ssg_manager()->initializeNewEdge();
                new_edge->label = 1;
                new_edge->source_vertex = new_vertex;
                new_edge->target_vertex = comm_->ssg_manager()->getVertex(id);
                new_edge->weight = 1.0;

                comm_->ssg_manager()->addEdge(new_edge);

                // Not useing current_compartment_vertex_id_ (with underscore)
                // because we want the old one that will then be replaced when we update the entry in tracked_compartments
                tracked_compartments_[current_compartment_vertex_id].associated_wall_vertex_ids.insert(id);
              }
            }
          }
          else
          {
            current_compartment_vertex_id_ = current_compartment_vertex_id;
            comm_->ssg_manager()->getVertex(current_compartment_vertex_id_)->state = tracked_compartments_[current_compartment_vertex_id].state;

            // Add edges
            for(int id : matched_wall_instances)
            {
              std::shared_ptr<SemanticVertex> current_vertex = comm_->ssg_manager()->getVertex(current_compartment_vertex_id);
              auto n_it = current_vertex->neighbor_map.find(id);
              if(n_it != current_vertex->neighbor_map.end())  
                continue;  // This wall is already a neighbor

              if(id >= low_certainty_wall_count_)
                continue;
              
              Eigen::Vector3d wall_normal(tracked_walls_[id].plane.coefficients.values[0],
                                          tracked_walls_[id].plane.coefficients.values[1],
                                          tracked_walls_[id].plane.coefficients.values[2]);
              // Not pointing towards the compartment center
              if(tracked_walls_[id].plane.normal.dot((tracked_compartments_[current_compartment_vertex_id].state.head(3) - tracked_walls_[id].plane.centroid).normalized()) < 0)  
                continue;
              

              // Same side check
              bool same_side_check_passed = true;

              int num_wall_neighbors = 0;
              for(auto n_it : current_vertex->neighbor_map)
              {
                if(comm_->ssg_manager()->getVertex(n_it.first)->label == L_WALL)
                  ++num_wall_neighbors;
              }

              if(num_wall_neighbors < 4)
              {
                std::shared_ptr<EdgeRelation> new_edge = comm_->ssg_manager()->initializeNewEdge();
                new_edge->label = 1;
                new_edge->source_vertex = comm_->ssg_manager()->getVertex(current_compartment_vertex_id_);
                new_edge->target_vertex = comm_->ssg_manager()->getVertex(id);
                new_edge->weight = 1.0;

                comm_->ssg_manager()->addEdge(new_edge);

                tracked_compartments_[current_compartment_vertex_id_].associated_wall_vertex_ids.insert(id);
              }
            }
          }
        }
        else
        {
          ++tracked_compartments_[current_compartment_vertex_id].num_detections;
        }

        if(update_tracked_compartment)
        {
          CompartmentInstance temp = tracked_compartments_[current_compartment_vertex_id];
          tracked_compartments_.erase(current_compartment_vertex_id);
          temp.id = current_compartment_vertex_id_;
          tracked_compartments_[current_compartment_vertex_id_] = temp;
        }
        
      }

    }
  }

  // visualizeGraph();
  // return;

  t2 = std::chrono::high_resolution_clock::now();

  //// EXTRACT STIFFENERS ////
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented_cloud_vis (new pcl::PointCloud<pcl::PointXYZRGB> ());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr extracted_lines_cloud_vis (new pcl::PointCloud<pcl::PointXYZRGB> ());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr close_to_wall_cloud_vis (new pcl::PointCloud<pcl::PointXYZRGB> ());
  pcl::PointCloud<pcl::PointXYZ>::Ptr stitched_cloud_copy (new pcl::PointCloud<pcl::PointXYZ> (*stitched_cloud_small_fov));
  std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr> close_to_wall_points;

  auto t1_s = std::chrono::high_resolution_clock::now();
  auto t2_s = t1_s;
  std::map<int, std::map<int, double>> wall_to_wall_distance_map;
  for(auto w_it : tracked_walls_)
  {
    for(auto w_it2 : tracked_walls_)
    {
      if(w_it.first == w_it2.first) continue;  // Same wall
      Eigen::Vector3d pt_q(w_it2.second.plane.points->points[0].x,
                           w_it2.second.plane.points->points[0].y,
                           w_it2.second.plane.points->points[0].z);
      double wall_centroid_dist = w_it2.second.plane.normal.dot(w_it.second.plane.centroid - pt_q);
      wall_to_wall_distance_map[w_it.first][w_it2.first] = wall_centroid_dist;
    }
  }

  t2_s = std::chrono::high_resolution_clock::now();

  // if(matched_wall_instances.size() != 4)
  //   ROS_WARN("Don't have 4 walls. Be carefull");

  if(config_->ssg_params.long_only_on_specific_walls)
  {
    // for(int w_id : matched_wall_instances) 
    // {
    // }

    std::vector<int> filtered_matched_wall_instances;
    double cos_30 = std::cos(30.0*M_PI/180);
    for(int w_id : matched_wall_instances)
    {
      Eigen::Vector3d wall_normal(tracked_walls_[w_id].plane.coefficients.values[0], 
                                  tracked_walls_[w_id].plane.coefficients.values[1], 
                                  tracked_walls_[w_id].plane.coefficients.values[2]);
      if(std::abs(tracked_walls_[w_id].plane.normal.dot(Eigen::Vector3d::UnitX())) < cos_30)
      {
        filtered_matched_wall_instances.push_back(w_id);
      }
    }

    // for(int w_id : filtered_matched_wall_instances) 
    // {
    // }

    matched_wall_instances = filtered_matched_wall_instances;
  }


  
  for(int w_id : matched_wall_instances)
  {
    close_to_wall_points[w_id].reset(new pcl::PointCloud<pcl::PointXYZ>());
  }

  std::vector<int> walls_to_evaluate;
  for(auto w_it : tracked_walls_)
  {
    if((current_robot_state_.head(3) - w_it.second.plane.centroid).norm() <= config_->ssg_params.furthest_wall_to_consider)
    {
      walls_to_evaluate.push_back(w_it.first);
    }
  }


  for(auto pt : stitched_cloud_copy->points)
  {
    if(pt.z > config_->ssg_params.max_long_height)
       continue;
    // std::vector<double> plane_distances;
    std::map<int, double> plane_distances;
    bool keep = false;
    Eigen::Vector3d pt_p(pt.x, pt.y, pt.z);
    for(int i=0; i<matched_wall_instances.size(); ++i)
    {
      Eigen::Vector3d pt_q(tracked_walls_[matched_wall_instances[i]].plane.points->points[0].x,
                           tracked_walls_[matched_wall_instances[i]].plane.points->points[0].y,
                           tracked_walls_[matched_wall_instances[i]].plane.points->points[0].z);
      double plane_dist = tracked_walls_[matched_wall_instances[i]].plane.normal.dot(pt_p - pt_q);
      if(plane_dist < config_->ssg_params.wall_to_long_thr && plane_dist >= config_->ssg_params.wall_to_long_thr_min) keep = true;

      plane_distances[matched_wall_instances[i]] = plane_dist;
    }

    if(!keep)
      continue;

    std::map<int, double> wall_dist_map;
    // for(auto w_it : tracked_walls_)
    for(int w_id : walls_to_evaluate)
    {
      WallInstance w_inst = tracked_walls_[w_id];
      Eigen::Vector3d pt_q(w_inst.plane.points->points[0].x,
                           w_inst.plane.points->points[0].y,
                           w_inst.plane.points->points[0].z);
      double plane_dist = w_inst.plane.normal.dot(pt_p - pt_q);
      wall_dist_map[w_id] = plane_dist;
    }

    // for(int i=0; i<plane_distances.size(); ++i)
    for(auto p_it : plane_distances)
    {
      // if(plane_distances[i] < 0.4 && plane_distances[i] >= config_->ssg_params.wall_to_long_thr_min)
      if(p_it.second < config_->ssg_params.wall_to_long_thr && p_it.second >= config_->ssg_params.wall_to_long_thr_min)
      {
        bool belongs_to_wall = true;
        Eigen::Vector3d plane_normal(tracked_walls_[p_it.first].plane.coefficients.values[0],
                                      tracked_walls_[p_it.first].plane.coefficients.values[1],
                                      tracked_walls_[p_it.first].plane.coefficients.values[2]);
        // for(int j=0; j<plane_distances.size(); ++j)
        // for(auto w_it : tracked_walls_)
        for(int w_id : walls_to_evaluate)
        {
          WallInstance w_inst = tracked_walls_[w_id];

          if(p_it.first == w_id) continue;
          Eigen::Vector3d wall_normal(w_inst.plane.coefficients.values[0],
                                        w_inst.plane.coefficients.values[1],
                                        w_inst.plane.coefficients.values[2]);
          if(std::abs(w_inst.plane.normal.dot(plane_normal.normalized())) >= std::cos(config_->ssg_params.wall_normal_thr))  // No need to check for parallel walls
            continue;
          
          // if(wall_dist_map[w_id] * wall_to_wall_distance_map[p_it.first][w_id] < 0)
          // if((wall_dist_map[w_id] < 0.42 && wall_dist_map[w_id] > 0.0))
          if((wall_dist_map[w_id] < 0.42 && wall_dist_map[w_id] > 0.0) ||
          // if((std::abs(wall_dist_map[w_id]) < 0.42) ||
            wall_dist_map[w_id] * wall_to_wall_distance_map[p_it.first][w_id] < 0)
          {
            belongs_to_wall = false;
            break;
          }
        }
            
            
        if(belongs_to_wall)
        {
          close_to_wall_points[p_it.first]->points.push_back(pt);
          break;
        }
      }
    }
  }

  for(int w_id : matched_wall_instances)
  {
    for(auto pt : close_to_wall_points[w_id]->points)
    {
      pcl::PointXYZRGB pt_rgb;
      pt_rgb.x = pt.x;
      pt_rgb.y = pt.y;
      pt_rgb.z = pt.z;
      Eigen::Vector3i rgb = getColor(w_id*30+10);
      pt_rgb.r = rgb(0);
      pt_rgb.g = rgb(1);
      pt_rgb.b = rgb(2);
      close_to_wall_cloud_vis->points.push_back(pt_rgb);
    }
  }

  // t2_s = std::chrono::high_resolution_clock::now();

  t2 = std::chrono::high_resolution_clock::now();


  std::vector<Line> all_extracted_lines;
  for(int w_id : matched_wall_instances)
  {
    WallInstance &wall_instance = tracked_walls_[w_id];
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_wall_segmented_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());

    /// Extract Lines ///
    std::vector<Line> extracted_lines;
    for(int i=0; i<config_->ssg_params.num_lines_to_extract; ++i)
    {
      if(close_to_wall_points[w_id]->points.empty())
        break;

      
      Eigen::Vector3f desired_dir(wall_instance.plane.coefficients.values[1], -wall_instance.plane.coefficients.values[0], 0.0);
      pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
      // Create the segmentation object
      pcl::SACSegmentation<pcl::PointXYZ> seg;
      seg.setOptimizeCoefficients(true);
      seg.setModelType(pcl::SACMODEL_PARALLEL_LINE);
      // seg.setModelType(pcl::SACMODEL_PLANE);
      seg.setMethodType(pcl::SAC_RANSAC);
      // seg.setMethodType(pcl::SAC_PROSAC);
      seg.setDistanceThreshold(config_->ssg_params.long_width_thr);
      seg.setInputCloud(close_to_wall_points[w_id]);
      seg.setAxis(desired_dir);
      seg.setEpsAngle(config_->ssg_params.long_dir_ang_thr);
      seg.setMaxIterations(1000);
      try 
      {
        // auto t1_s = std::chrono::high_resolution_clock::now();
        // auto t2_s = t1_s;
        seg.segment(*inliers, *coefficients);
        // t2_s = std::chrono::high_resolution_clock::now();
      }
      catch(...)
      {
        // ROS_WARN("Line extraction failed");
        continue;
      }

      if(coefficients->values.size() < 6)
        continue;

      Eigen::Vector3d dir_vec(coefficients->values[3], coefficients->values[4], coefficients->values[5]);

      if(std::abs(desired_dir.cast<double>().dot(dir_vec)) < std::cos(config_->ssg_params.long_dir_ang_thr))
      {
        continue;
      }

      dir_vec.z() = 0.0;
      dir_vec.normalize();
      coefficients->values[3] = dir_vec(0);
      coefficients->values[4] = dir_vec(1);
      coefficients->values[5] = dir_vec(2);
      

      if(coefficients->values.size() == 6 && config_->ssg_params.manual_inlier_classification)
      {
        inliers->indices.clear();
        Eigen::Vector3d line_center(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
        for(int ind=0; ind < close_to_wall_points[w_id]->points.size(); ++ind)
        {
          pcl::PointXYZ pt = close_to_wall_points[w_id]->points[ind];
          Eigen::Vector3d pt_vec(pt.x, pt.y, pt.z);
          Eigen::Vector3d vec_c = pt_vec - line_center;
          double dist = std::abs(dir_vec.cross(vec_c.normalized()).norm());
          if(dist <= config_->ssg_params.long_width_thr)
          {
            inliers->indices.push_back(ind);
          }
        }
      }


      ////////////////////////////////
      if(coefficients->values.size() == 6 && inliers->indices.size() >= config_->ssg_params.min_points_long)
      {
        Line new_line;
        new_line.coefficients = *coefficients;
        new_line.center << coefficients->values[0], coefficients->values[1], coefficients->values[2];
        new_line.direction << coefficients->values[3], coefficients->values[4], coefficients->values[5];
        double min_dist = 999, max_dist = -999;
        for(int j=0; j<inliers->indices.size(); ++j) 
        {
          pcl::PointXYZ pt = close_to_wall_points[w_id]->points[inliers->indices[j]];
          new_line.points->points.push_back(pt);
          pcl::PointXYZRGB pt_rgb;
          pt_rgb.x = pt.x;
          pt_rgb.y = pt.y;
          pt_rgb.z = pt.z;
          Eigen::Vector3i rgb = getColor(i*30+10);
          pt_rgb.r = rgb(0);
          pt_rgb.g = rgb(1);
          pt_rgb.b = rgb(2);
          current_wall_segmented_cloud->points.push_back(pt_rgb);

          //// TODO: This calculation can be combined with the one in the next loop
          Eigen::Vector3d pt_vec(pt.x, pt.y, pt.z);
          pt_vec -= new_line.center;
          double dist = pt_vec.dot(new_line.direction.normalized());
          if(dist > max_dist)
            max_dist = dist;
          if(dist < min_dist)
            min_dist = dist;
        }
        Eigen::Vector3d geom_center(0.0, 0.0, 0.0);
        double d_pc = new_line.center.dot(new_line.direction.normalized());
        geom_center = new_line.center + new_line.direction * ((min_dist + max_dist) / 2.0 - d_pc);

        Eigen::Vector3d wall_normal(wall_instance.plane.coefficients.values[0],
                                    wall_instance.plane.coefficients.values[1],
                                    wall_instance.plane.coefficients.values[2]);
        if((new_line.direction.cross(wall_instance.plane.normal)).dot(Eigen::Vector3d(0.0, 0.0, 1.0)) <= 0.0)
        {
          new_line.direction *= -1.0;
          // new_line.coefficients.values[0] *= -1.0;
          // new_line.coefficients.values[1] *= -1.0;
          // new_line.coefficients.values[2] *= -1.0;
          new_line.coefficients.values[3] *= -1.0;
          new_line.coefficients.values[4] *= -1.0;
          new_line.coefficients.values[5] *= -1.0;
        }

        if(new_line.center.z() < config_->ssg_params.max_long_height
          && new_line.center.z() > config_->ssg_params.min_long_height
          && (max_dist - min_dist) >= config_->ssg_params.min_long_length)
          extracted_lines.push_back(new_line);
      }
      if(inliers->indices.size() > 0 && coefficients->values.size() == 6)
      {
        pcl::PointCloud<pcl::PointXYZ> cloud_F;
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(close_to_wall_points[w_id]);
        extract.setIndices(inliers);
        extract.setNegative(true);
        extract.filter(cloud_F);
        close_to_wall_points[w_id]->swap(cloud_F);
      }
      ////////////////////////////////
      



      


      // close_to_wall_points[w_id]->swap(cloud_F);
      



      
    }

    extracted_lines_cloud_vis->points.insert(
      extracted_lines_cloud_vis->points.begin(), current_wall_segmented_cloud->points.begin(), 
      current_wall_segmented_cloud->points.end()
    );

    /// Update vertices ///
    if(extracted_lines.size() < config_->ssg_params.min_num_longs) 
    {
      // ROS_WARN("Less than three lines (%d)", extracted_lines.size());
      continue;
    }
    
    for(Line &l : extracted_lines)
    {
      all_extracted_lines.push_back(l);
      bool found = false;

      std::vector<std::pair<int, int>> tracked_longs_to_update;
      bool skip = false;
      for(auto &v_it : tracked_longs_)
      {
        if(v_it.second.associated_wall_vertex_id != w_id) continue;

        double normal_dist;
        Eigen::Vector3d c1_c2 = l.center - v_it.second.line.center;
        normal_dist = (c1_c2.cross(v_it.second.line.direction)).norm() / v_it.second.line.direction.norm();
        double long_dir_diff = std::abs(v_it.second.line.direction.normalized().dot(l.direction.normalized()));

        if(normal_dist < config_->ssg_params.min_dist_bet_longs && normal_dist > config_->ssg_params.long_distance_thr)
        {
          skip = true;
          break;
        }


        if((normal_dist) < config_->ssg_params.long_distance_thr &&
            long_dir_diff >= std::cos(config_->ssg_params.long_direction_thr))
        {
          found = true;
          v_it.second.line.direction = (v_it.second.line.direction * v_it.second.num_detections + l.direction) / (v_it.second.num_detections + 1);
          v_it.second.line.coefficients.values[3] = v_it.second.line.direction(0);
          v_it.second.line.coefficients.values[4] = v_it.second.line.direction(1);
          v_it.second.line.coefficients.values[5] = v_it.second.line.direction(2);

          ++v_it.second.num_detections;
          v_it.second.seen = true;
          
          v_it.second.line.points->points.insert(v_it.second.line.points->points.begin()+v_it.second.line.points->points.size(), l.points->points.begin(), l.points->points.end());
          pcl::VoxelGrid<pcl::PointXYZ> vox_filter;
          vox_filter.setInputCloud (v_it.second.line.points);
          vox_filter.setLeafSize (0.1f, 0.1f, 0.1f);
          vox_filter.filter (*v_it.second.line.points);
          // Eigen::Vector4f centroid;
          // pcl::compute3DCentroid(*v_it.second.line.points, centroid);
          
          Eigen::Vector3d centroid(0.0, 0.0, 0.0);
          double min_dist = 9999999999.9, max_dist = -99999999999.9;
          for(auto pt : v_it.second.line.points->points)
          {
            Eigen::Vector3d pt_vec(pt.x, pt.y, pt.z);
            centroid += pt_vec;

            double dist = pt_vec.dot(v_it.second.line.direction.normalized());
            if(dist > max_dist)
              max_dist = dist;
            if(dist < min_dist)
              min_dist = dist;
          }
          centroid /= v_it.second.line.points->points.size();
          Eigen::Vector3d geom_center(0.0, 0.0, 0.0);
          double d_pc = v_it.second.line.center.dot(v_it.second.line.direction.normalized());
          geom_center = v_it.second.line.center + v_it.second.line.direction * ((min_dist + max_dist) / 2.0 - d_pc);
          
          v_it.second.line.center = centroid;
          v_it.second.line.coefficients.values[0] = v_it.second.line.center(0);
          v_it.second.line.coefficients.values[1] = v_it.second.line.center(1);
          v_it.second.line.coefficients.values[2] = v_it.second.line.center(2);
          if(v_it.second.num_detections > config_->ssg_params.min_long_detections)
          {
            if(v_it.second.just_verified)
            {
              auto it = comm_->ssg_manager()->vertices_map_.find(w_id);
              if(it != comm_->ssg_manager()->vertices_map_.end())
              {
                v_it.second.just_verified = false;
                std::shared_ptr<SemanticVertex> new_vertex = comm_->ssg_manager()->initializeNewVertex();
                new_vertex->label = L_LONG;
                new_vertex->state.head(3) = v_it.second.line.center;
                // new_vertex->state.tail(3) = v_it.second.line.direction;
                new_vertex->state(5) = std::atan2(v_it.second.line.direction.y(), v_it.second.line.direction.x());
                new_vertex->state(4) = 0.0;
                new_vertex->state(3) = 0.0;
                // new_vertex->state(3) = 
                //   std::atan2(-v_it.second.line.direction.z(), 
                //     Eigen::Vector2d(v_it.second.line.direction.y(), v_it.second.line.direction.x()).norm());
                new_vertex->seen = true;
                new_vertex->metadata = &v_it.second;
                new_vertex->bbox << max_dist - min_dist, 0.0, 0.0;
                comm_->ssg_manager()->addVertex(new_vertex);

                tracked_longs_to_update.push_back(std::make_pair(v_it.first, new_vertex->id));

                std::shared_ptr<EdgeRelation> new_edge = comm_->ssg_manager()->initializeNewEdge();
                new_edge->label = 1;
                new_edge->source_vertex = new_vertex;
                new_edge->target_vertex = comm_->ssg_manager()->getVertex(w_id);
                new_edge->weight = 1.0;
                comm_->ssg_manager()->addEdge(new_edge);

                tracked_walls_[w_id].associated_long_vertex_ids.insert(new_vertex->id);
              }
            }
            else
            {
              comm_->ssg_manager()->getVertex(v_it.first)->state.head(3) = v_it.second.line.center;
              comm_->ssg_manager()->getVertex(v_it.first)->bbox << max_dist - min_dist, 0.0, 0.0;
            }
          }
          break;
        }
      }

      if(skip)  // This is close enough to another one to be rejected
        continue;

      for(auto p : tracked_longs_to_update)
      {
        LongInstance temp = tracked_longs_[p.first];
        tracked_longs_.erase(p.first);
        temp.id = p.second;
        tracked_longs_[p.second] = temp;
        comm_->ssg_manager()->getVertex(p.second)->metadata = &tracked_longs_[p.second];
      }
      
      if(!found)
      {



        LongInstance new_long;
        new_long.line = l;
        new_long.seen = true;
        new_long.num_detections = 1;
        new_long.id = 999 + low_certainty_longs_count;
        low_certainty_longs_count++;
        new_long.associated_wall_vertex_id = w_id;
        tracked_longs_[new_long.id] = new_long;
      }
    }
  }


  uint32_t d2 = 256*256; 
  for(auto l_it : tracked_longs_)
  {
    if(l_it.second.num_detections >= config_->ssg_params.min_long_detections && l_it.first < init_low_certainty_longs_count)
    {
      for(auto pt : l_it.second.line.points->points)
      {
        pcl::PointXYZRGB pt_rgb;
        pt_rgb.x = pt.x;
        pt_rgb.y = pt.y;
        pt_rgb.z = pt.z;
        Eigen::Vector3i rgb = getColor(l_it.first*30+10);
        uint32_t r = l_it.first / (d2);
        uint32_t remainder = l_it.first - r * (d2);
        pt_rgb.r = (uint8_t)r;
        uint32_t g = remainder / 256;
        remainder = remainder - r * 256;
        pt_rgb.g = (uint8_t)g;
        pt_rgb.b = (uint8_t)(remainder);
        // pt_rgb.r = rgb(0);
        // pt_rgb.g = rgb(1);
        // pt_rgb.b = rgb(2);
        segmented_cloud_vis->points.push_back(pt_rgb);
      }
    }
  }

  sensor_msgs::PointCloud2 segmented_cloud_msg;
  pcl::toROSMsg(*segmented_cloud_vis, segmented_cloud_msg);
  segmented_cloud_msg.header.frame_id = "world";
  segmented_cloud_msg.header.stamp = ros::Time::now();
  combined_segmented_cloud_pub_.publish(segmented_cloud_msg);
  sensor_msgs::PointCloud2 extracted_lines_cloud_msg;
  pcl::toROSMsg(*extracted_lines_cloud_vis, extracted_lines_cloud_msg);
  extracted_lines_cloud_msg.header.frame_id = "world";
  extracted_lines_cloud_msg.header.stamp = ros::Time::now();
  extracted_lines_cloud_pub_.publish(extracted_lines_cloud_msg);
  sensor_msgs::PointCloud2 close_to_wall_cloud_msg;
  pcl::toROSMsg(*close_to_wall_cloud_vis, close_to_wall_cloud_msg);
  close_to_wall_cloud_msg.header.frame_id = "world";
  close_to_wall_cloud_msg.header.stamp = ros::Time::now();
  close_to_wall_cloud_pub_.publish(close_to_wall_cloud_msg);
  // std::map<int, std::vector<int>> m;
  // m.begin()->second.push_back

  if(current_compartment_vertex_id_ >= 0)
  {
    geometry_msgs::PoseArray longs_in_this_compartment;
    for(int w_id : tracked_compartments_[current_compartment_vertex_id_].associated_wall_vertex_ids)
    {
      for(int l_id : tracked_walls_[w_id].associated_long_vertex_ids)
      {
        geometry_msgs::Pose long_msg;
        long_msg.position.x = tracked_longs_[l_id].line.center.x();
        long_msg.position.y = tracked_longs_[l_id].line.center.y();
        long_msg.position.z = tracked_longs_[l_id].line.center.z();
        long_msg.orientation.w = std::atan2(tracked_longs_[l_id].line.direction.y(), tracked_longs_[l_id].line.direction.x());

        long_msg.orientation.x = -comm_->ssg_manager()->getVertex(l_id)->bbox[0]/2.0;
        long_msg.orientation.y = comm_->ssg_manager()->getVertex(l_id)->bbox[0]/2.0;

        longs_in_this_compartment.poses.push_back(long_msg);
      }
    }
    
    current_compartment_longs_pub_.publish(longs_in_this_compartment);

  }

  t2 = std::chrono::high_resolution_clock::now();
  double proc_time = std::chrono::duration<double, std::milli>(t2 - t1).count();

  visualizeGraph();
}

geometry_msgs::PoseArray BWTSSGReal::getAllLongs()
{
  geometry_msgs::PoseArray longs;
  for(auto l_it : tracked_longs_)
  {
    if(l_it.second.num_detections >= config_->ssg_params.min_long_detections_for_overlap)
    {
      geometry_msgs::Pose long_msg;
      long_msg.position.x = l_it.second.line.center.x();
      long_msg.position.y = l_it.second.line.center.y();
      long_msg.position.z = l_it.second.line.center.z();
      long_msg.orientation.w = std::atan2(l_it.second.line.direction.y(), l_it.second.line.direction.x());

      double min_dist = 999, max_dist = -999;
      for(auto pt : l_it.second.line.points->points)
      {
        Eigen::Vector3d pt_vec(pt.x, pt.y, pt.z);
        pt_vec -= l_it.second.line.center;
        double dist = pt_vec.dot(l_it.second.line.direction.normalized());
        if(dist > max_dist)
          max_dist = dist;
        if(dist < min_dist)
          min_dist = dist;
      }
      double length = max_dist - min_dist;
      long_msg.orientation.x = -length / 2.0;
      long_msg.orientation.y = length / 2.0;
      longs.poses.push_back(long_msg);
    }
  }

  return longs;
}

geometry_msgs::PoseArray BWTSSGReal::getCompartmentLongs(int compartment_vertex_id)
{
  geometry_msgs::PoseArray longs_in_this_compartment;
  
  int v_id = compartment_vertex_id;
  if(v_id < 0)
  {
    if(current_compartment_vertex_id_ >= 0)
      v_id = current_compartment_vertex_id_;
    else
      return longs_in_this_compartment;
  }

  for(int w_id : tracked_compartments_[v_id].associated_wall_vertex_ids)
  {
    for(int l_id : tracked_walls_[w_id].associated_long_vertex_ids)
    {
      geometry_msgs::Pose long_msg;
      long_msg.position.x = tracked_longs_[l_id].line.center.x();
      long_msg.position.y = tracked_longs_[l_id].line.center.y();
      long_msg.position.z = tracked_longs_[l_id].line.center.z();
      long_msg.orientation.w = std::atan2(tracked_longs_[l_id].line.direction.y(), tracked_longs_[l_id].line.direction.x());

      long_msg.orientation.x = -comm_->ssg_manager()->getVertex(l_id)->bbox[0]/2.0;
      long_msg.orientation.y = comm_->ssg_manager()->getVertex(l_id)->bbox[0]/2.0;

      longs_in_this_compartment.poses.push_back(long_msg);
    }
  }

  return longs_in_this_compartment;
}

geometry_msgs::PoseArray BWTSSGReal::getFirstCompartmentLongs()
{
  geometry_msgs::PoseArray longs_in_this_compartment;
  if(first_compartment_vertex_id_ < 0)
  {
    ROS_WARN("NO COMPARTMENT DETECTED YET");
    return longs_in_this_compartment;
  }

  for(int w_id : tracked_compartments_[first_compartment_vertex_id_].associated_wall_vertex_ids)
  {
    for(int l_id : tracked_walls_[w_id].associated_long_vertex_ids)
    {
      geometry_msgs::Pose long_msg;
      long_msg.position.x = tracked_longs_[l_id].line.center.x();
      long_msg.position.y = tracked_longs_[l_id].line.center.y();
      long_msg.position.z = tracked_longs_[l_id].line.center.z();
      long_msg.orientation.w = std::atan2(tracked_longs_[l_id].line.direction.y(), tracked_longs_[l_id].line.direction.x());

      long_msg.orientation.x = -comm_->ssg_manager()->getVertex(l_id)->bbox[0]/2.0;
      long_msg.orientation.y = comm_->ssg_manager()->getVertex(l_id)->bbox[0]/2.0;

      longs_in_this_compartment.poses.push_back(long_msg);
    }
  }

  return longs_in_this_compartment;
}

Eigen::Vector6d BWTSSGReal::getFirstCompartmentState()
{
  if(first_compartment_vertex_id_ < 0)
  {
    ROS_WARN("NO COMPARTMENT DETECTED YET");
    Eigen::Vector6d empty_state;
    return empty_state;
  }
  else
  {
    return comm_->ssg_manager()->getVertex(first_compartment_vertex_id_)->state;
  }
}

void BWTSSGReal::visualizeGraph()
{
  visualization_msgs::MarkerArray marker_array;

  // Plot all edges
  visualization_msgs::Marker edge_marker;
  edge_marker.header.stamp = ros::Time::now();
  edge_marker.header.seq = 0;
  edge_marker.header.frame_id = "world";
  edge_marker.id = 0;
  edge_marker.ns = "edges";
  edge_marker.action = visualization_msgs::Marker::ADD;
  edge_marker.type = visualization_msgs::Marker::LINE_LIST;
  edge_marker.scale.x = 0.04;
  edge_marker.color.r = 200.0 / 255.0;
  edge_marker.color.g = 100.0 / 255.0;
  edge_marker.color.b = 0.0;
  edge_marker.color.a = 1.0;
  edge_marker.lifetime = ros::Duration(0.0);
  edge_marker.frame_locked = false;

  for (auto e_it : comm_->ssg_manager()->edge_map_) {
    geometry_msgs::Point p1;
    p1.x = e_it.second->source_vertex->state[0];
    p1.y = e_it.second->source_vertex->state[1];
    p1.z = e_it.second->source_vertex->state[2];
    geometry_msgs::Point p2;
    p2.x = e_it.second->target_vertex->state[0];
    p2.y = e_it.second->target_vertex->state[1];
    p2.z = e_it.second->target_vertex->state[2];
    edge_marker.points.push_back(p1);
    edge_marker.points.push_back(p2);
  }
  if(!edge_marker.points.empty())
    marker_array.markers.push_back(edge_marker);

  // Plot all vertices
  visualization_msgs::Marker vertex_marker;
  vertex_marker.header.stamp = ros::Time::now();
  vertex_marker.header.seq = 0;
  vertex_marker.header.frame_id = "world";
  vertex_marker.id = 0;
  vertex_marker.ns = "vertices";
  vertex_marker.action = visualization_msgs::Marker::ADD;
  vertex_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  vertex_marker.scale.x = 0.3;
  vertex_marker.scale.y = 0.3;
  vertex_marker.scale.z = 0.3;
  vertex_marker.lifetime = ros::Duration(0.0);
  vertex_marker.frame_locked = false;

  int marker_id = 0;
  for (auto v_it : comm_->ssg_manager()->vertices_map_) {
    geometry_msgs::Point p1;
    p1.x = v_it.second->state[0];
    p1.y = v_it.second->state[1];
    p1.z = v_it.second->state[2];
    std_msgs::ColorRGBA rgb_msg;
    Eigen::Vector3i rgb = getColor(v_it.second->label);
    rgb_msg.r = (double)rgb[0] / 255.0;
    rgb_msg.g = (double)rgb[1] / 255.0;
    rgb_msg.b = (double)rgb[2] / 255.0;
    rgb_msg.a = 1.0;
    vertex_marker.colors.push_back(rgb_msg);
    vertex_marker.points.push_back(p1);

    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.header.seq = 0;
    marker.header.frame_id = "world";
    marker.ns = "ids";
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.scale.z = 0.15;  // text height
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration(0.0);
    marker.frame_locked = false;
    marker.pose.position.x = v_it.second->state[0];
    marker.pose.position.y = v_it.second->state[1];
    marker.pose.position.z = v_it.second->state[2] + 0.1;
    // Show vertex gains.
    std::string text_display =
        std::to_string(v_it.second->id) + "," + std::to_string(v_it.second->label) + "," + std::to_string(v_it.second->associated_compartment_vertex_id);

    marker.text = text_display;
    marker.id = marker_id++;
    marker_array.markers.push_back(marker);
  }
  marker_array.markers.push_back(vertex_marker);

  for(auto v_it : comm_->ssg_manager()->vertices_map_)
  {
    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.header.seq = 0;
    marker.header.frame_id = "world";
    marker.ns = "headings";
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.scale.x = 0.3;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    std_msgs::ColorRGBA rgb_msg;
    Eigen::Vector3i rgb = getColor(v_it.second->label);
    marker.color.r = (float)rgb[0] / 255.0;
    marker.color.g = (float)rgb[1] / 255.0;
    marker.color.b = (float)rgb[2] / 255.0;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration(0.0);
    marker.frame_locked = false;
    convert(v_it.second->state, marker.pose);
    marker.id = marker_id++;
    marker_array.markers.push_back(marker);
  }

  graph_vis_pub_.publish(marker_array);
}


void BWTSSGReal::visualizeLines(std::vector<Line> lines)
{
  visualization_msgs::MarkerArray marker_array;

  int marker_id = 999;
  for(auto l : lines)
  {
    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.header.seq = 0;
    marker.header.frame_id = "world";
    marker.ns = "lines";
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.scale.x = 0.3;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.r = 255 / 255.0;
    marker.color.g = 0.0 / 255.0;
    marker.color.b = 0.0 / 255.0;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration(0.0);
    marker.frame_locked = false;
    geometry_msgs::Point p1;
    p1.x = l.center.x();
    p1.y = l.center.y();
    p1.z = l.center.z();
    marker.points.push_back(p1);
    geometry_msgs::Point p2;
    p2.x = l.direction.x() + l.center.x();
    p2.y = l.direction.y() + l.center.y();
    p2.z = l.direction.z() + l.center.z();
    marker.points.push_back(p2);
    marker.id = marker_id++;
    marker_array.markers.push_back(marker);
  }

  line_pub_.publish(marker_array);
}

double BWTSSGReal::compareOverlap(std::shared_ptr<SSGManager> candidate_graph)
{
  std::cout << "Performaing overlap check" << std::endl;
  double match_score = 0.0;

  int num_longs = 0;
  for(auto cv_it : candidate_graph->vertices_map_)
  {
    if(cv_it.second->label != L_LONG)
      continue;
    ++num_longs;


    Eigen::Vector3d direction(std::cos(cv_it.second->state[5]), std::sin(cv_it.second->state[5]), 0.0);
    bool match_found = false;
    int best_match_id = -1;
    double best_match_cost = config_->ssg_params.long_distance_thr * 2.0;
    for(auto l_it : tracked_longs_)
    {
      if(l_it.second.num_detections < config_->ssg_params.min_long_detections_for_overlap) 
        continue;
      
      double center_dist = (cv_it.second->state.head(3) - l_it.second.line.center).norm();

      if(center_dist > cv_it.second->bbox.x())
        continue;
      

      double cost = direction.normalized().cross(l_it.second.line.center - cv_it.second->state.head(3)).norm();
      if(cost < best_match_cost)
      {
        best_match_cost = cost;
        best_match_id = l_it.first;
        match_found = true;
      }
    }

    if(match_found)
    {
      match_score += 1.0;
    }
  }

  match_score /= num_longs;

  return match_score;
}