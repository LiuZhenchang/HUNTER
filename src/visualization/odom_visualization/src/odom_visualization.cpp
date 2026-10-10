#include "odom_visualization.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>
#include <functional>
#include <iostream>
#include <map>
#include <string>
#include "pose_utils/pose_utils.h"

using namespace arma;
using namespace std;

OdomVisualization::OdomVisualization()
    : node_(rclcpp::Node::make_shared("odom_visualization")),
      isOriginSet_(false),
      poseOrigin_(6, fill::zeros),
      sensor_(node_) {
  node_->declare_parameter("mesh_resource", "package://odom_visualization/meshes/hummingbird.mesh");
  node_->declare_parameter("color/r", 1.0);
  node_->declare_parameter("color/g", 0.0);
  node_->declare_parameter("color/b", 0.0);
  node_->declare_parameter("color/a", 1.0);
  node_->declare_parameter("origin", false);
  node_->declare_parameter("robot_scale", 2.0);
  node_->declare_parameter("frame_id", "world");
  node_->declare_parameter("cross_config", false);
  node_->declare_parameter("tf45", false);
  node_->declare_parameter("covariance_scale", 100.0);
  node_->declare_parameter("covariance_position", false);
  node_->declare_parameter("covariance_velocity", false);
  node_->declare_parameter("covariance_color", false);
  node_->declare_parameter("drone_id", -1);
  node_->declare_parameter("traj_scale", 1.0);
  node_->declare_parameter("drone_num", 4);

  node_->get_parameter("mesh_resource", mesh_resource_);
  node_->get_parameter("color/r", color_r_);
  node_->get_parameter("color/g", color_g_);
  node_->get_parameter("color/b", color_b_);
  node_->get_parameter("color/a", color_a_);
  node_->get_parameter("origin", origin_);
  node_->get_parameter("robot_scale", scale_);
  node_->get_parameter("frame_id", frame_id_);
  node_->get_parameter("cross_config", cross_config_);
  node_->get_parameter("tf45", tf45_);
  node_->get_parameter("covariance_scale", cov_scale_);
  node_->get_parameter("covariance_position", cov_pos_);
  node_->get_parameter("covariance_velocity", cov_vel_);
  node_->get_parameter("covariance_color", cov_color_);
  node_->get_parameter("drone_id", drone_id_);
  node_->get_parameter("traj_scale", traj_scale_);
  node_->get_parameter("drone_num", drone_num_);

  auto sensor_qos = rclcpp::SensorDataQoS();
  auto latched_qos = rclcpp::QoS(rclcpp::KeepLast(10)).transient_local().reliable();
  auto default_qos = rclcpp::QoS(rclcpp::KeepLast(10));

  subs_odom_.resize(drone_num_);
  for (int i = 0; i < drone_num_; i++) {
    std::string topic_name = "/drone_" + std::to_string(i) + "_communication_data";
    subs_odom_[i] = node_->create_subscription<custom_msgs::msg::CommunicationData>(
        topic_name, sensor_qos, [this, i](const custom_msgs::msg::CommunicationData::SharedPtr msg) {
          this->communicationDataCallback(msg, i);
        });
  }

  // sub_cmd_ = node_->create_subscription<quadrotor_msgs::msg::PositionCommand>("cmd", sensor_qos,
  // std::bind(&OdomVisualization::cmdCallback, this, std::placeholders::_1));
  posePub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("pose", latched_qos);
  pathPub_ = node_->create_publisher<nav_msgs::msg::Path>("path", latched_qos);
  velPub_ = node_->create_publisher<visualization_msgs::msg::Marker>("velocity", default_qos);
  covPub_ = node_->create_publisher<visualization_msgs::msg::Marker>("covariance", default_qos);
  covVelPub_ = node_->create_publisher<visualization_msgs::msg::Marker>("covariance_velocity", default_qos);
  trajPub_ = node_->create_publisher<visualization_msgs::msg::Marker>("trajectory", default_qos);
  sensorPub_ = node_->create_publisher<visualization_msgs::msg::Marker>("sensor", default_qos);
  meshPub_ = node_->create_publisher<visualization_msgs::msg::Marker>("robot", default_qos);
  heightPub_ = node_->create_publisher<sensor_msgs::msg::Range>("height", default_qos);
  broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
}

OdomVisualization::~OdomVisualization() {}

void OdomVisualization::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  if (msg->header.frame_id == "null")
    return;

  const rclcpp::Time current_stamp(msg->header.stamp);

  colvec pose(6);
  pose(0) = msg->pose.pose.position.x;
  pose(1) = msg->pose.pose.position.y;
  pose(2) = msg->pose.pose.position.z;
  colvec q(4);
  q(0) = msg->pose.pose.orientation.w;
  q(1) = msg->pose.pose.orientation.x;
  q(2) = msg->pose.pose.orientation.y;
  q(3) = msg->pose.pose.orientation.z;
  pose.rows(3, 5) = R_to_ypr(quaternion_to_R(q));
  colvec vel(3);
  vel(0) = msg->twist.twist.linear.x;
  vel(1) = msg->twist.twist.linear.y;
  vel(2) = msg->twist.twist.linear.z;

  if (origin_ && !isOriginSet_) {
    isOriginSet_ = true;
    poseOrigin_ = pose;
  }
  if (origin_) {
    vel = trans(ypr_to_R(pose.rows(3, 5))) * vel;
    pose = pose_update(pose_inverse(poseOrigin_), pose);
    vel = ypr_to_R(pose.rows(3, 5)) * vel;
  }

  poseROS_.header = msg->header;
  poseROS_.header.frame_id = "world";
  poseROS_.pose.position.x = pose(0);
  poseROS_.pose.position.y = pose(1);
  poseROS_.pose.position.z = pose(2);
  q = R_to_quaternion(ypr_to_R(pose.rows(3, 5)));
  poseROS_.pose.orientation.w = q(0);
  poseROS_.pose.orientation.x = q(1);
  poseROS_.pose.orientation.y = q(2);
  poseROS_.pose.orientation.z = q(3);
  posePub_->publish(poseROS_);

  // Velocity
  colvec yprVel(3);
  yprVel(0) = atan2(vel(1), vel(0));
  yprVel(1) = -atan2(vel(2), norm(vel.rows(0, 1), 2));
  yprVel(2) = 0;
  q = R_to_quaternion(ypr_to_R(yprVel));
  velROS_.header.frame_id = string("world");
  velROS_.header.stamp = msg->header.stamp;
  velROS_.ns = string("velocity");
  velROS_.id = 0;
  velROS_.type = visualization_msgs::msg::Marker::ARROW;
  velROS_.action = visualization_msgs::msg::Marker::ADD;
  velROS_.pose.position.x = pose(0);
  velROS_.pose.position.y = pose(1);
  velROS_.pose.position.z = pose(2);
  velROS_.pose.orientation.w = q(0);
  velROS_.pose.orientation.x = q(1);
  velROS_.pose.orientation.y = q(2);
  velROS_.pose.orientation.z = q(3);
  velROS_.scale.x = norm(vel, 2);
  velROS_.scale.y = 0.05;
  velROS_.scale.z = 0.05;
  velROS_.color.a = 1.0;
  velROS_.color.r = color_r_;
  velROS_.color.g = color_g_;
  velROS_.color.b = color_b_;
  velPub_->publish(velROS_);

  // Path
  static rclcpp::Time prevt(0, 0, RCL_ROS_TIME);
  static bool path_initialized = false;
  if (!path_initialized || (current_stamp - prevt).seconds() > 0.1) {
    prevt = current_stamp;
    path_initialized = true;
    pathROS_.header = poseROS_.header;
    pathROS_.poses.push_back(poseROS_);
    pathPub_->publish(pathROS_);
  }

  // Covariance color
  double r = 1;
  double g = 1;
  double b = 1;
  bool G = msg->twist.covariance[33];
  bool V = msg->twist.covariance[34];
  bool L = msg->twist.covariance[35];
  if (cov_color_) {
    r = G;
    g = V;
    b = L;
  }

  // Covariance Position
  if (cov_pos_) {
    mat P(6, 6);
    for (int j = 0; j < 6; j++)
      for (int i = 0; i < 6; i++)
        P(i, j) = msg->pose.covariance[i + j * 6];
    colvec eigVal;
    mat eigVec;
    eig_sym(eigVal, eigVec, P.submat(0, 0, 2, 2));
    if (det(eigVec) < 0) {
      for (int k = 0; k < 3; k++) {
        mat eigVecRev = eigVec;
        eigVecRev.col(k) *= -1;
        if (det(eigVecRev) > 0) {
          eigVec = eigVecRev;
          break;
        }
      }
    }
    covROS_.header.frame_id = string("world");
    covROS_.header.stamp = msg->header.stamp;
    covROS_.ns = string("covariance");
    covROS_.id = 0;
    covROS_.type = visualization_msgs::msg::Marker::SPHERE;
    covROS_.action = visualization_msgs::msg::Marker::ADD;
    covROS_.pose.position.x = pose(0);
    covROS_.pose.position.y = pose(1);
    covROS_.pose.position.z = pose(2);
    q = R_to_quaternion(eigVec);
    covROS_.pose.orientation.w = q(0);
    covROS_.pose.orientation.x = q(1);
    covROS_.pose.orientation.y = q(2);
    covROS_.pose.orientation.z = q(3);
    covROS_.scale.x = sqrt(eigVal(0)) * cov_scale_;
    covROS_.scale.y = sqrt(eigVal(1)) * cov_scale_;
    covROS_.scale.z = sqrt(eigVal(2)) * cov_scale_;
    covROS_.color.a = 0.4;
    covROS_.color.r = r * 0.5;
    covROS_.color.g = g * 0.5;
    covROS_.color.b = b * 0.5;
    covPub_->publish(covROS_);
  }

  // Covariance Velocity
  if (cov_vel_) {
    mat P(3, 3);
    for (int j = 0; j < 3; j++)
      for (int i = 0; i < 3; i++)
        P(i, j) = msg->twist.covariance[i + j * 6];
    mat R = ypr_to_R(pose.rows(3, 5));
    P = R * P * trans(R);
    colvec eigVal;
    mat eigVec;
    eig_sym(eigVal, eigVec, P);
    if (det(eigVec) < 0) {
      for (int k = 0; k < 3; k++) {
        mat eigVecRev = eigVec;
        eigVecRev.col(k) *= -1;
        if (det(eigVecRev) > 0) {
          eigVec = eigVecRev;
          break;
        }
      }
    }
    covVelROS_.header.frame_id = string("world");
    covVelROS_.header.stamp = msg->header.stamp;
    covVelROS_.ns = string("covariance_velocity");
    covVelROS_.id = 0;
    covVelROS_.type = visualization_msgs::msg::Marker::SPHERE;
    covVelROS_.action = visualization_msgs::msg::Marker::ADD;
    covVelROS_.pose.position.x = pose(0);
    covVelROS_.pose.position.y = pose(1);
    covVelROS_.pose.position.z = pose(2);
    q = R_to_quaternion(eigVec);
    covVelROS_.pose.orientation.w = q(0);
    covVelROS_.pose.orientation.x = q(1);
    covVelROS_.pose.orientation.y = q(2);
    covVelROS_.pose.orientation.z = q(3);
    covVelROS_.scale.x = sqrt(eigVal(0)) * cov_scale_;
    covVelROS_.scale.y = sqrt(eigVal(1)) * cov_scale_;
    covVelROS_.scale.z = sqrt(eigVal(2)) * cov_scale_;
    covVelROS_.color.a = 0.4;
    covVelROS_.color.r = r;
    covVelROS_.color.g = g;
    covVelROS_.color.b = b;
    covVelPub_->publish(covVelROS_);
  }

  // Color Coded Trajectory
  static colvec ppose = pose;
  static rclcpp::Time pt(0, 0, RCL_ROS_TIME);
  static bool traj_initialized = false;
  if (!traj_initialized) {
    pt = current_stamp;
    traj_initialized = true;
  }
  if ((current_stamp - pt).seconds() > 0.5) {
    trajROS_.header.frame_id = string("world");
    trajROS_.header.stamp = node_->now();
    trajROS_.ns = string("trajectory");
    trajROS_.type = visualization_msgs::msg::Marker::LINE_LIST;
    trajROS_.action = visualization_msgs::msg::Marker::ADD;
    trajROS_.pose.position.x = 0;
    trajROS_.pose.position.y = 0;
    trajROS_.pose.position.z = 0;
    trajROS_.pose.orientation.w = 1;
    trajROS_.pose.orientation.x = 0;
    trajROS_.pose.orientation.y = 0;
    trajROS_.pose.orientation.z = 0;
    trajROS_.scale.x = traj_scale_;
    trajROS_.scale.y = 0;
    trajROS_.scale.z = 0;
    trajROS_.color.r = 0.0;
    trajROS_.color.g = 1.0;
    trajROS_.color.b = 0.0;
    trajROS_.color.a = 0.8;
    geometry_msgs::msg::Point p;
    p.x = ppose(0);
    p.y = ppose(1);
    p.z = ppose(2);
    trajROS_.points.push_back(p);
    p.x = pose(0);
    p.y = pose(1);
    p.z = pose(2);
    trajROS_.points.push_back(p);
    std_msgs::msg::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = 1;
    trajROS_.colors.push_back(color);
    trajROS_.colors.push_back(color);
    ppose = pose;
    pt = current_stamp;
    trajPub_->publish(trajROS_);
  }

  // Sensor availability
  sensorROS_.header.frame_id = string("world");
  sensorROS_.header.stamp = msg->header.stamp;
  sensorROS_.ns = string("sensor");
  sensorROS_.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  sensorROS_.action = visualization_msgs::msg::Marker::ADD;
  sensorROS_.pose.position.x = pose(0);
  sensorROS_.pose.position.y = pose(1);
  sensorROS_.pose.position.z = pose(2) + 1.0;
  sensorROS_.pose.orientation.w = q(0);
  sensorROS_.pose.orientation.x = q(1);
  sensorROS_.pose.orientation.y = q(2);
  sensorROS_.pose.orientation.z = q(3);
  string strG = G ? string(" GPS ") : string("");
  string strV = V ? string(" Vision ") : string("");
  string strL = L ? string(" Laser ") : string("");
  sensorROS_.text = "| " + strG + strV + strL + " |";
  sensorROS_.color.a = 1.0;
  sensorROS_.color.r = 1.0;
  sensorROS_.color.g = 1.0;
  sensorROS_.color.b = 1.0;
  sensorROS_.scale.z = 0.5;
  sensorPub_->publish(sensorROS_);

  // Laser height measurement
  double H = msg->twist.covariance[32];
  heightROS_.header.frame_id = string("height");
  heightROS_.header.stamp = msg->header.stamp;
  heightROS_.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
  heightROS_.field_of_view = 5.0 * M_PI / 180.0;
  heightROS_.min_range = -100;
  heightROS_.max_range = 100;
  heightROS_.range = H;
  heightPub_->publish(heightROS_);

  // Mesh model
  meshROS_.header.frame_id = frame_id_;
  meshROS_.header.stamp = msg->header.stamp;
  meshROS_.ns = "mesh";
  meshROS_.id = 0;
  meshROS_.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
  meshROS_.action = visualization_msgs::msg::Marker::ADD;
  meshROS_.pose.position.x = msg->pose.pose.position.x;
  meshROS_.pose.position.y = msg->pose.pose.position.y;
  meshROS_.pose.position.z = msg->pose.pose.position.z;
  q(0) = msg->pose.pose.orientation.w;
  q(1) = msg->pose.pose.orientation.x;
  q(2) = msg->pose.pose.orientation.y;
  q(3) = msg->pose.pose.orientation.z;
  if (cross_config_) {
    colvec ypr = R_to_ypr(quaternion_to_R(q));
    ypr(0) += 45.0 * M_PI / 180.0;
    q = R_to_quaternion(ypr_to_R(ypr));
  }
  meshROS_.pose.orientation.w = q(0);
  meshROS_.pose.orientation.x = q(1);
  meshROS_.pose.orientation.y = q(2);
  meshROS_.pose.orientation.z = q(3);
  meshROS_.scale.x = scale_;
  meshROS_.scale.y = scale_;
  meshROS_.scale.z = scale_;
  meshROS_.color.a = color_a_;
  meshROS_.color.r = color_r_;
  meshROS_.color.g = color_g_;
  meshROS_.color.b = color_b_;
  meshROS_.mesh_resource = mesh_resource_;
  meshPub_->publish(meshROS_);

  // TF for raw sensor visualization
  if (tf45_) {
    geometry_msgs::msg::Quaternion base_quat;
    base_quat.w = q(0);
    base_quat.x = q(1);
    base_quat.y = q(2);
    base_quat.z = q(3);

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = msg->header.stamp;
    transform.header.frame_id = "world";
    transform.child_frame_id = drone_id_ == -1 ? "base" : "base" + std::to_string(drone_id_);
    transform.transform.translation.x = pose(0);
    transform.transform.translation.y = pose(1);
    transform.transform.translation.z = pose(2);
    transform.transform.rotation = base_quat;

    tf2::Quaternion q45_tf;
    q45_tf.setRPY(0.0, 0.0, 45.0 * M_PI / 180.0);
    geometry_msgs::msg::TransformStamped transform45;
    transform45.header.stamp = msg->header.stamp;
    transform45.header.frame_id = transform.child_frame_id;
    transform45.transform.translation.x = 0.0;
    transform45.transform.translation.y = 0.0;
    transform45.transform.translation.z = 0.0;
    transform45.transform.rotation = tf2::toMsg(q45_tf);

    tf2::Quaternion q90_tf;
    q90_tf.setRPY(0.0, 90.0 * M_PI / 180.0, 0.0);
    geometry_msgs::msg::TransformStamped transform90;
    transform90.header.stamp = msg->header.stamp;
    transform90.header.frame_id = transform.child_frame_id;
    transform90.transform.translation.x = 0.0;
    transform90.transform.translation.y = 0.0;
    transform90.transform.translation.z = 0.0;
    transform90.transform.rotation = tf2::toMsg(q90_tf);

    const string base_s = transform.child_frame_id;
    const string laser_s = drone_id_ == -1 ? string("laser") : string("laser") + std::to_string(drone_id_);
    const string vision_s = drone_id_ == -1 ? string("vision") : string("vision") + std::to_string(drone_id_);
    const string height_s = drone_id_ == -1 ? string("height") : string("height") + std::to_string(drone_id_);

    broadcaster_->sendTransform(transform);
    transform45.child_frame_id = laser_s;
    broadcaster_->sendTransform(transform45);
    transform45.child_frame_id = vision_s;
    broadcaster_->sendTransform(transform45);
    transform90.child_frame_id = height_s;
    broadcaster_->sendTransform(transform90);
  }
}

void OdomVisualization::communicationDataCallback(const custom_msgs::msg::CommunicationData::SharedPtr msg,
                                                  int agent_id) {
  if (msg->agents_history.empty()) {
    return;
  }
  if (msg->agents_history.back().history.empty()) {
    return;
  }
  if (agent_id < 0 || agent_id >= static_cast<int>(msg->agents_history.back().history.size())) {
    return;
  }
  const custom_msgs::msg::AgentState *agent_state = nullptr;
  for (const auto &state : msg->agents_history.back().history) {
    if (state.agent_id == agent_id) {
      agent_state = &state;
      break;
    }
  }
  if (agent_state == nullptr) {
    std::cout << "ERROR: agent_id=" << agent_id << " not found in message history!" << std::endl;
    return;
  }

  odom.header.frame_id = "world";
  odom.header.stamp = agent_state->time_stamp;

  // double resolution = sensor_.getlength();
  // double grid_x = agent_state->grid_x;
  // double grid_y = agent_state->grid_y;
  double world_x = agent_state->x;
  double world_y = agent_state->y;
  if (std::abs(world_x) < 0.001 && std::abs(world_y) < 0.001) {
    // std::cout << "grid coordinates are (0,0), skipping this callback (likely initial odometry message)" << std::endl;
    return;
  }

  odom.pose.pose.position.x = world_x;
  odom.pose.pose.position.y = world_y;

  odom.pose.pose.position.z = 0.0;
  tf2::Quaternion q_tf;
  q_tf.setRPY(0.0, 0.0, agent_state->yaw);
  geometry_msgs::msg::Quaternion qua = tf2::toMsg(q_tf);
  odom.pose.pose.orientation.w = qua.w;
  odom.pose.pose.orientation.x = qua.x;
  odom.pose.pose.orientation.y = qua.y;
  odom.pose.pose.orientation.z = qua.z;

  if (odom.header.frame_id == "null")
    return;

  colvec pose(6);
  pose(0) = odom.pose.pose.position.x;
  pose(1) = odom.pose.pose.position.y;
  pose(2) = odom.pose.pose.position.z;
  colvec q(4);
  q(0) = odom.pose.pose.orientation.w;
  q(1) = odom.pose.pose.orientation.x;
  q(2) = odom.pose.pose.orientation.y;
  q(3) = odom.pose.pose.orientation.z;
  pose.rows(3, 5) = R_to_ypr(quaternion_to_R(q));

  if (origin_ && !isOriginSet_) {
    isOriginSet_ = true;
    poseOrigin_ = pose;
  }
  if (origin_) {
    pose = pose_update(pose_inverse(poseOrigin_), pose);
  }

  poseROS_.header = odom.header;
  poseROS_.header.frame_id = "world";
  poseROS_.pose.position.x = pose(0);
  poseROS_.pose.position.y = pose(1);
  poseROS_.pose.position.z = pose(2);
  q = R_to_quaternion(ypr_to_R(pose.rows(3, 5)));
  poseROS_.pose.orientation.w = q(0);
  poseROS_.pose.orientation.x = q(1);
  poseROS_.pose.orientation.y = q(2);
  poseROS_.pose.orientation.z = q(3);
  posePub_->publish(poseROS_);

  static std::map<int, colvec> agent_ppose;
  static std::map<int, rclcpp::Time> agent_pt;

  const rclcpp::Time t(odom.header.stamp);

  if (agent_ppose.find(agent_id) == agent_ppose.end()) {
    agent_ppose[agent_id] = pose;
    agent_pt[agent_id] = t;
  }

  if ((t - agent_pt[agent_id]).seconds() > 0.5) {
    if (agent_traj_markers.find(agent_id) == agent_traj_markers.end()) {
      visualization_msgs::msg::Marker traj_marker;
      traj_marker.header.frame_id = string("world");
      traj_marker.header.stamp = node_->now();
      traj_marker.ns = string("trajectory");
      traj_marker.id = agent_id;
      traj_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
      traj_marker.action = visualization_msgs::msg::Marker::ADD;
      traj_marker.pose.position.x = 0;
      traj_marker.pose.position.y = 0;
      traj_marker.pose.position.z = 0;
      traj_marker.pose.orientation.w = 1;
      traj_marker.pose.orientation.x = 0;
      traj_marker.pose.orientation.y = 0;
      traj_marker.pose.orientation.z = 0;
      traj_marker.scale.x = traj_scale_;
      traj_marker.scale.y = 0;
      traj_marker.scale.z = 0;
      traj_marker.color.r = 0.0;
      traj_marker.color.g = 1.0;
      traj_marker.color.b = 0.0;
      traj_marker.color.a = 0.8;
      agent_traj_markers[agent_id] = traj_marker;
    }

    geometry_msgs::msg::Point p;
    p.x = agent_ppose[agent_id](0);
    p.y = agent_ppose[agent_id](1);
    p.z = agent_ppose[agent_id](2);
    agent_traj_markers[agent_id].points.push_back(p);

    p.x = pose(0);
    p.y = pose(1);
    p.z = pose(2);
    agent_traj_markers[agent_id].points.push_back(p);

    std_msgs::msg::ColorRGBA color;
    if (agent_id == 0) {
      // color.r = 1.0; color.g = 0.0; color.b = 0.0;  // red
      color.r = 1.0;
      color.g = 1.0;
      color.b = 1.0;  // white
    } else if (agent_id == 1) {
      // color.r = 0.0; color.g = 1.0; color.b = 0.0;  // green
      color.r = 1.0;
      color.g = 1.0;
      color.b = 1.0;  // white
    } else if (agent_id == 2) {
      // color.r = 0.0; color.g = 0.0; color.b = 1.0;  // blue
      color.r = 1.0;
      color.g = 1.0;
      color.b = 1.0;  // white
    } else {
      color.r = 1.0;
      color.g = 1.0;
      color.b = 1.0;  // white
    }

    color.a = 0.8;
    agent_traj_markers[agent_id].colors.push_back(color);
    agent_traj_markers[agent_id].colors.push_back(color);

    agent_ppose[agent_id] = pose;
    agent_pt[agent_id] = t;

    agent_traj_markers[agent_id].header.stamp = node_->now();
    trajPub_->publish(agent_traj_markers[agent_id]);

    // Mesh model
    meshROS_.header.frame_id = frame_id_;
    meshROS_.header.stamp = odom.header.stamp;
    meshROS_.ns = "mesh";
    meshROS_.id = agent_id;
    meshROS_.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    meshROS_.action = visualization_msgs::msg::Marker::ADD;
    meshROS_.pose.position.x = odom.pose.pose.position.x;
    meshROS_.pose.position.y = odom.pose.pose.position.y;
    meshROS_.pose.position.z = odom.pose.pose.position.z;
    q(0) = odom.pose.pose.orientation.w;
    q(1) = odom.pose.pose.orientation.x;
    q(2) = odom.pose.pose.orientation.y;
    q(3) = odom.pose.pose.orientation.z;
    if (cross_config_) {
      colvec ypr = R_to_ypr(quaternion_to_R(q));
      ypr(0) += 45.0 * M_PI / 180.0;
      q = R_to_quaternion(ypr_to_R(ypr));
    }
    meshROS_.pose.orientation.w = q(0);
    meshROS_.pose.orientation.x = q(1);
    meshROS_.pose.orientation.y = q(2);
    meshROS_.pose.orientation.z = q(3);
    meshROS_.scale.x = scale_;
    meshROS_.scale.y = scale_;
    meshROS_.scale.z = scale_;
    meshROS_.color.a = 1;
    meshROS_.color.r = 0;
    meshROS_.color.g = 0;
    meshROS_.color.b = 0;
    meshROS_.mesh_resource = mesh_resource_;
    meshPub_->publish(meshROS_);
  }
}

// void OdomVisualization::cmdCallback(const quadrotor_msgs::msg::PositionCommand::SharedPtr cmd)
// {
//   if (cmd.header.frame_id == "null")
//     return;

//   colvec pose(6);
//   pose(0) = cmd.position.x;
//   pose(1) = cmd.position.y;
//   pose(2) = cmd.position.z;
//   colvec q(4, fill::zeros);
//   q(0) = 1.0;
//   pose.rows(3, 5) = R_to_ypr(quaternion_to_R(q));

//   meshROS_.header.frame_id = frame_id_;
//   meshROS_.header.stamp = cmd.header.stamp;
//   meshROS_.ns = "mesh";
//   meshROS_.id = 0;
//   meshROS_.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
//   meshROS_.action = visualization_msgs::msg::Marker::ADD;
//   meshROS_.pose.position.x = cmd.position.x;
//   meshROS_.pose.position.y = cmd.position.y;
//   meshROS_.pose.position.z = cmd.position.z;

//   if (cross_config_)
//   {
//     colvec ypr = R_to_ypr(quaternion_to_R(q));
//     ypr(0) += 45.0 * M_PI / 180.0;
//     q = R_to_quaternion(ypr_to_R(ypr));
//   }
//   meshROS_.pose.orientation.w = q(0);
//   meshROS_.pose.orientation.x = q(1);
//   meshROS_.pose.orientation.y = q(2);
//   meshROS_.pose.orientation.z = q(3);
//   meshROS_.scale.x = 2.0;
//   meshROS_.scale.y = 2.0;
//   meshROS_.scale.z = 2.0;
//   meshROS_.color.a = color_a_;
//   meshROS_.color.r = color_r_;
//   meshROS_.color.g = color_g_;
//   meshROS_.color.b = color_b_;
//   meshROS_.mesh_resource = mesh_resource_;
//   meshPub_->publish(meshROS_);
// }

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto ov = std::make_shared<OdomVisualization>();
  rclcpp::spin(ov->get_node());
  rclcpp::shutdown();
  return 0;
}
