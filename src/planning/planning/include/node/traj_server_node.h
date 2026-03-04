/*
<traj_server_node.h>
This thread is command output interface. It has ros & ssdbus 2 versoin.
When turnning to ssdbus vesion, just replace the *ssdbus.cpp file in secrecy system.
This thread will read planned result saved in <ShareDataManager>.traj_info_, and publish control command in ros or ssdbus context.
In ssdbus version, this node translate trajectory to pos-command, and publish with OSDK
*/
#ifndef TRAJ_SERVER
#define TRAJ_SERVER

#include "util_gym/util_gym.hpp"
#include "util_gym/data_manager.hpp"
#include "parameter_server/parameter_server.hpp"
#include "rotation_util/rotation_util.hpp"

#include <thread>

#ifdef ROS
#include <ros/ros.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <std_msgs/Empty.h>
#endif

class TrajServer{
 private:
  // outer ptr
  std::shared_ptr<ShareDataManager> dataManagerPtr_;
  std::shared_ptr<parameter_server::ParaeterSerer> paraPtr_;

  const int cmd_hz_ = 100;
  double dt_to_future_s_ = 0;

  bool has_stop_propeller_ = false;

  TrajData traj_data_last_;
  bool has_last_traj_ = false;
  double last_yaw_;
  double dyaw_max_;

  /*********** Ros Variables ***************/
  #ifdef ROS
  ros::NodeHandle nh_;
  ros::Publisher pos_cmd_pub_, stop_pub_;
  ros::Publisher pos_cmd_pub;
  #endif
  /****************************************/

  /*********** SSDBUS Variables ***************/
  double Kp_horiz_, Kd_horiz_, Kp_vert_, Kd_vert_;
  #ifdef SS_DBUS
  #endif
  /****************************************/

 public:
  TrajServer(std::shared_ptr<ShareDataManager> dataManagerPtr,
             std::shared_ptr<parameter_server::ParaeterSerer> paraPtr);
  void cmd_thread();
  bool exe_traj(const Odom& odom_data, const TrajData& traj_data);


  /*********** ROS Functions ***************/
  #ifdef ROS
  void init_ros(ros::NodeHandle& nh);
  void traj2vis(const TrajData& traj_data);
  #endif
  /****************************************/

  /*********** SSDBUS Functions ***************/
  #ifdef SS_DBUS

  #endif
  /****************************************/

  /*********** Share Functions ***************/
  bool stop_propeller();
  Eigen::Quaterniond cmd2odom(const Eigen::Vector3d& acc, const double& yaw);
  // if look forward, the yaw cmd only enable when drone speed is small
  void publish_cmd(int traj_id,
                   const Eigen::Vector3d& p,
                   const Eigen::Vector3d& v,
                   const Eigen::Vector3d& a,
                   double& y, double yd, bool look_forward,
                   const Eigen::Vector3d& odom_p, const Eigen::Vector3d& odom_v, const Eigen::Vector3d& odom_a,
                   const double& odom_yaw, TimePoint& odom_timestamp_ms, TimePoint& sample_time);
  /****************************************/
};

#endif