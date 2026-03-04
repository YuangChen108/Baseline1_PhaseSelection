#ifndef TL_FSM
#define TL_FSM

#include "util_gym/util_gym.hpp"
#include "util_gym/data_manager.hpp"
#include "rotation_util/rotation_util.hpp"

#include <thread>

#include "node/plan_node.h"
#include "node/traj_server_node.h"
#include "node/data_callback_node.h"

class TLFSM{
 public:
  enum FsmState{
    IDLE,
    HOVER,
    GOAL,
    TRACK,
    LAND,
    STOP,
  };

 private:
  //  0: tl_mode, 1: target_list_mode, 2: eight_mode
  int fsm_mode_; 
  // landing pose offset
  Eigen::Vector3d land_dp_;

 public:

  std::shared_ptr<ShareDataManager> dataManagerPtr_;
  std::shared_ptr<parameter_server::ParaeterSerer> para_ptr_;
  std::shared_ptr<vis_interface::VisInterface> vis_ptr_;

  std::shared_ptr<DataCallBacks> data_callbacks_;
  std::shared_ptr<Planner> planner_;
  std::shared_ptr<TrajServer> traj_server_;

  FsmState state_ = IDLE;

  /*********** ROS Functions ***************/
  #ifdef ROS
  void init_ros(ros::NodeHandle& nh);
  #endif
  /****************************************/

  /*********** SSDBUS Functions ***************/
  #ifdef SS_DBUS

  #endif
  /****************************************/

 public:
  bool judge_to_stop();
  bool judge_to_track(const Eigen::Vector3d& odom_p, const Eigen::Vector3d& target_p);
  bool set_thread_para(std::shared_ptr<std::thread>& thread, const int priority, const char* name);

 public:
  TLFSM();
  void run();

};


#endif