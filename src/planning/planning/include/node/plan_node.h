#ifndef PLANNODE
#define PLANNODE

#include "util_gym/util_gym.hpp"
#include "util_gym/data_manager.hpp"

#include "planning/tlplanner.h"

#include <thread>

class Planner{
 public:
  enum PlanMode{
      IDLE,
      GOAL,
      HOVER,
      TRACK,
      LAND,
  };

  enum PlannerState{
    NOTNEEDPLAN,
    PLANFAIL,
    PLANSUCC,
  };

  std::shared_ptr<ShareDataManager> dataManagerPtr_;

 private:
  // plan frequency
  int plan_hz_;
  // if use map
  bool use_map_ = false;
  // estimated time comsume of plan one time
  double plan_estimated_duration_;
  // last goal stamp
  TimePoint last_goal_stamp_;
  bool has_last_goal_ = false; 
  Eigen::Vector3d last_goal_;
  // when traj duration less than this threshold, stop plan, execute the last traj
  double land_no_plan_time_;
  // tracking expected height, distance and relative angle
  double tracking_height_expect_, tracking_dis_expect_, tracking_angle_expect_;
  // traj last locally
  bool has_traj_last_ = false;
  TrajData traj_last_;
  // local goal horizon
  double local_horizon_;

  double set_car_v_;
  double land_roll_, land_pitch_;

  int web_vis_hz_;

 private:
  // plan mode & state
  PlanMode plan_mode_ = IDLE;
  PlannerState plan_state_;
  std::mutex mode_mutex_, state_mutex_;

  // outer ptr
  // std::shared_ptr<ShareDataManager> dataManagerPtr_;
  std::shared_ptr<parameter_server::ParaeterSerer> paraPtr_;
  std::shared_ptr<vis_interface::VisInterface> visPtr_;

  // inner ptr
  #ifdef ROS
  std::shared_ptr<mapping::OccGridMap> rawmapPtr_;
  #endif
  #ifdef SS_DBUS
  // std::shared_ptr<lmap> rawmapPtr_;
  #endif
  std::shared_ptr<map_interface::MapInterface> gridmapPtr_;
  std::shared_ptr<env::Env> envPtr_;
  std::shared_ptr<prediction::Predict> prePtr_;
  std::shared_ptr<traj_opt::TrajOpt> trajoptPtr_;
  std::shared_ptr<tlplanner::TLPlanner> tlplannerPtr_;

 public:
  Planner(std::shared_ptr<ShareDataManager> dataManagerPtr,
          std::shared_ptr<parameter_server::ParaeterSerer> paraPtr);

  void plan_thread();
  virtual void saveLandingData(const Odom& uav, const Odom& tar, bool success) {
        // 这是一个“虚”接口，留给 TLPlanner 去覆盖(Override)
        // 如果调用到这里，说明子类没覆盖，或者当前不是 TLPlanner
    }
  inline void set_vis_ptr(std::shared_ptr<vis_interface::VisInterface> visPtr){
    visPtr_ = visPtr;
    envPtr_->set_vis_ptr(visPtr_);
    prePtr_->set_vis_ptr(visPtr_);
    trajoptPtr_->set_vis_ptr(visPtr_);
    tlplannerPtr_->set_vis_ptr(visPtr_);

    
    #ifdef ROS
    std::vector<Eigen::Vector3d> path;
    visPtr_->visualize_path(path, "astar");
    visPtr_->visualize_pointcloud(path, "astar_vp");
    visPtr_->visualize_path(path, "car_predict");
    visPtr_->visualize_pointcloud(path, "car_predict_pt");
    visPtr_->visualize_pointcloud(path, "mid_waypts");

    Trajectory<7> traj;
    visPtr_->visualize_traj(traj, "traj");

    #endif
  }

  inline void set_mode(const PlanMode& mode){
    std::unique_lock<std::mutex> lck(mode_mutex_);
    plan_mode_ = mode;
  }

  inline void set_state(const PlannerState& state){
    std::unique_lock<std::mutex> lck(state_mutex_);
    plan_state_ = state;
  }

  inline PlanMode get_mode(){
    std::unique_lock<std::mutex> lck(mode_mutex_);
    return plan_mode_;
  }

  inline std::string get_mode_name(){
    std::unique_lock<std::mutex> lck(mode_mutex_);
    std::string name;
    switch (plan_mode_)
    {
    case IDLE:
      name = "IDLE";
      break;
    case GOAL:
      name = "GOAL";
      break;
    case HOVER:
      name = "HOVER";
      break;
    case TRACK:
      name = "TRACK";
      break;
    case LAND:
      name = "LAND";
      break;
    default:
      name = "Unknow";
    }
    return name;
  }

  inline PlannerState get_state(){
    std::unique_lock<std::mutex> lck(state_mutex_);
    return plan_state_;
  }
};



#endif