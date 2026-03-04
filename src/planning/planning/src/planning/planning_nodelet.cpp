#include <geometry_msgs/PoseStamped.h>
#include <mapping/mapping.h>
#include <nav_msgs/Odometry.h>
#include <nodelet/nodelet.h>
#include <quadrotor_msgs/OccMap3d.h>
#include <quadrotor_msgs/PolyTraj.h>
#include <quadrotor_msgs/ReplanState.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>

#include <traj_opt/traj_opt.h>
// #include <traj_opt_jerk/traj_opt_jerk.h>


#include <sensor_msgs/Joy.h>


#include <Eigen/Core>
#include <atomic>
#include <env/env.hpp>
// #include <prediction/prediction.hpp>
#include <prediction/prediction_car.hpp>
#include <thread>
#include <visualization/visualization.hpp>
#include <wr_msg/wr_msg.hpp>

namespace planning {
using rot_util = rotation_util::RotUtil;
Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", " << ", ";");

class Nodelet : public nodelet::Nodelet {
 private:
  std::thread initThread_;
  ros::Subscriber gridmap_sub_, odom_sub_, target_sub_, triger_sub_, land_triger_sub_, joy_sub_, goal_sub_;
  ros::Timer plan_timer_, plan_task_timer_;

  ros::Publisher traj_pub_, heartbeat_pub_, replanState_pub_;

  std::shared_ptr<mapping::OccGridMap> gridmapPtr_;
  std::shared_ptr<env::Env> envPtr_;
  std::shared_ptr<visualization::Visualization> visPtr_;
  std::shared_ptr<traj_opt::TrajOpt> trajOptPtr_;
  std::shared_ptr<prediction::Predict> prePtr_;
  std::shared_ptr<env::TargetBBX> targetBBXPtr_;
  std::shared_ptr<parameter_server::ParaeterSerer> para_ptr_;


  // NOTE planning or fake target
  int plan_hz = 10;
  bool fake_ = false;
  Eigen::Vector3d goal_;
  Eigen::Vector3d land_p_;
  Eigen::Quaterniond land_q_;
  double vmax_;

  // NOTE just for debug
  bool debug_ = false;
  quadrotor_msgs::ReplanState replanStateMsg_;
  ros::Publisher gridmap_pub_, inflate_gridmap_pub_;
  quadrotor_msgs::OccMap3d occmap_msg_;
  int plan_time_ = 0;

  double tracking_dur_, tracking_dist_, tracking_dt_;
  double landing_dur_;

  Trajectory<5> traj_jerk_;
  Trajectory<5> traj_track_;
  Trajectory<7> traj_land_;
  ros::Time replan_stamp_;
  int traj_id_ = 0;
  bool wait_hover_ = true;
  bool force_hover_ = true;
  bool first_land_ = false;
  bool first_land_traj_ = true;

  nav_msgs::Odometry odom_msg_, target_msg_;
  quadrotor_msgs::OccMap3d map_msg_;
  std::atomic_flag odom_lock_ = ATOMIC_FLAG_INIT;
  std::atomic_flag target_lock_ = ATOMIC_FLAG_INIT;
  std::atomic_flag gridmap_lock_ = ATOMIC_FLAG_INIT;
  std::atomic_bool odom_received_ = ATOMIC_VAR_INIT(false);
  std::atomic_bool map_received_ = ATOMIC_VAR_INIT(false);
  std::atomic_bool triger_received_ = ATOMIC_VAR_INIT(false);
  std::atomic_bool goal_received_ = ATOMIC_VAR_INIT(false);
  std::atomic_bool target_received_ = ATOMIC_VAR_INIT(false);
  std::atomic_bool land_triger_received_ = ATOMIC_VAR_INIT(false);

  struct joyState{
      double Y= 0.0;
      double X = 0.0;
      double Z = 0.0;
      double yaw = 0.0;
  };
  joyState _joyState;
  double track_angle_expect_ = -M_PI/1.0;

  void pub_hover_p(const Eigen::Vector3d& hover_p, const ros::Time& stamp) {
    quadrotor_msgs::PolyTraj traj_msg;
    traj_msg.hover = true;
    traj_msg.hover_p.resize(3);
    for (int i = 0; i < 3; ++i) {
      traj_msg.hover_p[i] = hover_p[i];
    }
    traj_msg.start_time = stamp;
    traj_msg.traj_id = traj_id_++;
    traj_pub_.publish(traj_msg);
  }

  // order = 5(jerk); order = 7(snap)
  void pub_traj(const Trajectory<5>& traj, const double& yaw, const ros::Time& stamp) {
    quadrotor_msgs::PolyTraj traj_msg;
    const int order = 5;
    traj_msg.hover = false;
    traj_msg.order = order;
    traj_msg.type = traj.getType();
    Eigen::VectorXd durs = traj.getDurations();
    int piece_num = traj.getPieceNum();
    traj_msg.duration.resize(piece_num);
    traj_msg.coef_x.resize((order+1) * piece_num);
    traj_msg.coef_y.resize((order+1) * piece_num);
    traj_msg.coef_z.resize((order+1) * piece_num);
    traj_msg.coef_psi.resize(4 * piece_num);
    for (int i = 0; i < piece_num; ++i) {
      traj_msg.duration[i] = durs(i);
      Eigen::MatrixXd cMat = traj[i].getCoeffMat();
      int i8 = i * (order+1);
      for (int j = 0; j < (order+1); j++) {
        traj_msg.coef_x[i8 + j] = cMat(0, j);
        traj_msg.coef_y[i8 + j] = cMat(1, j);
        traj_msg.coef_z[i8 + j] = cMat(2, j);
      }
      if (traj_msg.type == 2){
        AngleCoefficientMat cMat_angle = traj[i].getAngleCoeffMat();
        for(int j = 0; j < 4; j++) 
        {
          traj_msg.coef_psi[4 * i + j] = cMat_angle(0, j);
        }
      }
    }
    traj_msg.start_time = stamp;
    traj_msg.traj_id = traj_id_++;
    // NOTE yaw
    traj_msg.yaw = yaw;
    traj_pub_.publish(traj_msg);
  }

  // order = 5(jerk); order = 7(snap)
  void pub_traj(const Trajectory<7>& traj, const double& yaw, const ros::Time& stamp) {
    quadrotor_msgs::PolyTraj traj_msg;
    const int order = 7;
    traj_msg.hover = false;
    traj_msg.order = order;
    traj_msg.type = traj.getType();
    Eigen::VectorXd durs = traj.getDurations();
    int piece_num = traj.getPieceNum();
    traj_msg.duration.resize(piece_num);
    traj_msg.coef_x.resize((order+1) * piece_num);
    traj_msg.coef_y.resize((order+1) * piece_num);
    traj_msg.coef_z.resize((order+1) * piece_num);
    traj_msg.coef_psi.resize(4 * piece_num);
    for (int i = 0; i < piece_num; ++i) {
      traj_msg.duration[i] = durs(i);
      Eigen::MatrixXd cMat = traj[i].getCoeffMat();
      int i8 = i * (order+1);
      for (int j = 0; j < (order+1); j++) {
        traj_msg.coef_x[i8 + j] = cMat(0, j);
        traj_msg.coef_y[i8 + j] = cMat(1, j);
        traj_msg.coef_z[i8 + j] = cMat(2, j);
      }
      if (traj_msg.type == 2){
        AngleCoefficientMat cMat_angle = traj[i].getAngleCoeffMat();
        for(int j = 0; j < 4; j++) 
        {
          traj_msg.coef_psi[4 * i + j] = cMat_angle(0, j);
        }
      }
    }
    traj_msg.start_time = stamp;
    traj_msg.traj_id = traj_id_++;
    // NOTE yaw
    traj_msg.yaw = yaw;
    traj_pub_.publish(traj_msg);
  }

  void triger_callback(const geometry_msgs::PoseStampedConstPtr& msgPtr) {
    // goal_ << msgPtr->pose.position.x, msgPtr->pose.position.y, 0.9;
    triger_received_ = true;
  }

  void land_triger_callback(const geometry_msgs::PoseStampedConstPtr& msgPtr) {
    land_p_.x() = msgPtr->pose.position.x;
    land_p_.y() = msgPtr->pose.position.y;
    land_p_.z() = msgPtr->pose.position.z;
    land_q_.w() = msgPtr->pose.orientation.w;
    land_q_.x() = msgPtr->pose.orientation.x;
    land_q_.y() = msgPtr->pose.orientation.y;
    land_q_.z() = msgPtr->pose.orientation.z;
    land_triger_received_ = true;
  }

  void odom_callback(const nav_msgs::Odometry::ConstPtr& msgPtr) {
    while (odom_lock_.test_and_set())
      ;
    odom_msg_ = *msgPtr;
    odom_received_ = true;
    odom_lock_.clear();
  }

  void target_callback(const nav_msgs::Odometry::ConstPtr& msgPtr) {
    while (target_lock_.test_and_set())
      ;
    target_msg_ = *msgPtr;
    target_received_ = true;
    target_lock_.clear();
  }

  void goal_callback(const geometry_msgs::PoseStampedConstPtr& msgPtr) {
    goal_ << msgPtr->pose.position.x, msgPtr->pose.position.y, 1.0;
    INFO_MSG_GREEN("Goal: " << goal_.transpose());
    goal_received_ = true;
  }

  void gridmap_callback(const quadrotor_msgs::OccMap3dConstPtr& msgPtr) {
    while (gridmap_lock_.test_and_set())
      ;
    map_msg_ = *msgPtr;
    map_received_ = true;
    gridmap_lock_.clear();
  }

  void plan_callback(const ros::TimerEvent& event){
    heartbeat_pub_.publish(std_msgs::Empty());
    if (!odom_received_ || !map_received_ || !goal_received_) {
      return;
    }
    // goal_received_ = false;
    // obtain state of odom
    while (odom_lock_.test_and_set())
      ;
    auto odom_msg = odom_msg_;
    odom_lock_.clear();
    Eigen::Vector3d odom_p(odom_msg.pose.pose.position.x,
                           odom_msg.pose.pose.position.y,
                           odom_msg.pose.pose.position.z);
    Eigen::Vector3d odom_v(odom_msg.twist.twist.linear.x,
                           odom_msg.twist.twist.linear.y,
                           odom_msg.twist.twist.linear.z);
    Eigen::Quaterniond odom_q(odom_msg.pose.pose.orientation.w,
                              odom_msg.pose.pose.orientation.x,
                              odom_msg.pose.pose.orientation.y,
                              odom_msg.pose.pose.orientation.z);
    // NOTE obtain map
    while (gridmap_lock_.test_and_set())
      ;
    gridmapPtr_->from_msg(map_msg_);
    replanStateMsg_.occmap = map_msg_;
    gridmap_lock_.clear();

    // todo esdf check
    // for (double px = 2.0; px < 6.0; px += 0.2){
    //   Eigen::Vector3d p(px, -1.0, 1.0);
    //   INFO_MSG_RED("p: " << p.transpose());
    //   Eigen::Vector3d grad;
    //   double dis;

    //   grad.setZero();
    //   dis = gridmapPtr_->getCostWithGrad(p, grad);
    //   INFO_MSG("my esdf: " << dis <<", grad: " << grad.transpose());
    // }


    Eigen::Vector3d goal = goal_;
    if (std::fabs((goal - odom_p).norm() < 0.1 && odom_v.norm() < 0.1)) {
      if (!wait_hover_) {
        pub_hover_p(odom_p, ros::Time::now());
        wait_hover_ = true;
      }
      ROS_WARN("[planner] HOVERING...");
      return;
    }
    wait_hover_ = false;


    INFO_MSG_BLUE("in plan timer");
    // NOTE replan state
    double plan_estimated_duration = 0.07; // s
    Eigen::MatrixXd iniState;
    iniState.setZero(3, 4);
    ros::Time replan_stamp = ros::Time::now() + ros::Duration(plan_estimated_duration);
    double replan_t = (replan_stamp - replan_stamp_).toSec();
    if (replan_t > traj_jerk_.getTotalDuration()) {
      // should replan from the hover state
      iniState.col(0) = odom_p;
      iniState.col(1) = odom_v;
      ROS_ERROR_STREAM("replan from the hover state!");
    } else {
      // should replan from the last trajectory
      iniState.col(0) = traj_jerk_.getPos(replan_t);
      iniState.col(1) = traj_jerk_.getVel(replan_t);
      iniState.col(2) = traj_jerk_.getAcc(replan_t);
      INFO_MSG_GREEN("init p: "<<iniState.col(0).transpose() << ", goal: " << goal.transpose());
      // iniState.col(3) = traj_poly_.getJer(replan_t);
    }

    // NOTE path searching
    Eigen::Vector3d p_start = iniState.col(0);
    std::vector<Eigen::Vector3d> path, way_pts;

    INFO_MSG("short_astar");
    bool generate_new_traj_success = envPtr_->short_astar(p_start, goal, path); // get a path from current pose to target pose
    INFO_MSG("short_astar end, size: " << path.size());
    
    Trajectory<5> traj;
    if (generate_new_traj_success) {
        visPtr_->visualize_path(path, "astar");
        Eigen::MatrixXd finState;
        finState.setZero(3, 4);
        finState.col(0) = path.back();
        INFO_MSG("init p: " << iniState.col(0).transpose());
        INFO_MSG("end p: " << finState.col(0).transpose());
        std::vector<Eigen::Vector3d> target_predcit;
        // generate_new_traj_success = trajOptPtr_->generate_traj(iniState, finState, 2.0, path, target_predcit, traj);
        generate_new_traj_success = trajOptPtr_->generate_traj(iniState, finState, 2.0, path, traj);
    }else{
      ROS_ERROR_STREAM("a star error");
    }

      // NOTE collision check

      bool valid = false;
      if (generate_new_traj_success) {
        valid = validcheck(traj, replan_stamp);
        visPtr_->visualize_traj(traj, "traj");        
      } else {
        valid = false;
        replanStateMsg_.state = -2;
        replanState_pub_.publish(replanStateMsg_);
      }
      if (valid) {
        force_hover_ = false;
        
        ROS_WARN("[planner] REPLAN SUCCESS");
        replanStateMsg_.state = 0;
        replanState_pub_.publish(replanStateMsg_);

        double yaw = 0.0;
        pub_traj(traj, yaw, replan_stamp);
        traj_jerk_ = traj;
        replan_stamp_ = replan_stamp;

      } else if (force_hover_) {
        ROS_ERROR("[planner] REPLAN FAILED, HOVERING...");
        replanStateMsg_.state = 1;
        replanState_pub_.publish(replanStateMsg_);
        return;
      } else if (!validcheck(traj_jerk_, replan_stamp_)) {
        force_hover_ = true;
        ROS_FATAL("[planner] EMERGENCY STOP!!!");
        replanStateMsg_.state = 2;
        replanState_pub_.publish(replanStateMsg_);
        pub_hover_p(iniState.col(0), replan_stamp);
        return;
      } else {
        ROS_ERROR("[planner] REPLAN FAILED, EXECUTE LAST TRAJ...");
        replanStateMsg_.state = 3;
        replanState_pub_.publish(replanStateMsg_);
        return;  // current generated traj invalid but last is valid
      }

  }

  // NOTE main callback
  void plan_task_callback(const ros::TimerEvent& event){
    heartbeat_pub_.publish(std_msgs::Empty());
    if (!odom_received_ || !map_received_) {
      return;
    }
    // obtain state of odom
    while (odom_lock_.test_and_set())
      ;
    auto odom_msg = odom_msg_;
    odom_lock_.clear();
    Eigen::Vector3d odom_p(odom_msg.pose.pose.position.x,
                           odom_msg.pose.pose.position.y,
                           odom_msg.pose.pose.position.z);
    Eigen::Vector3d odom_v(odom_msg.twist.twist.linear.x,
                           odom_msg.twist.twist.linear.y,
                           odom_msg.twist.twist.linear.z);
    Eigen::Quaterniond odom_q(odom_msg.pose.pose.orientation.w,
                              odom_msg.pose.pose.orientation.x,
                              odom_msg.pose.pose.orientation.y,
                              odom_msg.pose.pose.orientation.z);
    double odom_yaw = rot_util::quaternion2euler(odom_q).z();
    if (!triger_received_) {
      return;
    }
    if (!target_received_) {
      return;
    }
    // NOTE obtain state of target
    while (target_lock_.test_and_set())
      ;
    replanStateMsg_.target = target_msg_;
    target_lock_.clear();
    Eigen::Vector3d target_p(replanStateMsg_.target.pose.pose.position.x,
                             replanStateMsg_.target.pose.pose.position.y,
                             replanStateMsg_.target.pose.pose.position.z);
    Eigen::Vector3d target_v(replanStateMsg_.target.twist.twist.linear.x,
                             replanStateMsg_.target.twist.twist.linear.y,
                             replanStateMsg_.target.twist.twist.linear.z);
    Eigen::Quaterniond target_q;
    target_q.w() = replanStateMsg_.target.pose.pose.orientation.w;
    target_q.x() = replanStateMsg_.target.pose.pose.orientation.x;
    target_q.y() = replanStateMsg_.target.pose.pose.orientation.y;
    target_q.z() = replanStateMsg_.target.pose.pose.orientation.z;
    Eigen::Vector3d target_rpy = rot_util::quaternion2euler(target_q);
    static double target_y_last = 0.0;
    double target_omega = (target_rpy(2) - target_y_last) / (1.0 / plan_hz);
    INFO_MSG_YELLOW("last_yaw: "<<target_y_last<<", nowyaw: "<<target_rpy(2)<<", w: "<<target_omega);
    target_y_last = target_rpy(2);

    ROS_WARN("[planner] LANDING...");
    if (land_triger_received_){
      target_p = target_p + target_q * land_p_;
    }else{
      target_p.z() += 1.0; // tracking height
    }
    wait_hover_ = false;


    // NOTE obtain map
    while (gridmap_lock_.test_and_set())
      ;
    gridmapPtr_->from_msg(map_msg_);
    replanStateMsg_.occmap = map_msg_;
    gridmap_lock_.clear();
    prePtr_->set_gridmap_ptr(gridmapPtr_);

    // todo esdf check
    // for (double px = 3.9; px < 4.1; px += 0.01){
    //   Eigen::Vector3d p(px, 6.0, 1.0);
    //   INFO_MSG_RED("p: " << p.transpose());
    //   Eigen::Vector3d grad;
    //   double dis;

    //   grad.setZero();
    //   dis = gridmapPtr_->getCostWithGrad(p, grad);
    //   INFO_MSG("my esdf: " << dis <<", grad: " << grad.transpose());
    // }

    // minco::MINCO_S2 minco_yaw;
    // Eigen::Matrix<double, 1, 2> headState, tailState;
    // headState.setZero();
    // tailState.setZero();
    // headState(0, 0) = 0;
    // tailState(0, 0) = 1;

    // minco_yaw.reset(headState, tailState, 2);
    // Eigen::Matrix<double, 1, 1> inPs;
    // inPs(0, 0) = 0.5;
    // Eigen::VectorXd ts;
    // ts.resize(2);
    // ts(0) = 1.0;
    // ts(1) = 1.0;
    
    // minco_yaw.generate(inPs, ts);
    // // Trajectory<3> traj_yaw;
    // // traj_yaw = minco_yaw.getTraj();
    // // visPtr_->visualize_traj(traj_yaw, "traj");

    // while(1)
    //   ;

    // NOTE prediction
    std::vector<Eigen::Vector3d> target_predcit;
    // ros::Time t_start = ros::Time::now();
    bool generate_new_traj_success;
    INFO_MSG("prediction");

    if (land_triger_received_) {
      // generate_new_traj_success = prePtr_->predict(target_p, target_v, target_predcit, landing_dur_);
      generate_new_traj_success = prePtr_->predict(target_p, 1.0, 0.0, 0.5, target_predcit, tracking_dur_);
    }else{
      // target_v.x() = 1.0;
      // generate_new_traj_success = prePtr_->predict(target_p, target_v, target_predcit, tracking_dur_);
      double target_car_v = target_v.head(2).norm();
      target_car_v = 1.0;
      INFO_MSG_RED("target_car_v: "<<target_car_v<<", target_omega: "<<target_omega);
      generate_new_traj_success = prePtr_->predict(target_p, target_car_v, target_rpy(2), target_omega, target_predcit, tracking_dur_);

    }
    INFO_MSG("prediction done");



    if (generate_new_traj_success) {
      Eigen::Vector3d observable_p = target_predcit.back();
      visPtr_->visualize_path(target_predcit, "car_predict");
      visPtr_->visualize_pointcloud(target_predcit, "car_predict_pt");
    }

    //todo visible path deform
    trajOptPtr_->set_track_angle(track_angle_expect_);
    // std::vector<Eigen::Vector3d> viewpoint_path;
    // bool deform_succ = trajOptPtr_->visible_path_deform(odom_p, target_predcit, viewpoint_path);
    // if (deform_succ){
    //   INFO_MSG_GREEN("visible_path_deform SUCC! size: " << viewpoint_path.size());
    // }else{
    //   INFO_MSG_RED("visible_path_deform FAIL! size: " << viewpoint_path.size());
    // }
    
    
    // visPtr_->visualize_pointcloud(viewpoint_path, "/astar_vp");

    // while(1)
    //   ;

    // NOTE replan state
    double plan_estimated_duration = 0.07; // s
    Eigen::MatrixXd iniState, init_yaw;
    iniState.setZero(3, 4);
    init_yaw.setZero(1, 2);
    ros::Time replan_stamp = ros::Time::now() + ros::Duration(plan_estimated_duration);
    double replan_t = (replan_stamp - replan_stamp_).toSec();

    if (land_triger_received_){
      if (force_hover_ || (!first_land_ && replan_t > traj_land_.getTotalDuration())) {
        // should replan from the hover state
        iniState.col(0) = odom_p;
        iniState.col(1) = odom_v;
        init_yaw(0, 0) = odom_yaw;
        ROS_ERROR_STREAM("replan from the hover state!");
      } else {
        if (first_land_){
          ROS_ERROR_STREAM("first_land_!");
          // should replan from the last trajectory
          iniState.col(0) = traj_track_.getPos(replan_t);
          iniState.col(1) = traj_track_.getVel(replan_t);
          iniState.col(2) = traj_track_.getAcc(replan_t);
          iniState.col(3) = traj_track_.getJer(replan_t);
          init_yaw(0, 0) = traj_track_.getAngle(replan_t);
          init_yaw(0, 1) = traj_track_.getAngleRate(replan_t);
          first_land_ = false;
        }else{
          // should replan from the last trajectory
          iniState.col(0) = traj_land_.getPos(replan_t);
          iniState.col(1) = traj_land_.getVel(replan_t);
          iniState.col(2) = traj_land_.getAcc(replan_t);
          iniState.col(3) = traj_land_.getJer(replan_t);
          init_yaw(0, 0) = traj_land_.getAngle(replan_t);
          init_yaw(0, 1) = traj_land_.getAngleRate(replan_t);
        }


        // predict target should also be aligned to replan time
        target_p = target_p + plan_estimated_duration * target_v;
        for(auto& target:target_predcit){
          target = target + plan_estimated_duration * target_v;
        }
      }
    }else{
      if (force_hover_ || replan_t > traj_track_.getTotalDuration()) {
        // should replan from the hover state
        iniState.col(0) = odom_p;
        iniState.col(1) = odom_v;
        init_yaw(0, 0) = odom_yaw;
        ROS_ERROR_STREAM("replan from the hover state!");
      } else {
        // should replan from the last trajectory
        iniState.col(0) = traj_track_.getPos(replan_t);
        iniState.col(1) = traj_track_.getVel(replan_t);
        iniState.col(2) = traj_track_.getAcc(replan_t);
        iniState.col(3) = traj_track_.getJer(replan_t);
        init_yaw(0, 0) = traj_track_.getAngle(replan_t);
        init_yaw(0, 1) = traj_track_.getAngleRate(replan_t);

        // predict target should also be aligned to replan time
        target_p = target_p + plan_estimated_duration * target_v;
        for(auto& target:target_predcit){
          target = target + plan_estimated_duration * target_v;
        }
      }
    }



    replanStateMsg_.header.stamp = ros::Time::now();
    replanStateMsg_.iniState.resize(9);
    // Eigen::Map<Eigen::MatrixXd>(replanStateMsg_.iniState.data(), 3, 3) = iniState;

    // NOTE path searching
    Eigen::Vector3d p_start = iniState.col(0);
    std::vector<Eigen::Vector3d> path, way_pts;

    ros::Time t_front0 = ros::Time::now();

    if (generate_new_traj_success) {
      if (land_triger_received_) {
        generate_new_traj_success = envPtr_->short_astar(p_start, target_p, path); // get a path from current pose to target pose
      } else {
        envPtr_->set_track_angle(track_angle_expect_);
        trajOptPtr_->set_track_angle(track_angle_expect_);
        visPtr_->visualize_arrow(Eigen::Vector3d(0,0,0), 5.0*Eigen::Vector3d(cos(track_angle_expect_),sin(track_angle_expect_),0),"/track_angle", visualization::blue, "view");
        INFO_MSG("findVisiblePath");
        generate_new_traj_success = envPtr_->findVisiblePath(p_start, target_predcit, tracking_dt_, way_pts, path); // way_pts correspond to target_predict, path is denser points
      
      
      
        // way_pts.pop_back();

        // envPtr_->pts2path(way_pts, path);

        // std::vector<Eigen::MatrixXd> hPolys;
        // std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> keyPts;
        // visPtr_->visualize_path(path, "path_for_SFC");
        // envPtr_->generateSFC(path, Eigen::Vector3d(5.0, 5.0, 3.0), hPolys, keyPts, false);
        // envPtr_->visCorridor(hPolys);
      
      }
    }

    if (generate_new_traj_success) {
      ROS_INFO_STREAM("a start succ");
      visPtr_->visualize_path(path, "astar");
      Eigen::Quaterniond land_q;
      land_q.w() = 1.0;
      land_q.x() = 0.0;
      land_q.y() = 0.0;
      land_q.z() = 0.0;

      Trajectory<5> traj_track;
      Trajectory<7> traj_land;

      if (land_triger_received_) {
        double replan_t = -1;
        if (!first_land_traj_ && traj_land_.getTotalDuration() < 2.0){
          replan_t = 0.2;
        }
        if (!first_land_traj_ && traj_land_.getTotalDuration() < 1.0){
          std::cout << "Stop Plan - traj.getTotalDuration(): " << traj_land_.getTotalDuration() << std::endl;
          while(1)
            ;
        }
        // target_v.y() = 1.0;
        // generate_new_traj_success = trajOptPtr_->generate_traj(iniState, target_p, target_v, land_q, 8, traj_land, replan_t);
        generate_new_traj_success = trajOptPtr_->generate_traj(iniState, init_yaw, target_p, target_v, land_q, 8, traj_land, replan_t);
        
        first_land_traj_ = false;





        bool valid = true;
        if (generate_new_traj_success) {
          visPtr_->visualize_traj(traj_land, "traj");
        } else {
          valid = false;
          replanStateMsg_.state = -2;
          replanState_pub_.publish(replanStateMsg_);
        }
        if (valid) {
          force_hover_ = false;
          ROS_WARN("[planner] REPLAN SUCCESS");
          replanStateMsg_.state = 0;
          replanState_pub_.publish(replanStateMsg_);
          Eigen::Vector3d dp = target_p + target_v * 0.03 - iniState.col(0);

          double yaw = std::atan2(dp.y(), dp.x());
          if (land_triger_received_) {
            yaw = 2 * std::atan2(target_q.z(), target_q.w());
          }
          pub_traj(traj_land, yaw, replan_stamp);
          traj_land_ = traj_land;
          replan_stamp_ = replan_stamp;


          // double dt = 0.01;
          // Eigen::Quaterniond q_last;
          // double max_omega = 0;
          // std::cout << "traj.getTotalDuration(): " << traj.getTotalDuration() << std::endl;
          // for (double t = 0; t <= traj.getTotalDuration(); t += dt) {
          //   Eigen::Vector3d p = traj.getPos(t);
          //   Eigen::Vector3d a = traj.getAcc(t);
          //   Eigen::Vector3d j = traj.getJer(t);
          //   Eigen::Vector3d g(0, 0, -9.8);
          //   Eigen::Vector3d thrust = a - g;
          //   if (trajOptPtr_->check_collilsion(p, a, target_p)) {
          //     std::cout << "collide!  t: " << t << std::endl;
          //   }
          // }
          // while(1){}



        } else if (force_hover_) {
          ROS_ERROR("[planner] REPLAN FAILED, HOVERING...");
          replanStateMsg_.state = 1;
          replanState_pub_.publish(replanStateMsg_);
          return;
        } else if (!validcheck(traj_land_, replan_stamp_)) {
          force_hover_ = true;
          ROS_FATAL("[planner] EMERGENCY STOP!!!");
          replanStateMsg_.state = 2;
          replanState_pub_.publish(replanStateMsg_);
          pub_hover_p(iniState.col(0), replan_stamp);
          return;
        } else {
          ROS_ERROR("[planner] REPLAN FAILED, EXECUTE LAST TRAJ...");
          replanStateMsg_.state = 3;
          replanState_pub_.publish(replanStateMsg_);
          return;  // current generated traj invalid but last is valid
        }

        // visual traj
        {
          visPtr_->visualize_traj(traj_land, "traj");
          std::vector<Eigen::Vector3d> wpt_on_traj;
          std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> rays;
          double t = 0;
          // std::cout<<"tracing dis: " << std::endl;
          for (int i = 0; i < (int)target_predcit.size(); ++i){
            if (t > traj_land.getTotalDuration()) break;
            wpt_on_traj.push_back(traj_land.getPos(t));
            rays.emplace_back(target_predcit[i], traj_land.getPos(t));
            // std::cout<< (traj.getPos(t) - target_predcit[i]).norm() << std::endl;
            t += tracking_dt_;
          }
          visPtr_->visualize_pointcloud(wpt_on_traj, "/wpt_on_traj");
          visPtr_->visualize_pairline(rays, "/track_rays");
        }

      }else{
        Eigen::MatrixXd finState, final_yaw;
        finState.setZero(3, 4);
        final_yaw.setZero(1, 2);
        finState.col(0) = path.back();
        finState.col(1) = target_v;
        Eigen::Vector3d dp = target_predcit.back() - path.back();
        final_yaw(0, 0) = atan2(dp.y(), dp.x());
        INFO_MSG("init p: " << iniState.col(0).transpose());
        INFO_MSG("end p: " << finState.col(0).transpose());
        // generate_new_traj_success = trajOptPtr_->generate_traj(iniState, finState, 1.0, path, target_predcit, traj_track);

        generate_new_traj_success = trajOptPtr_->generate_traj(iniState, finState, init_yaw, final_yaw, 1.0, path, way_pts, target_predcit, traj_track);
        ROS_WARN("[planner] generate_traj done");

      
      
        bool valid = true;
        if (generate_new_traj_success) {
          ROS_WARN("[planner] visualize_traj");
          visPtr_->visualize_traj(traj_track, "traj");
          ROS_WARN("[planner] visualize_traj done");
        } else {
          valid = false;
          replanStateMsg_.state = -2;
          replanState_pub_.publish(replanStateMsg_);
        }
        if (valid) {
          force_hover_ = false;
          ROS_WARN("[planner] REPLAN SUCCESS");
          replanStateMsg_.state = 0;
          replanState_pub_.publish(replanStateMsg_);
          Eigen::Vector3d dp = target_p + target_v * 0.03 - iniState.col(0);

          double yaw = std::atan2(dp.y(), dp.x());
          if (land_triger_received_) {
            yaw = 2 * std::atan2(target_q.z(), target_q.w());
          }
          pub_traj(traj_track, yaw, replan_stamp);
          traj_track_ = traj_track;
          replan_stamp_ = replan_stamp;


          // double dt = 0.01;
          // Eigen::Quaterniond q_last;
          // double max_omega = 0;
          // std::cout << "traj.getTotalDuration(): " << traj.getTotalDuration() << std::endl;
          // for (double t = 0; t <= traj.getTotalDuration(); t += dt) {
          //   Eigen::Vector3d p = traj.getPos(t);
          //   Eigen::Vector3d a = traj.getAcc(t);
          //   Eigen::Vector3d j = traj.getJer(t);
          //   Eigen::Vector3d g(0, 0, -9.8);
          //   Eigen::Vector3d thrust = a - g;
          //   if (trajOptPtr_->check_collilsion(p, a, target_p)) {
          //     std::cout << "collide!  t: " << t << std::endl;
          //   }
          // }
          // while(1){}
          if (traj_track.getTotalDuration() < 1.5){
            std::cout << "Stop Plan - traj.getTotalDuration(): " << traj_track.getTotalDuration() << std::endl;
            while(1)
              ;
          }


        } else if (force_hover_) {
          ROS_ERROR("[planner] REPLAN FAILED, HOVERING...");
          replanStateMsg_.state = 1;
          replanState_pub_.publish(replanStateMsg_);
          return;
        } else if (!validcheck(traj_track_, replan_stamp_)) {
          force_hover_ = true;
          ROS_FATAL("[planner] EMERGENCY STOP!!!");
          replanStateMsg_.state = 2;
          replanState_pub_.publish(replanStateMsg_);
          pub_hover_p(iniState.col(0), replan_stamp);
          return;
        } else {
          ROS_ERROR("[planner] REPLAN FAILED, EXECUTE LAST TRAJ...");
          replanStateMsg_.state = 3;
          replanState_pub_.publish(replanStateMsg_);
          return;  // current generated traj invalid but last is valid
        }

        // visual traj
        {
          visPtr_->visualize_traj(traj_track, "traj");
          std::vector<Eigen::Vector3d> wpt_on_traj;
          std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> rays;
          double t = 0;
          // std::cout<<"tracing dis: " << std::endl;
          for (int i = 0; i < (int)target_predcit.size(); ++i){
            wpt_on_traj.push_back(traj_track.getPos(t));
            rays.emplace_back(target_predcit[i], traj_track.getPos(t));
            // std::cout<< (traj.getPos(t) - target_predcit[i]).norm() << std::endl;
            t += tracking_dt_;
          }
          visPtr_->visualize_pointcloud(wpt_on_traj, "/wpt_on_traj");
          visPtr_->visualize_pairline(rays, "/track_rays");
        }
      
      }

      // ROS_INFO_STREAM("trajOptPtr_ succ");

    
      
    

      // NOTE collision check

    
    }
  }



  // NOTE main callback
  // void plan_timer_callback(const ros::TimerEvent& event) {
  //   heartbeat_pub_.publish(std_msgs::Empty());
  //   if (!odom_received_ || !map_received_) {
  //     return;
  //   }
  //   // obtain state of odom
  //   while (odom_lock_.test_and_set())
  //     ;
  //   auto odom_msg = odom_msg_;
  //   odom_lock_.clear();
  //   Eigen::Vector3d odom_p(odom_msg.pose.pose.position.x,
  //                          odom_msg.pose.pose.position.y,
  //                          odom_msg.pose.pose.position.z);
  //   Eigen::Vector3d odom_v(odom_msg.twist.twist.linear.x,
  //                          odom_msg.twist.twist.linear.y,
  //                          odom_msg.twist.twist.linear.z);
  //   Eigen::Quaterniond odom_q(odom_msg.pose.pose.orientation.w,
  //                             odom_msg.pose.pose.orientation.x,
  //                             odom_msg.pose.pose.orientation.y,
  //                             odom_msg.pose.pose.orientation.z);
  //   if (!triger_received_) {
  //     return;
  //   }
  //   if (!target_received_) {
  //     return;
  //   }
  //   // NOTE obtain state of target
  //   while (target_lock_.test_and_set())
  //     ;
  //   replanStateMsg_.target = target_msg_;
  //   target_lock_.clear();
  //   Eigen::Vector3d target_p(replanStateMsg_.target.pose.pose.position.x,
  //                            replanStateMsg_.target.pose.pose.position.y,
  //                            replanStateMsg_.target.pose.pose.position.z);
  //   Eigen::Vector3d target_v(replanStateMsg_.target.twist.twist.linear.x,
  //                            replanStateMsg_.target.twist.twist.linear.y,
  //                            replanStateMsg_.target.twist.twist.linear.z);
  //   Eigen::Quaterniond target_q;
  //   target_q.w() = replanStateMsg_.target.pose.pose.orientation.w;
  //   target_q.x() = replanStateMsg_.target.pose.pose.orientation.x;
  //   target_q.y() = replanStateMsg_.target.pose.pose.orientation.y;
  //   target_q.z() = replanStateMsg_.target.pose.pose.orientation.z;

  //   Eigen::Vector3d target_p_landing;

  //   targetBBXPtr_->updateBBX(target_p, target_v, target_q);
  //   envPtr_->set_odom(odom_p, odom_v);
  //   visPtr_->visualize_pointcloud(targetBBXPtr_->getVps(), "/TargetBBX");
  //   //! test the BBX
  //   std::vector<Eigen::Vector3d> inp, outp;
  //   for(double dx = -3.0; dx <= 3.0; dx += 0.2){
  //     for(double dy = -2.0; dy <= 2.0; dy += 0.2){
  //       for(double dz = -2.0; dz <= 2.0; dz += 0.05){
  //         Eigen::Vector3d p = target_p;
  //         p(0) = p(0) + dx;
  //         p(1) = p(1) + dy;
  //         p(2) = p(2) + dz;
  //         if(targetBBXPtr_->isInBBX(p)){
  //           inp.push_back(p);
  //         }else{
  //           outp.push_back(p);
  //         }
  //       }
  //     }
  //   }
  //   visPtr_->visualize_pointcloud(inp, "/debug_in");
  //   visPtr_->visualize_pointcloud(outp, "/debug_out");




  //   // NOTE force-hover: waiting for the speed of drone small enough
  //   if (force_hover_ && odom_v.norm() > 0.1) {
  //     return;
  //   }

  //   // NOTE just for landing on the car!
  //   if (land_triger_received_) {
  //     if (std::fabs((target_p - odom_p).norm() < 0.1 && odom_v.norm() < 0.1 && target_v.norm() < 0.2)) {
  //       if (!wait_hover_) {
  //         pub_hover_p(odom_p, ros::Time::now());
  //         wait_hover_ = true;
  //       }
  //       ROS_WARN("[planner] HOVERING...");
  //       return;
  //     }
  //     // TODO get the orientation fo target and calculate the pose of landing point
  //       ROS_WARN("[planner] LANDING...");
  //     target_p = target_p + target_q * land_p_;
  //     target_p_landing = target_p + target_v * landing_dur_ + target_q * land_p_;
  //     wait_hover_ = false;
  //   } else {
  //     target_p.z() += 1.0; // tracking height
  //     // NOTE determin whether to replan
  //     Eigen::Vector3d dp = target_p - odom_p;
  //     // std::cout << "dist : " << dp.norm() << std::endl;
  //     double desired_yaw = std::atan2(dp.y(), dp.x());
  //     Eigen::Vector3d project_yaw = odom_q.toRotationMatrix().col(0);  // NOTE ZYX
  //     double now_yaw = std::atan2(project_yaw.y(), project_yaw.x());
  //     if (std::fabs((target_p - odom_p).norm() - tracking_dist_) < tolerance_d_ &&
  //         odom_v.norm() < 0.1 && target_v.norm() < 0.2 &&
  //         std::fabs(desired_yaw - now_yaw) < 0.5) {
  //       if (!wait_hover_) {
  //         pub_hover_p(odom_p, ros::Time::now());
  //         wait_hover_ = true;
  //       }
  //       ROS_WARN("[planner] HOVERING...");
  //       replanStateMsg_.state = -1;
  //       replanState_pub_.publish(replanStateMsg_);
  //       return;
  //     } else {
  //       wait_hover_ = false;
  //     }
  //   }

  //   // NOTE obtain map
  //   while (gridmap_lock_.test_and_set())
  //     ;
  //   gridmapPtr_->from_msg(map_msg_);
  //   replanStateMsg_.occmap = map_msg_;
  //   gridmap_lock_.clear();
  //   prePtr_->setMap(*gridmapPtr_);

  //   // visualize the ray from drone to target
  //   if (envPtr_->checkRayValid(odom_p, target_p)) {
  //     visPtr_->visualize_arrow(odom_p, target_p, "ray", visualization::yellow);
  //   } else {
  //     visPtr_->visualize_arrow(odom_p, target_p, "ray", visualization::red);
  //   }

  //   // NOTE prediction
  //   std::vector<Eigen::Vector3d> target_predcit;
  //   // ros::Time t_start = ros::Time::now();
  //   bool generate_new_traj_success;
  //   if (land_triger_received_) {
  //     generate_new_traj_success = prePtr_->predict(target_p, target_v, target_predcit, landing_dur_);
  //   }else{
  //     generate_new_traj_success = prePtr_->predict(target_p, target_v, target_predcit, tracking_dur_);
  //   }
  //   // ros::Time t_stop = ros::Time::now();
  //   // std::cout << "predict costs: " << (t_stop - t_start).toSec() * 1e3 << "ms" << std::endl;
  //   if (generate_new_traj_success) {
  //     Eigen::Vector3d observable_p = target_predcit.back();
  //     visPtr_->visualize_path(target_predcit, "car_predict");
  //     visPtr_->visualize_pointcloud(target_predcit, "car_predict_pt");
  //     std::vector<Eigen::Vector3d> observable_margin;
  //     for (double theta = 0; theta <= 2 * M_PI; theta += 0.01) {
  //       observable_margin.emplace_back(observable_p + tracking_dist_ * Eigen::Vector3d(cos(theta), sin(theta), 0));
  //     }
  //     visPtr_->visualize_path(observable_margin, "observable_margin");
  //   }

  //   // NOTE replan state
  //   double plan_estimated_duration = 0.07; // s
  //   Eigen::MatrixXd iniState;
  //   iniState.setZero(3, 3);
  //   ros::Time replan_stamp = ros::Time::now() + ros::Duration(plan_estimated_duration);
  //   double replan_t = (replan_stamp - replan_stamp_).toSec();
  //   if (force_hover_ || replan_t > traj_poly_.getTotalDuration()) {
  //     // should replan from the hover state
  //     iniState.col(0) = odom_p;
  //     iniState.col(1) = odom_v;
  //     ROS_ERROR_STREAM("replan from the hover state!");
  //   } else {
  //     // should replan from the last trajectory
  //     iniState.col(0) = traj_poly_.getPos(replan_t);
  //     iniState.col(1) = traj_poly_.getVel(replan_t);
  //     iniState.col(2) = traj_poly_.getAcc(replan_t);
  //     // predict target should also be aligned to replan time
  //     target_p = target_p + plan_estimated_duration * target_v;
  //     for(auto& target:target_predcit){
  //       target = target + plan_estimated_duration * target_v;
  //     }
  //   }
  //   replanStateMsg_.header.stamp = ros::Time::now();
  //   replanStateMsg_.iniState.resize(9);
  //   Eigen::Map<Eigen::MatrixXd>(replanStateMsg_.iniState.data(), 3, 3) = iniState;

  //   // NOTE path searching
  //   Eigen::Vector3d p_start = iniState.col(0);
  //   std::vector<Eigen::Vector3d> path, way_pts;

  //   // NOTE calculate time of path searching, corridor generation and optimization

  //   // static double t_path_ = 0;
  //   // static double t_corridor_ = 0;
  //   // static double t_optimization_ = 0;
  //   // static int times_path_ = 0;
  //   // static int times_corridor_ = 0;
  //   // static int times_optimization_ = 0;
  //   // double t_path = 0;

  //   ros::Time t_front0 = ros::Time::now();

  //   if (generate_new_traj_success) {
  //     if (land_triger_received_) {
  //       generate_new_traj_success = envPtr_->short_astar(p_start, target_p, path); // get a path from current pose to target pose

  //       // generate_new_traj_success = envPtr_->short_astar(p_start, target_predcit.back(), path); // get a path from current pose to target pose
  //     } else {
  //       envPtr_->set_track_angle(track_angle_expect_);
  //       trajOptPtr_->set_track_angle(track_angle_expect_);
  //       visPtr_->visualize_arrow(Eigen::Vector3d(0,0,0), 5.0*Eigen::Vector3d(cos(track_angle_expect_),sin(track_angle_expect_),0),"/track_angle", visualization::blue, "view");
  //       generate_new_traj_success = envPtr_->findVisiblePath(p_start, target_predcit, tracking_dt_, way_pts, path); // way_pts correspond to target_predict, path is denser points
  //     }
  //     // ros::Time t_end0 = ros::Time::now();
  //     // t_path += (t_end0 - t_front0).toSec() * 1e3;
  //   }

  //   // std::cout << "short_astar path: ";
  //   // for(auto p:path){
  //   //   std::cout << p.transpose()<< std::endl;
  //   // }
  //   // std::cout << std::endl;

  //   std::vector<Eigen::Vector3d> visible_ps;
  //   std::vector<double> thetas;
  //   Trajectory traj;
  //   if (generate_new_traj_success) {
  //     visPtr_->visualize_path(path, "astar");
  //     if (land_triger_received_) {
  //       for (const auto& p : target_predcit) {
  //         path.push_back(p); // add target predict pose to path to generate SFC
  //       }
  //     } else {
  //       // NOTE generate visible regions
  //       target_predcit.pop_back();
  //       way_pts.pop_back();
  //       // ros::Time t_front1 = ros::Time::now();
  //       envPtr_->generate_visible_regions(target_predcit, way_pts,
  //                                         visible_ps, thetas);
  //       // ros::Time t_end1 = ros::Time::now();
  //       // t_path += (t_end1 - t_front1).toSec() * 1e3;
  //       visPtr_->visualize_pointcloud(visible_ps, "visible_ps");
  //       visPtr_->visualize_fan_shape_meshes(target_predcit, visible_ps, thetas, "visible_region");

  //       // TODO change the final state
  //       std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> rays;
  //       for (int i = 0; i < (int)way_pts.size(); ++i) {
  //         rays.emplace_back(target_predcit[i], way_pts[i]);
  //       }
  //       visPtr_->visualize_pointcloud(way_pts, "way_pts");
  //       visPtr_->visualize_arrows(rays, "/rays", visualization::green);
  //       way_pts.insert(way_pts.begin(), p_start);
  //       // ros::Time t_front2 = ros::Time::now();
  //       envPtr_->pts2path(way_pts, path);
  //       // ros::Time t_end2 = ros::Time::now();
  //       // t_path += (t_end2 - t_front2).toSec() * 1e3;
  //     }
  //     // NOTE corridor generating
  //     std::vector<Eigen::MatrixXd> hPolys;
  //     std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> keyPts;

  //     // ros::Time t_front3 = ros::Time::now();
  //     visPtr_->visualize_path(path, "path_for_SFC");
  //     envPtr_->generateSFC(path, Eigen::Vector3d(5.0, 5.0, 3.0), hPolys, keyPts, false);


  //     //! Edit SFC
  //     //! cut all z
  //     for(auto& poly:hPolys){
  //       Eigen::Vector3d n(0.0, 0.0, -1.0);
  //       Eigen::Vector3d p = target_p;
  //       poly.conservativeResize(poly.rows(), poly.cols()+1);
  //       poly.col(poly.cols()-1) << n, p;
  //     }
  //     //! Edit SFC

  //     // ros::Time t_end3 = ros::Time::now();
  //     // double t_corridor = (t_end3 - t_front3).toSec() * 1e3;


  //   std::cout << "pgid filter: "<< hPolys.size()<< std::endl;

  //     std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> gmid_lines;
  //     Eigen::Vector3d tempPoly_last;
  //     bool first = true;
  //     for(auto poly:hPolys){
  //       Eigen::MatrixXd v_temp;
  //       if(!geoutils::enumerateVs(poly, v_temp)) std::cout << " vis Poly_keyPts fail! "<< std::endl;
  //       Eigen::Vector3d tempPoly_mid;
  //       tempPoly_mid = v_temp.rowwise().sum() / v_temp.cols();
  //       if(!first){
  //         gmid_lines.emplace_back(tempPoly_last, tempPoly_mid);
  //       }
  //       first = false;
  //       tempPoly_last = tempPoly_mid;
  //       std::cout << "son: "<<tempPoly_mid.transpose() << std::endl;
  //     }
  //       std::cout << "size: "<<gmid_lines.size()<< std::endl;

  //     visPtr_->visualize_arrows(gmid_lines, "/Poly_keyPts", visualization::green);

  //     envPtr_->visCorridor(hPolys);
  //     // visPtr_->visualize_pairline(keyPts, "/keyPts");

  //     // NOTE trajectory optimization
  //     Eigen::MatrixXd finState;
  //     finState.setZero(3, 3);
  //     finState.col(0) = path.back();
  //     finState.col(1) = target_v;

  //     std::cout <<  "replan stamp: " << replan_stamp << std::endl;
  //     std::cout <<  "plan from: " << std::endl;
  //     std::cout <<  iniState << std::endl;
  //     std::cout <<  "plan to: " << std::endl;
  //     std::cout <<  finState << std::endl;

  //     // ros::Time t_front4 = ros::Time::now();
  //     if (land_triger_received_) {
  //       finState.col(0) = target_predcit.back(); //
  //       // generate_new_traj_success = trajOptPtr_->generate_traj(iniState, finState, target_predcit, target_v, hPolys, traj);
  //     } else {
  //       // generate_new_traj_success = trajOptPtr_->generate_traj(iniState, finState,
  //       //                                                        target_predcit, visible_ps, thetas,
  //       //                                                        hPolys, traj);
  //     }
  //     ros::Time t_end4 = ros::Time::now();
  //     double t_optimization = (t_end4 - t_front0).toSec();
  //     if(t_optimization > plan_estimated_duration){
  //       ROS_WARN_STREAM("Plan time exceed!!");
  //     }

  //     // NOTE average calculating time of path searching, corridor generation and optimization

  //     // t_path_ = (t_path_ * times_path_ + t_path) / (++times_path_);
  //     // t_corridor_ = (t_corridor_ * times_corridor_ + t_corridor) / (++times_corridor_);
  //     // t_optimization_ = (t_optimization_ * times_optimization_ + t_optimization) / (++times_optimization_);

  //     // std::cout << "t_path_: " << t_path_ << " ms" << std::endl;
  //     // std::cout << "t_corridor_: " << t_corridor_ << " ms" << std::endl;
  //     std::cout << "t_optimization_: " << t_optimization * 1e3 << " ms" << std::endl;

  //     visPtr_->visualize_traj(traj, "traj");
  //   }

  //   // NOTE collision check
  //   bool valid = false;
  //   if (generate_new_traj_success) {
  //     valid = validcheck(traj, replan_stamp);
  //   } else {
  //     replanStateMsg_.state = -2;
  //     replanState_pub_.publish(replanStateMsg_);
  //   }
  //   if (valid) {
  //     force_hover_ = false;
  //     ROS_WARN("[planner] REPLAN SUCCESS");
  //     replanStateMsg_.state = 0;
  //     replanState_pub_.publish(replanStateMsg_);
  //     Eigen::Vector3d dp = target_p + target_v * 0.03 - iniState.col(0);
  //     // NOTE : if the drone is going to unknown areas, watch that direction
  //     // Eigen::Vector3d un_known_p = traj.getPos(1.0);
  //     // if (gridmapPtr_->isUnKnown(un_known_p)) {
  //     //   dp = un_known_p - odom_p;
  //     // }
  //     double yaw = std::atan2(dp.y(), dp.x());
  //     if (land_triger_received_) {
  //       yaw = 2 * std::atan2(target_q.z(), target_q.w());
  //     }
  //     pub_traj(traj, yaw, replan_stamp);
  //     traj_poly_ = traj;
  //     replan_stamp_ = replan_stamp;
  //   } else if (force_hover_) {
  //     ROS_ERROR("[planner] REPLAN FAILED, HOVERING...");
  //     replanStateMsg_.state = 1;
  //     replanState_pub_.publish(replanStateMsg_);
  //     return;
  //   } else if (!validcheck(traj_poly_, replan_stamp_)) {
  //     force_hover_ = true;
  //     ROS_FATAL("[planner] EMERGENCY STOP!!!");
  //     replanStateMsg_.state = 2;
  //     replanState_pub_.publish(replanStateMsg_);
  //     pub_hover_p(iniState.col(0), replan_stamp);
  //     return;
  //   } else {
  //     ROS_ERROR("[planner] REPLAN FAILED, EXECUTE LAST TRAJ...");
  //     replanStateMsg_.state = 3;
  //     replanState_pub_.publish(replanStateMsg_);
  //     return;  // current generated traj invalid but last is valid
  //   }
  //     // visual traj
  //     {
  //       visPtr_->visualize_traj(traj, "traj");
  //       std::vector<Eigen::Vector3d> wpt_on_traj;
  //       std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> rays;
  //       double t = 0;
  //       std::cout<<"tracing dis: " << std::endl;
  //       for (int i = 0; i < (int)target_predcit.size(); ++i){
  //         wpt_on_traj.push_back(traj.getPos(t));
  //         rays.emplace_back(target_predcit[i], traj.getPos(t));
  //         std::cout<< (traj.getPos(t) - target_predcit[i]).norm() << std::endl;
  //         t += tracking_dt_;
  //       }
  //       visPtr_->visualize_pointcloud(wpt_on_traj, "/wpt_on_traj");
  //       visPtr_->visualize_pairline(rays, "/track_rays");
  //     }
      
  //     if(land_triger_received_ ){ //traj_id_ == 10
  //       trajOptPtr_->set_debug();
  //       // envPtr_->set_debug();
  //     //   while (1)
  //     //   {
  //     //     ;
  //     //   }
  //     }

      
  // }

  // void debug_timer_callback(const ros::TimerEvent& event) {
  //   inflate_gridmap_pub_.publish(replanStateMsg_.occmap);
  //   Eigen::MatrixXd iniState;
  //   iniState.setZero(3, 3);
  //   ros::Time replan_stamp = ros::Time::now() + ros::Duration(0.03);

  //   iniState = Eigen::Map<Eigen::MatrixXd>(replanStateMsg_.iniState.data(), 3, 3);
  //   Eigen::Vector3d target_p(replanStateMsg_.target.pose.pose.position.x,
  //                            replanStateMsg_.target.pose.pose.position.y,
  //                            replanStateMsg_.target.pose.pose.position.z);
  //   Eigen::Vector3d target_v(replanStateMsg_.target.twist.twist.linear.x,
  //                            replanStateMsg_.target.twist.twist.linear.y,
  //                            replanStateMsg_.target.twist.twist.linear.z);
  //   // std::cout << "target_p: " << target_p.transpose() << std::endl;
  //   // std::cout << "target_v: " << target_v.transpose() << std::endl;

  //   // visualize the target and the drone velocity
  //   visPtr_->visualize_arrow(iniState.col(0), iniState.col(0) + iniState.col(1), "drone_vel");
  //   visPtr_->visualize_arrow(target_p, target_p + target_v, "target_vel");

  //   // visualize the ray from drone to target
  //   if (envPtr_->checkRayValid(iniState.col(0), target_p)) {
  //     visPtr_->visualize_arrow(iniState.col(0), target_p, "ray", visualization::yellow);
  //   } else {
  //     visPtr_->visualize_arrow(iniState.col(0), target_p, "ray", visualization::red);
  //   }

  //   // NOTE prediction
  //   std::vector<Eigen::Vector3d> target_predcit;
  //   if (gridmapPtr_->isOccupied(target_p)) {
  //     std::cout << "target is invalid!" << std::endl;
  //     assert(false);
  //   }
  //   bool generate_new_traj_success = prePtr_->predict(target_p, target_v, target_predcit, tracking_dur_);

  //   if (generate_new_traj_success) {
  //     Eigen::Vector3d observable_p = target_predcit.back();
  //     visPtr_->visualize_path(target_predcit, "car_predict");
  //     std::vector<Eigen::Vector3d> observable_margin;
  //     for (double theta = 0; theta <= 2 * M_PI; theta += 0.01) {
  //       observable_margin.emplace_back(observable_p + tracking_dist_ * Eigen::Vector3d(cos(theta), sin(theta), 0));
  //     }
  //     visPtr_->visualize_path(observable_margin, "observable_margin");
  //   }

  //   // NOTE path searching
  //   Eigen::Vector3d p_start = iniState.col(0);
  //   std::vector<Eigen::Vector3d> path, way_pts;
  //   if (generate_new_traj_success) {
  //     generate_new_traj_success = envPtr_->findVisiblePath(p_start, target_predcit, tracking_dt_, way_pts, path);
  //   }

  //   std::vector<Eigen::Vector3d> visible_ps;
  //   std::vector<double> thetas;
  //   Trajectory traj;
  //   if (generate_new_traj_success) {
  //     visPtr_->visualize_path(path, "astar");
  //     // NOTE generate visible regions
  //     target_predcit.pop_back();
  //     way_pts.pop_back();
  //     envPtr_->generate_visible_regions(target_predcit, way_pts,
  //                                       visible_ps, thetas);
  //     visPtr_->visualize_pointcloud(visible_ps, "visible_ps");
  //     visPtr_->visualize_fan_shape_meshes(target_predcit, visible_ps, thetas, "visible_region");
  //     // NOTE corridor generating
  //     std::vector<Eigen::MatrixXd> hPolys;
  //     std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> keyPts;
  //     // TODO change the final state
  //     std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> rays;
  //     for (int i = 0; i < (int)way_pts.size(); ++i) {
  //       rays.emplace_back(target_predcit[i], way_pts[i]);
  //     }
  //     visPtr_->visualize_pointcloud(way_pts, "way_pts");
  //     way_pts.insert(way_pts.begin(), p_start);
  //     envPtr_->pts2path(way_pts, path);
  //     visPtr_->visualize_path(path, "corridor_path");
  //     envPtr_->generateSFC(path, Eigen::Vector3d(2.0, 2.0, 2.0), hPolys, keyPts);
  //     envPtr_->visCorridor(hPolys);
  //     visPtr_->visualize_pairline(keyPts, "keyPts");

  //     // NOTE trajectory optimization
  //     Eigen::MatrixXd finState;
  //     finState.setZero(3, 3);
  //     finState.col(0) = path.back();
  //     finState.col(1) = target_v;

  //     generate_new_traj_success = trajOptPtr_->generate_traj(iniState, finState,
  //                                                            target_predcit, visible_ps, thetas,
  //                                                            hPolys, traj);
  //     visPtr_->visualize_traj(traj, "traj");
  //   }
  //   if (!generate_new_traj_success) {
  //     return;
  //     // assert(false);
  //   }
  //   // check
  //   bool valid = true;
  //   std::vector<Eigen::Vector3d> check_pts, invalid_pts;
  //   double t0 = (ros::Time::now() - replan_stamp).toSec();
  //   t0 = t0 > 0.0 ? t0 : 0.0;
  //   double check_dur = 1.0;
  //   double delta_t = check_dur < traj.getTotalDuration() ? check_dur : traj.getTotalDuration();
  //   for (double t = t0; t < t0 + delta_t; t += 0.1) {
  //     Eigen::Vector3d p = traj.getPos(t);
  //     check_pts.push_back(p);
  //     if (gridmapPtr_->isOccupied(p)) {
  //       invalid_pts.push_back(p);
  //     }
  //   }
  //   visPtr_->visualize_path(invalid_pts, "invalid_pts");
  //   visPtr_->visualize_path(check_pts, "check_pts");
  //   valid = validcheck(traj, replan_stamp);
  //   if (!valid) {
  //     std::cout << "invalid!" << std::endl;
  //   }
  // }

  bool validcheck(const Trajectory<5>& traj, const ros::Time& t_start, const double& check_dur = 1.0) {
    double t0 = (ros::Time::now() - t_start).toSec();
    t0 = t0 > 0.0 ? t0 : 0.0;
    double delta_t = check_dur < traj.getTotalDuration() ? check_dur : traj.getTotalDuration();
    for (double t = t0; t < t0 + delta_t; t += 0.01) {
      Eigen::Vector3d p = traj.getPos(t);
      if (gridmapPtr_->isOccupied(p)) {
        std::cout << "Occupied at: " << p.transpose() <<std::endl;
        return false;
      }
    }
    return true;
  }

  bool validcheck(const Trajectory<7>& traj, const ros::Time& t_start, const double& check_dur = 1.0) {
    double t0 = (ros::Time::now() - t_start).toSec();
    t0 = t0 > 0.0 ? t0 : 0.0;
    double delta_t = check_dur < traj.getTotalDuration() ? check_dur : traj.getTotalDuration();
    for (double t = t0; t < t0 + delta_t; t += 0.01) {
      Eigen::Vector3d p = traj.getPos(t);
      if (gridmapPtr_->isOccupied(p)) {
        std::cout << "Occupied at: " << p.transpose() <<std::endl;
        return false;
      }
    }
    return true;
  }

  void joy_callback(const sensor_msgs::Joy::ConstPtr& msg){
      _joyState.X = msg->axes.at(3);
      _joyState.Y = msg->axes.at(4);
      _joyState.Z = msg->axes.at(1);
      _joyState.yaw = msg->axes.at(0);
      track_angle_expect_ += _joyState.X/4.5*10/180*M_PI;
      while(track_angle_expect_ > M_PI){
        track_angle_expect_ -= 2*M_PI;
      }
      while(track_angle_expect_ <= -M_PI){
        track_angle_expect_ += 2*M_PI;
      }
      
      std::cout<<"track_angle_expect_: "<<track_angle_expect_*180/M_PI<<std::endl;
      // visPtr_->visualize_arrow(Eigen::Vector3d(0,0,0), 5.0*Eigen::Vector3d(cos(track_angle_expect_),sin(track_angle_expect_),0),"/track_angle", visualization::blue, "view");
      // cout << "joy callback: " << _joyState.X << " | " << _joyState.Y << " | " << _joyState.Z << " | " << _joyState.yaw << endl;
  }

  void init(ros::NodeHandle& nh) {
    // set parameters of planning
    
    nh.getParam("plan_hz", plan_hz);
    nh.getParam("tracking_dur", tracking_dur_);
    nh.getParam("landing_dur", landing_dur_);
    nh.getParam("tracking_dist", tracking_dist_);
    nh.getParam("debug", debug_);
    nh.getParam("fake", fake_);
    nh.getParam("vmax", vmax_);
    nh.getParam("tracking_dt", tracking_dt_);

    para_ptr_ = std::make_shared<parameter_server::ParaeterSerer>(
        "/home/ningshan/VM-UAV/Elastic-Tracker/src/planning/planning/config/config.yaml");

    // gridmapPtr_ = std::make_shared<mapping::OccGridMap>();
    // envPtr_ = std::make_shared<env::Env>(nh, gridmapPtr_);
    // visPtr_ = std::make_shared<visualization::Visualization>(nh);
    // trajOptPtr_ = std::make_shared<traj_opt::TrajOpt>(nh);
    // trajOptPtr_->set_gridmap_ptr(gridmapPtr_);
    // prePtr_ = std::make_shared<prediction::Predict>(nh);

    gridmapPtr_ = std::make_shared<mapping::OccGridMap>();
    envPtr_ = std::make_shared<env::Env>(para_ptr_, gridmapPtr_);
    prePtr_ = std::make_shared<prediction::Predict>(para_ptr_);
    trajOptPtr_ = std::make_shared<traj_opt::TrajOpt>(para_ptr_);
    prePtr_->set_gridmap_ptr(gridmapPtr_);
    trajOptPtr_->set_gridmap_ptr(gridmapPtr_);

    targetBBXPtr_ = std::make_shared<env::TargetBBX>();
    targetBBXPtr_->setBBX(2.0, 1.0, 1.0);
    envPtr_->set_target_ptr(targetBBXPtr_);
    envPtr_->set_vis_ptr(visPtr_);
    prePtr_->set_vis_ptr(visPtr_);

    heartbeat_pub_ = nh.advertise<std_msgs::Empty>("heartbeat", 10);
    traj_pub_ = nh.advertise<quadrotor_msgs::PolyTraj>("trajectory", 1);
    replanState_pub_ = nh.advertise<quadrotor_msgs::ReplanState>("replanState", 1);

    // if (debug_) {
    //   plan_timer_ = nh.createTimer(ros::Duration(1.0 / plan_hz), &Nodelet::debug_timer_callback, this);
    //   // TODO read debug data from files
    //   wr_msg::readMsg(replanStateMsg_, ros::package::getPath("planning") + "/../../../debug/replan_state.bin");
    //   inflate_gridmap_pub_ = nh.advertise<quadrotor_msgs::OccMap3d>("gridmap_inflate", 10);
    //   gridmapPtr_->from_msg(replanStateMsg_.occmap);
    //   prePtr_->setMap(*gridmapPtr_);
    //   std::cout << "plan state: " << replanStateMsg_.state << std::endl;
    // }else 
    
    {
      plan_task_timer_ = nh.createTimer(ros::Duration(1.0 / plan_hz), &Nodelet::plan_task_callback, this);
      // plan_timer_ = nh.createTimer(ros::Duration(1.0 / plan_hz), &Nodelet::plan_callback, this);
    }
    gridmap_sub_ = nh.subscribe<quadrotor_msgs::OccMap3d>("gridmap_inflate", 1, &Nodelet::gridmap_callback, this, ros::TransportHints().tcpNoDelay());
    odom_sub_ = nh.subscribe<nav_msgs::Odometry>("odom", 10, &Nodelet::odom_callback, this, ros::TransportHints().tcpNoDelay());
    target_sub_ = nh.subscribe<nav_msgs::Odometry>("target", 10, &Nodelet::target_callback, this, ros::TransportHints().tcpNoDelay());
    goal_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("goal", 10, &Nodelet::goal_callback, this, ros::TransportHints().tcpNoDelay());
    triger_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("triger", 10, &Nodelet::triger_callback, this, ros::TransportHints().tcpNoDelay());
    land_triger_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("land_triger", 10, &Nodelet::land_triger_callback, this, ros::TransportHints().tcpNoDelay());
    joy_sub_ = nh.subscribe<sensor_msgs::Joy>("joy", 10, &Nodelet::joy_callback, this);
    ROS_WARN("Planning node initialized!");

    Trajectory<5> traj;
    visPtr_->visualize_traj(traj, "traj");
    std::vector<Eigen::Vector3d> target_predcit;
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> target_predcit_pair;
    visPtr_->visualize_path(target_predcit, "car_predict");
    visPtr_->visualize_pointcloud(target_predcit, "car_predict_pt");
    visPtr_->visualize_pointcloud(target_predcit, "/wpt_on_traj");
    visPtr_->visualize_pairline(target_predcit_pair, "/Poly_keyPts");
    visPtr_->visualize_pairline(target_predcit_pair, "/track_rays");
    visPtr_->visualize_path(target_predcit, "astar");
    visPtr_->visualize_path(target_predcit, "path_for_SFC");
    visPtr_->visualize_pointcloud(target_predcit, "/debug_in");
    visPtr_->visualize_pointcloud(target_predcit, "/debug_out");
    visPtr_->visualize_pointcloud(target_predcit, "/astar_vp");
    visPtr_->visualize_pointcloud(target_predcit, "/int_waypts");
    visPtr_->visualize_pointcloud(target_predcit, "int_waypts");
    visPtr_->visualize_pointcloud(target_predcit, "mid_waypts");
    visPtr_->visualize_pointcloud(target_predcit, "openset");
    visPtr_->visualize_pointcloud(target_predcit, "curnode");

    
    // visPtr_->visualize_pairline(target_predcit_pair, "visible_pts");


  }

 public:
  void onInit(void) {
    ros::NodeHandle nh(getMTPrivateNodeHandle());
    initThread_ = std::thread(std::bind(&Nodelet::init, this, nh));
  }
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace planning

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(planning::Nodelet, nodelet::Nodelet);