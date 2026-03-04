#include "node/plan_node.h"

using rot_util = rotation_util::RotUtil;

Planner::Planner(std::shared_ptr<ShareDataManager> dataManagerPtr,
                 std::shared_ptr<parameter_server::ParaeterSerer> paraPtr):
                 dataManagerPtr_(dataManagerPtr),
                 paraPtr_(paraPtr)
{
  paraPtr_->get_para("plan_hz", plan_hz_);
  paraPtr_->get_para("use_map", use_map_);
  paraPtr_->get_para("plan_estimated_duration", plan_estimated_duration_);   
  paraPtr_->get_para("land_no_plan_time", land_no_plan_time_);   
  paraPtr_->get_para("tracking_height_expect", tracking_height_expect_); 
  paraPtr_->get_para("tracking_dis_expect", tracking_dis_expect_); 
  paraPtr_->get_para("tracking_angle_expect", tracking_angle_expect_); 
  paraPtr_->get_para("Webvis_hz", web_vis_hz_);   
  paraPtr_->get_para("local_horizon", local_horizon_);   

  paraPtr_->get_para("set_car_v", set_car_v_);   
  paraPtr_->get_para("land_roll", land_roll_);
  paraPtr_->get_para("land_pitch", land_pitch_);

  #ifdef ROS
  rawmapPtr_ = std::make_shared<mapping::OccGridMap>();
  gridmapPtr_ = std::make_shared<map_interface::MapInterface>(rawmapPtr_);
  #endif
  #ifdef SS_DBUS
  // rawmapPtr_ = std::make_shared<lmap>();
  // gridmapPtr_ = std::make_shared<map_interface::MapInterface>(rawmapPtr_);
  gridmapPtr_ = std::make_shared<map_interface::MapInterface>();
  #endif
  
  envPtr_ = std::make_shared<env::Env>(paraPtr_, gridmapPtr_);
  prePtr_ = std::make_shared<prediction::Predict>(paraPtr_);
  trajoptPtr_ = std::make_shared<traj_opt::TrajOpt>(paraPtr_);
  tlplannerPtr_ = std::make_shared<tlplanner::TLPlanner>(paraPtr_);

  prePtr_->set_gridmap_ptr(gridmapPtr_);
  trajoptPtr_->set_gridmap_ptr(gridmapPtr_);
  tlplannerPtr_->set_gridmap_ptr(gridmapPtr_);
  tlplannerPtr_->set_env_ptr(envPtr_);
  tlplannerPtr_->set_pre_ptr(prePtr_);
  tlplannerPtr_->set_trajopt_ptr(trajoptPtr_);
  tlplannerPtr_->set_data_ptr(dataManagerPtr_);
}

void Planner::plan_thread(){
  while (!dataManagerPtr_->s_exit_)
  {    
    int thread_dur = (int) (1000.0 / (double) plan_hz_);
    if (plan_hz_ < 0) continue;
    TimePoint t0 = TimeNow();
    PlanMode mode = get_mode();
    INFO_MSG("------------------- Plan Thread(" <<  get_mode_name() <<") -----------------------");
    #ifdef SS_DBUS
    INFO_MSG("[systime] " << TimeNow() / 1e6);
    #endif
    if (mode == IDLE){
      std::this_thread::sleep_for(std::chrono::milliseconds(thread_dur));
      continue;
    }

    //! 1. obtain odom
    Odom odom_data;
    if (!dataManagerPtr_->get_odom(dataManagerPtr_->odom_info_, odom_data)){
      set_state(PLANFAIL);
      std::this_thread::sleep_for(std::chrono::milliseconds(thread_dur));
      continue;
    } 
    Eigen::Vector3d odom_p = odom_data.odom_p_;
    Eigen::Vector3d odom_v = odom_data.odom_v_;
    // double odom_yaw = rot_util::quaternion2yaw(odom_data.odom_q_);
    INFO_MSG("[plan node] get odom: " << odom_p.transpose());
    if (mode == HOVER){
      dataManagerPtr_->save_hover_p(odom_p, TimeNow(), dataManagerPtr_->traj_info_);
      set_state(NOTNEEDPLAN);
      INFO_MSG("[plan node] hovering...");
    }

    if (mode == HOVER){
      BAG(dataManagerPtr_->BagPtr_->write_float_flatmsg("/state", HOVER));
      std::this_thread::sleep_for(std::chrono::milliseconds(thread_dur));
      continue;
    }


    //! 2. obtain target
    Odom target_data;
    Odom goal_data;
    Eigen::Vector3d land_p;
    // if (mode == GOAL && !dataManagerPtr_->get_odom(dataManagerPtr_->goal_info_, goal_data)){
    //   set_state(PLANFAIL);
    // }
    // if (mode != GOAL && !dataManagerPtr_->get_car_odom(target_data)){
    //   set_state(PLANFAIL);
    // }
    if (!dataManagerPtr_->get_car_odom(target_data)){
      set_state(PLANFAIL);
    }


    // Eigen::Vector3d car_p_start(1, 0, 0.8);
    // double car_theta_start = 0.0;
    // double car_v_value_ = set_car_v_;
    // double car_omega_ = 0.4;
    // static bool is_first_plan_ = false;
    // static TimePoint t_start_ = TimeNow();
    // if (is_first_plan_){
    //     t_start_ = TimeNow();
    //     target_data.odom_p_ = car_p_start;
    //     target_data.odom_v_ << car_v_value_, 0, 0;
    //     target_data.odom_q_ = rot_util::yaw2quaternion(0.0);
    //     target_data.odom_dyaw_ = car_omega_;
    //     is_first_plan_ = false;
    // }
    // else{
    //     TimePoint t_cur = TimeNow();
    //     double car_dt = durationSecond(t_cur, t_start_);
    //     double theta_ = car_theta_start + car_omega_ * car_dt;
    //     target_data.odom_q_ = rot_util::yaw2quaternion(theta_);
    //     target_data.odom_v_ << car_v_value_*cos(theta_), car_v_value_*sin(theta_), 0.0;
    //     target_data.odom_dyaw_ = car_omega_;
    //     target_data.odom_p_.x() = car_p_start.x() + car_v_value_ / car_omega_ * sin(car_theta_start + car_omega_*car_dt) - car_v_value_ / car_omega_ *sin(car_theta_start);
    //     target_data.odom_p_.y() = car_p_start.y() - car_v_value_ / car_omega_ * cos(car_theta_start + car_omega_*car_dt) + car_v_value_ / car_omega_ *cos(car_theta_start);
    //     target_data.odom_p_.z() = car_p_start.z();
    // }

    // Eigen::Vector3d rpy(land_roll_, land_pitch_, rot_util::quaternion2yaw(target_data.odom_q_));
    // Eigen::Quaterniond land_q = rot_util::euler2quaternion(rpy);
    // visPtr_->visualize_odom(target_data.odom_p_, target_data.odom_v_, land_q, "/fake_tar_odom");


    // Eigen::Vector3d p0, p1;
    // p0 = target_data.odom_p_;
    // p1 = p0;
    // p1.x() += cos(rot_util::quaternion2yaw(target_data.odom_q_));
    // p1.y() += sin(rot_util::quaternion2yaw(target_data.odom_q_)); 
    // visPtr_->visualize_arrow(p0, p1, "/ekf_target");
    // if (mode == GOAL){
    //   INFO_MSG("[plan node] get goal: " << goal_data.odom_p_.transpose());
    // }else{
      INFO_MSG("[plan node] get target: " << target_data.odom_p_.transpose() << ", yaw: " << rot_util::rad2deg(rot_util::quaternion2yaw(target_data.odom_q_)));
      INFO_MSG("[plan node] target dp: " << (target_data.odom_p_ - odom_data.odom_p_).transpose());
    // }
    visPtr_->cam_vis_down_->visualize_camera_img(odom_data.odom_p_, odom_data.odom_q_, target_data.odom_p_);
    visPtr_->cam_vis_front_->visualize_camera_img(odom_data.odom_p_, odom_data.odom_q_, target_data.odom_p_);

    if (mode == HOVER){
      BAG(dataManagerPtr_->BagPtr_->write_float_flatmsg("/state", HOVER));
      std::this_thread::sleep_for(std::chrono::milliseconds(thread_dur));
      continue;
    }

    //! 2.5 obtain map
    // lock until this plan loop finish
    if (use_map_){
      if (!dataManagerPtr_->map_received_){
        set_state(PLANFAIL);
        std::this_thread::sleep_for(std::chrono::milliseconds(thread_dur));
        continue;
      }
      std::lock_guard<std::mutex> map_lck(dataManagerPtr_->map_mutex_);
      gridmapPtr_->from_msg(dataManagerPtr_->map_msg_);
      INFO_MSG("[plan node] get map");
      static int map_cnt = 0;
      if (map_cnt++ < 2){
        std::this_thread::sleep_for(std::chrono::milliseconds(thread_dur));
        continue;
      }
    }

    //! 3. get init state from last traj
    TimePoint t00 = TimeNow();
    TimePoint replan_stamp = addDuration(t00, plan_estimated_duration_);
    double replan_t = 0.0;
    Odom init_state;
    TrajData traj_now;
    bool traj_last_valid = has_traj_last_ && 
                           (traj_last_.state_ == TrajData::D5 || traj_last_.state_ == TrajData::D7);
    if (traj_last_valid){
      replan_t = durationSecond(replan_stamp, traj_last_.start_time_);
      traj_last_valid = traj_last_valid && (replan_t <= traj_last_.getTotalDuration());
    }
    // last traj not valid, plan from current odom
    if (!traj_last_valid){
      init_state = odom_data;
      init_state.odom_a_.setZero();
      init_state.odom_dyaw_ = 0.0;
    }
    // plan from last traj
    else{
      init_state.odom_p_ = traj_last_.getPos(replan_t);
      init_state.odom_v_ = traj_last_.getVel(replan_t);
      init_state.odom_a_ = traj_last_.getAcc(replan_t);
      init_state.odom_j_ = traj_last_.getJer(replan_t);
      init_state.odom_q_ = rot_util::yaw2quaternion(traj_last_.getAngle(replan_t));
      init_state.odom_dyaw_ = traj_last_.getAngleRate(replan_t);
      if (traj_last_.getTrajType() == TrajType::NOYAW){
        init_state.odom_q_ = odom_data.odom_q_;
        init_state.odom_dyaw_ = 0.0;
      }
    }
    INFO_MSG("[plan node] get init state");

    //! 4. tlplanner
    tlplanner::TLPlanner::PlanResState plan_res;
    bool no_need_plan = false;
    static bool has_plan_land_ = false;
    switch (mode)
    {
    case GOAL:
      {
        GData<double> track_dis_data, track_h_data;
        dataManagerPtr_->get_data(dataManagerPtr_->tracking_dis_info_, track_dis_data);
        dataManagerPtr_->get_data(dataManagerPtr_->tracking_height_info_, track_h_data);
        tracking_dis_expect_ = track_dis_data.data_;
        tracking_height_expect_ = track_h_data.data_;

        Eigen::Vector3d target_p = target_data.odom_p_;
        target_p.z() += tracking_height_expect_;

        Eigen::Vector3d delta2car = target_p - odom_p;
        double dis2car = delta2car.norm();
        if (dis2car <= tracking_dis_expect_){
          INFO_MSG_RED("[plan node] ERROR! GOAL mode, but drone is in the trakcing range, judge function error!");
          no_need_plan = true;
          break;
        }
        Eigen::Vector3d goal = odom_p + delta2car.normalized() * (dis2car - 1.0 * tracking_dis_expect_);
        Eigen::Vector3d delta2goal = goal - odom_p;
        double dis2goal = delta2goal.norm();
        INFO_MSG("[plan node] dis2car: " << dis2car << ", dis2goal: " << dis2goal);
        if (dis2goal < local_horizon_){
          goal_data.odom_p_ = goal;
          goal_data.odom_v_ = target_data.odom_v_;
        }else{
          goal_data.odom_p_ = odom_p + delta2goal.normalized() * local_horizon_;
          goal_data.odom_v_ = Eigen::Vector3d::Zero();
        }
        bool get_new_goal = (!has_last_goal_ || (goal_data.odom_p_ - last_goal_).norm() > 0.2);

        // replan if get new goal
        if (get_new_goal) {
          INFO_MSG("[plan node] get new goal: " << goal_data.odom_p_.transpose());
          plan_res = tlplannerPtr_->plan_goal(init_state, goal_data, traj_now);
          if (plan_res == tlplanner::TLPlanner::PlanResState::PLANSUCC){
            has_last_goal_ = true;
            last_goal_stamp_ = goal_data.odom_time_stamp_ms_;
            last_goal_ = goal_data.odom_p_;
          }
          INFO_MSG_YELLOW("traj_cur duration: " << traj_now.getTotalDuration());
        }
        // else replan if last traj get occ
        else{
          bool valid = false;
          if (has_traj_last_){
            valid = tlplannerPtr_->valid_cheack(traj_last_);
          }
          if (!valid){
            plan_res = tlplannerPtr_->plan_goal(init_state, goal_data, traj_now);
            if (plan_res == tlplanner::TLPlanner::PlanResState::PLANSUCC){
              has_last_goal_ = true;
              last_goal_stamp_ = goal_data.odom_time_stamp_ms_;
              last_goal_ = goal_data.odom_p_;
            }
            INFO_MSG_YELLOW("traj_cur duration: " << traj_now.getTotalDuration());
          }else{
            no_need_plan = true;
          }
        }
        break;
      }
    case TRACK:
    {
      GData<double> track_dis_data, track_ang_data, track_h_data;
      dataManagerPtr_->get_data(dataManagerPtr_->tracking_dis_info_, track_dis_data);
      dataManagerPtr_->get_data(dataManagerPtr_->tracking_angle_info_, track_ang_data);
      dataManagerPtr_->get_data(dataManagerPtr_->tracking_height_info_, track_h_data);
      tracking_dis_expect_ = track_dis_data.data_;
      tracking_angle_expect_ = track_ang_data.data_;
      tracking_height_expect_ = track_h_data.data_;

      dataManagerPtr_->car_ekf_ptr_->set_track_mode();

      tlplannerPtr_->set_track_angle(tracking_angle_expect_ + rot_util::quaternion2yaw(target_data.odom_q_));
      tlplannerPtr_->set_track_dis(tracking_dis_expect_);
      tlplannerPtr_->set_track_height(tracking_height_expect_);
      TimePoint t_front0 = TimeNow();
      plan_res = tlplannerPtr_->plan_track(init_state, target_data, traj_now);
      TimePoint t_end4 = TimeNow();
      double t_optimization = durationSecond(t_end4, t_front0);
      BAG(dataManagerPtr_->BagPtr_->write_float_flatmsg("/state", TRACK));
      BAG(dataManagerPtr_->BagPtr_->write_float_flatmsg("/time_monitor/plan/opt", t_optimization));

      if (plan_res == tlplanner::TLPlanner::PlanResState::PLANSUCC){
        BAG(dataManagerPtr_->BagPtr_->write_float_flatmsg("traj_opt/succ", 1));
      }else{
        BAG(dataManagerPtr_->BagPtr_->write_float_flatmsg("traj_opt/succ", 0));
      }
      break;
    }
    case LAND:
      {
        // if (has_plan_land_){
        //   INFO_MSG_RED("has plan land, skip");
        //   plan_res = tlplanner::TLPlanner::PlanResState::FAIL;
        //   break;
        // }

        dataManagerPtr_->car_ekf_ptr_->set_land_mode();

        double replan_dt = -1;
        INFO_MSG_YELLOW("traj_last duration: "<<traj_last_.getTotalDuration());
        if (has_traj_last_ && traj_last_.getTrajState() == TrajData::D7){
          // replan_dt = (1.0 / (double) plan_hz_);
          replan_dt = durationSecond(replan_stamp, traj_last_.start_time_);
        }
        // check if too short to replan
        if (has_traj_last_ && traj_last_.getTrajState() == TrajData::D7 && traj_last_.getTotalDuration() - replan_dt < land_no_plan_time_){
          INFO_MSG_GREEN("[plan node] Close to end. Stop Plan.");
          no_need_plan = true;
          // vis virtual camera img
          if (web_vis_hz_ > 0){
            bool is_virtual_tag_detected = 
              visPtr_->cam_vis_down_->visualize_camera_img(odom_data.odom_p_, odom_data.odom_q_, target_data.odom_p_);
              visPtr_->cam_vis_front_->visualize_camera_img(odom_data.odom_p_, odom_data.odom_q_, target_data.odom_p_);
            if (is_virtual_tag_detected){
              INFO_MSG_GREEN("[cam_vis] Tag Detected!");
            }
          }
          break;
        }

        // add noise
        // if (!visPtr_->cam_vis_down_->is_tag_visible(odom_data.odom_p_, odom_data.odom_q_, target_data.odom_p_)){
        //   target_data.odom_p_.x() -= 0.2;
        //   INFO_MSG_RED("[cam_vis] Tag Lost!");
        // }

        // call replan
        TimePoint t_front0 = TimeNow();
        plan_res = tlplannerPtr_->plan_land(init_state, target_data, traj_now, replan_dt);
        TimePoint t_end4 = TimeNow();
        double t_optimization = durationSecond(t_end4, t_front0);
        BAG(dataManagerPtr_->BagPtr_->write_float_flatmsg("/state", LAND));
        BAG(dataManagerPtr_->BagPtr_->write_float_flatmsg("/time_monitor/plan/opt", t_optimization));

        if (plan_res == tlplanner::TLPlanner::PlanResState::PLANSUCC){
          BAG(dataManagerPtr_->BagPtr_->write_float_flatmsg("traj_opt/succ", 1));
        }else{
          BAG(dataManagerPtr_->BagPtr_->write_float_flatmsg("traj_opt/succ", 0));
        }
        INFO_MSG_YELLOW("traj_cur duration: " << traj_now.getTotalDuration());

        // vis virtual camera img
        if (web_vis_hz_ > 0){
          bool is_virtual_tag_detected = 
            visPtr_->cam_vis_down_->visualize_camera_img(odom_data.odom_p_, odom_data.odom_q_, target_data.odom_p_);
            visPtr_->cam_vis_front_->visualize_camera_img(odom_data.odom_p_, odom_data.odom_q_, target_data.odom_p_);
          if (is_virtual_tag_detected){
            INFO_MSG_GREEN("[cam_vis] Tag Detected!");
          }
        }
        
        break;
      }
    default:
      break;
    }

    bool plan_time_valid = true;
    double plan_duration = durationSecond(TimeNow(), t00);
    if (plan_duration > plan_estimated_duration_){
      INFO_MSG_RED("[plan node] Plan Time Exceed: " << plan_duration);
      plan_time_valid = false;
      INFO_MSG_RED("[plan node] Plan Time Exceed Cause Replan Fail.");
    }
    INFO_MSG("[plan node] plan done");

    //! 5. check
    // if no need to replan, directly return
    if (no_need_plan){
      INFO_MSG_GREEN("[plan node] NO NEED TO REPLAN.");
      set_state(NOTNEEDPLAN);
    }
    // if get the new plan result, check it and publish it
    else{
      bool valid = false;
      if (plan_res == tlplanner::TLPlanner::PLANSUCC){
        valid = tlplannerPtr_->valid_cheack(traj_now);
        // if (!plan_time_valid){
        //   valid = false;
        // }
      }
      if (valid){
        traj_now.start_time_ = replan_stamp;
        dataManagerPtr_->write_traj(traj_now, dataManagerPtr_->traj_info_);
        traj_last_ = traj_now;
        has_traj_last_ = true;
        INFO_MSG_GREEN("[plan node] REPLAN SUCCESS.");
        set_state(PLANSUCC);
        if (mode == LAND){
          has_plan_land_ = true;
        }
      }else if (has_traj_last_ && !tlplannerPtr_->valid_cheack(traj_last_)){
        dataManagerPtr_->save_hover_p(odom_p, TimeNow(), dataManagerPtr_->traj_info_);
        INFO_MSG_RED("[plan node] EMERGENCY STOP!");
        set_state(PLANFAIL);
      }else{
        INFO_MSG_RED("[plan node] REPLAN FAIL, EXECUTE LAST TRAJ!");
        set_state(PLANFAIL);
      }
    }

    TimePoint t1 = TimeNow();
    double d0 = durationSecond(t1, t0) * 1e3; // ms
    if (d0 > thread_dur){
      INFO_MSG_RED("[plan node] Error! thread time exceed! " << d0 << "ms");
      continue;
    }else{
      int tr = floor(thread_dur - d0);
      std::this_thread::sleep_for(std::chrono::milliseconds(tr));
    }
  }
  INFO_MSG_RED("[plan node] Thread Exit.");
   
}