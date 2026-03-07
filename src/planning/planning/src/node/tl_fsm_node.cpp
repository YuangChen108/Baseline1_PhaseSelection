#include "node/tl_fsm_node.h"
using rot_util = rotation_util::RotUtil;

TLFSM::TLFSM(){
    #ifdef ROS
    para_ptr_ = std::make_shared<parameter_server::ParaeterSerer>(
        "/home/Quadrotor-Landing-with-Minco/src/planning/planning/config/config.yaml");
    #endif
    #ifdef SS_DBUS
    para_ptr_ = std::make_shared<parameter_server::ParaeterSerer>(
        "/blackbox/config/config.yaml");
    #endif
    dataManagerPtr_ = std::make_shared<ShareDataManager>(para_ptr_);
    
    planner_ = std::make_shared<Planner>(dataManagerPtr_, para_ptr_);
    data_callbacks_ = std::make_shared<DataCallBacks>(dataManagerPtr_, para_ptr_);
    traj_server_ = std::make_shared<TrajServer>(dataManagerPtr_, para_ptr_);

    para_ptr_->get_para("fsm_mode", fsm_mode_);
    para_ptr_->get_para("land_p_x", land_dp_.x());
    para_ptr_->get_para("land_p_y", land_dp_.y());
    para_ptr_->get_para("land_p_z", land_dp_.z());
}

void TLFSM::run(){
    while (!dataManagerPtr_->s_exit_)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        switch (state_)
        {
        case IDLE:
            if (dataManagerPtr_->auto_mode_){
                INFO_MSG_YELLOW("[FSM] IDLE -> HOVER");
                planner_->set_mode(Planner::HOVER);
                state_ = HOVER;
            }
            break;
        case HOVER:
        {
            Odom odom_data;
            if (!dataManagerPtr_->get_odom(dataManagerPtr_->odom_info_, odom_data)){
                break;
            }
            Odom target_data;
            if (!dataManagerPtr_->get_car_odom(target_data)){
                break;
            }
            if (dataManagerPtr_->auto_mode_ && fsm_mode_ == 0 && dataManagerPtr_->plan_trigger_received_){
                bool is_switch_to_track = judge_to_track(odom_data.odom_p_, target_data.odom_p_);
                if (is_switch_to_track){
                    INFO_MSG_YELLOW("[FSM] HOVER -> TRACK");
                    planner_->set_mode(Planner::TRACK);
                    state_ = TRACK;
                }else{
                    INFO_MSG_YELLOW("[FSM] HOVER -> GOAL");
                    planner_->set_mode(Planner::GOAL);
                    state_ = GOAL;
                }
            }
            if (dataManagerPtr_->auto_mode_ && fsm_mode_ == 1 && dataManagerPtr_->plan_trigger_received_){
                INFO_MSG_YELLOW("[FSM] HOVER -> GOAL");
                planner_->set_mode(Planner::GOAL);
                state_ = GOAL;
            }
            break;
        }
        case TRACK:
            if (!dataManagerPtr_->auto_mode_){
                INFO_MSG_YELLOW("[FSM] TRACK -> HOVER");
                planner_->set_mode(Planner::HOVER);
                state_ = HOVER;
            }
            if (dataManagerPtr_->land_trigger_received_){
                INFO_MSG_YELLOW("[FSM] TRACK -> LAND");
                planner_->set_mode(Planner::LAND);
                state_ = LAND;
            }
            break;
        case LAND:
        {
            // Odom odom_data;
            // if (!dataManagerPtr_->get_odom(dataManagerPtr_->odom_info_, odom_data)){
            //     continue;
            // }
            // Odom target_data;
            // if (!dataManagerPtr_->get_car_odom(target_data)){
            //     continue;
            // }
            // Eigen::Vector3d land_pose;
            // double theta = rot_util::quaternion2yaw(target_data.odom_q_);
            // land_pose = target_data.odom_p_ + rot_util::yaw2quaternion(theta) * land_dp_;
            // std::vector<Eigen::Vector3d> land_pose_pc;
            // land_pose_pc.push_back(land_pose);
            // VIS(vis_ptr_->visualize_path(land_pose_pc, "/land_pose"));

            // auto format_double_value = [](double val, int fixed){
            //     auto str = std::to_string(val);
            //     return str.substr(0, str.find(".") + fixed + 1);
            // };
            // std::string oss = "Pose Error: " + format_double_value((odom_data.odom_p_ - land_pose).x(), 3) + ", " + format_double_value((odom_data.odom_p_ - land_pose).y(), 3);
            // APPLOG(AppDebugLog::instance(.log(oss.c_str())));
            // INFO_MSG(oss.c_str());

            if (!dataManagerPtr_->auto_mode_){
                INFO_MSG_YELLOW("[FSM] LAND -> HOVER");
                planner_->set_mode(Planner::HOVER);
                state_ = HOVER;
            }
            bool stopped = judge_to_stop();
            if (stopped){
                // INFO_MSG_YELLOW("[FSM] LAND -> STOP");
                // planner_->set_mode(Planner::IDLE);
                // state_ = STOP;
                INFO_MSG_YELLOW("[FSM] Landed! Force Switching to TRACK (Glued mode).");
                planner_->set_mode(Planner::IDLE); // 让规划器休息
                state_ = STOP;                     // 切入停止状态
            }
            break;
        }
        case STOP:
        {
            BAG(dataManagerPtr_->BagPtr_->write_float_flatmsg("/state", STOP));
            Odom odom_data;
            if (!dataManagerPtr_->get_odom(dataManagerPtr_->odom_info_, odom_data)){
                break;
            }
            Odom target_data;
            if (!dataManagerPtr_->get_car_odom(target_data)){
                break;
            }
            Eigen::Vector3d land_pose;
            double theta = rot_util::quaternion2yaw(target_data.odom_q_);
            land_pose = target_data.odom_p_ + rot_util::yaw2quaternion(theta) * land_dp_;
            auto format_double_value = [](double val, int fixed){
                auto str = std::to_string(val);
                return str.substr(0, str.find(".") + fixed + 1);
            };
            std::string oss = "Pose Error: " + format_double_value((odom_data.odom_p_ - land_pose).x(), 3) + ", " + format_double_value((odom_data.odom_p_ - land_pose).y(), 3);
            APPLOG(AppDebugLog::instance().log(oss.c_str()));
            INFO_MSG(oss.c_str());
            break;
        }
            
        case GOAL:
        {
            Odom odom_data;
            if (!dataManagerPtr_->get_odom(dataManagerPtr_->odom_info_, odom_data)){
                break;
            }
            Odom target_data;
            if (!dataManagerPtr_->get_car_odom(target_data)){
                break;
            }
            bool is_switch_to_track = judge_to_track(odom_data.odom_p_, target_data.odom_p_);
            if (is_switch_to_track){
                INFO_MSG_YELLOW("[FSM] GOAL -> TRACK");
                planner_->set_mode(Planner::TRACK);
                state_ = TRACK;
            }
            break;
        }
            
        default:
            break;
        }
    }
    INFO_MSG_RED("[FSM] Thread Exit.");
}

bool TLFSM::judge_to_track(const Eigen::Vector3d& odom_p, const Eigen::Vector3d& target_p){
    Eigen::Vector3d tar_p = target_p;
    bool switch_to_track = false;
    GData<double> track_dis_data, track_h_data;
    dataManagerPtr_->get_data(dataManagerPtr_->tracking_dis_info_, track_dis_data);
    dataManagerPtr_->get_data(dataManagerPtr_->tracking_height_info_, track_h_data);
    tar_p.z() += track_h_data.data_;
    double dis2car = (odom_p - tar_p).norm();
    INFO_MSG("[FSM] dis2car: " << dis2car <<", track_dis: " << track_dis_data.data_);
    static std::deque<std::pair<double, TimePoint>> dis_vec;
    static double vec_len = 30;
    dis_vec.emplace_back(dis2car, TimeNow());
    bool stable_state = false;
    if (dis_vec.size() > vec_len){
        dis_vec.pop_front();
        double dv = abs(dis_vec.back().first - dis_vec.front().first) / (durationSecond(dis_vec.back().second, dis_vec.front().second));
        INFO_MSG("[plan node] dv: " << dv);
        if (dv < 0.5){
            stable_state = true;
        }
    }
    if ( dis2car <= track_dis_data.data_ + 9.0 && stable_state){
        switch_to_track = true;
    }
    return switch_to_track;
}


bool TLFSM::judge_to_stop(){
    //! obtain traj
    TrajData traj_data;
    if (!dataManagerPtr_->get_traj(dataManagerPtr_->traj_info_, traj_data)){
        return false;
    }
    if (traj_data.getTrajState() != TrajData::D7){
        return false;
    }

    // 1. 获取当前飞机和船的真实状态
    Odom odom_data, target_data;
    if (!dataManagerPtr_->get_odom(dataManagerPtr_->odom_info_, odom_data) || 
        !dataManagerPtr_->get_car_odom(target_data)) {
        return false;
    }

    Eigen::Vector3d drone_p = odom_data.odom_p_;
    Eigen::Vector3d boat_p  = target_data.odom_p_;
    Eigen::Vector3d drone_v = odom_data.odom_v_;
    Eigen::Vector3d boat_v  = target_data.odom_v_;

    // ==================== 🛠️ 补回：导出实际飞行轨迹 (高频记录) ====================
    // 因为 FSM 在 LAND 阶段是 100Hz 运行的，所以这里会密密麻麻地记录下飞机的真实红线
    std::string real_path = "/home/Quadrotor-Landing-with-Minco/actual_traj.csv";
    std::ofstream real_file(real_path, std::ios::out | std::ios::app);
    if (real_file.is_open()) {
        real_file.seekp(0, std::ios::end);
        if (real_file.tellp() == 0) { // 空文件时写表头
            real_file << "SysTime,Real_UAV_X,Real_UAV_Y,Real_UAV_Z,Real_Boat_X,Real_Boat_Y,Real_Boat_Z\n";
        }
        real_file << ros::Time::now().toSec() << ","
                  << drone_p.x() << "," << drone_p.y() << "," << drone_p.z() << ","
                  << boat_p.x() << "," << boat_p.y() << "," << boat_p.z() << "\n";
        real_file.close();
    }
    // =========================================================================

    double dist_z  = abs(drone_p.z() - boat_p.z());
    double dist_xy = (drone_p.head(2) - boat_p.head(2)).norm();
    double rel_speed = (drone_v - boat_v).norm();

    // ==================== 🛠️ 记录降落状态的小工具 (单次记录) ====================
    auto record_landing_state = [&](bool is_success, const std::string& reason) {
        std::string state_path = "/home/Quadrotor-Landing-with-Minco/landing_states.csv";
        std::ofstream state_file(state_path, std::ios::out | std::ios::app);
        if (state_file.is_open()) {
            state_file.seekp(0, std::ios::end);
            if (state_file.tellp() == 0) {
                state_file << "Time,UAV_X,UAV_Y,UAV_Z,Boat_X,Boat_Y,Boat_Z,Dist_XY,Dist_Z,Rel_Speed,Success,Reason\n";
            }
            state_file << ros::Time::now().toSec() << ","
                       << drone_p.x() << "," << drone_p.y() << "," << drone_p.z() << ","
                       << boat_p.x() << "," << boat_p.y() << "," << boat_p.z() << ","
                       << dist_xy << "," << dist_z << "," << rel_speed << ","
                       << (is_success ? 1 : 0) << "," << reason << "\n";
            state_file.close();
        }
    };
    // =========================================================================

    // 3. 判断条件
    TimePoint sample_time = TimeNow();
    double t = durationSecond(sample_time, traj_data.start_time_);
    bool time_is_up = (traj_data.getTotalDuration() < 1.5 && t > traj_data.getTotalDuration() - 0.05);

    // 4. 逻辑 A：时间到了，结算这次降落
    if (time_is_up) {
        if (dist_xy < 0.25 && rel_speed < 1.5) { 
            INFO_MSG_RED("[FSM] 降落完美！对准且速度稳定。Z_err: " << dist_z);
            if (planner_) planner_->saveLandingData(odom_data, target_data, true); 
            dataManagerPtr_->save_end_landing(TimeNow(), dataManagerPtr_->traj_info_);
            record_landing_state(true, "Perfect_Timeout");
            INFO_MSG_RED("[FSM] STOP Propeller!");
            return true;
        } else {
            INFO_MSG_YELLOW("[FSM] 降落失败！水平误差: " << dist_xy << "m, 相对速度: " << rel_speed << "m/s");
            if (planner_) planner_->saveLandingData(odom_data, target_data, false); 
            record_landing_state(false, "Timeout_Fail");
            return true; 
        }
    }

    // 5. 逻辑 B：防砸甲板（时间没到，但已经贴脸了）
    if (dist_z < 0.20 && dist_xy < 0.25 && rel_speed < 1.5) {
        ROS_INFO_STREAM("\033[1;32m[FSM] 真正满足物理条件！XY误差:" << dist_xy << " 记录Success=1\033[0m");
        INFO_MSG_RED("[FSM] 提前安全触地！");
        if (planner_) planner_->saveLandingData(odom_data, target_data, true); 
        record_landing_state(true, "Early_Touchdown");
        return true;
    }

    return false;
}

bool TLFSM::set_thread_para(std::shared_ptr<std::thread>& thread, const int priority, const char* name){
    pthread_setname_np(thread->native_handle(), name);
    struct sched_param thread_param = {};
    thread_param.sched_priority = priority;
    bool succ = false;
    if (pthread_setschedparam(thread->native_handle(), SCHED_RR, &thread_param) == 0){
        INFO_MSG("Set thread priority "<<priority);
        succ = true;
    }else{
        INFO_MSG("Fail to set thread priority "<<priority);
        succ = false;
    }
    return succ;
}

  #ifdef ROS
  void TLFSM::init_ros(ros::NodeHandle& nh){    
    data_callbacks_->init_ros(nh);
    traj_server_->init_ros(nh);

    vis_ptr_ = std::make_shared<vis_interface::VisInterface>(nh);
    planner_->set_vis_ptr(vis_ptr_);
  }
  #endif