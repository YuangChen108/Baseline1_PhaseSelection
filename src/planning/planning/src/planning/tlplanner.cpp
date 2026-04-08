#include "planning/tlplanner.h"
using rot_util = rotation_util::RotUtil;

namespace tlplanner{
TLPlanner::TLPlanner(std::shared_ptr<parameter_server::ParaeterSerer>& para_ptr):paraPtr_(para_ptr){
    paraPtr_->get_para("Webvis_hz", web_vis_hz_);   
    paraPtr_->get_para("is_use_viewpoint", is_use_viewpoint_);   
    paraPtr_->get_para("plan_estimated_duration", plan_estimated_duration_);   

    paraPtr_->get_para("tracking_dur", tracking_dur_);   
    paraPtr_->get_para("tracking_dt", tracking_dt_); 
    paraPtr_->get_para("tracking_height_expect", tracking_height_expect_); 
    paraPtr_->get_para("tracking_dis_expect", tracking_dis_expect_); 
    paraPtr_->get_para("tracking_angle_expect", tracking_angle_expect_); 
    paraPtr_->get_para("is_with_perception", is_with_perception_); 

    paraPtr_->get_para("land_p_x", land_dp_.x());
    paraPtr_->get_para("land_p_y", land_dp_.y());
    paraPtr_->get_para("land_p_z", land_dp_.z());

    paraPtr_->get_para("land_roll", land_roll_);
    paraPtr_->get_para("land_pitch", land_pitch_);

    std::string res_file_name;
    paraPtr_->get_para("res_file_name", res_file_name);
    INFO_MSG_RED("file name: " << res_file_name << ".csv");
    csvWriterPtr_ = std::make_shared<csv_writer::CsvWriter>(res_file_name);
}


TLPlanner::PlanResState TLPlanner::plan_goal(const Odom& init_state_in, const Odom& target_data, TrajData& traj_data){
    //! 3. get inital state
    Eigen::MatrixXd init_state, init_yaw;
    init_state.setZero(3, 3);
    init_yaw.setZero(1, 2);
    init_state.col(0) = init_state_in.odom_p_;
    init_state.col(1) = init_state_in.odom_v_;
    init_state.col(2) = init_state_in.odom_a_;
    init_yaw(0, 0)    = rot_util::quaternion2yaw(init_state_in.odom_q_);
    init_yaw(0, 1)    = init_state_in.odom_dyaw_;

    INFO_MSG("init p: " << init_state.col(0).transpose());
    INFO_MSG("init v: " << init_state.col(1).transpose());
    INFO_MSG("init a: " << init_state.col(2).transpose());
    INFO_MSG("init yaw: " << init_yaw);
    INFO_MSG("tar p: " << target_data.odom_p_.transpose());
    INFO_MSG("tar v: " << target_data.odom_v_.transpose());
    INFO_MSG("tar a: " << target_data.odom_a_.transpose());
    INFO_MSG("tar theta: " << rot_util::quaternion2yaw(target_data.odom_q_));

    //! 4. path search
    Eigen::Vector3d p_start = init_state.col(0);
    std::vector<Eigen::Vector3d> path, way_pts;
    bool generate_new_traj_success = envPtr_->short_astar(p_start, target_data.odom_p_, path); // get a path from current pose to target pose
    if (!generate_new_traj_success) {
        INFO_MSG_RED("[tlplanner] A path fail!");
        return FAIL;
    }

    //! 5. traj opt
    Trajectory<5> traj_goal;
    Eigen::MatrixXd final_state, final_yaw;
    final_state.setZero(3, 4);
    final_yaw.setZero(1, 2);
    final_state.col(0) = path.back();
    final_state.col(1) = target_data.odom_v_;
    final_yaw(0, 0) = rot_util::quaternion2yaw(target_data.odom_q_);
    INFO_MSG("init p: " << init_state.col(0).transpose());
    INFO_MSG("end p: " << final_state.col(0).transpose());

    generate_new_traj_success = trajoptPtr_->generate_traj(init_state, final_state, 2.0, path, traj_goal);
    //todo goal yaw still has problem
    // generate_new_traj_success = trajoptPtr_->generate_traj(init_state, final_state, init_yaw, final_yaw, 2.0, path, traj_goal);

    if (!generate_new_traj_success) {
        INFO_MSG_RED("[tlplanner] Traj opt fail!");
        return FAIL;
    }
    traj_data.traj_d5_ = traj_goal;
    traj_data.state_ = TrajData::D5;

    if (web_vis_hz_ > 0){
        visPtr_->visualize_path(path, "astar");
        visPtr_->visualize_traj(traj_goal, "traj");
    }

    return PLANSUCC;
}

// car tracking plan
TLPlanner::PlanResState TLPlanner::plan_track(const Odom& init_state_in, const Odom& target_data, TrajData& traj_data){
    //! 1. target fix (水平运动预测)
    prediction::State tar;
    double theta = rot_util::quaternion2yaw(target_data.odom_q_);
    
    // 提取船的真实水平线速度
    double v = target_data.odom_v_.head(2).dot(Eigen::Vector2d(cos(theta), sin(theta)));
    
    // 传入小船当前真实的 X,Y,Z 和速度 (Vz 默认为 0)
    prediction::State tar_in(target_data.odom_p_, v, 0.0, 
                             theta, target_data.odom_dyaw_);
                             
    // ❌ 注意：这里绝对不要把 Z 轴加上 tracking_height_expect_！
    // 否则会触发“2米高幽灵巨浪”Bug
                             
    // 运行底层的 CYRA 模型预测 (这一步主要是为了拿准确的 X 和 Y)
    prediction::Predict::CYRA_model(tar_in, plan_estimated_duration_, tar);

    //! 2. prediction (生成预测轨迹并“降维打击”)
    bool generate_new_traj_success;
    std::vector<Eigen::Vector3d> target_predcit;
    
    generate_new_traj_success = prePtr_->predict(tar.p_, tar.v_, 0.0, tar.theta_, tar.omega_, target_predcit, tracking_dur_);

    if (!generate_new_traj_success) {
        INFO_MSG_RED("[tlplanner] Predict fail!");
        return FAIL;
    }
    INFO_MSG("[tlplanner] prediction done");

    // ==================== ✅ 核心“压路机”魔法 ====================
    // 我们锁定一个绝对平滑的高空 Z 值：小船当前实际高度 + 期望伴飞高度
    // 为了防止飞机在伴飞时产生任何微小的上下抖动，我们直接写死这个平面
    double flat_tracking_z =  tracking_height_expect_;
    
    // 把整条预测轨迹的 Z 轴全部碾平！变成纯粹的 2D 平滑曲线
    for (auto& pt : target_predcit) {
        pt.z() = flat_tracking_z;
    }
    
    // 把端点也修正到同一高度，防止 MINCO 结尾处掉高度
    tar.p_.z() = flat_tracking_z;
    // =============================================================

    //! 3. get inital state
    Eigen::MatrixXd init_state, init_yaw;
    init_state.setZero(3, 3);
    init_yaw.setZero(1, 2);
    init_state.col(0) = init_state_in.odom_p_;
    init_state.col(1) = init_state_in.odom_v_;
    init_state.col(2) = init_state_in.odom_a_;
    init_yaw(0, 0)    = rot_util::quaternion2yaw(init_state_in.odom_q_);
    init_yaw(0, 1)    = init_state_in.odom_dyaw_;

    //! 4. path search (使用抬高后的轨迹去搜路)
    Eigen::Vector3d p_start = init_state.col(0);
    std::vector<Eigen::Vector3d> path, way_pts;
    envPtr_->set_track_angle(tracking_angle_expect_);
    envPtr_->set_track_dis(tracking_dis_expect_);
    
    // ✅ 这里传入 target_predict_high
    generate_new_traj_success = envPtr_->findVisiblePath(p_start, target_predcit, tracking_dt_, way_pts, path, is_use_viewpoint_);
    
    if (!generate_new_traj_success) {
        INFO_MSG_RED("[tlplanner] A path fail!");
        return FAIL;
    }
    
    #ifdef ROS
    if (web_vis_hz_ > 0){
        // 可视化原始的水面轨迹 (方便调试对比)
        visPtr_->visualize_path(target_predcit, "car_predict");
        visPtr_->visualize_pointcloud(target_predcit, "car_predict_pt");
        visPtr_->visualize_path(path, "astar");
        visPtr_->visualize_pointcloud(way_pts, "astar_wpt");
    }
    #endif

    //! 5. traj opt
    Trajectory<5> traj_track;
    Eigen::MatrixXd final_state, final_yaw;
    final_state.setZero(3, 4);
    final_yaw.setZero(1, 2);

    // ✅ 二次保险：清洗 A* 路径，确保所有点都在高空
    for (auto& pt : path) {
        pt.z() = std::max(pt.z(), tracking_height_expect_);
    }
    
    final_state.col(0) = path.back(); // 此时已经是抬高过的了
    final_state.col(1) = Eigen::Vector3d(tar.v_*cos(tar.theta_), tar.v_*sin(tar.theta_), 0.0);
    final_yaw(0, 0) = tar.theta_;
    
    trajoptPtr_->set_track_angle(tracking_angle_expect_);
    trajoptPtr_->set_track_dis(tracking_dis_expect_);

    // 注意：这里传给 MINCO 的 target_predcit 还是原始的（水面的），但这通常只影响可视性权重
    // 如果想要完全一致，这里也可以传 target_predict_high，但前面 path 抬高了就足够了
    generate_new_traj_success = trajoptPtr_->generate_traj(init_state, final_state, init_yaw, final_yaw, 2.0, path, way_pts, target_predcit, traj_track);
    
    if (!generate_new_traj_success) {
        INFO_MSG_RED("[tlplanner] Traj opt fail!");
        return FAIL;
    }

    // ==================== ✅ 事后防钻地检查 ====================
    traj_data.traj_d5_ = traj_track;
    traj_data.state_ = TrajData::D5;
    traj_data.start_time_ = TimeNow(); // 使用 TimeNow()

    if (!valid_cheack(traj_data, TimeNow())) { 
        INFO_MSG_RED("[plan_track] ❌ Trajectory hits WATER! Rejecting.");
        return FAIL; 
    }
    // =========================================================

    Eigen::Vector3d vis_arrow_p1 = tar.p_;
    Eigen::Vector3d vis_arrow_p2 = tar.p_ + 2.0 * Eigen::Vector3d(cos(tracking_angle_expect_), sin(tracking_angle_expect_), 0.0);

    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> rays;
    double t = 0;
    for (int i = 0; i < (int)target_predcit.size(); ++i){
        rays.emplace_back(target_predcit[i], traj_track.getPos(t));
        t += tracking_dt_;
    }

    BAG(dataManagerPtr_->BagPtr_->write_path_msg("/car_predict", target_predcit));
    BAG(dataManagerPtr_->BagPtr_->write_pointcloud_msg("/car_predict_pt", target_predcit));
    BAG(dataManagerPtr_->BagPtr_->write_path_msg("/astar", path));
    BAG(dataManagerPtr_->BagPtr_->write_traj("/traj", traj_track));
    BAG(dataManagerPtr_->BagPtr_->write_arrow("/track_angle", vis_arrow_p1, vis_arrow_p2));
    BAG(dataManagerPtr_->BagPtr_->write_pairline("/track_rays", rays));

    if (web_vis_hz_ > 0){
        visPtr_->visualize_traj(traj_track, "traj");
        visPtr_->visualize_arrow(vis_arrow_p1, vis_arrow_p2, "/track_angle");
        visPtr_->visualize_pairline(rays, "/track_rays");
    }

    return PLANSUCC;
}

TLPlanner::PlanResState TLPlanner::plan_land(const Odom& init_state_in, const Odom& target_data, TrajData& traj_data, const double& t_replan) {
    auto start_solve = std::chrono::high_resolution_clock::now();

    // ==================== 🧪 实验对照组开关 (Baseline 开关) ====================
    // true:  Baseline 1 (开启 XY 预测)
    // false: Baseline 0 (完全关闭预测，认为目标静止)
    bool use_prediction_logic = true; // 👈 在这里切换 Baseline 0 和 1
    // =========================================================================

    //! 1. target fix (水平运动预测)
    prediction::State tar;
    double theta = rot_util::quaternion2yaw(target_data.odom_q_);
    
    // 原始提取速度
    double v = target_data.odom_v_.head(2).dot(Eigen::Vector2d(cos(theta), sin(theta)));
    double omega = target_data.odom_dyaw_;

    // ==================== //! 🛡️ 位置与速度双重锁 (加固版) ====================
    static Eigen::Vector3d last_boat_p = target_data.odom_p_;
    static bool first_frame = true;

    // 先准备一个要传给 tar_in 的位置变量
    Eigen::Vector3d sanitized_p = target_data.odom_p_;
    double pos_jump = (target_data.odom_p_ - last_boat_p).norm();

    // 1. 位置锁：防止瞬移
    if (!first_frame && pos_jump > 0.4) { // 稍微放宽到0.4，适配20Hz左右的波动
        sanitized_p = last_boat_p; 
    } else {
        last_boat_p = target_data.odom_p_;
        first_frame = false;
    }

    // 2. 速度锁：物理限制
    const double MAX_BOAT_V = 1.5; 
    const double MAX_BOAT_OMEGA = 0.5; 
    if (v > MAX_BOAT_V) v = MAX_BOAT_V;
    else if (v < -MAX_BOAT_V) v = -MAX_BOAT_V;

    if (std::abs(omega) > MAX_BOAT_OMEGA) {
        omega = (omega > 0) ? MAX_BOAT_OMEGA : -MAX_BOAT_OMEGA;
    }
    // =======================================================================

    // ✅ 现在使用经过清洗的 sanitized_p 构建状态
    prediction::State tar_in(sanitized_p, v, 0.0, theta, omega);
    
    // 加上降落板相对于船体中心的偏移
    tar_in.p_ = tar_in.p_ + rot_util::yaw2quaternion(tar_in.theta_) * land_dp_;

    // ==================== 🚀 预测分支逻辑 (Baseline 开关控制) ====================
    bool generate_new_traj_success = true;
    std::vector<Eigen::Vector3d> target_predcit;
    
    if (use_prediction_logic) {
        // 【Baseline 1】执行 CYRA 预测模型，并生成未来轨迹点阵
        prediction::Predict::CYRA_model(tar_in, plan_estimated_duration_, tar);
        generate_new_traj_success = prePtr_->predict(tar.p_, tar.v_, 0.0, tar.theta_, tar.omega_, target_predcit, tracking_dur_);
    } else {
        // 【Baseline 0】认为拦截点就是“此时此刻”的位置，速度设为 0
        tar = tar_in;
        tar.v_ = 0.0;
        tar.omega_ = 0.0;
        
        // 填充静态点阵，让优化器以为终点没动
        int pts_num = std::max(1, (int)(tracking_dur_ / tracking_dt_));
        for (int i = 0; i < pts_num; ++i) {
            target_predcit.push_back(tar_in.p_);
        }
    }

    if (!generate_new_traj_success) {
        INFO_MSG_RED("[tlplanner] Predict fail!");
        return FAIL;
    }
    // =======================================================================

    // ==================== ✅ 核心“压路机” (降落目标：高度 0.0) ====================
    // 目标高度为降落板实际高度
    double flat_land_z = tar_in.p_.z(); 
    
    for (auto& pt : target_predcit) {
        pt.z() = flat_land_z;
    }
    tar.p_.z() = flat_land_z;
    // =======================================================================

    //! 3. get inital state
    Eigen::MatrixXd init_state, init_yaw;
    init_state.setZero(3, 3);
    init_yaw.setZero(1, 2);
    init_state.col(0) = init_state_in.odom_p_;
    init_state.col(1) = init_state_in.odom_v_;
    init_state.col(2) = init_state_in.odom_a_;
    init_yaw(0, 0)    = rot_util::quaternion2yaw(init_state_in.odom_q_);
    init_yaw(0, 1)    = init_state_in.odom_dyaw_;

    //! 4. path search (使用 A* 寻找下降走廊)
    Eigen::Vector3d p_start = init_state.col(0);
    std::vector<Eigen::Vector3d> path, way_pts;
    
    envPtr_->set_track_angle(0.0);
    envPtr_->set_track_dis(0.0);
    
    generate_new_traj_success = envPtr_->findVisiblePath(p_start, target_predcit, tracking_dt_, way_pts, path, is_use_viewpoint_);
    
    if (!generate_new_traj_success) {
        INFO_MSG_RED("[tlplanner] A path fail!");
        return FAIL;
    }

    //! 5. traj opt
    Trajectory<5> traj_land; 
    Eigen::MatrixXd final_state, final_yaw;
    final_state.setZero(3, 4);
    final_yaw.setZero(1, 2);

    for (auto& pt : path) {
        pt.z() = std::max(pt.z(), flat_land_z);
    }
    
    final_state.col(0) = path.back(); 
    
    // ==================== 🚀 预期末端速度约束 (Baseline 开关控制) ====================
    if (use_prediction_logic) {
        final_state.col(1) = Eigen::Vector3d(tar.v_*cos(tar.theta_), tar.v_*sin(tar.theta_), 0.0);
    } else {
        final_state.col(1) = Eigen::Vector3d(0.0, 0.0, 0.0);
    }
    // =======================================================================
    
    final_yaw(0, 0) = tar.theta_;
    
    trajoptPtr_->set_track_angle(0.0);
    trajoptPtr_->set_track_dis(0.0);

    generate_new_traj_success = trajoptPtr_->generate_traj(init_state, final_state, init_yaw, final_yaw, 2.0, path, way_pts, target_predcit, traj_land);
    
    auto end_solve = std::chrono::high_resolution_clock::now();
    double solve_time_ms = std::chrono::duration<double, std::milli>(end_solve - start_solve).count();

    if (!generate_new_traj_success) {
        INFO_MSG_RED("[tlplanner] Traj opt fail!");
        return FAIL;
    }

    // ==================== 状态机赋值 ====================
    traj_data.traj_d5_ = traj_land;
    traj_data.state_ = TrajData::D5; 
    traj_data.start_time_ = TimeNow();

    // ==================== 为了 CSV 日志不报错的计算与占位 ====================
    bool is_use_predict = use_prediction_logic; // 🚀 自动同步 CSV 标志位
    double dist_3d = (init_state_in.odom_p_ - tar_in.p_).norm();
    double dist_xy = (init_state_in.odom_p_.head(2) - tar_in.p_.head(2)).norm();
    Eigen::Vector3d target_p_final = final_state.col(0);
    Eigen::Vector3d target_v_final = final_state.col(1);
    double target_theta_final = final_yaw(0,0);
    double target_omega_final = tar.omega_;
    double estimated_T = plan_estimated_duration_;
    int seg_num = 5;
    double z_offset = 0.0;
    double pred_offset = (target_p_final - tar_in.p_).norm();

    double current_traj_cost = 0.0; 
    if (traj_land.getTotalDuration() > 0.001) {
        double dt = 0.05; 
        double total_dur = traj_land.getTotalDuration();
        for (double t = 0.0; t <= total_dur; t += dt) {
            Eigen::Vector3d jerk = traj_land.getJer(t); 
            current_traj_cost += jerk.squaredNorm() * dt; 
        }
    }
    
    prediction::State tar_now = tar_in;

    // ==================== 📊 CSV 导出 (全过程记录版) ====================
    auto now_chrono = TimeNow();
    auto dur_since_epoch = now_chrono.time_since_epoch();
    double time_sec = std::chrono::duration_cast<std::chrono::microseconds>(dur_since_epoch).count() / 1e6;

    static double run_id = time_sec; 

    static std::ofstream detail_file;
    const std::string file_path = "/home/Quadrotor-Landing-with-Minco/experiments_data/landing_deep_analysis.csv";
    
    std::ifstream check_file(file_path);
    bool file_exists = check_file.good();
    check_file.close();

    if (!detail_file.is_open()) {
        detail_file.open(file_path, std::ios::out | std::ios::app);
        if (!file_exists) {
            detail_file << "Run_ID,Time,Is_Predict,Dist_3D,Dist_XY,UAV_X,UAV_Y,UAV_Z,UAV_VX,UAV_VY,UAV_VZ,"
                        << "Boat_X,Boat_Y,Boat_Z,Boat_VX,Boat_VY,Boat_VZ,Target_PX,Target_PY,Target_PZ,"
                        << "Target_VZ,Estimated_T,Seg_Num,Z_Offset,Traj_Duration,Solve_Time_ms,Jerk_Cost,Pred_Offset" << std::endl;
        }
    }
    
    detail_file << std::fixed << std::setprecision(6)
                << run_id << "," << time_sec << "," << (is_use_predict ? 1 : 0) << "," << dist_3d << "," << dist_xy << ","
                << init_state_in.odom_p_.x() << "," << init_state_in.odom_p_.y() << "," << init_state_in.odom_p_.z() << ","
                << init_state_in.odom_v_.x() << "," << init_state_in.odom_v_.y() << "," << init_state_in.odom_v_.z() << ","
                << tar_now.p_.x() << "," << tar_now.p_.y() << "," << tar_now.p_.z() << ","
                << target_data.odom_v_.x() << "," << target_data.odom_v_.y() << "," << target_data.odom_v_.z() << ","
                << target_p_final.x() << "," << target_p_final.y() << "," << target_p_final.z() << ","
                << target_v_final.z() << "," << estimated_T << "," << seg_num << "," << z_offset << ","
                << traj_land.getTotalDuration() << "," 
                << solve_time_ms << "," << current_traj_cost << "," << pred_offset << std::endl;
    detail_file.flush();

    // ==================== 7. Bag 与 Web 记录 ====================
    csvWriterPtr_->set_target_state(target_p_final, target_v_final, target_theta_final, target_omega_final);

    std::string plan_path = "/home/Quadrotor-Landing-with-Minco/experiments_data/planned_traj.csv";
    std::ofstream plan_file(plan_path, std::ios::out); 
    if (plan_file.is_open()) {
        plan_file << "Time,UAV_Plan_X,UAV_Plan_Y,UAV_Plan_Z,UAV_Plan_VX,UAV_Plan_VY,UAV_Plan_VZ\n";
        for (double t = 0; t <= traj_land.getTotalDuration(); t += 0.05) {
            Eigen::Vector3d p = traj_land.getPos(t);
            Eigen::Vector3d v = traj_land.getVel(t);
            plan_file << t << "," << p.x() << "," << p.y() << "," << p.z() << ","
                      << v.x() << "," << v.y() << "," << v.z() << "\n";
        }
        plan_file.close();
    }

    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> rays;
    double t = 0;
    for (int i = 0; i < (int)target_predcit.size(); ++i){
        if (t > traj_land.getTotalDuration()) break;
        rays.emplace_back(target_predcit[i], traj_land.getPos(t));
        t += tracking_dt_;
    }

    BAG(dataManagerPtr_->BagPtr_->write_path_msg("/car_predict", target_predcit));
    BAG(dataManagerPtr_->BagPtr_->write_pointcloud_msg("/car_predict_pt", target_predcit));
    BAG(dataManagerPtr_->BagPtr_->write_path_msg("/astar", path));
    BAG(dataManagerPtr_->BagPtr_->write_traj("/traj", traj_land));
    BAG(dataManagerPtr_->BagPtr_->write_pairline("/track_rays", rays));

    if (web_vis_hz_ > 0){
        visPtr_->visualize_path(target_predcit, "car_predict");
        visPtr_->visualize_pointcloud(target_predcit, "car_predict_pt");
        visPtr_->visualize_path(path, "astar");
        visPtr_->visualize_traj(traj_land, "traj");
        visPtr_->visualize_pairline(rays, "/track_rays");
    }

    return PLANSUCC;
}

bool TLPlanner::valid_cheack(const TrajData& traj_data, const TimePoint& cur_t){
    double t0 = durationSecond(cur_t, traj_data.start_time_);
    t0 = t0 > 0.0 ? t0 : 0.0;
    INFO_MSG("------------valid check");
    for (double t = t0; t < traj_data.getTotalDuration(); t += 0.01){
        Eigen::Vector3d p = traj_data.getPos(t);
        if (gridmapPtr_->isOccupied(p)){
            return false;
        }
    }
    return true;
}

} // namespace tlplanner