#include <node/traj_server_node.h>

using rot_util = rotation_util::RotUtil;

TrajServer::TrajServer(std::shared_ptr<ShareDataManager> dataManagerPtr,
                       std::shared_ptr<parameter_server::ParaeterSerer> paraPtr):
                       dataManagerPtr_(dataManagerPtr),
                       paraPtr_(paraPtr)
{
    paraPtr_->get_para("PD_Ctrl/Kp_horiz", Kp_horiz_);
    paraPtr_->get_para("PD_Ctrl/Kd_horiz", Kd_horiz_);
    paraPtr_->get_para("PD_Ctrl/Kp_vert", Kp_vert_);
    paraPtr_->get_para("PD_Ctrl/Kd_vert", Kd_vert_);
    paraPtr_->get_para("TrajServer/dyaw_max", dyaw_max_);
}

bool TrajServer::exe_traj(const Odom& odom_data, const TrajData& traj_data){
    TimePoint sample_time = addDuration(TimeNow(), dt_to_future_s_);
    double t = durationSecond(sample_time, traj_data.start_time_);

    Eigen::Vector3d odom_p_raw = odom_data.odom_p_;
    Eigen::Vector3d odom_v = odom_data.odom_v_;
    Eigen::Vector3d odom_a = odom_data.odom_a_;
    double odom_yaw = rot_util::quaternion2yaw(odom_data.odom_q_);
    TimePoint odom_stamp_ms = odom_data.odom_time_stamp_ms_;

    if (t > 0){
        if (traj_data.state_ == TrajData::EndLanding){
            has_stop_propeller_ = stop_propeller();
            return true;
        }
        if (traj_data.state_ == TrajData::Hover){
            Eigen::Vector3d zero;
            zero.setZero();
            publish_cmd(traj_data.traj_id_, traj_data.hover_p_, zero, zero,
                        last_yaw_, 0, false, odom_p_raw, odom_v, odom_a, odom_yaw,
                        odom_stamp_ms, sample_time);
            return true;
        }
        if (t > traj_data.getTotalDuration()){
            // INFO_MSG_RED("[traj_server] Traj execute done!  Hovering...");
            Eigen::Vector3d p, zero;
            zero.setZero();
            p = traj_data.getPos(traj_data.getTotalDuration());
            publish_cmd(traj_data.traj_id_, p, zero, zero,
                        last_yaw_, 0, false, odom_p_raw, odom_v, odom_a, odom_yaw,
                        odom_stamp_ms, sample_time);
            return true;
        }
        Eigen::Vector3d p, v, a;
        double yaw, dyaw;
        p = traj_data.getPos(t);
        v = traj_data.getVel(t);
        a = traj_data.getAcc(t);
        yaw = traj_data.getAngle(t);
        dyaw = traj_data.getAngleRate(t);

        if (traj_data.getTrajType() == TrajType::NOYAW){
            double last_yaw = last_yaw_;
            publish_cmd(traj_data.traj_id_, p, v, a,
                        last_yaw, 0, true, odom_p_raw, odom_v, odom_a, odom_yaw,
                        odom_stamp_ms, sample_time);
            last_yaw_ = last_yaw;
        }else{
            publish_cmd(traj_data.traj_id_, p, v, a,
                        yaw, 0, false, odom_p_raw, odom_v, odom_a, odom_yaw,
                        odom_stamp_ms, sample_time);
            last_yaw_ = yaw;
        }
        return true;
    }
    return false;
}

void TrajServer::cmd_thread(){
  while (!dataManagerPtr_->s_exit_)
  {    
    int thread_dur = (int) (1000.0 / (double) cmd_hz_);
    
    TimePoint t0 = TimeNow();
    //! 1. obtain odom
    Odom odom_data;
    if (!dataManagerPtr_->get_odom(dataManagerPtr_->odom_info_, odom_data)){
        std::this_thread::sleep_for(std::chrono::milliseconds(thread_dur));
        continue;
    }

    //! 2. obtain traj
    TrajData traj_data;
    if (!dataManagerPtr_->get_traj(dataManagerPtr_->traj_info_, traj_data)){
        last_yaw_ = rot_util::quaternion2yaw(odom_data.odom_q_);
        std::this_thread::sleep_for(std::chrono::milliseconds(thread_dur));
        continue;
    }

    //! 3. execute traj
    if (exe_traj(odom_data, traj_data)){
        // vis traj
        if (traj_data.traj_id_ > traj_data_last_.traj_id_){
            traj2vis(traj_data);
        }
        traj_data_last_ = traj_data;
        has_last_traj_ = true;
    }else if (has_last_traj_){
        if (traj_data.traj_id_ - traj_data_last_.traj_id_ > 1){
            INFO_MSG_RED("[traj_server] Error! trajid jump, please adjust the plan_hz and plan_estimated_time.");
        }
        exe_traj(odom_data, traj_data_last_);
    }

    TimePoint t1 = TimeNow();
    double d0 = durationSecond(t1, t0) * 1e3;
    if (d0 > thread_dur){
        INFO_MSG_RED("[traj_server] Error! thread time exceed " << d0 << "ms!");
        continue;
    }
    else{
        int tr_nano = floor(thread_dur*1e6 - d0*1e6);
        std::this_thread::sleep_for(std::chrono::nanoseconds(tr_nano));
    }
  }
  INFO_MSG_RED("[traj_server] Thread Exit.");
}

Eigen::Quaterniond TrajServer::cmd2odom(const Eigen::Vector3d& acc, const double& yaw){
    Eigen::Vector3d alpha = Eigen::Vector3d(acc.x(), acc.y(), acc.z()) + 9.8*Eigen::Vector3d(0,0,1);
    Eigen::Vector3d xC(cos(yaw), sin(yaw), 0);
    Eigen::Vector3d yC(-sin(yaw), cos(yaw), 0);
    Eigen::Vector3d xB = (yC.cross(alpha)).normalized();
    Eigen::Vector3d yB = (alpha.cross(xB)).normalized();
    Eigen::Vector3d zB = xB.cross(yB);
    Eigen::Matrix3d R;
    R.col(0) = xB;
    R.col(1) = yB;
    R.col(2) = zB;
    Eigen::Quaterniond q(R);
    return q;
}
