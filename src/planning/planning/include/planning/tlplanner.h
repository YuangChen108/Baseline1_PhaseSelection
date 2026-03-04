#ifndef TLPLANNER
#define TLPLANNER

#include "util_gym/util_gym.hpp"
#include "util_gym/data_manager.hpp"

#include "rotation_util/rotation_util.hpp"
#include "map_interface/map_interface.h"
#include "traj_opt/traj_opt.h"
#include "parameter_server/parameter_server.hpp"
#include "prediction/prediction_car.hpp"
#include "traj_opt/csv_writer.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <env/env.hpp>

#include <thread>
#include <csignal>
#include <cmath>
#include <ctime>
#include <memory>

namespace tlplanner{

class TLPlanner{
public:
    enum PlanResState{
        PLANSUCC,
        FAIL,
    };
    std::shared_ptr<traj_opt::TrajOpt> trajoptPtr_;
private:
    std::shared_ptr<map_interface::MapInterface> gridmapPtr_;
    std::shared_ptr<env::Env> envPtr_;
    std::shared_ptr<parameter_server::ParaeterSerer> paraPtr_;
    std::shared_ptr<prediction::Predict> prePtr_;
    std::shared_ptr<vis_interface::VisInterface> visPtr_;
    std::shared_ptr<ShareDataManager> dataManagerPtr_;
    std::shared_ptr<csv_writer::CsvWriter> csvWriterPtr_;

    // tracking plan duration
    double tracking_dur_;
    // sample dt on target trajectory
    double tracking_dt_;
    // tracking expected height, distance and relative angle
    double tracking_height_expect_, tracking_dis_expect_, tracking_angle_expect_;
    
    // landing pos offset
    Eigen::Vector3d land_dp_;
    double land_roll_, land_pitch_;
    
    // estimated time comsume of plan one time
    double plan_estimated_duration_;

    int web_vis_hz_;
    
    bool is_use_viewpoint_ = false;

    bool is_with_perception_;

public:
    TLPlanner(std::shared_ptr<parameter_server::ParaeterSerer>& para_ptr);
    ~TLPlanner(){}
    inline void set_gridmap_ptr(std::shared_ptr<map_interface::MapInterface>& gridmap_ptr){
        gridmapPtr_ = gridmap_ptr;
    }
    inline void set_env_ptr(std::shared_ptr<env::Env>& env_ptr){
        envPtr_ = env_ptr;
    }
    inline void set_trajopt_ptr(std::shared_ptr<traj_opt::TrajOpt>& trajopt_ptr){
        trajoptPtr_ = trajopt_ptr;
    }
    inline void set_pre_ptr(std::shared_ptr<prediction::Predict>& pre_ptr){
        prePtr_ = pre_ptr;
    }
    inline void set_vis_ptr(std::shared_ptr<vis_interface::VisInterface>& vis_ptr){
        visPtr_ = vis_ptr;
        // csvWriterPtr_->set_camvis_ptr(visPtr_->cam_vis_front_);
        csvWriterPtr_->set_camvis_ptr(visPtr_->cam_vis_down_);
    }
    inline void set_data_ptr(std::shared_ptr<ShareDataManager>& dataManagerPtr){
        dataManagerPtr_ = dataManagerPtr;
    }
    inline void set_track_dis(const double& dis){
        tracking_dis_expect_ = dis;
    }
    inline void set_track_height(const double& h){
        tracking_height_expect_ = h;
    }
    inline void set_track_angle(const double& angle){
        tracking_angle_expect_ = angle;
        tracking_angle_expect_ = rotation_util::RotUtil::rad_limit(tracking_angle_expect_);
    }

    PlanResState plan_goal(const Odom& init_state_in, const Odom& target_data, TrajData& traj_data);

    PlanResState plan_track(const Odom& init_state_in, const Odom& target_data, TrajData& traj_data);

    PlanResState plan_land(const Odom& init_state_in, const Odom& target_data, TrajData& traj_data, const double& t_replan = -1.0);

    bool valid_cheack(const TrajData& traj_data, const TimePoint& cur_t = TimeNow());
    void saveLandingData(const Odom& uav, const Odom& boat, bool success);
};
} // namespace tlplanner

#endif