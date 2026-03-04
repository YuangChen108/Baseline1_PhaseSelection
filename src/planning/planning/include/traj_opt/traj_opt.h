#pragma once

#include <chrono>
#include <thread>
#include <mutex>

#include "minco.hpp"
#include <traj_opt/flatness.hpp>
#include "util_gym/util_gym.hpp"
#include "rotation_util/rotation_util.hpp"
#include "parameter_server/parameter_server.hpp"
#include "map_interface/map_interface.h"
#include "visualization_interface/vis_interface.h"


namespace traj_opt {

class TrajOpt {
 public:
  std::shared_ptr<map_interface::MapInterface> gridmapPtr_;
  std::shared_ptr<vis_interface::VisInterface> visPtr_;
  bool pause_debug_ = false;
  // # pieces and # key points
  int N_, K_, dim_t_, dim_p_;
  // cost record
  std::atomic_flag cost_lock_ = ATOMIC_FLAG_INIT;
  double cost_t_rec_, cost_v_rec_, cost_a_rec_, cost_thrust_rec_, cost_omega_rec_, cost_perching_collision_rec_, cost_snap_rec_;
  double cost_perching_precep_rec_;
  double cost_tracking_dis_rec_, cost_tracking_ang_rec_, cost_tracking_vis_rec_, cost_collision_rec_;
  double cost_yaw_rec_, cost_dyaw_rec_;
  double deltaT_rec_;
  // weight for time regularization term
  double rhoT_track_, rhoT_land_;
  double rhoT_land_origin_;
  double rhoVt_, rhoAt_, rhoPt_;
  // collision avoiding and dynamics paramters
  double vmax_, amax_;
  double dv_max_horiz_track_, dv_max_horiz_land_, v_max_vert_track_, v_max_vert_land_;
  double dyaw_max_;
  double rhoP_, rhoV_, rhoA_;
  double rhoYaw_, rhoDyaw_;
  double rhoThrust_, rhoOmega_;
  double rhoPerchingCollision_;
  double rhoPerchingPreception_;
  // deform parameters
  double rhoDeformDis_, rhoDeformP_, rhoDeformVisibility_, rhoDeformAngle_;
  double rhoDeformConsistAngle_;
  Eigen::Vector3d cur_tar_, last_ego_p_, last_tar_p_;
  // tracking parameters
  double rhoTrackingDis_, rhoTrackingVisibility_, rhoTrackingAngle_;
  double tracking_dur_;
  double tracking_dist_, tolerance_tracking_d_;
  double track_angle_expect_;
  double tracking_dt_;
  std::vector<Eigen::Vector3d> tracking_ps_;
  std::vector<double> tracking_yaws_;
  // landing parameters
  double v_plus_, robot_l_, robot_r_, platform_r_;
  double preception_d_max_, preception_d_min_;
  bool with_perception_ = false;
  bool short_mode_with_perception_ = false;
  bool short_mode_ = false;
  double short_mode_time_;
  double land_z_min_, land_z_max_;
  double land_z_down_relative_, land_z_up_relative_;
  double eps_pz_;
  // camera parameters
  Eigen::Matrix3d cam2body_R_down_, cam2body_R_front_;
  Eigen::Vector3d cam2body_p_down_, cam2body_p_front_;
  double fx_down_, fy_down_, cx_down_, cy_down_;
  double fx_front_, fy_front_, cx_front_, cy_front_;
  // SE3 dynamic limitation parameters
  double thrust_max_, thrust_min_;
  double omega_max_, omega_yaw_max_;
  // flatness map
  flatness::FlatnessMap flatmap_;
  // MINCO Optimizer
  minco::MINCO_S4_Uniform minco_s4_opt_;
  minco::MINCO_S3 minco_s3_opt_;
  minco::MINCO_S2 minco_s2_yaw_opt_;
  minco::MINCO_S2_Uniform minco_s2u_yaw_opt_;
  minco::MINCO_S3_Uniform minco_s3u_opt_;
  Eigen::MatrixXd initS_, finalS_;
  Eigen::MatrixXd init_yaw_, final_yaw_;
  // weight for each vertex
  Eigen::MatrixXd p_;
  // duration of each piece of the trajectory
  Eigen::VectorXd t_;
  double* x_;
  double sum_T_;
  // opt value
  int max_iter_;
  TimePoint opt_start_t_;
  int iter_times_;

  int use_which_img_; // 0 for front, 1 for down

 private:
  void setBoundConds(const Eigen::MatrixXd& iniState,
                    const Eigen::MatrixXd& finState);


  bool grad_cost_v(const Eigen::Vector3d& v,
                   Eigen::Vector3d& gradv,
                   double& costv);
  bool grad_cost_v(const Eigen::Vector3d& v,
                   const double& v_max_horiz,
                   const double& v_max_vert,
                   Eigen::Vector3d& gradv,
                   double& costv);
                   
  bool grad_cost_a(const Eigen::Vector3d& a,
                   Eigen::Vector3d& grada,
                   double& costa);

  bool grad_cost_yaw(const double& yaw,
                    const Eigen::Vector3d& p,
                    const Eigen::Vector3d& target_p,
                    double& grad_yaw,
                    double& cost_yaw);

  bool grad_cost_dyaw(const double& dyaw,
                      double& grad_dyaw,
                      double& cost_dyaw);

  bool grad_cost_thrust(const Eigen::Vector3d& a,
                        Eigen::Vector3d& grada,
                        double& costa);
  bool grad_cost_omega(const Eigen::Vector3d& a,
                       const Eigen::Vector3d& j,
                       Eigen::Vector3d& grada,
                       Eigen::Vector3d& gradj,
                       double& cost);
  bool grad_cost_rate(const Eigen::Vector3d& omg,
                      Eigen::Vector3d& gradomg,
                      double& cost);
  bool grad_cost_omega_yaw(const Eigen::Vector3d& a,
                           const Eigen::Vector3d& j,
                           Eigen::Vector3d& grada,
                           Eigen::Vector3d& gradj,
                           double& cost);
  bool grad_cost_floor(const Eigen::Vector3d& p,
                       Eigen::Vector3d& gradp,
                       double& costp);
  bool grad_cost_ceil(const Eigen::Vector3d& p,
                      Eigen::Vector3d& gradp,
                      double& costp);

  bool grad_cost_perching_collision(const Eigen::Vector3d& pos,
                                    const Eigen::Vector3d& acc,
                                    const Eigen::Vector3d& car_p,
                                    const Eigen::Vector3d& car_zd,
                                    Eigen::Vector3d& gradp,
                                    Eigen::Vector3d& grada,
                                    Eigen::Vector3d& grad_car_p,
                                    Eigen::Vector3d& grad_car_zd,
                                    double& cost);
  bool grad_cost_preception(const Eigen::Vector3d& pos,
                            const Eigen::Vector3d& acc,
                            const Eigen::Vector3d& car_p,
                            Eigen::Vector3d& gradp,
                            Eigen::Vector3d& grada,
                            Eigen::Vector3d& grad_car_p,
                            double& cost);
  bool grad_cost_preception(const Eigen::Vector3d& pos,
                            const Eigen::Vector4d& q,
                            const Eigen::Vector3d& car_p,
                            Eigen::Vector3d& gradp,
                            Eigen::Vector4d& gradQuat,
                            Eigen::Vector3d& grad_car_p,
                            double& cost);
  bool grad_cost_tracking_p(const Eigen::Vector3d& p,
                            const Eigen::Vector3d& target_p,
                            Eigen::Vector3d& gradp,
                            double& costp);
                            
  bool grad_cost_tracking_angle(const Eigen::Vector3d& p,
                                const Eigen::Vector3d& target_p,
                                Eigen::Vector3d& gradp,
                                double& costp);

  bool grad_cost_tracking_visibility(const Eigen::Vector3d& p,
                                    const Eigen::Vector3d& target_p,
                                    Eigen::Vector3d& gradp,
                                    double& costp);

  bool grad_cost_deform_dis(const Eigen::Vector3d& p,
                            const Eigen::Vector3d& target_p,
                            Eigen::Vector3d& gradp,
                            double& costp);
                            
  bool grad_cost_deform_angle(const Eigen::Vector3d& p,
                              const Eigen::Vector3d& target_p,
                              Eigen::Vector3d& gradp,
                              double& costp);

  bool grad_cost_deform_visibility(const Eigen::Vector3d& p,
                                  const Eigen::Vector3d& target_p,
                                  Eigen::Vector3d& gradp,
                                  double& costp);

 public:
  TrajOpt(std::shared_ptr<parameter_server::ParaeterSerer>& paraPtr);
  ~TrajOpt() {}

  bool grad_cost_collision(const Eigen::Vector3d& p,
                           Eigen::Vector3d& gradp,
                           double& costp);

  inline void set_gridmap_ptr(std::shared_ptr<map_interface::MapInterface> gridmapPtr){
    gridmapPtr_ = gridmapPtr;
  }

  inline void set_vis_ptr(std::shared_ptr<vis_interface::VisInterface> visPtr){
    visPtr_ = visPtr;
    visPtr_->cam_vis_down_->set_para(cam2body_R_down_, cam2body_p_down_, fx_down_, fy_down_, cx_down_, cy_down_);
    visPtr_->cam_vis_front_->set_para(cam2body_R_front_, cam2body_p_front_, fx_front_, fy_front_, cx_front_, cy_front_);
    std::vector<Eigen::Vector3d> int_waypts;
    visPtr_->visualize_pointcloud(int_waypts, "int_waypts");
  }

  inline void set_track_dis(const double track_dis_expect){
    tracking_dist_ = track_dis_expect;
  }

  inline void set_track_angle(const double track_angle_expect){
    track_angle_expect_ = track_angle_expect;
  }

  inline void set_with_perception(const bool& is_with_perception){
    with_perception_ = is_with_perception;
  }

  inline void set_rho_T_land(const double& rhoT_land){
    rhoT_land_ = rhoT_land;
  }

  int optimize(const double& delta = 1e-4);
  
  void extract_mid_pts_from_apath(const std::vector<Eigen::Vector3d>& ego_path,
                                const double& seg_per_dis,
                                std::vector<Eigen::Vector3d>& mid_q_vec,
                                int& N_ret);

  void get_init_taj(const int& N,
                    const Eigen::MatrixXd& init_P_vec,
                    Eigen::VectorXd& init_T_vec);
  // goal traj generation
  bool generate_traj(const Eigen::MatrixXd& iniState,
                    const Eigen::MatrixXd& finState,
                    const double& seg_per_dis,
                    const std::vector<Eigen::Vector3d>& ego_path,
                    Trajectory<5>& traj);
  // goal-yaw traj generation
  bool generate_traj(const Eigen::MatrixXd& iniState,
                    const Eigen::MatrixXd& finState,
                    const Eigen::MatrixXd& init_yaw,
                    const Eigen::MatrixXd& final_yaw,
                    const double& seg_per_dis,
                    const std::vector<Eigen::Vector3d>& ego_path,
                    Trajectory<5>& traj);
  // landing traj generation
  bool generate_traj(const Eigen::MatrixXd& iniState,
                     const Eigen::MatrixXd& init_yaw,
                     const Eigen::Vector3d& car_p,
                     const Eigen::Vector3d& car_v,
                     const double& car_theta,
                     const double& car_omega,
                     const Eigen::Quaterniond& land_q,
                     const int& N,
                     Trajectory<7>& traj, 
                     const double& t_replan = -1.0);

  // landing-yaw traj generation
  bool generate_traj(const Eigen::MatrixXd& iniState,
                     const Eigen::Vector3d& car_p,
                     const Eigen::Vector3d& car_v,
                     const Eigen::Quaterniond& land_q,
                     const int& N,
                     Trajectory<7>& traj, 
                     const double& t_replan = -1.0);

  // tracking traj generation
  bool generate_traj(const Eigen::MatrixXd& iniState,
                    const Eigen::MatrixXd& finState,
                    const double& seg_per_dis,
                    const std::vector<Eigen::Vector3d>& ego_path,
                    const std::vector<Eigen::Vector3d>& target_predcit,
                    Trajectory<5>& traj);

  // tracking-yaw traj generation
  bool generate_traj(const Eigen::MatrixXd& iniState,
                    const Eigen::MatrixXd& finState,
                    const Eigen::MatrixXd& init_yaw,
                    const Eigen::MatrixXd& final_yaw,
                    const double& seg_per_dis,
                    const std::vector<Eigen::Vector3d>& ego_path,
                    const std::vector<Eigen::Vector3d>& viewpts,
                    const std::vector<Eigen::Vector3d>& target_predcit,
                    Trajectory<5>& traj);

  bool visible_path_deform(const Eigen::Vector3d& ego_p, 
                           const std::vector<Eigen::Vector3d>& target_predcit,
                           std::vector<Eigen::Vector3d>& viewpoint_path);
  
  bool generate_viewpoint(const Eigen::Vector3d& cur_tar,
                          const Eigen::Vector3d& last_ego_p,
                          const Eigen::Vector3d& last_tar_p,
                          Eigen::Vector3d& viewpoint);

  void addTimeIntPenaltyGoal(double& cost);
  void addTimeIntPenaltyTracking(double& cost);
  void addTimeIntPenaltyLanding(double& cost);

  void addTimeCostTracking(double& cost); 
  void addDeformCost(const Eigen::Vector3d& p,
                    const Eigen::Vector3d& target_p,
                    Eigen::Vector3d& gradp,
                    double& costp);

  Eigen::MatrixXd cal_timebase_jerk(const int order, const double& t);
  Eigen::MatrixXd cal_timebase_acc(const int order, const double& t);
  Eigen::MatrixXd cal_timebase_snap(const int order, const double& t);

  void getJacobian(const Eigen::Vector3d& p,
                  const Eigen::Quaterniond& q,
                  Eigen::MatrixXd& Jacobian);

  bool feasibleCheck(Trajectory<7>& traj);


  bool check_collilsion(const Eigen::Vector3d& pos,
                        const Eigen::Vector3d& acc,
                        const Eigen::Vector3d& car_p);


  inline void clear_cost_rec(){
    cost_snap_rec_ = 0.0;
    cost_v_rec_ = 0.0;
    cost_a_rec_ = 0.0;
    cost_thrust_rec_ = 0.0;
    cost_omega_rec_ = 0.0;
    cost_perching_collision_rec_ = 0.0;
    cost_perching_precep_rec_ = 0.0;
    cost_t_rec_ = 0.0;
    deltaT_rec_ = 0.0;
    cost_tracking_dis_rec_ = 0.0;
    cost_collision_rec_ = 0.0;
    cost_tracking_ang_rec_ = 0.0;
    cost_tracking_vis_rec_ = 0.0;
    cost_yaw_rec_ = 0.0;
    cost_dyaw_rec_ = 0.0;
  }
  inline double get_snap_cost(){
    return cost_snap_rec_;
  }
  inline double get_v_cost(){
    return cost_v_rec_;
  }
  inline double get_a_cost(){
    return cost_a_rec_;
  }
  inline double get_thrust_cost(){
    return cost_thrust_rec_;
  }
  inline double get_omega_cost(){
    return cost_omega_rec_;
  }
  inline double get_perching_collision_cost(){
    return cost_perching_collision_rec_;
  }
  inline double get_perching_precept_cost(){
    return cost_perching_precep_rec_;
  }
  inline double get_t_cost(){
    return cost_t_rec_;
  }
  inline double get_tracking_dis_cost(){
    return cost_tracking_dis_rec_;
  }
  inline double get_tracking_ang_cost(){
    return cost_tracking_ang_rec_;
  }
  inline double get_tracking_vis_cost(){
    return cost_tracking_vis_rec_;
  }
  inline double get_collision_cost(){
    return cost_collision_rec_;
  }
  inline double get_yaw_cost(){
    return cost_yaw_rec_;
  }
  inline double get_dyaw_cost(){
    return cost_dyaw_rec_;
  }
  inline void dashboard_cost_print(){  
    // return;
    if (cost_lock_.test_and_set()) return;
    INFO_MSG("-------------------[Traj Opt Cost]----------------");
    INFO_MSG("Snap: " << get_snap_cost());
    INFO_MSG("dT(" << deltaT_rec_ << "): " << get_t_cost());
    INFO_MSG("Vel: " << get_v_cost());
    INFO_MSG("Acc: " << get_a_cost());
    INFO_MSG("ESDF: " << get_collision_cost());
    INFO_MSG("Track Dis: " << get_tracking_dis_cost());
    INFO_MSG("Track Ang: " << get_tracking_ang_cost());
    INFO_MSG("Track Vis: " << get_tracking_vis_cost());
    INFO_MSG("Thrust: " << get_thrust_cost());
    INFO_MSG("Omega: " << get_omega_cost());
    INFO_MSG("Perching Collision: " << get_perching_collision_cost());
    INFO_MSG("Perching precept: " << get_perching_precept_cost());
    INFO_MSG("Yaw: " << get_yaw_cost());
    INFO_MSG("Dyaw: " << get_dyaw_cost());
    cost_lock_.clear();
  }
};

inline double objectiveFuncGoal(void* ptrObj,
                                const double* x,
                                double* grad,
                                const int n);
inline int earlyExitGoal(void* ptrObj,
                        const double* x,
                        const double* grad,
                        const double fx,
                        const double xnorm,
                        const double gnorm,
                        const double step,
                        int n,
                        int k,
                        int ls);

inline double objectiveFuncTracking(void* ptrObj,
                                    const double* x,
                                    double* grad,
                                    const int n);
inline int earlyExitTracking(void* ptrObj,
                            const double* x,
                            const double* grad,
                            const double fx,
                            const double xnorm,
                            const double gnorm,
                            const double step,
                            int n,
                            int k,
                            int ls);

inline double objectiveFuncLanding(void* ptrObj,
                                   const double* x,
                                   double* grad,
                                   const int n);
inline int earlyExitLanding(void* ptrObj,
                            const double* x,
                            const double* grad,
                            const double fx,
                            const double xnorm,
                            const double gnorm,
                            const double step,
                            int n,
                            int k,
                            int ls);

inline double objectiveFuncDeform(void* ptrObj,
                                  const double* x,
                                  double* grad,
                                  const int n);

inline int earlyExitDeform(void* ptrObj,
                          const double* x,
                          const double* grad,
                          const double fx,
                          const double xnorm,
                          const double gnorm,
                          const double step,
                          int n,
                          int k,
                          int ls);

double expC2(double t);
double logC2(double T);
double smoothedL1(const double& x, double& grad);
double smoothed01(const double& x, double& grad, const double mu = 0.01);
double penF(const double& x, double& grad);
double penF2(const double& x, double& grad);
double gdT2t(double t);
void forwardT(const double& t, double& T);
void backwardT(const double& T, double& t);
void forwardT_sum(const Eigen::Ref<const Eigen::VectorXd>& t, const double& sT, Eigen::Ref<Eigen::VectorXd> vecT);
void backwardT_sum(const Eigen::Ref<const Eigen::VectorXd>& vecT, Eigen::Ref<Eigen::VectorXd> t);
void forwardT(const Eigen::Ref<const Eigen::VectorXd>& t, Eigen::Ref<Eigen::VectorXd> vecT);
void backwardT(const Eigen::Ref<const Eigen::VectorXd>& vecT, Eigen::Ref<Eigen::VectorXd> t);
void addLayerTGrad(const Eigen::Ref<const Eigen::VectorXd>& t,
                    const Eigen::Ref<const Eigen::VectorXd>& gradT,
                    Eigen::Ref<Eigen::VectorXd> gradt);
void addLayerTGrad(const Eigen::Ref<const Eigen::VectorXd>& t,
                  const double& sT,
                  const Eigen::Ref<const Eigen::VectorXd>& gradT,
                  Eigen::Ref<Eigen::VectorXd> gradt);

Trajectory<5> getS3TrajWithYaw(minco::MINCO_S3& mincos3_opt, minco::MINCO_S2& mincoyaw_opt);
Trajectory<7> getS4TrajWithYaw(minco::MINCO_S4_Uniform& mincos4u_opt, minco::MINCO_S2& mincoyaw_opt);
Trajectory<7> getS4TrajWithYaw(minco::MINCO_S4_Uniform& mincos4u_opt, minco::MINCO_S2_Uniform& mincoyaw_u_opt);
}  // namespace traj_opt