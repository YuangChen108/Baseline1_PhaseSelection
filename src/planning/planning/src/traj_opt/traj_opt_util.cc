#include <traj_opt/traj_opt.h>
#include <traj_opt/lbfgs_raw.hpp>

namespace traj_opt {

using rot_util = rotation_util::RotUtil;

TrajOpt::TrajOpt(std::shared_ptr<parameter_server::ParaeterSerer>& paraPtr) {
  paraPtr->get_para("K", K_);
  paraPtr->get_para("max_iter", max_iter_);

  paraPtr->get_para("vmax", vmax_);

  paraPtr->get_para("dv_max_horiz_track", dv_max_horiz_track_);
  paraPtr->get_para("dv_max_horiz_land", dv_max_horiz_land_);
  paraPtr->get_para("v_max_vert_track", v_max_vert_track_);
  paraPtr->get_para("v_max_vert_land", v_max_vert_land_);

  paraPtr->get_para("amax", amax_);
  paraPtr->get_para("dyaw_max", dyaw_max_);
  paraPtr->get_para("thrust_max", thrust_max_);
  paraPtr->get_para("thrust_min", thrust_min_);
  paraPtr->get_para("omega_max", omega_max_);
  paraPtr->get_para("omega_yaw_max", omega_yaw_max_);
  paraPtr->get_para("v_plus", v_plus_);
  paraPtr->get_para("robot_l", robot_l_);
  paraPtr->get_para("robot_r", robot_r_);
  paraPtr->get_para("platform_r", platform_r_);
  paraPtr->get_para("preception_d_max", preception_d_max_);
  paraPtr->get_para("preception_d_min", preception_d_min_);
  paraPtr->get_para("land_z_down_relative", land_z_down_relative_);
  paraPtr->get_para("land_z_up_relative", land_z_up_relative_);
  paraPtr->get_para("short_mode_with_perception", short_mode_with_perception_);

  paraPtr->get_para("rhoT_track", rhoT_track_);
  paraPtr->get_para("rhoT_land", rhoT_land_origin_);
  rhoT_land_ = rhoT_land_origin_;
  paraPtr->get_para("rhoVt", rhoVt_);
  paraPtr->get_para("rhoAt", rhoAt_);
  paraPtr->get_para("rhoPt", rhoPt_);

  paraPtr->get_para("rhoP", rhoP_);
  paraPtr->get_para("rhoV", rhoV_);
  paraPtr->get_para("rhoA", rhoA_);
  paraPtr->get_para("rhoThrust", rhoThrust_);
  paraPtr->get_para("rhoOmega", rhoOmega_);
  paraPtr->get_para("rhoPerchingCollision", rhoPerchingCollision_);
  paraPtr->get_para("rhoPerchingPreception", rhoPerchingPreception_);

  paraPtr->get_para("rhoYaw", rhoYaw_);
  paraPtr->get_para("rhoDyaw", rhoDyaw_);

  paraPtr->get_para("rhoDeformConsistAngle", rhoDeformConsistAngle_);
  paraPtr->get_para("rhoDeformDis", rhoDeformDis_);
  paraPtr->get_para("rhoDeformP", rhoDeformP_);
  paraPtr->get_para("rhoDeformVisibility", rhoDeformVisibility_);
  paraPtr->get_para("rhoDeformAngle", rhoDeformAngle_);

  paraPtr->get_para("rhoTrackingDis", rhoTrackingDis_);
  paraPtr->get_para("rhoTrackingAngle", rhoTrackingAngle_);
  paraPtr->get_para("rhoTrackingVisibility", rhoTrackingVisibility_);
  paraPtr->get_para("tracking_dur", tracking_dur_);
  paraPtr->get_para("tracking_dis_expect", tracking_dist_);
  paraPtr->get_para("tolerance_tracking_d", tolerance_tracking_d_);
  paraPtr->get_para("tracking_dt", tracking_dt_);

  paraPtr->get_para("pause_debug", pause_debug_);
  paraPtr->get_para("short_mode_time", short_mode_time_);

  paraPtr->get_para("eps_pz", eps_pz_);

  paraPtr->get_para("use_which_img", use_which_img_);

  // note that the orientation definition in hopf fibration is different from real drone's NED frame
  // we choose to plan in the hopf-fibration-definition frame, and change the cmd to NED frame when publish control command
  // in fact, the cam2bodyR didnot affect the gradient, beacause it is just the hypothetical model
  // cam2body_R_ << 0.0, -1.0, 0.0,
  //                -1.0, 0.0, 0.0,
  //                0.0, 0.0, -1.0;

  cam2body_R_down_ << 1.0,  0.0, 0.0,
                      0.0, -1.0, 0.0,
                      0.0, 0.0, -1.0;

  cam2body_p_down_ << 0.0, 0.0, 0.0;
  fx_down_ = 320;
  fy_down_ = 320;
  cx_down_ = 320;
  cy_down_ = 240;

  cam2body_R_front_ << 0.0,  0.0,  1.0,
                      -1.0,  0.0,  0.0,
                       0.0, -1.0,  0.0;

  cam2body_p_front_ << 0.0, 0.0, 0.0;
  fx_front_ = 240;
  fy_front_ = 240;
  cx_front_ = 240;
  cy_front_ = 240;

}

double expC2(double t) {
  return t > 0.0 ? ((0.5 * t + 1.0) * t + 1.0)
                 : 1.0 / ((0.5 * t - 1.0) * t + 1.0);
}

double logC2(double T) {
  return T > 1.0 ? (sqrt(2.0 * T - 1.0) - 1.0) : (1.0 - sqrt(2.0 / T - 1.0));
}

double smoothedL1(const double& x,
                         double& grad) {
  static double mu = 0.01;
  if (x < 0.0) {
    return 0.0;
  } else if (x > mu) {
    grad = 1.0;
    return x - 0.5 * mu;
  } else {
    const double xdmu = x / mu;
    const double sqrxdmu = xdmu * xdmu;
    const double mumxd2 = mu - 0.5 * x;
    grad = sqrxdmu * ((-0.5) * xdmu + 3.0 * mumxd2 / mu);
    return mumxd2 * sqrxdmu * xdmu;
  }
}

double smoothed01(const double& x,
                         double& grad, const double mu) {
  static double mu4 = mu * mu * mu * mu;
  static double mu4_1 = 1.0 / mu4;
  if (x < -mu) {
    grad = 0;
    return 0;
  } else if (x < 0) {
    double y = x + mu;
    double y2 = y * y;
    grad = y2 * (mu - 2 * x) * mu4_1;
    return 0.5 * y2 * y * (mu - x) * mu4_1;
  } else if (x < mu) {
    double y = x - mu;
    double y2 = y * y;
    grad = y2 * (mu + 2 * x) * mu4_1;
    return 0.5 * y2 * y * (mu + x) * mu4_1 + 1;
  } else {
    grad = 0;
    return 1;
  }
}

double penF(const double& x, double& grad) {
  static double eps = 0.05;
  static double eps2 = eps * eps;
  static double eps3 = eps * eps2;
  if (x < 2 * eps) {
    double x2 = x * x;
    double x3 = x * x2;
    double x4 = x2 * x2;
    grad = 12 / eps2 * x2 - 4 / eps3 * x3;
    return 4 / eps2 * x3 - x4 / eps3;
  } else {
    grad = 16;
    return 16 * (x - eps);
  }
}

double penF2(const double& x, double& grad) {
  double x2 = x * x;
  grad = 3 * x2;
  return x * x2;
}

/*
* function getJacobian
* for a quaternion rotation: r = q*p*q^(-1)
* returns dr_dq
*/
void TrajOpt::getJacobian(const Eigen::Vector3d& p,
                          const Eigen::Quaterniond& q,
                          Eigen::MatrixXd& Jacobian)
{
    Jacobian.resize(4,3);
    Jacobian.row(0) << p(0)*q.w() + p(2)*q.y() - p(1)*q.z(), p(1)*q.w() + p(0)*q.z() - p(2)*q.x(), p(1)*q.x() - p(0)*q.y() + p(2)*q.w();
    Jacobian.row(1) << p(0)*q.x() + p(1)*q.y() + p(2)*q.z(), p(0)*q.y() - p(1)*q.x() - p(2)*q.w(), p(1)*q.w() + p(0)*q.z() - p(2)*q.x();
    Jacobian.row(2) << p(1)*q.x() - p(0)*q.y() + p(2)*q.w(), p(0)*q.x() + p(1)*q.y() + p(2)*q.z(), p(1)*q.z() - p(0)*q.w() - p(2)*q.y();
    Jacobian.row(3) << p(2)*q.x() - p(0)*q.z() - p(1)*q.w(), p(0)*q.w() - p(1)*q.z() + p(2)*q.y(), p(0)*q.x() + p(1)*q.y() + p(2)*q.z();
    Jacobian = 2*Jacobian;
}

Eigen::MatrixXd TrajOpt::cal_timebase_acc(const int order, const double& t){
  double s1, s2, s3;
  s1 = t;
  s2 = s1 * s1;
  s3 = s2 * s1;
  Eigen::Matrix<double, 4, 1> beta;
  switch(order){
    case 0:
      beta << 1.0, s1, s2, s3;
      break;
    case 1:
      beta << 0.0, 1.0, 2.0*s1, 3.0*s2;
      break;
    case 2:
      beta << 0.0, 0.0, 2.0, 6.0 * s1;
      break;
    default:
      std::cout << "[trajopt] cal_timebase error." << std::endl;
      break;
  }
  return beta;
}

Eigen::MatrixXd TrajOpt::cal_timebase_jerk(const int order, const double& t){
  double s1, s2, s3, s4, s5;
  s1 = t;
  s2 = s1 * s1;
  s3 = s2 * s1;
  s4 = s2 * s2;
  s5 = s4 * s1;
  Eigen::Matrix<double, 6, 1> beta;
  switch(order){
    case 0:
      beta << 1.0, s1, s2, s3, s4, s5;
      break;
    case 1:
      beta << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
      break;
    case 2:
      beta << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3;
      break;
    case 3:
      beta << 0.0, 0.0, 0.0, 6.0, 24.0 * s1, 60.0 * s2;
      break;
    default:
      std::cout << "[trajopt] cal_timebase error." << std::endl;
      break;
  }
  return beta;
}

Eigen::MatrixXd TrajOpt::cal_timebase_snap(const int order, const double& t){
  double s1, s2, s3, s4, s5, s6, s7;
  s1 = t;
  s2 = s1 * s1;
  s3 = s2 * s1;
  s4 = s2 * s2;
  s5 = s4 * s1;
  s6 = s4 * s2;
  s7 = s4 * s3;
  Eigen::Matrix<double, 8, 1> beta;
  switch(order){
    case 0:
      beta << 1.0, s1, s2, s3, s4, s5, s6, s7;
      break;
    case 1:
      beta << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4, 6.0 * s5, 7.0 * s6;
      break;
    case 2:
      beta << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3, 30.0 * s4, 42.0 * s5;
      break;
    case 3:
      beta << 0.0, 0.0, 0.0, 6.0, 24.0 * s1, 60.0 * s2, 120.0 * s3, 210.0 * s4;
      break;
    case 4:
      beta << 0.0, 0.0, 0.0, 0.0, 24.0, 120.0 * s1, 360.0 * s2, 840.0 * s3;
      break;
    default:
      std::cout << "[trajopt] cal_timebase error." << std::endl;
      break;
  }
  return beta;
}

void forwardT_sum(const Eigen::Ref<const Eigen::VectorXd>& t, const double& sT, Eigen::Ref<Eigen::VectorXd> vecT) {
  int M = t.size();
  for (int i = 0; i < M; ++i) {
    vecT(i) = expC2(t(i));
  }
  vecT(M) = 0.0;
  vecT /= 1.0 + vecT.sum();
  vecT(M) = 1.0 - vecT.sum();
  vecT *= sT;
  return;
}
void backwardT_sum(const Eigen::Ref<const Eigen::VectorXd>& vecT, Eigen::Ref<Eigen::VectorXd> t) {
  int M = t.size();
  t = vecT.head(M) / vecT(M);
  for (int i = 0; i < M; ++i) {
    t(i) = logC2(vecT(i));
  }
  return;
}
void forwardT(const Eigen::Ref<const Eigen::VectorXd>& t, Eigen::Ref<Eigen::VectorXd> vecT) {
  int M = t.size();
  for (int i = 0; i < M; ++i) {
    vecT(i) = expC2(t(i));
  }
  return;
}
void backwardT(const Eigen::Ref<const Eigen::VectorXd>& vecT, Eigen::Ref<Eigen::VectorXd> t) {
  int M = vecT.size();
  for (int i = 0; i < M; ++i) {
    t(i) = logC2(vecT(i));
  }
  return;
}
void forwardT(const double& t, double& T){
  T = expC2(t);
}
void backwardT(const double& T, double& t){
  t = logC2(T);
}

double gdT2t(double t) {
  if (t > 0) {
    return t + 1.0;
  } else {
    double denSqrt = (0.5 * t - 1.0) * t + 1.0;
    return (1.0 - t) / (denSqrt * denSqrt);
  }
}

void addLayerTGrad(const Eigen::Ref<const Eigen::VectorXd>& t,
                    const Eigen::Ref<const Eigen::VectorXd>& gradT,
                    Eigen::Ref<Eigen::VectorXd> gradt) {
  int M = t.size();
  for (int i = 0; i < M; ++i) {
    gradt(i) = gradT(i) * gdT2t(t(i));
  }
  return;
}

void addLayerTGrad(const Eigen::Ref<const Eigen::VectorXd>& t,
                  const double& sT,
                  const Eigen::Ref<const Eigen::VectorXd>& gradT,
                  Eigen::Ref<Eigen::VectorXd> gradt) {
  int Ms1 = t.size();
  Eigen::VectorXd gFree = sT * gradT.head(Ms1);
  double gTail = sT * gradT(Ms1);
  Eigen::VectorXd dExpTau(Ms1);
  double expTauSum = 0.0, gFreeDotExpTau = 0.0;
  double denSqrt, expTau;
  for (int i = 0; i < Ms1; i++) {
    if (t(i) > 0) {
      expTau = (0.5 * t(i) + 1.0) * t(i) + 1.0;
      dExpTau(i) = t(i) + 1.0;
      expTauSum += expTau;
      gFreeDotExpTau += expTau * gFree(i);
    } else {
      denSqrt = (0.5 * t(i) - 1.0) * t(i) + 1.0;
      expTau = 1.0 / denSqrt;
      dExpTau(i) = (1.0 - t(i)) / (denSqrt * denSqrt);
      expTauSum += expTau;
      gFreeDotExpTau += expTau * gFree(i);
    }
  }
  denSqrt = expTauSum + 1.0;
  gradt = (gFree.array() - gTail) * dExpTau.array() / denSqrt -
          (gFreeDotExpTau - gTail * expTauSum) * dExpTau.array() / (denSqrt * denSqrt);
}

void TrajOpt::extract_mid_pts_from_apath(const std::vector<Eigen::Vector3d>& ego_path,
                                        const double& seg_per_dis,
                                        std::vector<Eigen::Vector3d>& mid_q_vec,
                                        int& N_ret){
  int N = 0;
  double dis_sum = 0.0, dis_sum_cur = 0.0;
  std::vector<Eigen::Vector3d> mid_Q_vec;
  Eigen::Vector3d last_p = ego_path.front();
  Eigen::Vector3d last_mid_p = ego_path.front();
  mid_Q_vec.push_back(ego_path.front());
  for (size_t i = 0; i < ego_path.size(); i++){
    Eigen::Vector3d p = ego_path[i];
    if (!gridmapPtr_->checkRayValid(last_mid_p, p)){
      mid_Q_vec.push_back(last_p);
      INFO_MSG("push midp: " << last_p.transpose());
      last_mid_p = last_p;
    }
    last_p = p;
  }
  mid_Q_vec.push_back(ego_path.back());

  for (size_t i = 0; i < mid_Q_vec.size()-1; i++){
    Eigen::Vector3d ps = mid_Q_vec[i];
    Eigen::Vector3d pt = mid_Q_vec[i+1];
    if (i != 0) mid_q_vec.push_back(ps);
    int n = round((ps - pt).norm() / seg_per_dis);
    if (n > 1){
      Eigen::Vector3d dir = (pt - ps).normalized();
      double dis_thres = (ps - pt).norm() / n;
      for (int k = 1; k < n; k++){
        Eigen::Vector3d pk = ps + dir * dis_thres * k;
        mid_q_vec.push_back(pk);
      }
    }
  }

  N = mid_q_vec.size() + 1;


  // for (size_t i; i < path_s.size(); i++){
  //   int n = round(dis_s[i] / seg_per_dis);
  //   if (n > 1){
  //     double dis_thres = dis_s[i] / n;
  //     last_mid_p = path_s[i].front();
  //     dis_sum_cur = 0.0;
  //     for (auto p:path_s[i]){
  //       if ((p - last_mid_p).norm() > dis_thres){
  //         mid_q_vec.push_back(p);
  //         last_mid_p = p;
  //       }
  //     }
  //   }

  // }


  if (N <= 1){
    N_ret = 2;
    dis_sum = (ego_path.front() - ego_path.back()).norm();
    dis_sum_cur = 0.0;
    last_p = ego_path.front();
    for (auto p:ego_path){
      dis_sum_cur += (p - last_p).norm();
      last_p = p;
      if (dis_sum_cur >= dis_sum/2.0){
        mid_q_vec.push_back(p);
        INFO_MSG("push midp: " << p.transpose());
        break;
      }
    }
  }else{
    N_ret = N;
  }
}

void TrajOpt::get_init_taj(const int& N,
                           const Eigen::MatrixXd& init_P_vec,
                           Eigen::VectorXd& init_T_vec){
  minco_s3_opt_.reset(initS_.block<3,3>(0,0), finalS_.block<3,3>(0,0), N);
  double desire_vel = vmax_;
  double max_vel, max_acc;
  int cnt = 0;
  do{
    INFO_MSG("1111");
    for (size_t i = 0; i < N; i++){
      if (i == 0){
        init_T_vec(i) = (init_P_vec.col(i) - initS_.col(0)).norm() / desire_vel;
      }else if (i == N-1){
        init_T_vec(i) = (finalS_.col(0) - init_P_vec.col(i-1)).norm() / desire_vel;
      }else{
        init_T_vec(i) = (init_P_vec.col(i) - init_P_vec.col(i-1)).norm() / desire_vel;
      }
    }
    INFO_MSG("desire_vel: " << desire_vel);
    INFO_MSG("init_T_vec: " << init_T_vec.transpose());

    minco_s3_opt_.generate(init_P_vec, init_T_vec);
    desire_vel /= 1.5;
    INFO_MSG("generate done");
    max_vel = minco_s3_opt_.getTraj().getMaxVelRate();
    max_acc = minco_s3_opt_.getTraj().getMaxAccRate();
    cnt++;
    INFO_MSG("max_vel: " << max_vel<<", max_acc: " << max_acc);

  }while(cnt < 3 && (max_vel > vmax_ || max_acc > amax_));
}

bool TrajOpt::grad_cost_collision(const Eigen::Vector3d& p,
                                  Eigen::Vector3d& gradp,
                                  double& costp){
  costp = 0.0;
  gradp.setZero();
  double dist_threshold = 0.8;
  double          dist = 0;
  Eigen::Vector3d dist_grad;
  dist_grad.setZero();
  dist = gridmapPtr_->getCostWithGrad(p, dist_grad);
  if (dist < dist_threshold){
    double pen =  dist_threshold - dist;
    costp += pen * pen;
    
    gradp += -2.0 * pen * dist_grad;
    // INFO_MSG("p: "<<p.transpose()<<", esdf: "<< dist <<", pen: "<<pen<<", costp: "<<costp<<", grad: "<<gradp.transpose());
  }
  
  gradp *= rhoP_;
  costp *= rhoP_;  
  return true;
}

bool TrajOpt::grad_cost_v(const Eigen::Vector3d& v,
                          Eigen::Vector3d& gradv,
                          double& costv) {
  gradv.setZero();
  costv = 0;
  double vpen = v.squaredNorm() - vmax_ * vmax_;

  if (vpen > 0) {
    // INFO_MSG("v: " << v);
    // INFO_MSG("vmax: " << vmax_);
    double grad = 0;
    costv = smoothedL1(vpen, grad);
    gradv = rhoV_ * grad * 2 * v;
    costv *= rhoV_;
    return true;
  }
  return false;


  // double vpen = v.squaredNorm() - vmax_ * vmax_;
  // if (vpen > 0) {
  //   gradv = rhoV_ * 6 * vpen * vpen * v;
  //   costv = rhoV_ * vpen * vpen * vpen;
  //   return true;
  // }
  // return false;

}

bool TrajOpt::grad_cost_v(const Eigen::Vector3d& v,
                          const double& v_max_horiz,
                          const double& v_max_vert,
                          Eigen::Vector3d& gradv,
                          double& costv) {
  gradv.setZero();
  costv = 0;
  bool ret = false;
  double v_max_horiz_use = std::min(vmax_, v_max_horiz);
  double vpen = v.head(2).squaredNorm() - v_max_horiz_use * v_max_horiz_use;
  if (vpen > 0) {
    // double grad = 0;
    // costv += smoothedL1(vpen, grad);
    // gradv.head(2) = grad * 2 * v.head(2);

    costv += vpen * vpen * vpen;
    gradv.head(2) = 6 * vpen * vpen * v.head(2);
    
    ret = true;
  }

  vpen = v.z() * v.z() - v_max_vert * v_max_vert;
  if (vpen > 0){
    // double grad = 0;
    // costv += smoothedL1(vpen, grad);
    // gradv.z() = grad * 2 * v.z();

    costv += vpen * vpen * vpen;
    gradv.z() = 6 * vpen * vpen * v.z();

    ret = true;
  }

  costv *= rhoV_;
  gradv *= rhoV_;

  return ret;
}

bool TrajOpt::grad_cost_a(const Eigen::Vector3d& a,
                          Eigen::Vector3d& grada,
                          double& costa) {
  grada.setZero();
  costa = 0;
  double apen = a.squaredNorm() - amax_ * amax_;

  if (apen > 0) {
    // INFO_MSG("a: " << a);
    // INFO_MSG("amax: " << amax_);
    double grad = 0;
    costa = smoothedL1(apen, grad);
    grada = rhoA_ * grad * 2 * a;
    costa *= rhoA_;
    return true;
  }
  return false;

  // double apen = a.squaredNorm() - amax_ * amax_;
  // if (apen > 0) {
  //   grada = rhoA_ * 6 * apen * apen * a;
  //   costa = rhoA_ * apen * apen * apen;
  //   return true;
  // }
  // return false;
}

bool TrajOpt::grad_cost_dyaw(const double& dyaw,
                             double& grad_dyaw,
                             double& cost_dyaw){
  grad_dyaw = 0;
  cost_dyaw = 0;
  double vpen = dyaw * dyaw - dyaw_max_ * dyaw_max_;

  if (vpen > 0) {
    // INFO_MSG("v: " << v);
    // INFO_MSG("vmax: " << vmax_);
    double grad = 0;
    cost_dyaw = smoothedL1(vpen, grad);
    grad_dyaw = rhoDyaw_ * grad * 2 * dyaw;
    cost_dyaw *= rhoDyaw_;
    return true;
  }
  return false;
}

bool TrajOpt::grad_cost_yaw(const double& yaw,
                            const Eigen::Vector3d& p,
                            const Eigen::Vector3d& target_p,
                            double& grad_yaw,
                            double& cost_yaw){
  grad_yaw = 0;
  cost_yaw = 0;
  Eigen::Vector3d dp = target_p - p;
  double exp_yaw = atan2(dp.y(), dp.x());
  double pen = -rot_util::error_angle(yaw, exp_yaw);

  // INFO_MSG_GREEN("p: "<<p.transpose()<<", tar: "<<target_p.transpose());
  std::string sgn;
  cost_yaw = rhoYaw_ * pen * pen;
  grad_yaw = rhoYaw_ * 2 * pen;
  if (grad_yaw >= 0){
    sgn = "-";
  }else{
    sgn = "+";
  }
  // INFO_MSG("("<<sgn << ")yaw: " << yaw << ", exp_yaw: " << exp_yaw << ", error_angle: " << pen);

  return true;
}

Trajectory<5> getS3TrajWithYaw(minco::MINCO_S3& mincos3_opt, minco::MINCO_S2& mincoyaw_opt){
  Trajectory<5> traj;
  traj.clear();
  traj.reserve(mincos3_opt.N);
  for (int i = 0; i < mincos3_opt.N; i++)
  {
      traj.emplace_back(mincos3_opt.T1(i),
                        mincos3_opt.b.block<6, 3>(6 * i, 0).transpose().rowwise().reverse(),
                        mincoyaw_opt.b.block<4, 1>(4 * i, 0).transpose().rowwise().reverse());
  }
  return traj;
}

Trajectory<7> getS4TrajWithYaw(minco::MINCO_S4_Uniform& mincos4u_opt, minco::MINCO_S2& mincoyaw_opt){
  Trajectory<7> traj;
  traj.clear();
  traj.reserve(mincos4u_opt.N);
  for (int i = 0; i < mincos4u_opt.N; i++)
  {
      traj.emplace_back(mincos4u_opt.t(1),
                        mincos4u_opt.c.block<8, 3>(8 * i, 0).transpose().rowwise().reverse(),
                        mincoyaw_opt.b.block<4, 1>(4 * i, 0).transpose().rowwise().reverse());
  }
  return traj;
}

Trajectory<7> getS4TrajWithYaw(minco::MINCO_S4_Uniform& mincos4u_opt, minco::MINCO_S2_Uniform& mincoyaw_u_opt){
  Trajectory<7> traj;
  traj.clear();
  traj.reserve(mincos4u_opt.N);
  for (int i = 0; i < mincos4u_opt.N; i++)
  {
      traj.emplace_back(mincos4u_opt.t(1),
                        mincos4u_opt.c.block<8, 3>(8 * i, 0).transpose().rowwise().reverse(),
                        mincoyaw_u_opt.c.block<4, 1>(4 * i, 0).transpose().rowwise().reverse());
  }
  return traj;
}
} // namespace traj_opts