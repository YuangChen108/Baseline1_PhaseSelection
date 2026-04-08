#pragma once

#include <Eigen/Core>
#include <queue>
#include <thread>
#include <cmath>
#include <iostream>

#include "util_gym/util_gym.hpp"
#include "parameter_server/parameter_server.hpp"
#include "map_interface/map_interface.h"

namespace prediction {

// ==================== State 结构体 ====================
class State {
 public:
  State(){}
  State(const Eigen::Vector3d& p,
        const double& v,
        const double& a,
        const double& theta,
        const double& omega,
        const double& vz = 0.0): p_(p), v_(v), a_(a), theta_(theta), omega_(omega), vz_(vz){}

  Eigen::Vector3d p_;
  double v_;      
  double a_;      
  double theta_;  
  double omega_;  
  double vz_;     // 垂直速度
};

struct Node {
  State state_;
  double t_;
  double score_;
  double h_;
  Node* parent_ = nullptr;
};
typedef Node* NodePtr;

class NodeComparator {
 public:
  bool operator()(NodePtr& lhs, NodePtr& rhs) {
    return lhs->score_ + lhs->h_ > rhs->score_ + rhs->h_;
  }
};

struct Predict {
 private:
  static constexpr int MAX_MEMORY = 1000;
  
  double dt;
  double rho_a, rho_domega;
  double vmax, omega_max, amax;
  double init_omega, init_acc;
  
  std::shared_ptr<map_interface::MapInterface> gridmapPtr_;
  std::shared_ptr<vis_interface::VisInterface> visPtr_;
  NodePtr data[MAX_MEMORY];
  int stack_top;
  bool is_debug_ = false;

  inline bool isValid(const State& s, const State& t) const {
    if (std::abs(t.v_) > vmax || std::abs(t.omega_) > omega_max){
      return false;
    }
    if (gridmapPtr_ && (!gridmapPtr_->checkRayValid(s.p_, t.p_) || gridmapPtr_->isOccupied(t.p_))){
      return false;
    }
    return true;
  }

 public:
  // 🚀 核心改动 1：使用静态引用函数代替 static inline 变量 (兼容 C++11)
  // 这种写法确保了无论多少个文件引用此头文件，g_wave_freq 永远指向同一个地址
  static double& wave_freq_ref() {
      static double val = 1.0;
      return val;
  }
  static double& z_mean_ref() {
      static double val = 0.0;
      return val;
  }

  // 构造函数
  inline Predict(std::shared_ptr<parameter_server::ParaeterSerer>& paraPtr) {
    paraPtr->get_para("tracking_dt", dt);
    paraPtr->get_para("prediction/rho_a", rho_a);
    paraPtr->get_para("prediction/rho_domega", rho_domega);
    paraPtr->get_para("prediction/vmax", vmax);
    paraPtr->get_para("prediction/acc_max", amax);
    paraPtr->get_para("prediction/omega_max", omega_max);
    paraPtr->get_para("prediction/pause_debug", is_debug_);
    
    for (int i = 0; i < MAX_MEMORY; ++i) {
      data[i] = new Node;
    }

    // 🚀 核心改动 2：修复 get_para 返回 void 的报错
    // 既然不能放在 if 里，我们先设置默认值，读取后再打印出来检验
    wave_freq_ref() = 1.0; 
    paraPtr->get_para("prediction/wave_freq", wave_freq_ref());

    z_mean_ref() = 0.0;
    paraPtr->get_para("prediction/z_mean", z_mean_ref());

    // 打印结果以便排查
    std::cout << "\033[32m[Prediction] Current g_wave_freq: " << wave_freq_ref() << " Hz\033[0m" << std::endl;
  }

  inline void set_gridmap_ptr(std::shared_ptr<map_interface::MapInterface>& gridmap_ptr){
      gridmapPtr_ = gridmap_ptr;
  }
  inline void set_vis_ptr(std::shared_ptr<vis_interface::VisInterface> visPtr){
    visPtr_ = visPtr;
  }

  // predict 函数
  inline bool predict(const Eigen::Vector3d& target_p,
                      const double& target_v, 
                      const double& target_a,
                      const double& target_theta,
                      const double& target_omega, 
                      std::vector<Eigen::Vector3d>& target_predcit, 
                      const double& pre_dur,
                      const double& max_time = 0.1) {
    
    State state_start(target_p, target_v, 0.0, target_theta, target_omega, 0.0); 

    if (state_start.omega_ > omega_max) state_start.omega_ = omega_max;
    else if (state_start.omega_ < -omega_max) state_start.omega_ = -omega_max;
    
    if (state_start.a_ > amax) state_start.a_ = amax;
    else if (state_start.a_ < -amax) state_start.a_ = -amax;

    init_omega = state_start.omega_;
    init_acc = state_start.a_;

    auto score_ = [&](const NodePtr& ptr) -> double {
      return rho_a * std::abs(ptr->state_.a_ - init_acc) + rho_domega * std::abs(ptr->state_.omega_ - init_omega); 
    };

    State state_end;
    CYRA_model(state_start, pre_dur, state_end); 

    auto calH = [&](const NodePtr& ptr) -> double {
      return 0.001 * (ptr->state_.p_ - state_end.p_).norm();
    };

    TimePoint t_start = TimeNow(); 
    
    std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator> open_set;

    double input_a, input_domega;

    stack_top = 0;
    NodePtr curPtr = data[stack_top++];
    curPtr->state_ = state_start;
    curPtr->parent_ = nullptr;
    curPtr->score_ = 0;
    curPtr->h_ = 0;
    curPtr->t_ = 0;

    while (curPtr->t_ < pre_dur) {
      for (input_a = -3; input_a <= 3; input_a += 3){
        for (input_domega = -0.4; input_domega <= 0.4; input_domega += 0.4){
          State state_change_input;
          State state_now;
          state_change_input = curPtr->state_;
          state_change_input.a_ = init_acc + input_a;
          state_change_input.omega_ = init_omega + input_domega;
          
          CYRA_model(state_change_input, dt, state_now);
          
          if (!isValid(curPtr->state_, state_now)) {
            continue;
          }
          if (stack_top == MAX_MEMORY) {
            std::cout << "[prediction] out of memory!" << std::endl;
            return false;
          }
          
          double t_cost = durationSecond(TimeNow(), t_start);
          
          if (!is_debug_ && t_cost > max_time) {
            std::cout << "[prediction] too slow!" << std::endl;
            return false;
          }
          
          NodePtr ptr = data[stack_top++];
          ptr->state_ = state_now;
          ptr->parent_ = curPtr;
          ptr->t_ = curPtr->t_ + dt;
          ptr->score_ = curPtr->score_ + score_(ptr);
          ptr->h_ = calH(ptr);
          open_set.push(ptr);
        }
      }
      if (open_set.empty()) {
        std::cout << "[prediction] no way!" << std::endl;
        return false;
      }
      curPtr = open_set.top();
      open_set.pop();
    }
    
    target_predcit.clear();
    while (curPtr != nullptr) {
      target_predcit.push_back(curPtr->state_.p_);
      curPtr = curPtr->parent_;
    }
    std::reverse(target_predcit.begin(), target_predcit.end());
    return true;
  }

  // ==================== CYRA_model (静态函数) ====================
  static void CYRA_model(const State& state_in, const double& dT, State& state_out){
    state_out = state_in;
    
    // 水平运动
    state_out.v_ = state_in.v_ + state_in.a_ * dT;
    if (std::abs(state_in.omega_) < 1e-2){
      state_out.p_.x() = state_in.p_.x() + (0.5 * state_in.a_ * dT * dT + state_in.v_ * dT) * std::cos(state_in.theta_);
      state_out.p_.y() = state_in.p_.y() + (0.5 * state_in.a_ * dT * dT + state_in.v_ * dT) * std::sin(state_in.theta_);
      state_out.theta_ = state_in.theta_; 
    }else{
      double cx = state_in.p_.x() - (state_in.v_ / state_in.omega_) * std::sin(state_in.theta_) - (state_in.a_ / std::pow(state_in.omega_, 2)) * std::cos(state_in.theta_);
      double cy = state_in.p_.y() + (state_in.v_ / state_in.omega_) * std::cos(state_in.theta_) - (state_in.a_ / std::pow(state_in.omega_, 2)) * std::sin(state_in.theta_);

      state_out.theta_ = state_in.theta_ + state_in.omega_ * dT;
      state_out.p_.x() = (state_in.a_ / std::pow(state_in.omega_, 2)) * std::cos(state_out.theta_) + (state_out.v_ / state_in.omega_) * std::sin(state_out.theta_) + cx;
      state_out.p_.y() = (state_in.a_ / std::pow(state_in.omega_, 2)) * std::sin(state_out.theta_) - (state_out.v_ / state_in.omega_) * std::cos(state_out.theta_) + cy;
    }

    // 🚀 核心改动 3：引用静态函数 wave_freq_ref() 获取全局唯一的频率值
    double w = 2.0 * M_PI * wave_freq_ref();
    if (std::abs(w) < 1e-4) w = 1e-4; 

    double dz = state_in.p_.z() - z_mean_ref();
    
    state_out.p_.z() = dz * std::cos(w * dT) + (state_in.vz_ / w) * std::sin(w * dT) + z_mean_ref();
    state_out.vz_ = -dz * w * std::sin(w * dT) + state_in.vz_ * std::cos(w * dT);
  }

  // getPredState 接口
  inline void getPredState(const double& t, 
                           const Eigen::Vector3d& start_p, 
                           const Eigen::Vector3d& start_v, 
                           Eigen::Vector3d& pred_p, 
                           Eigen::Vector3d& pred_v,
                           const double& omega_yaw = 0.0) {
      
      double v_horiz = start_v.head(2).norm();
      double theta = std::atan2(start_v.y(), start_v.x());
      State s_in(start_p, v_horiz, 0.0, theta, omega_yaw, start_v.z());
      State s_out;
      
      CYRA_model(s_in, t, s_out);
      
      pred_p = s_out.p_;
      pred_v.x() = s_out.v_ * std::cos(s_out.theta_);
      pred_v.y() = s_out.v_ * std::sin(s_out.theta_);
      pred_v.z() = s_out.vz_; 
  }

  template<typename T> void vis_openset(T open_set) {} 
  template<typename T> void vis_curnode(T curnode) {}
};

}  // namespace prediction