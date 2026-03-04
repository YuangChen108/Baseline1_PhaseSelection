#pragma once
#include "util_gym/util_gym.hpp"
#ifdef ROS
#include <mapping/mapping.h>
namespace map_interface
{
  
class MapInterface{
 private:
  std::shared_ptr<mapping::OccGridMap> gridmapPtr_;
 public:
  MapInterface(std::shared_ptr<mapping::OccGridMap> gridmapPtr):gridmapPtr_(gridmapPtr){}

  inline void from_msg(const quadrotor_msgs::OccMap3d& msg) {
    gridmapPtr_->from_msg(msg);
  }
  inline const bool isOccupied(const Eigen::Vector3d& p) const {
    return gridmapPtr_->isOccupied(p);
  }
  inline const bool isOccupied(const Eigen::Vector3i& id) const {
    return gridmapPtr_->isOccupied(id);
  }
  inline bool checkRayValid(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1) const {
    return gridmapPtr_->checkRayValid(p0, p1);
  }
  inline double getCostWithGrad(const Eigen::Vector3d& pos, Eigen::Vector3d& grad) const{
    return gridmapPtr_->getCostWithGrad(pos, grad);
  }
  inline const Eigen::Vector3i pos2idx(const Eigen::Vector3d& pt) const {
    return gridmapPtr_->pos2idx(pt);
  }
  inline const Eigen::Vector3d idx2pos(const Eigen::Vector3i& id) const {
    return gridmapPtr_->idx2pos(id);
  }
  inline double resolution() const{
    return gridmapPtr_->resolution;
  }
};

} // namespace map_interface
#endif