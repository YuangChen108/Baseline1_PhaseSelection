#include <mapping/mapping.h>
#include <quadrotor_msgs/OccMap3d.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <mapping/visualization.hpp>

ros::Publisher esdf_vs_pub, gridmap_vs_pub, gridmap_inflate_vs_pub;
bool remove_floor_ceil_ = false;

std::shared_ptr<visualization::Visualization> vis_ptr_;

void gridmap_callback(const quadrotor_msgs::OccMap3dConstPtr& msgPtr) {
  mapping::OccGridMap gridmap;
  gridmap.from_msg(*msgPtr);
  sensor_msgs::PointCloud2 pc;
  gridmap.occ2pc(pc);
  pc.header.frame_id = "world";
  gridmap_vs_pub.publish(pc);
}
void gridmap_inflate_callback(const quadrotor_msgs::OccMap3dConstPtr& msgPtr) {
  mapping::OccGridMap gridmap;
  // ROS_WARN_STREAM("VS gridmap_inflate_callback");
  gridmap.from_msg(*msgPtr);
  // ROS_WARN_STREAM("VS from_msg done");
  sensor_msgs::PointCloud2 pc;
  if (remove_floor_ceil_) {
    gridmap.occ2pc(pc, 0.5, 2.5);
  } else {
    gridmap.occ2pc(pc);
  }
  pc.header.frame_id = "world";
  gridmap_inflate_vs_pub.publish(pc);
  // ROS_WARN_STREAM("VS publish inflate");

  gridmap.get_sdf_msg(pc);
  pc.header.frame_id = "world";
  esdf_vs_pub.publish(pc);
  // ROS_WARN_STREAM("VS publish esdf");

  // std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> grad_msg;
  // gridmap.get_sdf_gradient_msg(grad_msg);
  // vis_ptr_->visualize_arrows(grad_msg, "/esdf_grad");
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "mapping_vis");
  ros::NodeHandle nh("~");
  nh.getParam("remove_floor_ceil", remove_floor_ceil_);

  vis_ptr_ = std::make_shared<visualization::Visualization>(nh);

  esdf_vs_pub = nh.advertise<sensor_msgs::PointCloud2>("vs_sdf_map", 1);
  gridmap_vs_pub = nh.advertise<sensor_msgs::PointCloud2>("vs_gridmap", 1);
  gridmap_inflate_vs_pub = nh.advertise<sensor_msgs::PointCloud2>("vs_gridmap_inflate", 1);
  ros::Subscriber gridmap_sub = nh.subscribe<quadrotor_msgs::OccMap3d>("gridmap", 1, gridmap_callback);
  ros::Subscriber gridmap_inflate_sub = nh.subscribe<quadrotor_msgs::OccMap3d>("gridmap_inflate", 1, gridmap_inflate_callback);

  ros::spin();
  return 0;
}
