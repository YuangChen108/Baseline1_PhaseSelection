#include <fstream>
#include <iostream>
#include <vector>
//include ros dep.
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <tf/transform_broadcaster.h>

#include "tf/tf.h"
#include "tf/transform_datatypes.h"
//include pcl dep
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
//include opencv and eigen
#include <cv_bridge/cv_bridge.h>

#include <Eigen/Eigen>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

#include "opencv2/highgui/highgui.hpp"

//#include <cloud_banchmark/cloud_banchmarkConfig.h>
#include "depth_render.cuh"
#include "quadrotor_msgs/PositionCommand.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace cv;
using namespace std;
using namespace Eigen;

// quaternion to euler: roll, pitch, yaw
static Eigen::Matrix<double, 3, 1> quaternion2euler(const Eigen::Quaterniond &q){
    Eigen::Matrix<double, 3, 1> e;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w() * q.x() + q.y() * q.z());
    double cosr_cosp = 1 - 2 * (q.x() * q.x() + q.y() * q.y());
    e(0) = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q.w() * q.y() - q.z() * q.x());
    if (std::abs(sinp) >= 1)
        e(1) = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        e(1) = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
    e(2) = std::atan2(siny_cosp, cosy_cosp);

    return e;
}

// euler to quaternion: roll, pitch, yaw
static Eigen::Quaterniond euler2quaternion(const double& roll, const double& pitch, const double& yaw){
    // Abbreviations for the various angular functions
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    Eigen::Quaterniond q;
    q.w() = cr * cp * cy + sr * sp * sy;
    q.x() = sr * cp * cy - cr * sp * sy;
    q.y() = cr * sp * cy + sr * cp * sy;
    q.z() = cr * cp * sy - sr * sp * cy;

    return q;
}

int *depth_hostptr;
cv::Mat depth_mat;
cv::Mat depth_mat_vis;

//camera param
int width, height;
double fx, fy, cx, cy;

DepthRender depthrender;
ros::Publisher pub_depth;
ros::Publisher pub_color;
ros::Publisher pub_pose;
ros::Publisher pub_pcl_wolrd;
ros::Publisher pub_potentail_occ_pixel;
ros::Publisher pub_target_pose_body_frame;
ros::Publisher pub_target_dis;
ros::Publisher pub_target_yaw_err;
ros::Publisher pub_is_out_fov;
ros::Publisher pub_nearest_occ_uv;
ros::Publisher pub_tar_uv;

sensor_msgs::PointCloud2 local_map_pcl;
sensor_msgs::PointCloud2 local_depth_pcl;

ros::Subscriber odom_sub;
ros::Subscriber odom_target_sub;
ros::Subscriber global_map_sub, local_map_sub;

ros::Timer local_sensing_timer, estimation_timer;

bool has_global_map(false);
bool has_local_map(false);
bool has_odom(false);

Matrix4d cam02body;
Matrix4d cam2world;
Eigen::Quaterniond cam2world_quat;
nav_msgs::Odometry _odom;

double sensing_horizon, sensing_rate, estimation_rate;

ros::Time last_odom_stamp = ros::TIME_MAX;
Eigen::Vector3d last_pose_world;

Eigen::Vector3d target_p;
Eigen::Quaterniond target_q;
bool is_target_recieved = false;

Eigen::Vector3d odom_p;
Eigen::Quaterniond odom_q;
Matrix4d cam_render2world;
Eigen::Vector3d render_p;
bool is_has_render_p = false;

double tar_dis = 5.0;
double cam_yaw;

void render_currentpose();
void render_pcl_world();

void rcvTargetOdometryCallbck(const nav_msgs::Odometry &odom) {
  target_p << odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z;
  target_q.w() = odom.pose.pose.orientation.w;
  target_q.x() = odom.pose.pose.orientation.x;
  target_q.y() = odom.pose.pose.orientation.y;
  target_q.z() = odom.pose.pose.orientation.z;
  is_target_recieved = true;
}

void rcvOdometryCallbck(const nav_msgs::Odometry &odom) {
  /*if(!has_global_map)
    return;*/
  has_odom = true;
  _odom = odom;
  Matrix4d Pose_receive = Matrix4d::Identity();

  Eigen::Vector3d request_position;
  Eigen::Quaterniond request_pose;
  request_position.x() = odom.pose.pose.position.x;
  request_position.y() = odom.pose.pose.position.y;
  request_position.z() = odom.pose.pose.position.z;
  
  odom_p = request_position;

  Eigen::Quaterniond q(odom.pose.pose.orientation.w, odom.pose.pose.orientation.x, 
    odom.pose.pose.orientation.y, odom.pose.pose.orientation.z);
  Eigen::Vector3d rpy = quaternion2euler(q);
  // To make the coloreddepth image more stable
  rpy(0) *= 0.5;
  rpy(1) *= 0.5;
  cam_yaw = rpy(2);
  Eigen::Quaterniond q_fix;
  q_fix = euler2quaternion(rpy(0), rpy(1), rpy(2));
  request_pose.x() = q_fix.x();
  request_pose.y() = q_fix.y();
  request_pose.z() = q_fix.z();
  request_pose.w() = q_fix.w();

  odom_q = q_fix;

  if (is_target_recieved){
    double add_d = 5.0 - tar_dis;
    // if (add_d > 0){ //TODO
    //   render_p = odom_p - Eigen::Vector3d(add_d*cos(rpy(2)), add_d*sin(rpy(2)), 0.0);
    // }else{
    //   render_p = odom_p;
    // }

    render_p = odom_p;


    Pose_receive.block<3, 3>(0, 0) = request_pose.toRotationMatrix();
    Pose_receive(0, 3) = render_p(0);
    Pose_receive(1, 3) = render_p(1);
    Pose_receive(2, 3) = render_p(2);

    Matrix4d body_pose = Pose_receive;
    //convert to cam pose
    cam_render2world = body_pose * cam02body;
    is_has_render_p = true;
  }

  // request_pose.x() = odom.pose.pose.orientation.x;
  // request_pose.y() = odom.pose.pose.orientation.y;
  // request_pose.z() = odom.pose.pose.orientation.z;
  // request_pose.w() = odom.pose.pose.orientation.w;
  Pose_receive.block<3, 3>(0, 0) = request_pose.toRotationMatrix();
  Pose_receive(0, 3) = request_position(0);
  Pose_receive(1, 3) = request_position(1);
  Pose_receive(2, 3) = request_position(2);

  Matrix4d body_pose = Pose_receive;
  //convert to cam pose
  cam2world = body_pose * cam02body;
  cam2world_quat = cam2world.block<3, 3>(0, 0);

  last_odom_stamp = odom.header.stamp;

  last_pose_world(0) = odom.pose.pose.position.x;
  last_pose_world(1) = odom.pose.pose.position.y;
  last_pose_world(2) = odom.pose.pose.position.z;
}

void pubCameraPose(const ros::TimerEvent &event) {
  //cout<<"pub cam pose"
  geometry_msgs::PoseStamped camera_pose;
  camera_pose.header = _odom.header;
  camera_pose.header.frame_id = "world";
  camera_pose.pose.position.x = cam2world(0, 3);
  camera_pose.pose.position.y = cam2world(1, 3);
  camera_pose.pose.position.z = cam2world(2, 3);
  camera_pose.pose.orientation.w = cam2world_quat.w();
  camera_pose.pose.orientation.x = cam2world_quat.x();
  camera_pose.pose.orientation.y = cam2world_quat.y();
  camera_pose.pose.orientation.z = cam2world_quat.z();
  pub_pose.publish(camera_pose);
}

void renderSensedPoints(const ros::TimerEvent &event) {
  //if(! has_global_map || ! has_odom) return;
  if (!has_global_map && !has_local_map) return;

  if (!has_odom) return;
  render_currentpose();
  render_pcl_world();
}

vector<float> cloud_data;
void rcvGlobalPointCloudCallBack(const sensor_msgs::PointCloud2 &pointcloud_map) {
  if (has_global_map)
    return;

  ROS_WARN("Global Pointcloud received..");
  //load global map
  pcl::PointCloud<pcl::PointXYZ> cloudIn;
  pcl::PointXYZ pt_in;
  //transform map to point cloud format
  pcl::fromROSMsg(pointcloud_map, cloudIn);
  for (int i = 0; i < int(cloudIn.points.size()); i++) {
    pt_in = cloudIn.points[i];
    cloud_data.push_back(pt_in.x);
    cloud_data.push_back(pt_in.y);
    cloud_data.push_back(pt_in.z);
  }
  printf("global map has points: %d.\n", (int)cloud_data.size() / 3);
  //pass cloud_data to depth render
  depthrender.set_data(cloud_data);
  depth_hostptr = (int *)malloc(width * height * sizeof(int));

  has_global_map = true;
  // ROS_ERROR("GLOBAL!!!!");
}

void rcvLocalPointCloudCallBack(const sensor_msgs::PointCloud2 &pointcloud_map) {
  //ROS_WARN("Local Pointcloud received..");
  //load local map
  pcl::PointCloud<pcl::PointXYZ> cloudIn;
  pcl::PointXYZ pt_in;
  //transform map to point cloud format
  pcl::fromROSMsg(pointcloud_map, cloudIn);

  if (cloudIn.points.size() == 0) return;
  for (int i = 0; i < int(cloudIn.points.size()); i++) {
    pt_in = cloudIn.points[i];
    Eigen::Vector3d pose_pt(pt_in.x, pt_in.y, pt_in.z);
    //pose_pt = gridIndex2coord(coord2gridIndex(pose_pt));
    cloud_data.push_back(pose_pt(0));
    cloud_data.push_back(pose_pt(1));
    cloud_data.push_back(pose_pt(2));
  }
  //printf("local map has points: %d.\n", (int)cloud_data.size() / 3 );
  //pass cloud_data to depth render
  depthrender.set_data(cloud_data);
  depth_hostptr = (int *)malloc(width * height * sizeof(int));

  has_local_map = true;
}

void render_pcl_world() {
  //for debug purpose
  pcl::PointCloud<pcl::PointXYZ> localMap;
  pcl::PointXYZ pt_in;

  Eigen::Vector4d pose_in_camera;
  Eigen::Vector4d pose_in_world;
  Eigen::Vector3d pose_pt;

  for (int u = 0; u < width; u++)
    for (int v = 0; v < height; v++) {
      float depth = depth_mat.at<float>(v, u);

      if (depth == 0.0)
        continue;

      pose_in_camera(0) = (u - cx) * depth / fx;
      pose_in_camera(1) = (v - cy) * depth / fy;
      pose_in_camera(2) = depth;
      pose_in_camera(3) = 1.0;

      pose_in_world = cam2world * pose_in_camera;

      if ((pose_in_world.segment(0, 3) - last_pose_world).norm() > sensing_horizon)
        continue;

      pose_pt = pose_in_world.head(3);
      pt_in.x = pose_pt(0);
      pt_in.y = pose_pt(1);
      pt_in.z = pose_pt(2);

      localMap.points.push_back(pt_in);
    }

  localMap.width = localMap.points.size();
  localMap.height = 1;
  localMap.is_dense = true;

  pcl::toROSMsg(localMap, local_map_pcl);
  local_map_pcl.header.frame_id = "world";
  local_map_pcl.header.stamp = last_odom_stamp;

  pub_pcl_wolrd.publish(local_map_pcl);
}

double cal_dis(const int& u0, const int& v0, const int& u1, const int& v1){
  return sqrt((u0-u1) * (u0-u1) + (v0-v1) * (v0-v1));
}

bool find_closet_pixel(double& dis_min, int& row, int& col, double& d, const int& row_tar, const int& col_tar, const double& depth_thres){
  dis_min = 99999;
  bool is_has_occ = false;
  for (int i = 0; i < height; i++)
    for (int j = 0; j < width; j++) {
      double d_cur = (double)depth_mat.at<float>(i, j);
      if (d_cur < depth_thres){
        double dis = cal_dis(i, j, row_tar, col_tar);
        if (dis < dis_min){
          dis_min = dis;
          row = i;
          col = j;
          d = d_cur;
          is_has_occ = true;
        }
      }
    }
    return is_has_occ;
}

void get_target_pixel(double& u, double& v, double& d, double& r){
  static double eps = 1e-6;
  Eigen::Quaterniond qbw;
  qbw.w() = odom_q.w(); qbw.x() = -odom_q.x(); qbw.y() = -odom_q.y(); qbw.z() = -odom_q.z();
  Eigen::Vector3d pc; // target in camera frame
  Eigen::Matrix3d cam2body_R_front_;
  cam2body_R_front_ << 0.0,  0.0,  1.0,
                      -1.0,  0.0,  0.0,
                       0.0, -1.0,  0.0;

  pc = cam2body_R_front_.transpose() * (qbw * (target_p - render_p)); 

  nav_msgs::Odometry odom_msg;
  odom_msg.header.frame_id = "base";
  odom_msg.header.stamp = ros::Time::now();
  odom_msg.pose.pose.position.x = pc.x();
  odom_msg.pose.pose.position.y = pc.y();
  odom_msg.pose.pose.position.z = pc.z();

  pub_target_pose_body_frame.publish(odom_msg);

  std_msgs::Float32 target_dis_msg;
  target_dis_msg.data = pc.norm();
  pub_target_dis.publish(target_dis_msg);
  tar_dis = pc.norm();

  Eigen::Vector2d v1(pc.x(), pc.z());
  Eigen::Vector2d v2(0, 1);
  double yaw_err = acos(v1.dot(v2)/(v1.norm()));
  std_msgs::Float32 target_yaw_err_msg;
  target_yaw_err_msg.data = yaw_err;
  pub_target_yaw_err.publish(target_yaw_err_msg);

  u = pc.x() / (pc.z() + eps) * fx + cx;
  v = pc.y() / (pc.z() + eps) * fy + cy;  
  d = pc.z();
  r = 0.2 / (pc.z() + eps) * fx;

  std_msgs::Int8 int_msg;
  if (d < 0.0 || d > 10.0 || u < 0 || u > 640 || v < 0 || v > 480){
    int_msg.data = 1;
  }else{
    int_msg.data = 0;
  }
  pub_is_out_fov.publish(int_msg);
}

void render_currentpose() {
  double this_time = ros::Time::now().toSec();

  Matrix4d cam_pose = cam2world.inverse();

  double pose[4 * 4];

  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++)
      pose[j + 4 * i] = cam_pose(i, j);

  depthrender.render_pose(pose, depth_hostptr);
  //depthrender.render_pose(cam_pose, depth_hostptr);

  depth_mat = cv::Mat::zeros(height, width, CV_32FC1);
  double min = 0.2;
  double max = 1.0f;
  double invalid_depth = 10.0f;
  const double potential_occulusion_dis = 5.0;
  for (int i = 0; i < height; i++)
    for (int j = 0; j < width; j++) {
      float depth = (float)depth_hostptr[i * width + j] / 1000.0f;
      depth = depth < invalid_depth ? depth : invalid_depth;
      max = depth > max ? depth : max;
      if (depth < 1e-3){ // empty pixel
        depth = invalid_depth;
      } 
      depth_mat.at<float>(i, j) = depth;
    }


  if (is_has_render_p){

    Matrix4d cam_pose = cam_render2world.inverse();

    double pose[4 * 4];

    for (int i = 0; i < 4; i++)
      for (int j = 0; j < 4; j++)
        pose[j + 4 * i] = cam_pose(i, j);

    depthrender.render_pose(pose, depth_hostptr);
    //depthrender.render_pose(cam_pose, depth_hostptr);

    depth_mat_vis = cv::Mat::zeros(height, width, CV_32FC1);
    double min = 0.2;
    double max = 1.0f;
    double invalid_depth = 10.0f;
    const double potential_occulusion_dis = 5.0;
    for (int i = 0; i < height; i++)
      for (int j = 0; j < width; j++) {
        float depth = (float)depth_hostptr[i * width + j] / 1000.0f;
        depth = depth < invalid_depth ? depth : invalid_depth;
        max = depth > max ? depth : max;
        if (depth < 1e-3){ // empty pixel
          depth = invalid_depth;
        } 
        depth_mat_vis.at<float>(i, j) = depth;
      }

    // add target on the img
    // depth_mat.copyTo(depth_mat_vis);
    double u_tar, v_tar, r_tar, d_tar;
    get_target_pixel(u_tar, v_tar, d_tar, r_tar);
    for (int i = u_tar - r_tar; i < u_tar + r_tar; i++){
      for (int j = v_tar - r_tar; j < v_tar + r_tar; j++){
        if (i >= 0 && i < width && j >= 0 && j < height && (i-u_tar)*(i-u_tar) + (j-v_tar)*(j-v_tar) <= r_tar*r_tar){
          // depth_mat.at<float>(i, j) = 1;
          depth_mat_vis.at<float>(j, i) = min;
        }
      }
    }

    double row_tar, col_tar;
    row_tar = v_tar;
    col_tar = u_tar;




    int row_close, col_close;
    double dis_min;
    double proj_dis_w;
    std_msgs::Float32 msg_dis;
    // d_tar = 5.0; //TODO
    double d_close;
    if (find_closet_pixel(dis_min, row_close, col_close, d_close, (int)row_tar, (int)col_tar, d_tar)){
      //calculate minoccdis
      depth_mat_vis.at<float>(row_close, col_close) = min;
      double row_diff = ((double)row_close - row_tar);
      double col_diff = ((double)col_close - col_tar);
      double y_diff = row_diff / fy * d_close;
      double x_diff = col_diff / fx * d_close;
      double dis_w = sqrt(x_diff * x_diff + y_diff * y_diff);
      // ROS_INFO_STREAM("x_diff: " << x_diff << ", y_diff: " << y_diff<<", d: " << d_close << ", d_tar: " << d_tar<<", dw: " << dis_w);

      msg_dis.data = dis_w;

      geometry_msgs::PoseStamped tar_uv;
      tar_uv.pose.position.x = u_tar;
      tar_uv.pose.position.y = v_tar;
      pub_tar_uv.publish(tar_uv);
      geometry_msgs::PoseStamped occ_closest;
      occ_closest.pose.position.x = col_close;
      occ_closest.pose.position.y = row_close;
      pub_nearest_occ_uv.publish(occ_closest);
    }else{
      msg_dis.data = -1;
    }
    pub_potentail_occ_pixel.publish(msg_dis);
  }


  //ROS_INFO("render cost %lf ms.", (ros::Time::now().toSec() - this_time) * 1000.0f);
  //printf("max_depth %lf.\n", max);

  cv_bridge::CvImage out_msg;
  out_msg.header.stamp = last_odom_stamp;
  out_msg.header.frame_id = "camera";
  out_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  out_msg.image = depth_mat.clone();
  pub_depth.publish(out_msg.toImageMsg());

  cv::Mat adjMap;
  // depth_mat.convertTo(adjMap,CV_8UC1, 255 / (max-min), -min);
  depth_mat_vis.convertTo(adjMap, CV_8UC1, 255 / 13.0, -min);
  cv::Mat falseColorsMap;
  cv::applyColorMap(adjMap, falseColorsMap, cv::COLORMAP_RAINBOW);
  cv_bridge::CvImage cv_image_colored;
  cv_image_colored.header.frame_id = "depthmap";
  cv_image_colored.header.stamp = last_odom_stamp;
  cv_image_colored.encoding = sensor_msgs::image_encodings::BGR8;
  cv_image_colored.image = falseColorsMap;
  pub_color.publish(cv_image_colored.toImageMsg());
  //cv::imshow("depth_image", adjMap);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "pcl_render");
  ros::NodeHandle nh("~");

  nh.getParam("cam_width", width);
  nh.getParam("cam_height", height);
  nh.getParam("cam_fx", fx);
  nh.getParam("cam_fy", fy);
  nh.getParam("cam_cx", cx);
  nh.getParam("cam_cy", cy);
  nh.getParam("sensing_horizon", sensing_horizon);
  nh.getParam("sensing_rate", sensing_rate);
  nh.getParam("estimation_rate", estimation_rate);

  depthrender.set_para(fx, fy, cx, cy, width, height);

  // cam02body <<  0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975,
  //               0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768,
  //               -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949,
  //               0.0, 0.0, 0.0, 1.0;

  cam02body << 0.0, 0.0, 1.0, 0.0,
      -1.0, 0.0, 0.0, 0.0,
      0.0, -1.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 1.0;

  //init cam2world transformation
  cam2world = Matrix4d::Identity();
  //subscribe point cloud
  global_map_sub = nh.subscribe("global_map", 1, rcvGlobalPointCloudCallBack);
  local_map_sub = nh.subscribe("local_map", 1, rcvLocalPointCloudCallBack);
  odom_sub = nh.subscribe("odometry", 50, rcvOdometryCallbck);
  odom_target_sub = nh.subscribe("/target/odom", 50, rcvTargetOdometryCallbck);

  //publisher depth image and color image
  pub_depth = nh.advertise<sensor_msgs::Image>("depth", 1000);
  pub_color = nh.advertise<sensor_msgs::Image>("colordepth", 1000);
  pub_pose = nh.advertise<geometry_msgs::PoseStamped>("camera_pose", 1000);
  pub_pcl_wolrd = nh.advertise<sensor_msgs::PointCloud2>("local_cloud", 1);
  pub_potentail_occ_pixel = nh.advertise<std_msgs::Float32>("/pixel_occ_dis", 1);
  pub_target_pose_body_frame = nh.advertise<nav_msgs::Odometry>("/target_pwb", 1);
  pub_target_dis = nh.advertise<std_msgs::Float32>("/target_dis", 1);
  pub_target_yaw_err = nh.advertise<std_msgs::Float32>("/target_yaw_err", 1);

  pub_is_out_fov = nh.advertise<std_msgs::Int8>("/is_out_fov", 1);
  pub_nearest_occ_uv = nh.advertise<geometry_msgs::PoseStamped>("/nearest_occ_uv", 1000);
  pub_tar_uv = nh.advertise<geometry_msgs::PoseStamped>("/tar_uv", 1000);


  double sensing_duration = 1.0 / sensing_rate;
  double estimate_duration = 1.0 / estimation_rate;

  local_sensing_timer = nh.createTimer(ros::Duration(sensing_duration), renderSensedPoints);
  estimation_timer = nh.createTimer(ros::Duration(estimate_duration), pubCameraPose);
  //cv::namedWindow("depth_image",1);

  ros::Rate rate(100);
  bool status = ros::ok();
  while (status) {
    ros::spinOnce();
    status = ros::ok();
    rate.sleep();
  }
}
