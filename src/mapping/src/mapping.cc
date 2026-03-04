#include <mapping/mapping.h>
#include <pcl_conversions/pcl_conversions.h>

#include <iostream>

namespace mapping {
void OccGridMap::updateMap(const Eigen::Vector3d& sensor_p,
                           const std::vector<Eigen::Vector3d>& pc) {
  vis.fillData(0);

  Eigen::Vector3i sensor_idx = pos2idx(sensor_p);
  Eigen::Vector3i offset = sensor_idx - Eigen::Vector3i(size_x / 2, size_y / 2, size_z / 2);
  // NOTE clear the updated part
  if (init_finished) {
    Eigen::Vector3i move = offset - Eigen::Vector3i(offset_x, offset_y, offset_z);
    Eigen::Vector3i from, to, from_small, to_small;
    Eigen::Vector3i size_xyz(size_x, size_y, size_z);
    for (int i = 0; i < 3; ++i) {
      if (move[i] >= 0) {
        from[i] = 0;
        from_small[i] = inflate_size;
        to[i] = move[i];
        to_small[i] = move[i] + inflate_size;
      } else {
        from[i] = move[i] + size_xyz[i];
        from_small[i] = move[i] + size_x - inflate_size;
        to[i] = size_xyz[i];
        to_small[i] = size_xyz[i] - inflate_size;
      }
    }

    for (int x = from.x(); x < to.x(); ++x) {
      for (int y = 0; y < size_y; ++y) {
        for (int z = 0; z < size_z; ++z) {
          Eigen::Vector3i id = Eigen::Vector3i(offset_x + x, offset_y + y, offset_z + z);
          occ.atId(id) = 0;
          infocc.atId(id) = 0;
        }
      }
    }
    for (int x = from_small.x(); x < to_small.x(); ++x) {
      for (int y = inflate_size; y < size_y - inflate_size; ++y) {
        for (int z = inflate_size; z < size_z - inflate_size; ++z) {
          Eigen::Vector3i id = Eigen::Vector3i(offset_x + x, offset_y + y, offset_z + z);
          pro.atId(id) = p_def;
        }
      }
    }
    offset_x = offset.x();
    for (int y = from.y(); y < to.y(); ++y) {
      for (int x = 0; x < size_x; ++x) {
        for (int z = 0; z < size_z; ++z) {
          Eigen::Vector3i id = Eigen::Vector3i(offset_x + x, offset_y + y, offset_z + z);
          occ.atId(id) = 0;
          infocc.atId(id) = 0;
        }
      }
    }
    for (int y = from_small.y(); y < to_small.y(); ++y) {
      for (int x = inflate_size; x < size_x - inflate_size; ++x) {
        for (int z = inflate_size; z < size_z - inflate_size; ++z) {
          Eigen::Vector3i id = Eigen::Vector3i(offset_x + x, offset_y + y, offset_z + z);
          pro.atId(id) = p_def;
        }
      }
    }
    offset_y = offset.y();
    for (int z = from.z(); z < to.z(); ++z) {
      for (int x = 0; x < size_x; ++x) {
        for (int y = 0; y < size_y; ++y) {
          Eigen::Vector3i id = Eigen::Vector3i(offset_x + x, offset_y + y, offset_z + z);
          occ.atId(id) = 0;
          infocc.atId(id) = 0;
        }
      }
    }
    for (int z = from_small.z(); z < to_small.z(); ++z) {
      for (int x = inflate_size; x < size_x - inflate_size; ++x) {
        for (int y = inflate_size; y < size_y - inflate_size; ++y) {
          Eigen::Vector3i id = Eigen::Vector3i(offset_x + x, offset_y + y, offset_z + z);
          pro.atId(id) = p_def;
        }
      }
    }
    offset_z = offset.z();
  } else {
    offset_x = offset.x();
    offset_y = offset.y();
    offset_z = offset.z();
    init_finished = true;
  }

  // set occupied
  for (const auto& p : pc) {
    Eigen::Vector3d pt;
    bool inrange = filter(sensor_p, p, pt);
    Eigen::Vector3i idx = pos2idx(pt);
    if (vis.atId(idx) != 1) {
      if (inrange) {
        hit(idx);
      } else {
        mis(idx);
      }
    } else {
      continue;
    }

    // ray casting
    Eigen::Vector3i d_idx = sensor_idx - idx;
    Eigen::Vector3i step = d_idx.array().sign().cast<int>();
    Eigen::Vector3d delta_t;
    Eigen::Vector3d dp = sensor_p - pt;
    for (int i = 0; i < 3; ++i) {
      delta_t(i) = dp(i) == 0 ? std::numeric_limits<double>::max() : 1.0 / std::fabs(dp(i));
    }
    Eigen::Vector3d t_max;
    for (int i = 0; i < 3; ++i) {
      t_max(i) = step(i) > 0 ? (idx(i) + 1) - pt(i) / resolution : pt(i) / resolution - idx(i);
    }
    t_max = t_max.cwiseProduct(delta_t);
    Eigen::Vector3i rayIdx = idx;
    while ((rayIdx - sensor_idx).squaredNorm() != 1) {
      // find the shortest t_max
      int s_dim = 0;
      for (int i = 1; i < 3; ++i) {
        s_dim = t_max(i) < t_max(s_dim) ? i : s_dim;
      }
      rayIdx(s_dim) += step(s_dim);
      t_max(s_dim) += delta_t(s_dim);
      Eigen::Vector3i rayAdd = occ.idx2add(rayIdx);
      if (vis.at(rayAdd) == -1) {
        break;
      }
      if (vis.at(rayAdd) != 1) {
        mis(rayIdx);
      }
    }
  }

  // ESDF update
  static bool  esdf_need_update_ = true;
  if (esdf_need_update_){
    // esdf_need_update_ = false;
    // std::cout << "ESDF UPdate begin" << std::endl;
    ros::Time t1 = ros::Time::now();
    // Eigen::Vector3i min_esdf = Eigen::Vector3i(offset_x + inflate_size, offset_y + inflate_size, offset_z + offset_z);
    // Eigen::Vector3i max_esdf = Eigen::Vector3i(offset_x + size_x - inflate_size, offset_y + size_y - inflate_size, offset_z + size_z - offset_z);
    Eigen::Vector3i min_esdf = offset;
    Eigen::Vector3i max_esdf = offset + Eigen::Vector3i(size_x, size_y, size_z);

    // ROS_INFO_STREAM("offset: "<<offset.transpose()<<", size: "<<size_x<<", "<<size_y<<", "<<size_z);
    // ROS_INFO_STREAM("occ size: "<<occ_buffer_.size_x_<<", "<<occ_buffer_.size_y_<<", "<<occ_buffer_.size_z_);
    // ROS_INFO_STREAM("min_esdf: " << min_esdf.transpose());
    // ROS_INFO_STREAM("max_esdf: " << max_esdf.transpose());
    occ_buffer_.setup(size_x, size_y, size_z);
    occ_neg_buffer_.setup(size_x, size_y, size_z);
    tmp_buffer1_.setup(size_x, size_y, size_z);
    tmp_buffer2_.setup(size_x, size_y, size_z);
    distance_buffer_.setup(size_x, size_y, size_z);
    distance_buffer_neg_.setup(size_x, size_y, size_z);
    distance_buffer_all_.setup(size_x, size_y, size_z);

    // clean ESDF
    /* ========== clear ESDF ========== */
    tmp_buffer1_.fillData(0);
    tmp_buffer2_.fillData(0);
    distance_buffer_.fillData(0);
    distance_buffer_neg_.fillData(0);
    distance_buffer_all_.fillData(0);
    occ_neg_buffer_.fillData(0);

    int bx = 0, by = 0, bz = 0;
    bool no_occ = true;
    for (int x = min_esdf[0]; x < max_esdf[0]; x++) {
      by = 0;
      for (int y = min_esdf[1]; y < max_esdf[1]; y++) {
        bz = 0;
        for(int z = min_esdf[2]; z < max_esdf[2]; z++){
          Eigen::Vector3i idx(x, y, z);
          Eigen::Vector3i bidx(bx, by, bz);
          // ROS_INFO_STREAM("bidx: "<<bidx.transpose()<<", idx: "<<idx.transpose());
          // ROS_INFO_STREAM("infocc: "<<(int)infocc.atId(idx));
          int8_t is_occ = infocc.atId(idx);
          occ_buffer_.atId(bidx) = is_occ;
          if (is_occ) no_occ = false;
          bz++;
        }
        by++;
      }
      bx++;
    }

    // std::cout << "clear done" << std::endl;
    // if no occ in local region
    if (no_occ){
      double max_dis = std::max(std::max(size_x, size_y), size_z) * resolution;
      distance_buffer_all_.fillData(max_dis);
      return;
    } 



    min_esdf << 0, 0, 0;
    max_esdf << size_x-1, size_y-1, size_z-1;

    /* ========== compute positive DT ========== */
    for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
      for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
        fillESDF(
            [&](int z) {
              return occ_buffer_.atId(Eigen::Vector3i(x,y,z)) == 1 ?
                  0 :
                  std::numeric_limits<double>::max();
            },
            [&](int z, double val) { tmp_buffer1_.atId(Eigen::Vector3i(x,y,z)) = val; }, min_esdf[2],
            max_esdf[2], 2);
      }
    }

    for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
      for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
        fillESDF([&](int y) { return tmp_buffer1_.atId(Eigen::Vector3i(x,y,z)); },
                [&](int y, double val) { tmp_buffer2_.atId(Eigen::Vector3i(x,y,z)) = val; }, min_esdf[1],
                max_esdf[1], 1);
      }
    }

    for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
      for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
        fillESDF([&](int x) { return tmp_buffer2_.atId(Eigen::Vector3i(x,y,z)); },
                [&](int x, double val) {
                  distance_buffer_.atId(Eigen::Vector3i(x,y,z)) = resolution * std::sqrt(val);
                  //  min(mp_.resolution_ * std::sqrt(val),
                  //      distance_buffer_[toAddress(x, y, z)]);
                },
                min_esdf[0], max_esdf[0], 0);
      }
    }

    /* ========== compute negative distance ========== */
    for (int x = min_esdf(0); x <= max_esdf(0); ++x)
      for (int y = min_esdf(1); y <= max_esdf(1); ++y)
        for (int z = min_esdf(2); z <= max_esdf(2); ++z) {

          Eigen::Vector3i idx = Eigen::Vector3i(x, y, z);
          if (occ_buffer_.atId(idx) == 0) {
            occ_neg_buffer_.atId(idx) = 1;
          } else if (occ_buffer_.atId(idx) == 1) {
            occ_neg_buffer_.atId(idx) = 0;
          } else {
            ROS_ERROR("what?");
          }
        }

    tmp_buffer1_.fillData(0);
    tmp_buffer2_.fillData(0);

    for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
      for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
        fillESDF(
            [&](int z) {
              return occ_neg_buffer_.atId(Eigen::Vector3i(x,y,z)) == 1 ?
                  0 :
                  std::numeric_limits<double>::max();
            },
            [&](int z, double val) { tmp_buffer1_.atId(Eigen::Vector3i(x,y,z)) = val; }, min_esdf[2],
            max_esdf[2], 2);
      }
    }

    for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
      for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
        fillESDF([&](int y) { return tmp_buffer1_.atId(Eigen::Vector3i(x,y,z)); },
                [&](int y, double val) { tmp_buffer2_.atId(Eigen::Vector3i(x,y,z)) = val; }, min_esdf[1],
                max_esdf[1], 1);
      }
    }

    for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
      for (int z = min_esdf[2]; z <= max_esdf[2]; z++) {
        fillESDF([&](int x) { return tmp_buffer2_.atId(Eigen::Vector3i(x,y,z)); },
                [&](int x, double val) {
                  distance_buffer_neg_.atId(Eigen::Vector3i(x,y,z)) = resolution * std::sqrt(val);
                },
                min_esdf[0], max_esdf[0], 0);
      }
    }

    /* ========== combine pos and neg DT ========== */
    for (int x = min_esdf(0); x <= max_esdf(0); ++x)
      for (int y = min_esdf(1); y <= max_esdf(1); ++y)
        for (int z = min_esdf(2); z <= max_esdf(2); ++z) {

          Eigen::Vector3i idx = Eigen::Vector3i(x, y, z);
          distance_buffer_all_.atId(idx) = distance_buffer_.atId(idx);

          if (distance_buffer_neg_.atId(idx) > 0.0)
            distance_buffer_all_.atId(idx) += (-distance_buffer_neg_.atId(idx) + resolution);
        }

    // std::cout << "ESDF UPdate Done!!!!!!" << std::endl;
    ros::Time t2 = ros::Time::now();
    // std::cout << "time: " << (t2-t1).toSec() << std::endl;
  }

}

template <typename F_get_val, typename F_set_val>
void OccGridMap::fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim) {
  int voxel_num;
  if (dim == 0){
    voxel_num = size_x;
  }else if (dim == 1){
    voxel_num = size_y;
  }else if (dim == 2){
    voxel_num = size_z;
  }
  int v[voxel_num];
  double z[voxel_num + 1];

  int k = start;
  v[start] = start;
  z[start] = -std::numeric_limits<double>::max();
  z[start + 1] = std::numeric_limits<double>::max();

  for (int q = start + 1; q <= end; q++) {
    k++;
    double s;

    do {
      k--;
      s = ((f_get_val(q) + q * q) - (f_get_val(v[k]) + v[k] * v[k])) / (2 * q - 2 * v[k]);
    } while (s <= z[k]);

    k++;

    v[k] = q;
    z[k] = s;
    z[k + 1] = std::numeric_limits<double>::max();
  }

  k = start;

  for (int q = start; q <= end; q++) {
    while (z[k + 1] < q) k++;
    double val = (q - v[k]) * (q - v[k]) + f_get_val(v[k]);
    f_set_val(q, val);
  }
}

double OccGridMap::getFineDistance(const Eigen::Vector3d& pos) const {
  Eigen::Vector3d pos_m = pos - 0.5 * resolution * Eigen::Vector3d::Ones();
  Eigen::Vector3i idx = pos2idx(pos_m);

  Eigen::Vector3d idx_p = idx2pos(idx);
  Eigen::Vector3d diff = (pos - idx_p) / resolution;
  double values[2][2][2];
  double max_dis = std::max(std::max(size_x, size_y), size_z) * resolution;
  for (int x = 0; x < 2; x++) {
    for (int y = 0; y < 2; y++) {
      for (int z = 0; z < 2; z++) {
        Eigen::Vector3i current_idx = idx + Eigen::Vector3i(x, y, z);
        if (isInMap(current_idx)) {
          Eigen::Vector3i idx_local = current_idx - Eigen::Vector3i(offset_x, offset_y, offset_z);
          values[x][y][z] = distance_buffer_all_.atId(idx_local);
        } else {
          // values[x][y][z] = 0;
          values[x][y][z] = max_dis;
        }
      }
    }
  }
  double v00 = (1 - diff[0]) * values[0][0][0] + diff[0] * values[1][0][0];
  double v01 = (1 - diff[0]) * values[0][0][1] + diff[0] * values[1][0][1];
  double v10 = (1 - diff[0]) * values[0][1][0] + diff[0] * values[1][1][0];
  double v11 = (1 - diff[0]) * values[0][1][1] + diff[0] * values[1][1][1];
  double v0 = (1 - diff[1]) * v00 + diff[1] * v10;
  double v1 = (1 - diff[1]) * v01 + diff[1] * v11;
  double dist = (1 - diff[2]) * v0 + diff[2] * v1;

  return dist;
}

double OccGridMap::getCostWithGrad(const Eigen::Vector3d& pos, Eigen::Vector3d& grad) const {
  Eigen::Vector3d pos_m = pos - 0.5 * resolution * Eigen::Vector3d::Ones();
  Eigen::Vector3i idx = pos2idx(pos_m);

  Eigen::Vector3d idx_p = idx2pos(idx);
  Eigen::Vector3d diff = (pos - idx_p) / resolution;
  double values[2][2][2];
  double max_dis = std::max(std::max(size_x, size_y), size_z) * resolution;
  for (int x = 0; x < 2; x++) {
    for (int y = 0; y < 2; y++) {
      for (int z = 0; z < 2; z++) {
        Eigen::Vector3i current_idx = idx + Eigen::Vector3i(x, y, z);
        if (isInMap(current_idx)) {
          Eigen::Vector3i idx_local = current_idx - Eigen::Vector3i(offset_x, offset_y, offset_z);
          values[x][y][z] = distance_buffer_all_.atId(idx_local);
        } else {
          // values[x][y][z] = 0;
          values[x][y][z] = max_dis;
        }
      }
    }
  }
  double v00 = (1 - diff[0]) * values[0][0][0] + diff[0] * values[1][0][0];
  double v01 = (1 - diff[0]) * values[0][0][1] + diff[0] * values[1][0][1];
  double v10 = (1 - diff[0]) * values[0][1][0] + diff[0] * values[1][1][0];
  double v11 = (1 - diff[0]) * values[0][1][1] + diff[0] * values[1][1][1];
  double v0 = (1 - diff[1]) * v00 + diff[1] * v10;
  double v1 = (1 - diff[1]) * v01 + diff[1] * v11;
  double dist = (1 - diff[2]) * v0 + diff[2] * v1;

  grad[2] = (v1 - v0) / resolution;
  grad[1] = ((1 - diff[2]) * (v10 - v00) + diff[2] * (v11 - v01)) / resolution;
  grad[0] = (1 - diff[2]) * (1 - diff[1]) * (values[1][0][0] - values[0][0][0]);
  grad[0] += (1 - diff[2]) * diff[1] * (values[1][1][0] - values[0][1][0]);
  grad[0] += diff[2] * (1 - diff[1]) * (values[1][0][1] - values[0][0][1]);
  grad[0] += diff[2] * diff[1] * (values[1][1][1] - values[0][1][1]);

  grad[0] /= resolution;
  // grad.normalize();

  // check if at the grad peak, to avoid grad jump between 1 and -1
  // if (grad.squaredNorm() < 1e-4){
  //   std::cout <<"x comp!!!"<<std::endl;
  //   double dis_0, dis_2;
  //   Eigen::Vector3d p0 = pos;
  //   Eigen::Vector3d p2 = pos;
  //   p0.x() = pos.x() - resolution/2.0;
  //   p2.x() = pos.x() + resolution/2.0;
  //   dis_0 = getFineDistance(p0);
  //   dis_2 = getFineDistance(p2);
  //   grad.x() = (dis_2 - dis_0) / (resolution/2.0);
  // }
  // if (grad.squaredNorm() < 1e-4){
  //   std::cout <<"y comp!!!"<<std::endl;
  //   double dis_0, dis_2;
  //   Eigen::Vector3d p0 = pos;
  //   Eigen::Vector3d p2 = pos;
  //   p0.y() = pos.y() - resolution/2.0;
  //   p2.y() = pos.y() + resolution/2.0;
  //   dis_0 = getFineDistance(p0);
  //   dis_2 = getFineDistance(p2);
  //   grad.y() = (dis_2 - dis_0) / (resolution/2.0);
  // }

  return dist;
}

void OccGridMap::get_sdf_msg(sensor_msgs::PointCloud2& msg) {
  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::PointXYZI pt;

  Eigen::Vector3d p_min = idx2pos(Eigen::Vector3i(offset_x + inflate_size, offset_y + inflate_size, 0));
  Eigen::Vector3d p_max = idx2pos(Eigen::Vector3i(offset_x + size_x - inflate_size, offset_y + size_y - inflate_size, 0));
  // std::cout << "offset: "<<offset_x<<", "<<offset_y<<", "<<offset_z<<std::endl;
  // std::cout << "size: "<<size_x<<", "<<size_y<<", "<<size_z<<std::endl;
  // std::cout << "inflate size: "<<inflate_size<<std::endl;
  // std::cout << "p_min: "<<p_min.transpose() <<", pmax: "<<p_max.transpose()<<std::endl;
  double max_dis = std::max(std::max(size_x, size_y), size_z) * resolution;
  Eigen::Vector3d pos(0, 0, 1);
  for (pos.x() = p_min.x(); pos.x() <= p_max.x(); pos.x() += 0.1)
    for (pos.y() = p_min.y(); pos.y() <= p_max.y(); pos.y() += 0.1) {
      double dist = getFineDistance(pos);
      if (abs(dist) > 2.0*max_dis){
        std::cout << "ERROR!!! pos: "<<pos.transpose() <<", dist: "<<dist<<std::endl;
      }
      // if (dist >= 0) {
        pt.x = pos.x();
        pt.y = pos.y();
        pt.z = pos.z();
        pt.intensity = dist/(size_x*resolution);
        cloud.push_back(pt);
      // }
    }

  cloud.width = cloud.points.size();
  // ROS_ERROR_STREAM("get_sdf_msg size: " << cloud.points.size());
  cloud.height = 1;
  cloud.is_dense = true;
  pcl::toROSMsg(cloud, msg);
  return;
}

void OccGridMap::get_sdf_gradient_msg(std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>& msg){
  msg.clear();
  Eigen::Vector3d p_min = idx2pos(Eigen::Vector3i(offset_x + inflate_size, offset_y + inflate_size, 0));
  Eigen::Vector3d p_max = idx2pos(Eigen::Vector3i(offset_x + size_x - inflate_size, offset_y + size_y - inflate_size, 0));
  // std::cout << "offset: "<<offset_x<<", "<<offset_y<<", "<<offset_z<<std::endl;
  // std::cout << "size: "<<size_x<<", "<<size_y<<", "<<size_z<<std::endl;
  // std::cout << "inflate size: "<<inflate_size<<std::endl;
  // std::cout << "p_min: "<<p_min.transpose() <<", pmax: "<<p_max.transpose()<<std::endl;
  double max_dis = std::max(std::max(size_x, size_y), size_z) * resolution;
  Eigen::Vector3d pos(0, 0, 1);
  for (pos.x() = p_min.x(); pos.x() <= p_max.x(); pos.x() += resolution)
    for (pos.y() = p_min.y(); pos.y() <= p_max.y(); pos.y() += resolution) {
      double dist = getFineDistance(pos);
      if (abs(dist) > 2.0*max_dis){
        std::cout << "ERROR!!! pos: "<<pos.transpose() <<", dist: "<<dist<<std::endl;
      }
      // if (dist >= 0) {
        Eigen::Vector3d grad;
        getCostWithGrad(pos, grad);
        Eigen::Vector3d p1 = pos;
        Eigen::Vector3d p2 = pos + 0.5*resolution*grad.normalized();
        msg.emplace_back(p1, p2);
      // }
    }
}

void OccGridMap::occ2pc(sensor_msgs::PointCloud2& msg) {
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> pcd;
  for (int x = 0; x < size_x; ++x) {
    for (int y = 0; y < size_y; ++y) {
      for (int z = 0; z < size_z; ++z) {
        Eigen::Vector3i idx(offset_x + x, offset_y + y, offset_z + z);
        if (infocc.atId(idx) == 1) {
          pt.x = (offset_x + x + 0.5) * resolution;
          pt.y = (offset_y + y + 0.5) * resolution;
          pt.z = (offset_z + z + 0.5) * resolution;
          pcd.push_back(pt);
        }
      }
    }
  }
  pcd.width = pcd.points.size();
  pcd.height = 1;
  pcd.is_dense = true;
  pcl::toROSMsg(pcd, msg);
  msg.header.frame_id = "world";
}

void OccGridMap::occ2pc(sensor_msgs::PointCloud2& msg, double floor, double ceil) {
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> pcd;
  for (int x = 0; x < size_x; ++x) {
    for (int y = 0; y < size_y; ++y) {
      for (int z = 0; z < size_z; ++z) {
        Eigen::Vector3i idx(offset_x + x, offset_y + y, offset_z + z);
        if (infocc.atId(idx) == 1) {
          pt.x = (offset_x + x + 0.5) * resolution;
          pt.y = (offset_y + y + 0.5) * resolution;
          pt.z = (offset_z + z + 0.5) * resolution;
          if (pt.z > floor && pt.z < ceil) {
            pcd.push_back(pt);
          }
        }
      }
    }
  }
  pcd.width = pcd.points.size();
  pcd.height = 1;
  pcd.is_dense = true;
  pcl::toROSMsg(pcd, msg);
  msg.header.frame_id = "world";
}

void OccGridMap::inflate_once() {
  static Eigen::Vector3i p;
  for (const auto& id : v0) {
    int x0 = id.x() - 1 >= offset_x ? id.x() - 1 : offset_x;
    int y0 = id.y() - 1 >= offset_y ? id.y() - 1 : offset_y;
    int z0 = id.z() - 1 >= offset_z ? id.z() - 1 : offset_z;
    int x1 = id.x() + 1 <= offset_x + size_x - 1 ? id.x() + 1 : offset_x + size_x - 1;
    int y1 = id.y() + 1 <= offset_y + size_y - 1 ? id.y() + 1 : offset_y + size_y - 1;
    int z1 = id.z() + 1 <= offset_z + size_z - 1 ? id.z() + 1 : offset_z + size_z - 1;
    for (p.x() = x0; p.x() <= x1; p.x()++)
      for (p.y() = y0; p.y() <= y1; p.y()++)
        for (p.z() = z0; p.z() <= z1; p.z()++) {
          auto ptr = infocc.atIdPtr(p);
          if ((*ptr) != 1) {
            *ptr = 1;
            v1.push_back(p);
          }
        }
  }
}

void OccGridMap::inflate_xy() {
  static Eigen::Vector3i p;
  for (const auto& id : v0) {
    int x0 = id.x() - 1 >= offset_x ? id.x() - 1 : offset_x;
    int y0 = id.y() - 1 >= offset_y ? id.y() - 1 : offset_y;
    int x1 = id.x() + 1 <= offset_x + size_x - 1 ? id.x() + 1 : offset_x + size_x - 1;
    int y1 = id.y() + 1 <= offset_y + size_y - 1 ? id.y() + 1 : offset_y + size_y - 1;
    p.z() = id.z();
    for (p.x() = x0; p.x() <= x1; p.x()++)
      for (p.y() = y0; p.y() <= y1; p.y()++) {
        auto ptr = infocc.atIdPtr(p);
        if ((*ptr) != 1) {
          *ptr = 1;
          v1.push_back(p);
        }
      }
  }
}

void OccGridMap::inflate_last() {
  static Eigen::Vector3i p;
  for (const auto& id : v1) {
    int x0 = id.x() - 1 >= offset_x ? id.x() - 1 : offset_x;
    int y0 = id.y() - 1 >= offset_y ? id.y() - 1 : offset_y;
    int z0 = id.z() - 1 >= offset_z ? id.z() - 1 : offset_z;
    int x1 = id.x() + 1 <= offset_x + size_x - 1 ? id.x() + 1 : offset_x + size_x - 1;
    int y1 = id.y() + 1 <= offset_y + size_y - 1 ? id.y() + 1 : offset_y + size_y - 1;
    int z1 = id.z() + 1 <= offset_z + size_z - 1 ? id.z() + 1 : offset_z + size_z - 1;
    for (p.x() = x0; p.x() <= x1; p.x()++)
      for (p.y() = y0; p.y() <= y1; p.y()++)
        for (p.z() = z0; p.z() <= z1; p.z()++)
          infocc.atId(p) = 1;
  }
}

void OccGridMap::inflate(int inflate_size) {
  if (inflate_size < 1) {
    return;
  }
  Eigen::Vector3i idx;
  v1.clear();
  for (idx.x() = offset_x; idx.x() < offset_x + size_x; ++idx.x())
    for (idx.y() = offset_y; idx.y() < offset_y + size_y; ++idx.y())
      for (idx.z() = offset_z; idx.z() < offset_z + size_z; ++idx.z()) {
        if (infocc.atId(idx) == 1) {
          v1.push_back(idx);
        }
      }
  for (int i = 0; i < inflate_size - 1; ++i) {
    std::swap(v0, v1);
    v1.clear();
    inflate_once();
  }
  inflate_last();
}

}  // namespace mapping