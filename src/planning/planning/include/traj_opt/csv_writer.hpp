#include <Eigen/Geometry>
#include <iostream>
#include <string>

#include <traj_opt/trajectory.hpp>
#include <visualization/camera_vis.hpp>

namespace csv_writer{
class CsvWriter{
 private:
    std::string file_name_; 
    double g_ = 9.8;
    Eigen::Vector3d car_p_, car_v_;
    double car_theta_, car_omega_;

    std::shared_ptr<visualization::CameraVis> camVisPtr_;

 public:
    CsvWriter(const std::string& file_name):file_name_(file_name){}

    inline void set_camvis_ptr(std::shared_ptr<visualization::CameraVis> camVisPtr){
        camVisPtr_ = camVisPtr;
    }

    inline void set_target_state(const Eigen::Vector3d& car_p,
                                 const Eigen::Vector3d& car_v,
                                 const double& car_theta,
                                 const double& car_omega)
    {
        car_p_ = car_p;
        car_v_ = car_v;
        car_theta_ = car_theta;
        car_omega_ = car_omega;
        if (car_omega_ > 0.5){
            car_omega_ = 0.5;
        }
    }

    static Eigen::MatrixXd f_DN(const Eigen::Vector3d& x) {
    double x_norm_2 = x.squaredNorm();
    return (Eigen::MatrixXd::Identity(3, 3) - x * x.transpose() / x_norm_2) / sqrt(x_norm_2);
    }

    static Eigen::Vector3d f_N(const Eigen::Vector3d& x) {
        return x.normalized();
    }

    // ref: Vehicle Trajectory Prediction based on Motion Model and Maneuver Recognition
    static void CYRV_model(const Eigen::Vector3d& p0, const Eigen::Vector3d& v0, const double& theta, const double& omega, const double& t,
                        Eigen::Vector3d& p1)
    {
        if (omega > 1e-2){
            double v = v0.head(2).dot(Eigen::Vector2d(cos(theta), sin(theta)));
            p1.x() = p0.x() + v / omega * sin(theta + omega*t) - v / omega *sin(theta);
            p1.y() = p0.y() - v / omega * cos(theta + omega*t) + v / omega *cos(theta);
            p1.z() = p0.z();
        }else{
            p1.x() = p0.x() + v0.x() * t;
            p1.y() = p0.y() + v0.y() * t;
            p1.z() = p0.z();// + v0.z() * t;
        }
    }

    void record_state(FILE *fp, const Trajectory<7> traj_land, const double t)
    {
        // slove orientation
        Eigen::Vector3d acc = traj_land.getAcc(t);
        double yaw = traj_land.getAngle(t);
        double yaw_dot = traj_land.getAngleRate(t);
        Eigen::Vector3d alpha = acc + g_ * Eigen::Vector3d(0,0,1);
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
        Eigen::Vector3d jerk = traj_land.getJer(t);
        Eigen::Vector3d thrust = acc + Eigen::Vector3d(0, 0, g_);  
        Eigen::Vector3d zb_dot = f_DN(thrust) * jerk;
        Eigen::Vector3d zb = f_N(thrust);
        double a = zb.x();
        double b = zb.y();
        double c = zb.z();
        double a_dot = zb_dot.x();
        double b_dot = zb_dot.y();
        double c_dot = zb_dot.z();
        double s_y = sin(yaw);
        double c_y = cos(yaw);
        double w_x = s_y*a_dot - c_y*b_dot - (a*s_y - b*c_y) * (c_dot/(c+1));
        double w_y = c_y*a_dot + s_y*b_dot - (a*c_y + b*s_y) * (c_dot/(c+1));
        double w_z = (b*a_dot - a*b_dot) / (1+c) + yaw_dot;

        // t
        fprintf(fp, "%e,", t);
        // p
        Eigen::Vector3d p = traj_land.getPos(t);
        for (int j = 0; j < 3; ++j)
            fprintf(fp, "%e,", p(j));
        // q
        fprintf(fp, "%e,", q.w());
        fprintf(fp, "%e,", q.x());
        fprintf(fp, "%e,", q.y());
        fprintf(fp, "%e,", q.z());
        // v
        Eigen::Vector3d v = traj_land.getVel(t);
        for (int j = 0; j < 3; ++j)
            fprintf(fp, "%e,", v(j));
        // w
        fprintf(fp, "%e,", w_x);
        fprintf(fp, "%e,", w_y);
        fprintf(fp, "%e,", w_z);
        // acc
        for (int j = 0; j < 3; ++j)
            fprintf(fp, "%e,", acc(j));
        // jerk
        for (int j = 0; j < 3; ++j)
            fprintf(fp, "%e,", jerk(j));
        // snap
        Eigen::Vector3d snap = traj_land.getSnap(t);
        for (int j = 0; j < 3; ++j)
            fprintf(fp, "%e,", snap(j));
        // u v
        Eigen::Vector3d car_p1;
        CYRV_model(car_p_, car_v_, car_theta_, car_omega_, t, car_p1);
        double u_cam = 0.0, v_cam = 0.0;
        camVisPtr_->is_tag_visible(p, q, car_p1, u_cam, v_cam);
        // std::cout << "p: " << p.transpose() << ", targetp: " << car_p1.transpose() << ", u: " << u_cam << ", v: " << v_cam << std::endl;
        fprintf(fp, "%e,", u_cam);
        fprintf(fp, "%e,", v_cam);


        fprintf(fp, "\n");
    }

    void exportToCSV(const Trajectory<7> traj_land, const double dt)
    {
        FILE *fp;
        fp = fopen((file_name_+".csv").c_str(), "w");
        fprintf(fp, "t,p_x,p_y,p_z,q_w,q_x,q_y,q_z,v_x,v_y,v_z,w_x,w_y,w_z,a_lin_x,a_lin_y,a_lin_z,jerk_x,jerk_y,jerk_z,snap_x,snap_y,snap_z,u,v\n");

        double t_all = traj_land.getTotalDuration();
        for (double t = 0.0; t <= t_all; t += dt){
            record_state(fp, traj_land, t);
        }
        record_state(fp, traj_land, t_all);

        fclose(fp);
        printf("Saved planning results in %s \n",(file_name_+".csv").c_str());
    }

};
} //namespace csv_writer