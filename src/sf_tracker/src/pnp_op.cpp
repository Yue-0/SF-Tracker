/* @author: YueLin */

#include <ros/ros.h>
#include <nlopt.hpp>
#include <Eigen/Eigen>

#include "nav_msgs/Path.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PoseStamped.h"

#include "sf_tracker/pnp_op.hpp"

#define ROS_INIT ros::init(argc, argv, "pnp_op")

typedef nav_msgs::Path Path;
typedef geometry_msgs::Twist Velocity;

double jv, js, jf;

namespace sf_tracker
{
    Vector OrientationPlanner::plan(const Vector& angles, 
                                    double omega, double alpha)
    {
        // Vector optimized(1, 0), b = optimize(bspline(
        //     samples(angles, omega), omega, alpha
        // ));
        // for(int i = 1; i < b.size() - 2; i++)
        //     optimized.push_back((
        //         b[i] + 4 * b[i + 1] + b[i + 2]
        //     ) / 6.0);
        // return optimized;
        Vector optimized(1, 0), b = optimize(angles);
        std::cout << "jf: " << jf
                  << "js: " << js
                  << "jv: " << jv << std::endl;
        return b;
    }

    Vector OrientationPlanner::bspline(const Vector& angles,
                                       double omega, double alpha)
    {
        int n = angles.size();
        Vector control(n + 2);
        Eigen::VectorXd yaw(n + 4);
        Eigen::Vector3d y(3), w(3), a(3);
        Eigen::MatrixXd matrix = Eigen::MatrixXd::Zero(n + 4, n + 2);
        y << 1, 4, 1; w << -1, 0, 1; a << 1, -2, 1;
        for(int i = 0; i < n; i++)
        {
            yaw(i) = angles[i];
            matrix.block(i, i, 1, 3) = (1.0 / 6) * y.transpose();
        }
        matrix.block(n, 0, 1, 3) = (0.5 / t) * w.transpose();
        matrix.block(n + 2, 0, 1, 3) = (1 / t / t) * a.transpose();
        matrix.block(n + 1, n - 1, 1, 3) = (0.5 / t) * w.transpose();
        matrix.block(n + 3, n - 1, 1, 3) = (1 / t / t) * a.transpose();
        yaw(n) = omega; yaw(n + 2) = alpha; yaw(n + 1) = yaw(n + 3) = 0;
        Eigen::VectorXd angle = matrix.colPivHouseholderQr().solve(yaw);
        for(int i = 0; i < n + 2; i++) control[i] = angle(i); return control;
    }

    Vector OrientationPlanner::samples(const Vector& orientations, double w0)
    {
        double w;
        const float t1 = 1 / t;
        Vector angles(orientations);
        const int n = orientations.size();
        for(int i = 1; i < n; i++)
        {
            w = t1 * sub(orientations[i], angles[i - 1]);
            CLIP(w, w0, t, max_alpha); w0 = w;
            angles[i] = clip(angles[i - 1] + w * t);
        }
        return angles;
    }

    Vector OrientationPlanner::optimize(const Vector& angles)
    {
        int num, n = angles.size();
        if(n < 4) return angles;
        Vector x(num = n - 1);
        best.assign(angles.begin(), angles.end());
        nlopt::opt optimizer(nlopt::LD_LBFGS, num);
        for(int i = 1; i < n; i++) p[i] = angles[i];
        for(int i = 0; i < num; i++) x[i] = p[i + 1];
        double cost = std::numeric_limits<double>::max();
        try
        {
            optimizer.set_min_objective(objective, this);
            optimizer.set_lower_bounds(Vector(num, -PI));
            optimizer.set_upper_bounds(Vector(num, PI));
            optimizer.set_maxtime(max_time);
            optimizer.set_xtol_rel(1e-7);
            optimizer.optimize(x, cost);
        }
        catch(std::exception& _){}
        Vector orientation(angles);
        for(int i = 0; i < num; i++)
            orientation[i + 1] = x[i];
        return orientation;
    }

    double objective(const Vector& x, Vector& grad, void* orientation_planner)
    {
        double cost, g, j = 0.0;
        const unsigned int n = x.size();
        OrientationPlanner* op = reinterpret_cast<
        OrientationPlanner*>(orientation_planner);
        Vector gradient(n + 1, 0), vel(n + 0), acc(n + (-1)), jerk(n + (-2));
        for(int i = 0; i < n; i++)
            op->p[i + 1] = x[i];
        for(int i = 0; i < n + 0; i++)
            vel[i] = sub(op->p[i + 1], op->p[i]);
        for(int i = 0; i < n + (-1); i++)
            acc[i] = vel[i + 1] - vel[i];
        for(int i = 0; i < n + (-2); i++)
            jerk[i] = acc[i + 1] - acc[i];
        
        /* Feasibility Cost */
        double t = op->t; double t2 = 1 / t / t; double t4 = pow2(t2);
        for(int i = 0; i < n + 0; i++)
            if((cost = pow2(vel[i]) * t2 - pow2(op->max_omega)) > 0)
            {
                j += op->mu_f * cost;
                g = 2 * op->mu_f * vel[i] * t2;
                gradient[i] -= g; gradient[i + 1] += g;
            }
        for(int i = 0; i < n + (-1); i++)
            if((cost = pow2(acc[i]) * t4 - pow2(op->max_alpha)) > 0)
            {
                j += op->mu_f * cost;
                g = 2 * op->mu_f * acc[i] * t4;
                gradient[i + 1] -= 2 * g;
                gradient[i + 2] += g;
                gradient[i] += g;
            }
        jf = j;

        /* Smoothness Cost */
        double t6 = t2 * t4;
        for(int i = 0; i < n + (-1); i++)
        {
            j += op->mu_s * pow2(acc[i]) * t4;
            g = 2 * op->mu_s * acc[i] * t4;
            gradient[i + 1] -= 2 * g;
            gradient[i + 2] += g;
            gradient[i] += g;
        }
        for(int i = 0; i < n + (-2); i++)
        {
            j += op->mu_s * pow2(jerk[i]) * t6;
            g = 2 * op->mu_s * jerk[i] * t6;
            gradient[i + 1] += 3 * g;
            gradient[i + 2] -= 3 * g;
            gradient[i + 3] += g;
            gradient[i] -= g;
        }
        js = j - jf;

        /* Visibility Cost */
        for(int i = 1; i < n + (-1); i++)
        {
            cost = op->best[i] - (
                op->p[i] + 4 * op->p[i + 1] + op->p[i + 2]
            ) / 6;
            g = -op->mu_v * std::sin(cost) / 6;
            j += op->mu_v * (1 - std::cos(cost));
            gradient[i + 1] += 4 * g;
            gradient[i + 2] += g;
            gradient[i] += g;
        }
        jv = j - jf - js;

        for(int i = 0; i < n; i++)
            grad[i] = gradient[i + 1];
        return j;
    }

    // Vector OrientationPlanner::optimize(const Vector& angles)
    // {
    //     int num, n = angles.size();
    //     if(n < 7) return angles;
    //     Vector x(num = n - 6);
    //     best.assign(angles.begin(), angles.end());
    //     nlopt::opt optimizer(nlopt::LD_LBFGS, num);
    //     for(int i = 1; i < n; i++) p[i] = angles[i];
    //     for(int i = 0; i < num; i++) x[i] = p[i + 3];
    //     double cost = std::numeric_limits<double>::max();
    //     try
    //     {
    //         optimizer.set_min_objective(objective, this);
    //         optimizer.set_lower_bounds(Vector(num, -PI));
    //         optimizer.set_upper_bounds(Vector(num, PI));
    //         optimizer.set_maxtime(max_time);
    //         optimizer.set_xtol_rel(1e-7);
    //         optimizer.optimize(x, cost);
    //     }
    //     catch(std::exception& _){}
    //     Vector orientation(angles);
    //     for(int i = 0; i < num; i++)
    //         orientation[i + 3] = x[i];
    //     return orientation;
    // }

    // double objective(const Vector& x, Vector& grad, void* orientation_planner)
    // {
    //     double cost, g, j = 0.0;
    //     const unsigned int n = x.size();
    //     OrientationPlanner* op = reinterpret_cast<
    //     OrientationPlanner*>(orientation_planner);
    //     Vector gradient(n + 6, 0), vel(n + 5), acc(n + 4), jerk(n + 3);
    //     for(int i = 0; i < n; i++)
    //         op->p[i + 3] = x[i];
    //     for(int i = 0; i < n + 5; i++)
    //         vel[i] = sub(op->p[i + 1], op->p[i]);
    //     for(int i = 0; i < n + 4; i++)
    //         acc[i] = vel[i + 1] - vel[i];
    //     for(int i = 0; i < n + 3; i++)
    //         jerk[i] = acc[i + 1] - acc[i];
        
    //     /* Feasibility Cost */
    //     double t = op->t; double t2 = 1 / t / t; double t4 = pow2(t2);
    //     for(int i = 0; i < n + 5; i++)
    //         if((cost = pow2(vel[i]) * t2 - pow2(op->max_omega)) > 0)
    //         {
    //             j += op->mu_f * cost;
    //             g = 2 * op->mu_f * vel[i] * t2;
    //             gradient[i] -= g; gradient[i + 1] += g;
    //         }
    //     for(int i = 0; i < n + 4; i++)
    //         if((cost = pow2(acc[i]) * t4 - pow2(op->max_alpha)) > 0)
    //         {
    //             j += op->mu_f * cost;
    //             g = 2 * op->mu_f * acc[i] * t4;
    //             gradient[i + 1] -= 2 * g;
    //             gradient[i + 2] += g;
    //             gradient[i] += g;
    //         }
    //     jf = j;

    //     /* Smoothness Cost */
    //     double t6 = t2 * t4;
    //     for(int i = 0; i < n + 4; i++)
    //     {
    //         j += op->mu_s * pow2(acc[i]) * t4;
    //         g = 2 * op->mu_s * acc[i] * t4;
    //         gradient[i + 1] -= 2 * g;
    //         gradient[i + 2] += g;
    //         gradient[i] += g;
    //     }
    //     for(int i = 0; i < n + 3; i++)
    //     {
    //         j += op->mu_s * pow2(jerk[i]) * t6;
    //         g = 2 * op->mu_s * jerk[i] * t6;
    //         gradient[i + 1] += 3 * g;
    //         gradient[i + 2] -= 3 * g;
    //         gradient[i + 3] += g;
    //         gradient[i] -= g;
    //     }
    //     js = j - jf;

    //     /* Visibility Cost */
    //     for(int i = 1; i < n + 4; i++)
    //     {
    //         cost = op->best[i] - (
    //             op->p[i] + 4 * op->p[i + 1] + op->p[i + 2]
    //         ) / 6;
    //         g = -op->mu_v * std::sin(cost) / 6;
    //         j += op->mu_v * (1 - std::cos(cost));
    //         gradient[i + 1] += 4 * g;
    //         gradient[i + 2] += g;
    //         gradient[i] += g;
    //     }
    //     jv = j - jf - js;

    //     for(int i = 0; i < n; i++)
    //         grad[i] = gradient[i + 3];
    //     return j;
    // }

    class VelocityController
    {
        public:
            Path path;
            double angles[0x400];
            float dis, dt, xt, yt;
            double max_omega, max_alpha, omega = 0, alpha = 0;
        
        private:
            int p = 0;
            bool lock = false;
            Velocity velocity;
        
        public:
            void releaseLock() {lock = false;}
            void applyForLock() {while(lock); lock = true;}
            void applyForLockAndResetPointer() {applyForLock(); p = 0;}

        public:
            VelocityController()
            {
                velocity.angular.x =
                velocity.angular.y =
                velocity.linear.z = 0;
            }
        
            Velocity control(float x0, float y0)
            {
                const int n = path.poses.size() - 1;
                if(p >= n)
                {
                    velocity.linear.x =
                    velocity.linear.y =
                    velocity.angular.z =
                    omega = 0; return velocity;
                }
                if(n * dt < dis)
                {
                    velocity.angular.z = max_omega * std::tanh(
                        std::atan2(yt - y0, xt - x0)
                    );
                    velocity.linear.x = velocity.linear.y = 0; return_:
                    CLIP(velocity.angular.z, omega, dt, max_alpha);
                    alpha = (velocity.angular.z - omega) / dt;
                    omega = velocity.angular.z;
                    return ++p, velocity;
                }
                double t1 = 1 / dt,
                theta = angles[p + 1],
                dx = (path.poses[p + 1].pose.position.x -
                      path.poses[p + 0].pose.position.x),
                dy = (path.poses[p + 1].pose.position.y -
                      path.poses[p + 0].pose.position.y);
                if(std::fabs(velocity.angular.z = theta * t1) > max_omega)
                    velocity.angular.z = max_omega * sign(theta);
                if(std::fabs(velocity.angular.z) < 1e-2)
                {
                    velocity.angular.z = 0;
                    velocity.linear.x = dx * t1;
                    velocity.linear.y = dy * t1;
                }
                else
                {
                    double z = 1.0 / velocity.angular.z;
                    double i1 = std::sin(theta) * z,
                    i2 = (1 - std::cos(theta)) * z;
                    double i = 1.0 / (pow2(i2) + pow2(i1));
                    velocity.linear.x = (dx * i1 + dy * i2) * i;
                    velocity.linear.y = (dy * i1 - dx * i2) * i;
                }
                goto return_;
            }
    };
}

int main(int argc, char* argv[])
{
    ROS_INIT;
    float x0, y0;
    ros::Time::init();
    ros::NodeHandle nh("~");
    sf_tracker::OrientationPlanner op;
    sf_tracker::VelocityController vc;
    vc.dt = op.t = nh.param("delta_t", 0.1);
    vc.dis = nh.param("observation_dis", 1);
    op.mu_v = nh.param("mu_visibility", 1e2);
    op.mu_s = nh.param("mu_smoothness", 1e-2);
    op.mu_f = nh.param("mu_feasibility", 1e-1);
    op.max_time = nh.param("solution_time_limit", 1e-2);
    op.max_alpha = vc.max_alpha = nh.param("max_alpha", 1.0);
    op.max_omega = vc.max_omega = nh.param("max_omega", 1.5);
    ros::Publisher velocity = nh.advertise<Velocity>("/cmd_vel", 1);
    ros::Publisher trajectory = nh.advertise<Path>("/trajectory", 1);
    ros::Subscriber subscriber = nh.subscribe<Path>(
        "/path", 1, [&](Path::ConstPtr points){
            int a = 0;
            vc.applyForLockAndResetPointer();
            Path path; path.header = points->header;
            vc.xt = points->poses.back().pose.position.x;
            vc.yt = points->poses.back().pose.position.y;
            vc.path = *points; vc.path.poses.pop_back();
            const int n = vc.path.poses.size() - 1;
            trajectory.publish(
                n * vc.dt < vc.dis? path: vc.path
            );
            sf_tracker::Vector orientations(1, 0);
            for(int i = 1; i <= n; i++)
                orientations.push_back(std::atan2(
                    vc.yt - points->poses[i].pose.position.y,
                    vc.xt - points->poses[i].pose.position.x
                ));
            for(double angle: op.plan(orientations, vc.omega, vc.alpha))
                vc.angles[a++] = angle;
            vc.releaseLock();
        }
    );
    ros::Subscriber map = nh.subscribe<nav_msgs::OccupancyGrid>(
        "/costmap", 1, [&x0, &y0]
        (nav_msgs::OccupancyGrid::ConstPtr og)
        {
            x0 = og->info.resolution * (og->info.width >> 1);
            y0 = og->info.resolution * (og->info.height >> 1);
        }
    );
    ros::Timer control = nh.createTimer(
        ros::Duration(vc.dt),
        [&](const ros::TimerEvent& _){
            vc.applyForLock(); velocity.publish
            (vc.control(x0, y0)); vc.releaseLock();
        }
    );
    return ros::spin(), 0;
}
