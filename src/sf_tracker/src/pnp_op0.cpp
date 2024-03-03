/* @author: YueLin */

#include <tf/tf.h>
#include <ros/ros.h>
#include <nlopt.hpp>
#include <Eigen/Eigen>
#include <tf/transform_listener.h>

#include "nav_msgs/Path.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"

#include "sf_tracker/pnp_op.hpp"

#define sgn(n) (n > 0? 1: -1)
#define eq(f1, f2) (std::fabs(f1 - f2) < 1e-6)

#define ROS_INIT ros::init(argc, argv, "pnp_op")

typedef nav_msgs::Path Path;
typedef geometry_msgs::Point Point;
typedef geometry_msgs::Twist Velocity;

namespace sf_tracker
{
    Vector OrientationPlanner::plan(double omega, double alpha)
    {
        Vector optimized,
        bsp = optimize(best);
        for(int i = 0; i < bsp.size() - 2; i++)
            optimized.push_back((
                bsp[i] + 4 * bsp[i + 1] + bsp[i + 2]
            ) / 6.0);
        return optimized;
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

    Vector OrientationPlanner::optimize(const Vector& angles)
    {
        int num, n = angles.size();
        if((num = n - 1) <= 0)
            return angles;
        Vector x(num);
        nlopt::opt optimizer(nlopt::LD_LBFGS, num);
        for(int i = 0; i < n; i++) p[i] = angles[i];
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
        OrientationPlanner* op = reinterpret_cast
        <OrientationPlanner*>(orientation_planner);
        Vector gradient(n + 1, 0), vel(n + 0), acc(n + -1), jerk(n + -2);
        for(int i = 0; i < n; i++)
            op->p[i + 1] = x[i];
        for(int i = 0; i < n + 0; i++)
            vel[i] = sub(op->p[i + 1], op->p[i]);
        for(int i = 0; i < n + -1; i++)
            acc[i] = vel[i + 1] - vel[i];
        for(int i = 0; i < n + -2; i++)
            jerk[i] = acc[i + 1] - acc[i];
        
        /* Feasibility Cost */
        double t = op->t; double t2 = 1 / pow2(t); double t4 = pow2(t2);
        for(int i = 0; i < n + 0; i++)
            if((cost = pow2(vel[i]) * t2 - pow2(op->max_omega)) > 0)
            {
                j += op->mu_f * cost;
                g = 2 * op->mu_f * vel[i] * t2;
                gradient[i] -= g; gradient[i + 1] += g;
            }
        for(int i = 0; i < n + -1; i++)
            if((cost = pow2(acc[i]) * t4 - pow2(op->max_alpha)) > 0)
            {
                j += op->mu_f * cost;
                g = 2 * op->mu_f * acc[i] * t4;
                gradient[i + 1] -= 2 * g;
                gradient[i + 2] += g;
                gradient[i] += g;
            }

        /* Smoothness Cost */
        double t6 = t2 * t4;
        for(int i = 0; i < n + -1; i++)
        {
            j += op->mu_s * pow2(acc[i]) * t4;
            g = 2 * op->mu_s * acc[i] * t4;
            gradient[i + 1] -= 2 * g;
            gradient[i + 2] += g;
            gradient[i] += g;
        }
        for(int i = 0; i < n + -2; i++)
        {
            j += op->mu_s * pow2(jerk[i]) * t6;
            g = 2 * op->mu_s * jerk[i] * t6;
            gradient[i + 1] += 3 * g;
            gradient[i + 2] -= 3 * g;
            gradient[i + 3] += g;
            gradient[i] -= g;
        }

        /* Visibility Cost */
        for(int i = 0; i < n + 1; i++)
        {
            // cost = op->best[i] - (
            //     op->p[i] + 4 * op->p[i + 1] + op->p[i + 2]
            // ) / 6;
            cost = op->best[i] - op->p[i];
            g = -std::sin(cost) * op->mu_v;
            j += (1 - std::cos(cost)) * op->mu_v;
            gradient[i] += g;
            // gradient[i + 2] += g / 6;
            // gradient[i + 1] += 2 * g / 3;
        }

        for(int i = 0; i < n; i++)
            grad[i] = gradient[i + 1];
        return j;
    }

    class VelocityController
    {
        public:
            Vector path[3];
            float dt, omega, alpha, vel;
            tf::TransformListener listener;
            std::string robot_frame, odom_frame;
            float max_vel, max_omega, max_alpha;
        
        private:
            bool lock;
            unsigned int p;
            Velocity velocity;
            tf::StampedTransform transform;
        
        private:
            void applyLock()
            {
                while(lock);
                lock = true;
            }

            void releaseLock()
            {
                lock = false;
            }

            void applyLockAndResetPointer(unsigned int pointer)
            {
                applyLock(); p = pointer;
            }

        public:
            VelocityController()
            {
                releaseLock();
                omega = alpha = 
                velocity.angular.x =
                velocity.angular.y =
                velocity.linear.z = 0;
            }

            void set()
            {
                applyLockAndResetPointer(0);
                for(int i = 0; i < 3; i++)
                    path[i].clear();
                releaseLock();
            }

            void set(const Path& trajectory, const Vector& angles)
            {
                int n = angles.size();
                applyLockAndResetPointer(0);
                for(int i = 0; i < 3; i++)
                    path[i].clear();
                for(int i = 0; i < n; i++)
                {
                    path[0].push_back(angles[i]);
                    path[1].push_back(trajectory.poses[i].pose.position.x);
                    path[2].push_back(trajectory.poses[i].pose.position.y);
                }
                releaseLock();
            }

            Velocity control(double xt, double yt)
            {
                applyLock();
                listener.lookupTransform(
                    odom_frame, robot_frame, ros::Time(0), transform
                );
                double t1 = 1 / dt,
                yaw = tf::getYaw(transform.getRotation());
                if(p + 1 < path->size())
                {
                    double
                    sin = t1 * std::sin(yaw),
                    cos = t1 * std::cos(yaw),
                    dx = path[1][p + 1] - path[1][p],
                    dy = path[2][p + 1] - path[2][p];
                    velocity.linear.x = cos * dx + sin * dy;
                    velocity.linear.y = cos * dy - sin * dx;
                    velocity.angular.z = t1 * sub(path[0][p + 1], path[0][p++]);
                }
                else
                {
                    double
                    x0 = transform.getOrigin().x(),
                    y0 = transform.getOrigin().y();
                    velocity.linear.x = velocity.linear.y = 0;
                    velocity.angular.z = max_omega * std::tanh(
                        sub(std::atan2(yt - y0, xt - x0), yaw)
                    );
                }
                if((vel = std::sqrt(pow2(velocity.linear.x) + 
                                    pow2(velocity.linear.y))) > max_vel)
                {
                    double k = max_vel / vel;
                    velocity.linear.x *= k;
                    velocity.linear.y *= k;
                }
                if(std::fabs(velocity.angular.z) > max_omega)
                    velocity.angular.z = max_omega * sgn(velocity.angular.z);
                alpha = t1 * (velocity.angular.z - omega);
                if(std::fabs(alpha) > max_alpha)
                {
                    alpha = max_alpha * sgn(alpha);
                    velocity.angular.z = omega + alpha * dt;
                }
                omega = velocity.angular.z;
                velocity.angular.z = 0;  /////////////////
                // velocity.linear.x = velocity.linear.y = 0;
                return releaseLock(), velocity;
            }
    };
}

int main(int argc, char* argv[])
{
    ROS_INIT;
    Path path;
    Velocity vel;
    float xt, yt;
    bool empty = true;
    ros::Time::init();
    ros::NodeHandle nh("~");
    sf_tracker::OrientationPlanner op;
    sf_tracker::VelocityController ctrl;
    ctrl.max_vel = nh.param("max_vel", 2.5);
    op.mu_v = nh.param("mu_visibility", 1e2);
    op.mu_s = nh.param("mu_smoothness", 1e-6);
    op.mu_f = nh.param("mu_feasibility", 1e-2);
    op.t = ctrl.dt = nh.param("delta_t", 1e-1);
    if(!nh.getParam("odom_frame", ctrl.odom_frame))
        ROS_ERROR("Missing parameter: odom_frame");
    if(!nh.getParam("robot_frame", ctrl.robot_frame))
        ROS_ERROR("Missing parameter: robot_frame");
    op.max_time = nh.param("solution_time_limit", 1e-2);
    int length = std::round(nh.param("distance", 0.5) / op.t);
    op.max_alpha = ctrl.max_alpha = nh.param("max_alpha", 1.5);
    op.max_omega = ctrl.max_omega = nh.param("max_omega", 2.0);
    ros::Publisher velocity = nh.advertise<Velocity>("/cmd_vel", 1);
    ros::Publisher trajectory = nh.advertise<Path>("/trajectory", 1);
    ctrl.listener.waitForTransform(
        ctrl.odom_frame, ctrl.robot_frame, ros::Time(0), ros::Duration(50)
    );
    ros::Subscriber planning = nh.subscribe<Path>(
        "/path", 1, [&](Path::ConstPtr points){
            path = *points;
            int n = path.poses.size();
            if(n)
            {
                path.poses.pop_back(); n--;
                xt = points->poses.back().pose.position.x;
                yt = points->poses.back().pose.position.y;
            }   n -= length;
            if(n > 3)
            {
                op.best.clear();
                op.best.push_back(0);
                path.poses.resize(n);
                for(int i = 1; i < n; i++)
                    op.best.push_back(std::atan2(
                        yt - path.poses[i].pose.position.y,
                        xt - path.poses[i].pose.position.x
                    ));
                ctrl.set(path, op.plan(ctrl.omega, ctrl.alpha));
            }
            else
            {
                ctrl.set();
                path.poses.clear();
            }
            if(!empty || path.poses.size())
                trajectory.publish(path);
            empty = path.poses.size() > 0;
        }
    );
    ros::Timer control = nh.createTimer(
        ros::Duration(ctrl.dt), [&](const ros::TimerEvent& _)
        {
            // if(x0 < 1e-6 && y0 < 1e-6) return;
            Velocity v = ctrl.control(xt, yt);
            if(!eq(vel.linear.x, v.linear.x) ||
               !eq(vel.linear.y, v.linear.y) ||
               !eq(vel.angular.z, v.angular.z))
                velocity.publish(vel = v);
        }
    );
    return ros::spin(), 0;
}
