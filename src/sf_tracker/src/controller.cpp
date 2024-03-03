/* @author: YueLin */

#include <cmath>
#include <vector>

#include <nlopt.hpp>
#include <ros/ros.h>

#include "nav_msgs/Path.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PoseStamped.h"

#define pow2(n) (n)*(n)

#define ROS_INIT ros::init(argc, argv, "controller")

typedef nav_msgs::Path Path;
typedef geometry_msgs::Twist Velocity;
typedef std::vector<geometry_msgs::PoseStamped> Points;

namespace sf_tracker
{
    typedef std::vector<double> Vector;

    const double PI = std::acos(-1);

    double loss(const Vector&, Vector&, void*);

    class Controller
    {
        public:
            Path path;
            int p = 0;
            double angle[0x400];
            double lambda1, lambda2, lambda3;
            double max_vel, max_acc, vel = 0;
            float max_time, dt, xt, yt;
        
        private:
            Velocity velocity;

        public:
            Controller()
            {
                velocity.angular.x =
                velocity.angular.y =
                velocity.linear.z = 0;
            }
            void optimize();
            Velocity control(float, float);
            double sub(double, double);

        private:
            double clip(double*);
    };

    Velocity Controller::control(float x, float y)
    {
        Points points = path.poses;
        const int n = points.size() - 1;
        if(p >= n)
        {
            velocity.linear.x =
            velocity.linear.y =
            velocity.angular.z =
            vel = 0; return velocity;
        }
        if(n * dt < 1)
        {
            velocity.angular.z = max_vel * std::tanh(
                std::atan2(yt - y, xt - x)
            );
            velocity.linear.x = velocity.linear.y = 0;
            vel = clip(&velocity.angular.z); p++;
            return velocity;
        }
        double t1 = 1 / dt,
        theta = angle[p + 1],
        dx = (points[p + 1].pose.position.x -
              points[p + 0].pose.position.x),
        dy = (points[p + 1].pose.position.y -
              points[p + 0].pose.position.y);
        velocity.angular.z = theta * t1;
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
        }   p++;
        vel = clip(&velocity.angular.z);
        return velocity;
    }

    void Controller::optimize()
    {
        int n = path.poses.size() - 1;
        angle[0] = 0;
        if(n * dt < 1)
            return;
        Vector var(n - 1);
        for(int p = 1; p <= n; p++)
        {
            angle[p] = std::atan2(
                yt - path.poses[p].pose.position.y,
                xt - path.poses[p].pose.position.x
            );
            if(p < n) var[p - 1] = angle[p];
        }
        nlopt::opt optimizer(nlopt::LD_LBFGS, --n);
        double cst = std::numeric_limits<double>::max();
        try
        {
            optimizer.set_min_objective(loss, this);
            optimizer.set_upper_bounds(Vector(n, PI));
            optimizer.set_lower_bounds(Vector(n, -PI));
            optimizer.set_maxtime(max_time);
            optimizer.set_xtol_rel(1e-7);
            optimizer.optimize(var, cst);
        }
        catch(std::exception& _){}
        for(int p = 1; p < n; p++)
            angle[p] = var[p - 1];
    }

    double Controller::sub(double rad1, double rad2)
    {
        double delta = rad1 - rad2;
        while(delta > PI) delta -= 2 * PI;
        while(delta <= -PI) delta += 2 * PI;
        return delta;
    }

    double Controller::clip(double* w)
    {
        double a = (*w - vel) / dt;
        if(std::fabs(a) > max_acc)*
            w = vel + max_acc * dt * (a > 0? 1: -1);
        return *w;
    }

    double loss(const Vector& x, Vector& grad, void* ptr)
    {
        double cost, g, j = 0.0;
        const unsigned int n = x.size();
        Controller* ctrl = reinterpret_cast<Controller*>(ptr);
        Vector gradient(n + 2, 0), vel(n + 1, 0), acc(n), jerk(n - 1);
        vel[0] = ctrl->sub(x[0], ctrl->angle[0]);
        for(int i = 1; i < n; i++)
            vel[i] = ctrl->sub(x[i], x[i - 1]);
        for(int i = 0; i < n; i++)
            acc[i] = vel[i + 1] - vel[i];
        for(int i = 0; i < n - 1; i++)
            jerk[i] = acc[i + 1] - acc[i];
        /* Feasibility Cost */
        double vm2 = pow2(ctrl->max_vel), am2 = pow2(ctrl->max_acc);
        double t = ctrl->dt; double t2 = 1 / t / t; double t4 = pow2(t2);
        for(int i = 0; i < n; i++)
            if((cost = pow2(vel[i]) * t2 - vm2) > 0)
            {
                j += ctrl->lambda1 * cost;
                g = 2 * ctrl->lambda1 * vel[i] * t2;
                gradient[i] -= g; gradient[i + 1] += g;
            }
        for(int i = 0; i < n - 1; i++)
            if((cost = pow2(acc[i]) * t4 - am2) > 0)
            {
                g = 2 * ctrl->lambda1 * acc[i] * t4;
                j += ctrl->lambda1 * cost;
                gradient[i + 1] -= 2 * g;
                gradient[i + 2] += g;
                gradient[i] += g;
            }
        /* Smoothness Cost */
        double t6 = t2 * t4;
        for(int i = 0; i < n; i++)
        {
            j += ctrl->lambda2 * pow2(acc[i]) * t4;
            g = 2 * ctrl->lambda2 * acc[i] * t4;
            gradient[i + 1] -= 2 * g;
            gradient[i + 2] += g;
            gradient[i] += g;
        }
        for(int i = 0; i < n - 1; i++)
        {
            g = 2 * ctrl->lambda2 * jerk[i] * t6;
            j += ctrl->lambda2 * pow2(jerk[i]) * t6;
            gradient[i] -= g; gradient[i + 1] += 3 * g;
            gradient[i + 2] -= 3 * g; gradient[i + 3] += g; 
        }
        /* Angle Cost */
        for(int i = 0; i < n; i++)
        {
            cost = ctrl->angle[i + 1] - x[i];
            grad[i] = -ctrl->lambda3 * std::sin(cost);
            j += (1 - std::cos(cost)) * ctrl->lambda3;
        }
        /* Structural Gradient */
        for(int i = 0; i < n; i++)
            grad[i] += gradient[i + 1];
        return j;
    }
}

int main(int argc, char* argv[])
{
    ROS_INIT;
    float x0, y0;
    bool lock = false;
    ros::Time::init();
    ros::NodeHandle nh("~");
    sf_tracker::Controller controller;
    controller.dt = nh.param("delta_t", 1e-1);
    controller.max_vel = nh.param("max_vel_omiga", 1.5);
    controller.max_acc = nh.param("max_acc_omiga", 1.0);
    controller.lambda3 = nh.param("lambda_angle", 1.00);
    controller.lambda2 = nh.param("lambda_smoothness", 0.10);
    controller.lambda1 = nh.param("lambda_feasibility", 1.00);
    controller.max_time = nh.param("solution_time_limit", 1e-2);
    ros::Publisher publisher = nh.advertise<Velocity>("/cmd_vel", 1);
    ros::Publisher trajectory = nh.advertise<Path>("/trajectory", 1);
    ros::Subscriber subscriber = nh.subscribe<Path>(
        "/path", 1, [&](Path::ConstPtr points){
            while(lock);
            lock = true;
            Path path;
            path.header = points->header;
            controller.p = 0;
            controller.path = *points;
            controller.xt = points->poses.back().pose.position.x;
            controller.yt = points->poses.back().pose.position.y;
            int n = controller.path.poses.size() - 2;
            controller.path.poses.pop_back();
            trajectory.publish(n * controller.dt < 1? path: controller.path);
            controller.optimize();
            lock = false;
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
        ros::Duration(controller.dt),
        [&](const ros::TimerEvent& _){
            while(lock);
            lock = true;
            publisher.publish(
                controller.control(x0, y0)
            );
            lock = false;
        }
    );
    ros::spin(); return 0;
}
