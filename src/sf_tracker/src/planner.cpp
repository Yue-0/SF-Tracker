/* @author: YueLin */

#include <tf/tf.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>

#include "nav_msgs/Path.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Float32MultiArray.h"

#include "sf_tracker/trajectory.hpp"
#include "sf_tracker/orientation.hpp"

#define TIME ros::Time::now().toSec
#define ROS_INIT(node) ros::init(argc, argv, node)

namespace sf_tracker
{
    double TrajectoryPlanner::f(void* trajectory_planner, 
                                const Eigen::VectorXd& x, 
                                Eigen::VectorXd& gradient)
    {
        /* Get the TrajectoryPlanner object */
        TrajectoryPlanner*
        p = reinterpret_cast
        <TrajectoryPlanner*>
        (trajectory_planner);

        /* Initialize */
        double cost, g, j = 0.;
        const int X = 0, Y = 1;
        const int N = x.size() >> 1;
        std::vector<std::vector<double>> 
        vel(N - 1, std::vector<double>(2)),
        acc(N - 2, std::vector<double>(2)),
        jerk(N - 3, std::vector<double>(2)),
        grad(N, std::vector<double>(2, 0));

        /* Calculate velocity */
        for(int i = 0; i < N - 1; i++)
        {
            vel[i][X] = x((i + 1) * 2 + X) - x(i * 2 + X);
            vel[i][Y] = x((i + 1) * 2 + Y) - x(i * 2 + Y);
        }

        /* Calculate acceleration */
        for(int i = 0; i < N - 2; i++)
        {
            acc[i][X] = vel[i + 1][X] - vel[i][X];
            acc[i][Y] = vel[i + 1][Y] - vel[i][Y];
        }

        /* Calculate jerk */
        for(int i = 0; i < N - 3; i++)
        {
            jerk[i][X] = acc[i + 1][X] - acc[i][X];
            jerk[i][Y] = acc[i + 1][Y] - acc[i][Y];
        }

        /* Feasibility Cost */
        double t2 = 1 / pow2(p->t); double t4 = pow2(t2);
        double vm2 = pow2(p->max_vel), am2 = pow2(p->max_acc);
        for(int i = 0; i < N - 1; i++)
            for(int z = 0; z < 2; z++)
                if((cost = pow2(vel[i][z]) * t2 - vm2) > 0)
                {
                    j += p->lambda_f * cost;
                    g = 2 * p->lambda_f * vel[i][z] * t2;
                    grad[i][z] -= g; grad[i + 1][z] += g;
                }
        for(int i = 0; i < N - 2; i++)
            for(int z = 0; z < 2; z++)
                if((cost = pow2(acc[i][z]) * t4 - am2) > 0)
                {
                    j += p->lambda_f * cost;
                    g = 2 * p->lambda_f * acc[i][z] * t4;
                    grad[i + 1][z] -= 2 * g;
                    grad[i + 2][z] += g;
                    grad[i][z] += g;
                }

        /* Smoothness Cost */
        double t6 = t2 * t4;
        for(int i = 0; i < N - 2; i++)
            for(int z = 0; z < 2; z++)
            {
                j += p->lambda_s * pow2(acc[i][z]) * t4;
                g = 2 * p->lambda_s * acc[i][z] * t4;
                grad[i + 1][z] -= 2 * g;
                grad[i + 2][z] += g;
                grad[i][z] += g;
            }
        for(int i = 0; i < N - 3; i++)
            for(int z = 0; z < 2; z++)
            {
                j += p->lambda_s * pow2(jerk[i][z]) * t6;
                g = 2 * p->lambda_s * jerk[i][z] * t6;
                grad[i + 1][z] += 3 * g;
                grad[i + 2][z] -= 3 * g;
                grad[i + 3][z] += g;
                grad[i][z] -= g;
            }

        /* Safety Cost */
        cv::Mat& esdf = p->sdf;
        for(int i = 3; i < N - 3; i++)
        {
            float
            x0 = std::max(std::min(x(i * 2 + X) / p->scale, esdf.cols - 2.), O),
            y0 = std::max(std::min(x(i * 2 + Y) / p->scale, esdf.rows - 2.), O);

            /* Linear interpolation */
            int x1 = x0, y1 = y0;
            int x2 = x1 + 1, y2 = y1 + 1;
            float u = x0 - x1, v = y1 - y0;
            float u_ = 1. - u, v_ = 1. - v;
            double sqr = std::sqrt(
                + u * v * esdf.at<float>(y2, x2)
                + u * v_ * esdf.at<float>(y1, x2)
                + u_ * v * esdf.at<float>(y2, x1)
                + u_ * v_ * esdf.at<float>(y1, x1)
            );
            
            /* Calculate gradients of ESDF */
            if((cost = p->safe - sqr) > 0)
            {
                j += p->lambda_d * cost;
                g = 0.5 * p->lambda_d / sqr;
                grad[i][X] += g * (
                    + v * esdf.at<float>(y2, x1) + v_ * esdf.at<float>(y1, x1) 
                    - v * esdf.at<float>(y2, x2) - v_ * esdf.at<float>(y1, x2)
                );
                grad[i][Y] += g * (
                    + u * esdf.at<float>(y1, x2) + u_ * esdf.at<float>(y1, x1)
                    - u * esdf.at<float>(y2, x2) - u_ * esdf.at<float>(y2, x1)
                );
            }
        }

        /* Structural gradient */
        for(int i = 0; i < N; i++)
        {
            gradient(i * 2 + X) = grad[i][X];
            gradient(i * 2 + Y) = grad[i][Y];
        }
        for(int i = 0; i < 3; i++)
            gradient[i * 2 + X] = gradient[i * 2 + Y] = 0;
        for(int i = N - 4; i < N; i++)
            gradient[i * 2 + X] = gradient[i * 2 + Y] = 0;

        return j;
    }

    double OrientationPlanner::f(void* orientation_planner, 
                                 const Eigen::VectorXd& xi, 
                                 Eigen::VectorXd& gradient)
    {
        /* Get the OrientationPlanner object */
        OrientationPlanner*
        p = reinterpret_cast
        <OrientationPlanner*>
        (orientation_planner);

        /* Initialize */
        const int N = xi.size();
        double cost, g, j = 0.0;
        std::vector<double> vel(N - 1);
        std::vector<double> acc(N - 2);
        std::vector<double> jerk(N - 3);

        /* Clear gradient */
        for(int i = 0; i < N; i++)
            gradient(i) = 0;

        /* Calculate velocity */
        for(int i = 0; i < N - 1; i++)
            vel[i] = sub(xi(i + 1), xi(i));

        /* Calculate acceleration */
        for(int i = 0; i < N - 2; i++)
            acc[i] = vel[i + 1] - vel[i];

        /* Calculate jerk */
        for(int i = 0; i < N - 3; i++)
            jerk[i] = acc[i + 1] - acc[i];

        /* Feasibility Cost */
        double t2 = 1 / pow2(p->t); double t4 = pow2(t2);
        double vm2 = pow2(p->max_omega), am2 = pow2(p->max_alpha);
        for(int i = 0; i < N - 1; i++)
            if((cost = pow2(vel[i]) * t2 - vm2) > 0)
            {
                j += p->mu_f * cost;
                g = 2 * p->mu_f * vel[i] * t2;
                gradient(i) -= g; gradient(i + 1) += g;
            }
        for(int i = 0; i < N - 2; i++)
            if((cost = pow2(acc[i]) * t4 - am2) > 0)
            {
                j += p->mu_f * cost;
                g = 2 * p->mu_f * acc[i] * t4;
                gradient(i + 1) -= 2 * g;
                gradient(i + 2) += g;
                gradient(i) += g;
            }

        /* Smoothness Cost */
        double t6 = t2 * t4;
        for(int i = 0; i < N - 2; i++)
        {
            j += p->mu_s * pow2(acc[i]) * t4;
            g = 2 * p->mu_s * acc[i] * t4;
            gradient(i + 1) -= 2 * g;
            gradient(i + 2) += g;
            gradient(i) += g;
        }
        for(int i = 0; i < N - 3; i++)
        {
            j += p->mu_s * pow2(jerk[i]) * t6;
            g = 2 * p->mu_s * jerk[i] * t6;
            gradient(i + 1) += 3 * g;
            gradient(i + 2) -= 3 * g;
            gradient(i + 3) += g;
            gradient(i) -= g;
        }

        /* Visibility Cost */
        for(int i = 2; i < N - 1; i++)
        {
            cost = p->a[i] - (xi[i - 1] + 4 * xi[i] + xi[i + 1]) / 6;
            j += p->mu_v * (1 - std::cos(cost));
            g = -p->mu_v * std::sin(cost);
            gradient(i) += 4 * g;
            gradient(i - 1) += g;
            gradient(i + 1) += g;
        }

        /* Clear first 3 gradients */
        for(int i = 0; i < 3; i++)
            gradient(i) = 0;

        return j;
    }
}

int main(int argc, char* argv[])
{
    /* Initialize */
    ROS_INIT("planner");
    ros::Time::init();
    ros::NodeHandle nh("~");

    /* Variables */
    cv::Mat map;
    bool track = false;
    cv::Point2d target;
    cv::Point3d vel, acc;
    double scale, time = 0;
    cv::Point start(0, 0), end(0, 0);

    /* Objects */
    nav_msgs::Path path;
    tf::StampedTransform transform;
    tf::TransformListener listener;
    sf_tracker::TrajectoryPlanner planner1;
    sf_tracker::OrientationPlanner planner2;
    
    /* Frames */
    std::string map_frame, odom_frame;
    if(!nh.getParam("map_frame", map_frame))
        ROS_ERROR("Missing parameter: map_frame");
    if(!nh.getParam("odom_frame", odom_frame))
        ROS_ERROR("Missing parameter: odom_frame");
    path.header.frame_id = map_frame;

    /* Hyper-parameters of trajectory planner */
    planner1.t = nh.param("delta_t", 0.1);
    planner1.max_vel = nh.param("max_vel", 2.5);
    planner1.max_acc = nh.param("max_acc", 1.5);
    planner1.lambda_d = nh.param("lambda_safety", 1e2);
    planner1.lambda_s = nh.param("lambda_smoothness", 1e-6);
    planner1.lambda_f = nh.param("lambda_feasibility", 1e-2);
    planner1.max_time = nh.param("solution_time_limit", 1e-2);
    planner1.safe = std::sqrt(nh.param("safe_distance", 0.4));
    planner1.dist = nh.param("observation_distance", 0.5);

    /* Hyper-parameters of orientation planner */
    planner2.t = nh.param("delta_t", 0.1);
    planner2.mu_v = nh.param("mu_visibility", 1e2);
    planner2.mu_s = nh.param("mu_smoothness", 1e-2);
    planner2.mu_f = nh.param("mu_feasibility", 1e-1);
    planner2.max_alpha = nh.param("max_alpha", 1.0);
    planner2.max_omega = nh.param("max_omega", 1.5);
    planner2.max_time = nh.param("solution_time_limit", 1e-2);

    /* Wait tf */
    while(!listener.canTransform(odom_frame, map_frame, ros::Time(0)))
    {
        if(time >= 60) throw "Transfrom error!";
        listener.waitForTransform(
            odom_frame, map_frame, ros::Time(0), ros::Duration(10)
        );
        time += 10;
    }
    time = 0;

    /* Publishers */
    ros::Publisher publisher = nh.advertise<nav_msgs::Path>("/bspline", 1);
    ros::Publisher visualizer = nh.advertise<nav_msgs::Path>("/trajectory", 1);
    
    /* Subscribe target's position */
    ros::Subscriber tracking = nh.subscribe<geometry_msgs::PoseStamped>(
        "/target", 1, [&target, &track, &planner1]
        (geometry_msgs::PoseStamped::ConstPtr pos)
        {
            if(zero(planner1.scale)) return;
            target.x = pos->pose.position.x;
            target.y = pos->pose.position.y; track = true;
        }
    );

    /* Subscribe costmap */
    ros::Subscriber scan = nh.subscribe<nav_msgs::OccupancyGrid>(
        "/costmap", 1, [&map, &start, &planner1, &scale]
        (nav_msgs::OccupancyGrid::ConstPtr costmap)
        {
            const int H = costmap->info.height, W = costmap->info.width;
            planner1.scale = costmap->info.resolution;
            start.x = W >> 1; start.y = H >> 1;
            map = cv::Mat(H, W, CV_8UC1, 0xFF);
            scale = 1 / planner1.scale;
            for(int y = 0; y < H; y++)
                for(int x = 0; x < W; x++)
                    if(costmap->data[astar::encode(x, y, W)])
                        map.at<uchar>(y, x) = 0;
        }
    );

    /* Subscribe ESDF */
    ros::Subscriber esdf = nh.subscribe<std_msgs::Float32MultiArray>(
        "/esdf", 1, [&planner1](std_msgs::Float32MultiArray::ConstPtr array)
        {
            float value;
            const int W = array->layout.data_offset;
            const int H = array->data.size() / W;
            planner1.sdf = cv::Mat(cv::Size(W, H), CV_32FC1);
            for(int y = 0; y < H; y++)
                for(int x = 0; x < W; x++)
                    planner1.sdf.at<float>(y, x) = zero(
                        value = array->data[astar::encode(x, y, W)]
                    )? O: value * planner1.scale;
        }
    );

    /* Subscribe velocity */
    ros::Subscriber kinetic = nh.subscribe<geometry_msgs::Twist>(
        "/cmd_vel", 1, [&vel, &acc, &time]
        (geometry_msgs::Twist::ConstPtr cmd_vel)
        {
            if(zero(time))
            {
                time = TIME();
                vel.x = cmd_vel->linear.x;
                vel.y = cmd_vel->linear.y;
                vel.z = cmd_vel->angular.z;
            }
            else
            {
                double t = 1.0 / (ros::Time::now().toSec() - time);
                acc.z = t * (cmd_vel->angular.z - vel.z);
                acc.y = t * (cmd_vel->linear.y - vel.y);
                acc.x = t * (cmd_vel->linear.x - vel.x);
                vel.z = cmd_vel->angular.z;
                vel.y = cmd_vel->linear.y;
                vel.x = cmd_vel->linear.x;
                time = TIME();
            }
        }
    );

    /* Main loop */
    ros::Timer planning = nh.createTimer(
        ros::Duration(nh.param("replan_interval", 0.1)), [
            &track, &scale, &map, &path,
            &target, &start, &end, &vel, &acc, 
            &planner1, &planner2, &publisher, &visualizer,
            &listener, &transform, &odom_frame, &map_frame
        ](const ros::TimerEvent&){

            /* Wait for initialization */
            if(!track || !(start.x || start.y))
                return;
            
            /* Get target's position */
            listener.lookupTransform(
                odom_frame, map_frame, ros::Time(0), transform
            );
            double yaw = tf::getYaw(transform.getRotation());
            double sin = std::sin(yaw), cos = std::cos(yaw);
            double x = target.x - transform.getOrigin().x();
            double y = target.y - transform.getOrigin().y();
            
            /* Get navigation goal */
            target.x = x * cos + y * sin;
            target.y = y * cos - x * sin;
            cv::Point2d goal = target * scale;
            cv::Point2d ob = goal - cv::Point2d(start.x, start.y);
            goal -= scale * planner1.dist / (std::sqrt(ob.dot(ob)) + O) * ob;
            end.x = std::round(std::max(std::min(goal.x, map.rows - 1.), 0.));
            end.y = std::round(std::max(std::min(goal.y, map.cols - 1.), 0.));
            if(distance(start.x, start.y, end.x, end.y) < scale * planner1.dist)
            {
                path.poses.clear();
                path.header.stamp = ros::Time::now();
                visualizer.publish(path); publisher.publish(path); return;
            }

            /* Trajectory planning */
            std::vector<cv::Point2d> bspline2 = planner1.plan(
                map, planner1.search(map, start, end),
                vel.x, vel.y, acc.x, acc.y
            );
            std::vector<cv::Point2d> trajectory = bspline::trajectory(bspline2);

            /* Orientation planning */
            std::vector<double> bspline1 = planner2.plan(
                trajectory, target, vel.z, acc.z
            );
            std::vector<double> angles = bspline::trajectory(bspline1);

            /* Publish trajectory */
            path.poses.clear();
            int length = trajectory.size();
            path.header.stamp = ros::Time::now();
            for(int point = 0; point < length; point++)
            {
                geometry_msgs::PoseStamped pose;
                pose.header.stamp = path.header.stamp;
                pose.pose.position.x = trajectory[point].x;
                pose.pose.position.y = trajectory[point].y;
                pose.header.frame_id = path.header.frame_id;
                pose.pose.orientation = tf::createQuaternionMsgFromYaw(
                    angles[point]
                );
                path.poses.push_back(pose);
            }
            visualizer.publish(path);
            
            /* Publish B-splines */
            path.poses.clear();
            length = bspline2.size();
            for(int point = 0; point < length; point++)
            {
                geometry_msgs::PoseStamped pose;
                pose.header.stamp = path.header.stamp;
                pose.pose.position.x = bspline2[point].x;
                pose.pose.position.y = bspline2[point].y;
                pose.header.frame_id = path.header.frame_id;
                pose.pose.orientation = tf::createQuaternionMsgFromYaw(
                    bspline1[point]
                );
                path.poses.push_back(pose);
            }
            publisher.publish(path);
        }
    );

    return ros::spin(), 0;
}
