/* @author: YueLin */

#include <tf/tf.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>

#include "nav_msgs/Path.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PoseStamped.h"

#include "sf_tracker/control.hpp"

#define ROS_INIT(node) ros::init(argc, argv, node)

int main(int argc, char* argv[])
{
    /* Initialize */
    ROS_INIT("controller");
    ros::Time::init();
    ros::NodeHandle nh("~");

    /* Objects */
    sf_tracker::Controller ctrl;
    geometry_msgs::Twist velocity;
    tf::StampedTransform transform;
    tf::TransformListener listener;

    /* Variables */
    double time = 0;
    std::vector<double> knots;
    cv::Point2d position(0, 0);
    std::vector<double> orientation;
    std::vector<cv::Point2d> points;
    std::vector<cv::Point3d> trajectory;

    /* Hyper-parameters */
    ctrl.dt = nh.param("delta_t", 0.1);
    ctrl.max_vel.z = nh.param("max_omega", 1.5);
    ctrl.max_acc.z = nh.param("max_alpha", 1.0);
    ctrl.max_vel.x = ctrl.max_vel.y = nh.param("max_vel", 2.5);
    ctrl.max_acc.x = ctrl.max_acc.y = nh.param("max_acc", 1.5);

    /* Frames */
    std::string map_frame, odom_frame;
    if(!nh.getParam("map_frame", map_frame))
        ROS_ERROR("Missing parameter: map_frame");
    if(!nh.getParam("odom_frame", odom_frame))
        ROS_ERROR("Missing parameter: odom_frame");
    
    /* Wait tf */
    while(!listener.canTransform(odom_frame, map_frame, ros::Time(0)))
    {
        if(time > 10)
            ROS_WARN("Wait for transfrom.");
        listener.waitForTransform(
            odom_frame, map_frame, ros::Time(0), ros::Duration(10)
        );
        time += 10;
    }
    time = 0;

    /* Velocity publisher */
    ros::Publisher cmd_vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    /* Subscribe target's position */
    ros::Subscriber tracking = nh.subscribe<geometry_msgs::PoseStamped>(
        "/target", 1, [&listener, &transform, &odom_frame, &map_frame, &ctrl]
        (geometry_msgs::PoseStamped::ConstPtr pos)
        {
            listener.lookupTransform(
                odom_frame, map_frame, ros::Time(0), transform
            );
            double x = pos->pose.position.x - transform.getOrigin().x();
            double y = pos->pose.position.y - transform.getOrigin().y();
            double yaw = tf::getYaw(transform.getRotation());
            double sin = std::sin(yaw), cos = std::cos(yaw);
            ctrl.target.x = x * cos + y * sin;
            ctrl.target.y = y * cos - x * sin;
        }
    );

    /* Subscribe robot's position */
    ros::Subscriber localization = nh.subscribe<nav_msgs::OccupancyGrid>(
        "/costmap", 1, [&position](nav_msgs::OccupancyGrid::ConstPtr map)
        {
            position.x = 0.5 * map->info.width * map->info.resolution;
            position.y = 0.5 * map->info.height * map->info.resolution;
        }
    );

    /* Subscribe path */
    ros::Subscriber planning = nh.subscribe<nav_msgs::Path>(
        "/bspline", 1, [
            &time, &points, &orientation, &trajectory, &knots, &ctrl
        ](nav_msgs::Path::ConstPtr path)
        {
            const int N = path->poses.size();
            if(!N) {time = 0; knots.clear(); return;}
            time = ros::Time::now().toSec();
            trajectory.resize(N); points.resize(N); orientation.resize(N);
            for(int p = 0; p < N; p++)
            {
                points[p].x = path->poses[p].pose.position.x;
                points[p].y = path->poses[p].pose.position.y;
                orientation[p] = tf::getYaw(path->poses[p].pose.orientation);
            }
            knots = ctrl.adjustment(orientation, points);
            for(int p = 0; p < N; p++)
            {
                trajectory[p].x = points[p].x;
                trajectory[p].y = points[p].y;
                trajectory[p].z = orientation[p];
            }
        }
    );

    /* Main loop */
    ros::Timer control = nh.createTimer(
        ros::Duration(nh.param("control_interval", 0.02)),
        [&ctrl, &trajectory, &knots, &time, &position, &velocity, &cmd_vel]
        (const ros::TimerEvent&){
            cv::Point3d vel = ctrl.control(
                trajectory, knots, position,
                ros::Time::now().toSec() - time
            );
            velocity.angular.z = vel.z;
            velocity.linear.x = vel.x;
            velocity.linear.y = vel.y;
            cmd_vel.publish(velocity);
        }
    );

    return ros::spin(), 0;
}
