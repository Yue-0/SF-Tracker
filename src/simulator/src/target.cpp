/* @author: YueLin */

#include <cmath>

#include <ros/ros.h>

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

#define ROS_INIT(node) ros::init(argc, argv, node)

int main(int argc, char* argv[])
{
    /* Initialize */
    ROS_INIT("target");
    ros::NodeHandle nh("~");

    /* Target's position */
    double x = nh.param("initial_x", 0.7);
    double y = nh.param("initial_y", 0.9);

    /* Message */
    geometry_msgs::PoseStamped target;
    target.header.frame_id = "odom";
    target.pose.orientation.z = 1.0;
    target.pose.position.x = x;
    target.pose.position.y = y;
    
    /* Hyper-parameter */
    const double RATE = 10;
    const double LENGTH = nh.param("velocity", 1.5) / RATE;

    /* Publisher */
    ros::Publisher pos = nh.advertise<geometry_msgs::PoseStamped>("/target", 1);
    
    /* Subscriber */
    ros::Subscriber p = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
        "/initialpose", 1, [&x, &y]
        (geometry_msgs::PoseWithCovarianceStamped::ConstPtr pose)
        {
            x = pose->pose.pose.position.x;
            y = pose->pose.pose.position.y;
        }
    );

    /* Publish target's position */
    ros::Timer publisher = nh.createTimer(
        ros::Duration(1 / RATE), [
            &LENGTH, &x, &y, &target, &pos
        ](const ros::TimerEvent&)
        {
            double k = LENGTH / std::sqrt(
                + (x - target.pose.position.x) 
                * (x - target.pose.position.x)
                + (y - target.pose.position.y) 
                * (y - target.pose.position.y)
            );
            if(k >= 1)
            {
                target.pose.position.x = x;
                target.pose.position.y = y;
            }
            else
            {
                double k_ = 1 - k;
                target.pose.position.x = k * x + k_ * target.pose.position.x;
                target.pose.position.y = k * y + k_ * target.pose.position.y;
            }
            pos.publish(target);
        }
    );

    return ros::spin(), 0;
}
