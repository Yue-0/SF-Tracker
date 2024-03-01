/* @author: YueLin */

#include <ros/ros.h>

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

#define ROS_INIT ros::init(argc, argv, "fake_target")

typedef geometry_msgs::PoseStamped Target;
typedef geometry_msgs::PoseWithCovarianceStamped Pose;

int main(int argc, char* argv[])
{
    ROS_INIT;
    Target target;
    ros::NodeHandle nh("~");
    ros::Publisher publisher = nh.advertise<Target>("/target", 1);
    ros::Subscriber subscriber = nh.subscribe<Pose>(
        "/initialpose", 1,
        [&publisher, &target]
        (Pose::ConstPtr pose)
        {
            target.pose = pose->pose.pose;
            target.header = pose->header;
            publisher.publish(target);
        }
    );
    return ros::spin(), 0;
}
