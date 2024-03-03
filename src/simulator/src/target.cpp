/* @author: YueLin */

#include <cmath>
#include <cstdlib>

#include <ros/ros.h>

#include "geometry_msgs/PoseStamped.h"

#define RATE 10
#define pow2(n) (n)*(n)
#define rand10 ((rand() % 10) * 1e-2)
#define ROS_INIT ros::init(argc, argv, "fake_target")
#define distance(x1, y1, x2, y2) std::sqrt(pow2(x1 - x2) + pow2(y1 - y2))

typedef geometry_msgs::PoseStamped Target;

int main(int argc, char* argv[])
{
    ROS_INIT;
    Target target;
    ros::Time::init();
    float x, y, x0, y0;
    ros::Rate sleeper(RATE);
    ros::NodeHandle nh("~");
    target.header.frame_id = "odom";
    x = x0 = nh.param("initial_x", 0.7);
    y = y0 = nh.param("initial_y", 0.9);
    const float length = nh.param("velocity", 1.5) / RATE;
    ros::Publisher publisher = nh.advertise<Target>("/target", 1);
    ros::Subscriber subscriber = nh.subscribe<Target>(
        "/move_base_simple/goal", 1, [&x, &y](Target::ConstPtr pose)
        {
            x = pose->pose.position.x;
            y = pose->pose.position.y;
        }
    );
    ros::Timer publish = nh.createTimer(
        ros::Duration(1 / RATE), [&](const ros::TimerEvent& _)
        {
            float noise = 0,
            k = length / distance(x, y, x0, y0);
            if(k >= 1) {x0 = x; y0 = y;} else
            {
                noise = rand10;
                x0 = k * x + (1 - k) * x0;
                y0 = k * y + (1 - k) * y0;
            }
            target.pose.position.x = x0 + noise;
            target.pose.position.y = y0 + noise;
            publisher.publish(target);
            sleeper.sleep();
        }
    );
    return ros::spin(), 0;
}
