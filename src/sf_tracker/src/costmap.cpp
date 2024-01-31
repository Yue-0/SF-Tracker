/* @author: YueLin */

#include <cmath>

#include <tf/tf.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>

#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"

#include "sf_tracker/bfs.hpp"

#define ROS_INIT ros::init(argc, argv, "costmap")

#define swap(a, b) {a += b; b = a - b; a -= b;}

typedef sensor_msgs::LaserScan Scan;
typedef nav_msgs::OccupancyGrid Map;

int main(int argc, char* argv[])
{
    ROS_INIT;
    Scan lidar;
    Map costmap;
    cv::Mat sdf;
    ros::Time::init();
    bool ready = false;
    double x0, y0, yaw;
    ros::NodeHandle nh("~");
    tf::StampedTransform transform;
    tf::TransformListener listener;
    std::string map_frame, lidar_frame, robot_frame;
    if(!nh.getParam("map_frame", map_frame))
        ROS_ERROR("Missing parameter: map_frame");
    if(!nh.getParam("robot_frame", robot_frame))
        ROS_ERROR("Missing parameter: robot_frame");
    if(!nh.getParam("lidar_frame", lidar_frame))
        ROS_ERROR("Missing parameter: lidar_frame");
    int width = 2 * nh.param("width", 1000);
    int height = 2 * nh.param("height", 1000);
    int expansion = nh.param("expansion", 40);
    float scale = 1 / nh.param("scale", 1e-2);
    cv::Mat map = cv::Mat(height, width, CV_8UC1, 0xFF);
    listener.waitForTransform(
        map_frame, robot_frame, ros::Time(0), ros::Duration(100)
    );
    listener.waitForTransform(
        map_frame, lidar_frame, ros::Time(0), ros::Duration(100)
    );
    costmap.info.width = width;
    costmap.info.height = height;
    costmap.info.resolution = 1e-2;
    costmap.data.resize(width * height);
    costmap.header.frame_id = map_frame;
    ros::Publisher publisher = nh.advertise<Map>("/costmap", 1);
    ros::Subscriber laser = nh.subscribe<Scan>(
        "/scan", 1, [&lidar, &ready](Scan::ConstPtr scan){
            lidar = *scan; ready = true;
        }
    );
    while(ros::ok())
    {
        ros::spinOnce();
        if(!ready) continue; map *= 0;
        cv::rectangle(
            map, cv::Rect(0, 0, width, height), 0x64, 10
        );
        listener.lookupTransform(
            map_frame, lidar_frame, ros::Time(0), transform
        );
        x0 = transform.getOrigin().x();
        y0 = transform.getOrigin().y();
        yaw = tf::getYaw(transform.getRotation()) + lidar.angle_min;
        for(double d: lidar.ranges)
        {
            if(d < lidar.range_max && d >= lidar.range_min)
            {
                int x = std::round((x0 + d * std::cos(yaw)) * scale);
                int y = std::round((y0 + d * std::sin(yaw)) * scale);
                if(std::min(x, y) >= 0 && x < map.cols && y < map.rows)
                    map.at<uchar>(y, x) = 0xFF;
            }
            yaw += lidar.angle_increment;
        }
        cv::distanceTransform(
            0xFF - map, sdf, cv::DIST_L2, cv::DIST_MASK_PRECISE
        );
        for(int y = 0; y < map.rows; y++)
            for(int x = 0; x < map.cols; x++)
            {
                float esdf = 0x32 * sdf.at<float>(y, x) / expansion;
                map.at<uchar>(y, x) = esdf > 0x32? 0: 0x64 - esdf;
            }
        listener.lookupTransform(
            map_frame, robot_frame, ros::Time(0), transform
        );
        int xs = std::round(scale * transform.getOrigin().x());
        int ys = std::round(scale * transform.getOrigin().y());
        xs = std::min(std::max(xs, 0), map.cols - 1);
        ys = std::min(std::max(ys, 0), map.rows - 1);
        if(map.at<uchar>(ys, xs))
        {
            sf_tracker::int2D p = sf_tracker::bfs(map, xs, ys, 0);
            int x = p.first, y = p.second;
            if(xs > x) swap(x, xs);
            if(ys > y) swap(y, ys);
            cv::rectangle(
                map, cv::Point2i(xs, ys), cv::Point2i(x + 1, y + 1), 0, -1
            );
        }
        for(int y = 0; y < map.rows; y++)
        {
            int z = y * costmap.info.width;
            for(int x = 0; x < map.cols; x++)
                costmap.data[z + x] = map.at<uchar>(y, x);
        }
        publisher.publish(costmap);
    }
    return 0;
}
