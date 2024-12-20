/* @author: YueLin */

#include <cmath>

#include <tf/tf.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>

#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/Float32MultiArray.h"

#include "utils/bfs.hpp"

#define ROS_INIT(node) ros::init(argc, argv, node)

#define swap(a, b) {a += b; b = a - b; a -= b;}

int main(int argc, char* argv[])
{
    /* Initialize */
    ROS_INIT("map");
    ros::Time::init();
    ros::NodeHandle nh("~");

    /* Variables */
    bool ready = false;
    double x0, y0, yaw;

    /* Hyper-parameters */
    const double SCALE = 1 / nh.param("scale", 1e-2);
    const int W = 2 * nh.param("width", 10.0) * SCALE;
    const int H = 2 * nh.param("height", 10.0) * SCALE;
    const int RECT = nh.param("expansion", 0.4) * SCALE;
    const uchar T = std::max(
        nh.param("attenuation", 0.25) * nh.param("update_rate", 10), 1.
    );

    /* Frames */
    tf::StampedTransform transform;
    tf::TransformListener listener;
    std::string map_frame, lidar_frame, robot_frame;
    if(!nh.getParam("map_frame", map_frame))
        ROS_ERROR("Missing parameter: map_frame");
    if(!nh.getParam("robot_frame", robot_frame))
        ROS_ERROR("Missing parameter: robot_frame");
    if(!nh.getParam("lidar_frame", lidar_frame))
        ROS_ERROR("Missing parameter: lidar_frame");
    listener.waitForTransform(
        map_frame, robot_frame, ros::Time(0), ros::Duration(100)
    );
    listener.waitForTransform(
        map_frame, lidar_frame, ros::Time(0), ros::Duration(100)
    );

    /* Objects */
    cv::Mat sdf;
    sensor_msgs::LaserScan lidar;
    nav_msgs::OccupancyGrid costmap;
    std_msgs::Float32MultiArray array;
    cv::Mat map = cv::Mat(H, W, CV_8UC1, 0xFF);
    ros::Rate sleep(nh.param("update_rate", 10));
    cv::Mat attenuator = 0 * map;

    /* Initialize costmaps */
    costmap.info.width = W;
    costmap.info.height = H;
    array.data.resize(W * H);
    costmap.data.resize(W * H);
    array.layout.data_offset = W;
    costmap.info.resolution = 1 / SCALE;
    costmap.header.frame_id = map_frame;

    /* Publishers */
    ros::Publisher
    publisher = nh.advertise<nav_msgs::OccupancyGrid>("/costmap", 1),
    constructor = nh.advertise<std_msgs::Float32MultiArray>("/esdf", 1);

    /* Subscriber */
    ros::Subscriber laser = nh.subscribe<sensor_msgs::LaserScan>(
        "/scan", 1, [&lidar, &ready](sensor_msgs::LaserScan::ConstPtr scan){
            lidar = *scan; ready = true;
        }
    );

    /* Main loop */
    while(ros::ok())
    {
        /* Lidar callback */
        ros::spinOnce();
        if(!ready) continue;

        /* Attenuate */
        for(int y = 0; y < H; y++)
            for(int x = 0; x < W; x++)
                if(attenuator.at<uchar>(y, x))
                    --attenuator.at<uchar>(y, x);
        cv::rectangle(attenuator, cv::Rect(0, 0, W, H), T, RECT >> 2);

        /* Get lidar's position */
        listener.lookupTransform(
            map_frame, lidar_frame, ros::Time(0), transform
        );
        x0 = transform.getOrigin().x();
        y0 = transform.getOrigin().y();
        yaw = tf::getYaw(transform.getRotation()) + lidar.angle_min;

        /* Add obstacles */
        for(double d: lidar.ranges)
        {
            if(d < lidar.range_max && d >= lidar.range_min)
            {
                int x = std::round((x0 + d * std::cos(yaw)) * SCALE);
                int y = std::round((y0 + d * std::sin(yaw)) * SCALE);
                if(std::min(x, y) >= 0 && x < map.cols && y < map.rows)
                    attenuator.at<uchar>(y, x) = T;
            }
            yaw += lidar.angle_increment;
        }

        /* Build map */
        map *= 0;
        for(int y = 0; y < H; y++)
            for(int x = 0; x < W; x++)
                if(attenuator.at<uchar>(y, x))
                    map.at<uchar>(y, x) = 0xFF;

        /* Build ESDF */
        cv::distanceTransform(
            0xFF - map, sdf, cv::DIST_L2, cv::DIST_MASK_PRECISE
        );

        /* Update map */
        for(int y = 0; y < map.rows; y++)
            for(int x = 0; x < map.cols; x++)
            {
                float esdf = 0x32 * sdf.at<float>(y, x) / RECT;
                map.at<uchar>(y, x) = esdf > 0x32? 0: 0x64 - esdf;
            }
        
        /* Get robot's position */
        listener.lookupTransform(
            map_frame, robot_frame, ros::Time(0), transform
        );
        int xs = std::round(SCALE * transform.getOrigin().x());
        int ys = std::round(SCALE * transform.getOrigin().y());
        xs = std::min(std::max(xs, 0), map.cols - 1);
        ys = std::min(std::max(ys, 0), map.rows - 1);

        /* Prevent obstacles from expanding into the robot */
        if(map.at<uchar>(ys, xs))
        {
            std::pair<int, int> p = bfs(map, xs, ys);
            int x = p.first, y = p.second;
            if(xs > x) swap(x, xs);
            if(ys > y) swap(y, ys);
            cv::rectangle(
                map, cv::Point(xs, ys), cv::Point(x + 1, y + 1), 0, -1
            );
        }

        /* Update constmaps */
        for(int y = 0; y < map.rows; y++)
        {
            int z = y * costmap.info.width;
            for(int x = 0; x < map.cols; x++)
            {
                array.data[z + x] = sdf.at<float>(y, x);
                costmap.data[z + x] = map.at<uchar>(y, x);
            }
        }

        /* Publish costmaps */
        publisher.publish(costmap);
        constructor.publish(array);

        /* Sleep */
        sleep.sleep();
    }

    return 0x0;
}
