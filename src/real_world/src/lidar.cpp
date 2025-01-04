/* @author: YueLin */

#include <vector>
#include <stdlib.h>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

typedef sensor_msgs::LaserScan LaserScan;

void inline init(int* pl, int* pf, LaserScan::ConstPtr data, float crop)
{
    int len = data->ranges.size();
    int f = len * crop;
    *pl = len - f;
    *pf = f >> 1;
}

LaserScan filter(LaserScan::ConstPtr data, int len, int f)
{
    LaserScan ls = *data;
    std::vector<float> ranges(len), intensities(len);
    for(int angle = 0; angle < len; angle++)
    {
        ranges[angle] = data->ranges[angle + f];
        intensities[angle] = data->intensities[angle + f];
    }
    ls.ranges = ranges;
    ls.intensities = intensities;
    ls.angle_min += f * ls.angle_increment;
    ls.angle_max = ls.angle_min + (len - 1) * ls.angle_increment;
    return ls;
}

int main(int argc, char* argv[])
{
    int f, len = 0;
    float crop = atof(argv[3]);
    ros::init(argc, argv, "lidar"); ros::NodeHandle handle("~");
    ros::Publisher publisher = handle.advertise<LaserScan>(argv[1], 1);
    ros::Subscriber subscriber = handle.subscribe<LaserScan>(
        argv[2], 1, [&](LaserScan::ConstPtr data)
        {
            if(!len) init(&len, &f, data, crop);
            publisher.publish(filter(data, len, f));
        }
    );
    return ros::spin(), 0;
}
