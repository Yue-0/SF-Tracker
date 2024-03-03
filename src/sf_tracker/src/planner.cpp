/* @author: YueLin */

#include <cmath>
#include <queue>
#include <thread>
#include <utility>

#include <tf/tf.h>
#include <ros/ros.h>
#include <nlopt.hpp>
#include <Eigen/Eigen>
#include <tf/transform_listener.h>

#include "nav_msgs/Path.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PoseStamped.h"

#include "sf_tracker/bfs.hpp"
#include "sf_tracker/planner.hpp"

#define TIME ros::Time::now().toSec()
#define ROS_INIT ros::init(argc, argv, "planner")

#define zero(f) std::fabs(f) < 1e-6
#define path_push(x0, y0) do\
{\
    geometry_msgs::PoseStamped pose;\
    pose.header.stamp = path.header.stamp;\
    pose.pose.position.x = x0 * optimizer.scale;\
    pose.pose.position.y = y0 * optimizer.scale;\
    pose.header.frame_id = path.header.frame_id;\
    path.poses.push_back(pose);\
}\
while(false)

typedef nav_msgs::Path Path;

namespace sf_tracker
{
    Points PathSearcher::plan(const cv::Mat& map, Point start, Point end)
    {
        if(!map.at<uchar>(end.y, end.x))
            FIX(map, end);
        if(!map.at<uchar>(start.y, start.x))
            FIX(map, start);
        Points path = keypoint(astar(map, start, end));
        return path.size()? dijkstra(graph(map, path), path): path;
    }

    Points PathSearcher::keypoint(const Points& path)
    {
        Points keypoints;
        int n = path.size();
        if(!n) return keypoints;
        keypoints.push_back(path[0]);
        Point p0 = path[0], p1 = path[1], p2 = path[2];
        for(int i = 3; i < path.size(); i++)
        {
            if((p1.x - p0.x) * (p2.y - p1.y) - (p1.y - p0.y) * (p2.x - p1.x))
                keypoints.push_back(p1);
            p0 = p1; p1 = p2; p2 = path[i];
        }
        keypoints.push_back(p2);
        return keypoints;
    }

    Points PathSearcher::astar(const cv::Mat& map, Point start, Point end)
    {
        const int cols = map.cols;
        const int rows = map.rows;
        std::vector<int> g(cols * rows, INF);
        std::vector<int> parent(cols * rows, -1);
        std::vector<bool> visited(cols * rows, false);
        std::priority_queue<std::pair<int, int>> queue;
        int x, y, x0, y0, idx, index = cols * start.y + start.x;
        visited[index] = !(g[index] = 0);
        queue.push(std::make_pair(
            -F(0, start.x, start.y, end),
            encode(start.x, start.y, cols)
        ));
        while(!queue.empty())
        {
            index = queue.top().second;
            decode(index, x0, y0, cols);
            visited[index] = true;
            queue.pop();
            if(x0 == end.x && y0 == end.y)
            {
                Points path;
                path.push_back(Point(x0, y0));
                while((index = parent[index]) != -1)
                {
                    decode(index, x0, y0, cols);
                    path.push_back(Point(x0, y0));
                }
                std::reverse(path.begin(), path.end());
                return path;
            }
            for(int dy = -1; dy <= 1; dy++)
            {
                y = y0 + dy;
                if(y < 0 || y >= rows) continue;
                for(int dx = -1; dx <= 1; dx++)
                {
                    x = x0 + dx;
                    idx = encode(x, y, cols);
                    if(x < 0 || x >= cols || !map.at<uchar>
                       (y, x) || visited[idx]) continue;
                    int cost = std::sqrt(
                        distance2(x, y, end.x, end.y)
                    ) + g[index];
                    if(cost < g[idx])
                    {
                        g[idx] = cost;
                        parent[idx] = index;
                        queue.push(std::make_pair(-F(cost, x, y, end), idx));
        }   }   }   }
        return Points();
    }

    Matrix PathSearcher::graph(const cv::Mat& map, const Points& points)
    {
        int n = points.size();
        Matrix g(n, std::vector<int>(n, 0));
        for(int i = 0; i < n; i++)
        {
            Point p1 = points[i];
            for(int j = i + 1; j < n; j++)
            {
                Point p2 = points[j];
                bool connected = true;
                if(j - i > 1)
                {
                    cv::LineIterator line(map, p1, p2);
                    for(int k = 0; k < line.count; k++, line++)
                    {
                        Point p = line.pos();
                        if(!map.at<uchar>(p.y, p.x))
                        {
                            connected = false; break;
                        }
                    }
                }
                if(connected)
                    g[i][j] = std::sqrt(distance2(p1.x, p1.y, p2.x, p2.y));
            }
        }
        return g;
    }

    Points PathSearcher::dijkstra(const Matrix& g, const Points& points)
    {
        int n = g.size();
        std::vector<int> dist(n, INF);
        std::vector<int> parent(n, -1);
        std::vector<bool> visited(n, false);
        std::priority_queue<std::pair<int, int>> queue;
        queue.push(std::make_pair(dist[0] = 0, 0));
        while(!queue.empty())
        {
            int u = queue.top().second; queue.pop();
            if(u == n - 1)
            {
                Points path;
                path.push_back(points[n - 1]);
                while(u = parent[u])
                    path.push_back(points[u]);
                path.push_back(points[0]);
                std::reverse(path.begin(), path.end());
                return path;
            }
            for(int v = u + 1; v < n; v++)
                if(!visited[v] && g[u][v])
                {
                    int d = dist[u] + g[u][v];
                    if(d < dist[v])
                    {
                        dist[v] = d;
                        parent[v] = u;
                        queue.push(std::make_pair(-d, v));
                    }
                }
            visited[u] = true;
        }
        return Points();
    }

    Points TrajectoryOptimizer::plan(const cv::Mat& map,
                                     const Points& path,
                                     double vel_x, double vel_y,
                                     double acc_x, double acc_y)
    {
        if(path.empty()) return path;
        std::thread thread([=, &map](){esdf(map);});
        Points b = bspline(
            samples(path), vel_x, vel_y, acc_x, acc_y
        ); thread.join(); b = optimize(b);
        const int n = b.size() - 3;
        if(n <= 1) return path; Points optimized;
        for(int p = 1; p < n; p++)
            optimized.push_back((
                b[p] + 4 * b[p + 1] + b[p + 2]
            ) / 6.0);
        return optimized;
    }

    void TrajectoryOptimizer::esdf(const cv::Mat& map)
    {
        cv::distanceTransform(
            map, sdf, cv::DIST_L2, cv::DIST_MASK_PRECISE
        );
        for(int y = 0; y < map.rows; y++)
            for(int x = 0; x < map.cols; x++)
                if(sdf.at<float>(y, x))
                    sdf.at<float>(y, x) *= scale;
                else
                    sdf.at<float>(y, x) = 1e-6;
    }

    Points TrajectoryOptimizer::bspline(const Points& points,
                                        double vel_x, double vel_y,
                                        double acc_x, double acc_y)
    {
        int n = points.size();
        Points control(n + 2);
        Eigen::Vector3d p(3), v(3), a(3);
        Eigen::VectorXd x(n + 4), y(n + 4);
        Eigen::MatrixXd matrix = Eigen::MatrixXd::Zero(n + 4, n + 2);
        p << 1, 4, 1; v << -1, 0, 1; a << 1, -2, 1;
        for(int i = 0; i < n; i++)
        {
            x(i) = points[i].x; y(i) = points[i].y;
            matrix.block(i, i, 1, 3) = (1.0 / 6) * p.transpose();
        }
        x(n) = vel_x; y(n) = vel_y;
        x(n + 2) = acc_x; y(n + 2) = acc_y;
        x(n + 1) = y(n + 1) = x(n + 3) = y(n + 3) = 0;
        matrix.block(n, 0, 1, 3) = (0.5 / t) * v.transpose();
        matrix.block(n + 2, 0, 1, 3) = (1 / t / t) * a.transpose();
        matrix.block(n + 1, n - 1, 1, 3) = (0.5 / t) * v.transpose();
        matrix.block(n + 3, n - 1, 1, 3) = (1 / t / t) * a.transpose();
        Eigen::ColPivHouseholderQR<Eigen::MatrixXd>
        qr = matrix.colPivHouseholderQr(); Eigen::VectorXd
        cx = qr.solve(x), cy = qr.solve(y);
        for(int i = 0; i < n + 2; i++)
        {
            control[i].x = cx(i);
            control[i].y = cy(i);
        }
        return control;
    }

    Points TrajectoryOptimizer::samples(const Points& keypoints)
    {
        Points points;
        Point p0, p2, p1;
        points.push_back(p1 = keypoints[0]);
        for(int p = 1; p < keypoints.size(); p1 = p2)
        {
            p2 = keypoints[p++];
            double k, dt, step = max_vel / (std::sqrt(
                distance2(p1.x, p1.y, p2.x, p2.y)
            ) * scale) * t;
            for(k = dt = std::min(step, 0.5); k < 1 + dt; k += dt)
            {
                p0.x = std::round(k * p2.x + (1 - k) * p1.x);
                p0.y = std::round(k * p2.y + (1 - k) * p1.y);
                points.push_back(p0);
            }
        }
        return points;
    }

    Points TrajectoryOptimizer::optimize(const Points& points)
    {
        int num, n = points.size();
        if((num = n - 6) <= 0)
            return points;
        Vector x(num << 1);
        Points path(points);
        double cost = std::numeric_limits<double>::max();
        nlopt::opt optimizer(nlopt::LD_LBFGS, num << 1);
        for(int i = 0; i < n; i++)
        {
            q[i].x = points[i].x * scale;
            q[i].y = points[i].y * scale;
        }
        for(int i = 0; i < num; i++)
        {
            x[i << 1] = q[i + 3].x;
            x[(i << 1) + 1] = q[i + 3].y;
        }
        Vector upper(num << 1);
        for(int i = 0; i < num; i++)
        {
            upper[i << 1] = sdf.cols;
            upper[(i << 1) + 1] = sdf.rows;
        }
        try
        {
            optimizer.set_min_objective(objective, this);
            optimizer.set_lower_bounds(Vector(num << 1, 0));
            optimizer.set_upper_bounds(upper);
            optimizer.set_maxtime(max_time);
            optimizer.set_xtol_rel(1e-7);
            optimizer.optimize(x, cost);
        }
        catch(std::exception& _){}
        for(int i = 0; i < num; i++)
        {
            path[i + 3].x = std::round(x[i << 1] / scale);
            path[i + 3].y = std::round(x[(i << 1) + 1] / scale);
        }
        return path;
    }

    double objective(const Vector& x, Vector& grad, void* trajectory_optimizer)
    {
        TrajectoryOptimizer*
        opt = reinterpret_cast
        <TrajectoryOptimizer*>
        (trajectory_optimizer);
        double cost, g, j = 0.0;
        const unsigned int n = x.size() >> 1;
        std::vector<Vector> gradient(n + 6, Vector(2, 0)),
        vel(n + 5, Vector(2)), acc(n + 4, Vector(2)), jerk(n + 3, Vector(2));
        for(int i = 0; i < n; i++)
        {
            int z = i << 1;
            opt->q[i + 3].x = x[z];
            opt->q[i + 3].y = x[z + 1];
        }
        for(int i = 0; i < n + 5; i++)
        {
            vel[i][0] = opt->q[i + 1].x - opt->q[i].x;
            vel[i][1] = opt->q[i + 1].y - opt->q[i].y;
        }
        for(int i = 0; i < n + 4; i++)
        {
            acc[i][0] = vel[i + 1][0] - vel[i][0];
            acc[i][1] = vel[i + 1][1] - vel[i][1];
        }
        for(int i = 0; i < n + 3; i++)
        {
            jerk[i][0] = acc[i + 1][0] - acc[i][0];
            jerk[i][1] = acc[i + 1][1] - acc[i][1];
        }

        /* Feasibility Cost */
        double vm2 = pow2(opt->max_vel), am2 = pow2(opt->max_acc);
        double t = opt->t; double t2 = 1 / t / t; double t4 = pow2(t2);
        for(int i = 0; i < n + 5; i++)
            for(int z = 0; z < 2; z++)
                if((cost = pow2(vel[i][z]) * t2 - vm2) > 0)
                {
                    j += opt->lambda_f * cost;
                    g = 2 * opt->lambda_f * vel[i][z] * t2;
                    gradient[i][z] -= g; gradient[i + 1][z] += g;
                }
        for(int i = 0; i < n + 4; i++)
            for(int z = 0; z < 2; z++)
                if((cost = pow2(acc[i][z]) * t4 - am2) > 0)
                {
                    j += opt->lambda_f * cost;
                    g = 2 * opt->lambda_f * acc[i][z] * t4;
                    gradient[i][z] += g;
                    gradient[i + 2][z] += g;
                    gradient[i + 1][z] -= 2 * g;
                }

        /* Smoothness Cost */
        double t6 = t2 * t4;
        for(int i = 0; i < n + 4; i++)
            for(int z = 0; z < 2; z++)
            {
                g = 2 * opt->lambda_s * acc[i][z] * t4;
                gradient[i][z] += g;
                gradient[i + 2][z] += g;
                gradient[i + 1][z] -= 2 * g;
                j += opt->lambda_s * pow2(acc[i][z]) * t4;
            }
        for(int i = 0; i < n + 3; i++)
            for(int z = 0; z < 2; z++)
            {
                g = 2 * opt->lambda_s * jerk[i][z] * t6;
                gradient[i][z] -= g;
                gradient[i + 3][z] += g;
                gradient[i + 1][z] += 3 * g;
                gradient[i + 2][z] -= 3 * g;
                j += opt->lambda_s * pow2(jerk[i][z]) * t6;
            }

        /* Safety Cost */
        const cv::Mat esdf = opt->sdf;
        for(int i = 3; i < n + 3; i++)
        {
            int x = std::round(opt->q[i].x / opt->scale);
            int y = std::round(opt->q[i].y / opt->scale);
            x = std::max(std::min(x, esdf.cols - 2), 1);
            y = std::max(std::min(y, esdf.rows - 2), 1);
            double sqr = std::sqrt(esdf.at<float>(y, x));
            if((cost = opt->safe - sqr) >= 0)
            {
                j += opt->lambda_d * cost;
                g = 0.25 * opt->lambda_d / sqr;
                gradient[i][0] += g * (
                    esdf.at<float>(y, x - 1) - esdf.at<float>(y, x + 1)
                );
                gradient[i][1] += g * (
                    esdf.at<float>(y - 1, x) - esdf.at<float>(y + 1, x)
                );
            }
        }

        /* Structural Gradient */
        for(int i = 0; i < n; i++)
        {
            int z = i << 1;
            grad[z] = gradient[i + 3][0];
            grad[z + 1] = gradient[i + 3][1];
        }
        return j;
    }
}

int main(int argc, char* argv[])
{
    ROS_INIT;
    Path path;
    cv::Mat map;
    double scale;
    ros::Time::init();
    bool track = false;
    ros::NodeHandle nh("~");
    geometry_msgs::Point target;
    tf::StampedTransform transform;
    tf::TransformListener listener;
    sf_tracker::PathSearcher planner;
    std::string map_frame, odom_frame;
    sf_tracker::Point start(0, 0), end(0, 0);
    sf_tracker::TrajectoryOptimizer optimizer;
    if(!nh.getParam("map_frame", map_frame))
        ROS_ERROR("Missing parameter: map_frame");
    if(!nh.getParam("odom_frame", odom_frame))
        ROS_ERROR("Missing parameter: odom_frame");
    path.header.frame_id = map_frame;
    optimizer.t = nh.param("delta_t", 0.1);
    optimizer.max_vel = nh.param("max_vel", 2.5);
    optimizer.max_acc = nh.param("max_acc", 1.5);
    optimizer.lambda_d = nh.param("lambda_safety", 1e2);
    optimizer.lambda_s = nh.param("lambda_smoothness", 1e-6);
    optimizer.lambda_f = nh.param("lambda_feasibility", 1e-2);
    optimizer.max_time = nh.param("solution_time_limit", 1e-2);
    optimizer.safe = std::sqrt(nh.param("safe_distance", 0.4));
    float vel_x = 0, vel_y = 0, acc_x = 0, acc_y = 0, cmd_vel_time = 0;
    listener.waitForTransform(
        odom_frame, map_frame, ros::Time(0), ros::Duration(10)
    );
    ros::Publisher publisher = nh.advertise<Path>("/path", 1);
    ros::Subscriber tracking = nh.subscribe<geometry_msgs::PoseStamped>(
        "/target", 1, [&target, &track, &optimizer]
        (geometry_msgs::PoseStamped::ConstPtr pos)
        {
            if(zero(optimizer.scale)) return;
            target = pos->pose.position; track = true;
        }
    );
    ros::Subscriber scan = nh.subscribe<nav_msgs::OccupancyGrid>(
        "/costmap", 1, [&map, &start, &optimizer, &scale]
        (nav_msgs::OccupancyGrid::ConstPtr costmap)
        {
            int h = costmap->info.height, w = costmap->info.width;
            optimizer.scale = costmap->info.resolution;
            start.x = w >> 1; start.y = h >> 1;
            map = cv::Mat(h, w, CV_8UC1, 0xFF);
            scale = 1 / optimizer.scale;
            for(int y = 0; y < h; y++)
            {
                int z = y * w;
                for(int x = 0; x < w; x++)
                    if(costmap->data[z + x])
                        map.at<uchar>(y, x) = 0;
            }
        }
    );
    ros::Subscriber kinetic = nh.subscribe<geometry_msgs::Twist>(
        "/cmd_vel", 1, [&vel_x, &vel_y, &acc_x, &acc_y, &cmd_vel_time]
        (geometry_msgs::Twist::ConstPtr cmd_vel)
        {
            if(!cmd_vel_time)
            {
                cmd_vel_time = TIME;
                vel_x = cmd_vel->linear.x;
                vel_y = cmd_vel->linear.y;
            }
            else
            {
                float t = 1.0 / (TIME - cmd_vel_time);
                acc_x = t * (cmd_vel->linear.x - vel_x);
                acc_y = t * (cmd_vel->linear.y - vel_y);
                vel_x = cmd_vel->linear.x;
                vel_y = cmd_vel->linear.y;
                cmd_vel_time = TIME;
            }
        }
    );
    ros::Timer planning = nh.createTimer(
        ros::Duration(nh.param("replan_interval", 0.1)),
        [&](const ros::TimerEvent& _){
            if(!track || (!start.x && !start.y))
                return;
            listener.lookupTransform(
                odom_frame, map_frame, ros::Time(0), transform
            );
            path.poses.clear();
            path.header.stamp = ros::Time::now();
            tf::Vector3 origin = transform.getOrigin();
            double yaw = tf::getYaw(transform.getRotation());
            double sin = std::sin(yaw), cos = std::cos(yaw),
            x = target.x - origin.x(), y = target.y - origin.y();
            end.x = std::round((x * cos + y * sin) * scale);
            end.y = std::round((y * cos - x * sin) * scale);
            end.x = std::max(std::min(end.x, map.rows - 1), 0);
            end.y = std::max(std::min(end.y, map.cols - 1), 0);
            for(sf_tracker::Point point: optimizer.plan(
                map, planner.plan(map, start, end),
                vel_x, vel_y, acc_x, acc_y
            )) path_push(point.x, point.y);
               path_push(end.x, end.y);
            publisher.publish(path);
        }
    );
    return ros::spin(), 0;
}
