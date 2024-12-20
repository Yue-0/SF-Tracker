/* @author: YueLin */

#include "utils/bfs.hpp"
#include "utils/astar.hpp"
#include "utils/dijkstra.hpp"

namespace path_searcher
{
    inline void correct(const cv::Mat& map, cv::Point& point, const int d = 0xa)
    {
        std::pair<int, int> corrected = bfs(0xFF - map, point.x, point.y, d);
        point.x = corrected.first; point.y = corrected.second;
    }

    std::vector<cv::Point> keypoint(const std::vector<cv::Point>& path)
    {
        std::vector<cv::Point> keypoints;

        const int N = path.size();
        if(N < 3) return keypoints;

        keypoints.push_back(path[0]);
        cv::Point p0 = path[0], p1 = path[1], p2 = path[2];
        for(int i = 3; i < N; i++)
        {
            if((p1.x - p0.x) * (p2.y - p1.y) - (p1.y - p0.y) * (p2.x - p1.x))
                keypoints.push_back(p1);
            p0 = p1; p1 = p2; p2 = path[i];
        }
        keypoints.push_back(p2);
        return keypoints;
    }

    cv::Mat graph(const cv::Mat& map, const std::vector<cv::Point>& points)
    {
        const int N = points.size();

        cv::Mat g(N, N, CV_32FC1); g *= 0;

        for(int i = 0; i < N; i++)
        {
            cv::Point p1 = points[i];
            for(int j = i + 1; j < N; j++)
            {
                cv::Point p2 = points[j];
                bool connected = true;
                if(j - i > 1)
                {
                    cv::LineIterator line(map, p1, p2);
                    for(int k = 0; k < line.count; k++, line++)
                    {
                        cv::Point p = line.pos();
                        if(!map.at<uchar>(p.y, p.x))
                        {
                            connected = false; break;
                        }
                    }
                }
                if(connected)
                    g.at<float>(i, j) = distance(p1.x, p1.y, p2.x, p2.y);
            }
        }
        
        return g;
    }

    std::vector<cv::Point> plan
    (const cv::Mat& map, cv::Point start, cv::Point goal)
    {
        if(!map.at<uchar>(goal.y, goal.x))
            correct(map, goal);
        if(!map.at<uchar>(start.y, start.x))
            correct(map, start);
        
        std::vector<cv::Point> path = keypoint(astar::plan(map, start, goal));
        return path.size()? dijkstra::plan(graph(map, path), path): path;
    }
}
