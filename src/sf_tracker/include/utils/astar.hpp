/* @author: YueLin */

#include <queue>
#include <vector>
#include <utility>

#include "opencv2/opencv.hpp"

#include "utils/utils.hpp"

namespace astar
{
    inline int encode(int x, int y, const int cols)
    {
        return x + y * cols;
    }

    inline void decode(int code, int* x, int* y, const int cols)
    {
        *x = code % cols; *y = code / cols;
    }

    inline double f(double g, int x, int y, const cv::Point& end)
    {
        return g + distance(x, y, end.x, end.y);
    }

    /* A-star Algorithm */
    std::vector<cv::Point> plan
    (const cv::Mat& map, const cv::Point& start, const cv::Point& goal)
    {
        /* Constants */
        const int W = map.cols;
        const int H = map.rows;

        /* Result */
        std::vector<cv::Point> path;

        /* Initialize arrays and queue */
        std::vector<double> g(W * H, INF);
        std::vector<int> parent(W * H, -1);
        std::vector<bool> visited(W * H, false);
        std::priority_queue<std::pair<double, int>> queue;

        /* Variables */
        int x, y, x0, y0, idx, index = encode(start.x, start.y, W);

        /* Push the first point into the queue */
        visited[index] = !(g[index] = 0);
        queue.push(std::make_pair(-f(0, start.x, start.y, goal), index));

        /* Main loop */
        while(!queue.empty())
        {
            /* Dequeue a point */
            index = queue.top().second;
            decode(index, &x0, &y0, W);
            visited[index] = true; queue.pop();

            /* If found a path */
            if(x0 == goal.x && y0 == goal.y)
            {
                path.push_back(cv::Point(x0, y0));
                while((index = parent[index]) != -1)
                {
                    decode(index, &x0, &y0, W);
                    path.push_back(cv::Point(x0, y0));
                }
                std::reverse(path.begin(), path.end());
                break;
            }

            /* Expand the point */
            for(int neighbor = 0; neighbor < 9; neighbor++)
            {
                x = x0 + neighbor % 3 - 1;
                y = y0 + neighbor / 3 - 1;
                idx = encode(x, y, W);
                
                /* Determine the legitimacy of the extension point */
                if(x < 0 || x >= W || y < 0 || y >= H ||  
                   !map.at<uchar>(y, x) || visited[idx])
                    continue;
                
                /* Calculate cost value */
                double cost = distance(x0, y0, x, y) + g[index];

                /* Update the point */
                if(cost < g[idx])
                {
                    g[idx] = cost;
                    parent[idx] = index;
                    queue.push(std::make_pair(-f(cost, x, y, goal), idx));
                }
            }
        }

        return path;
    }
}
