/* @author: YueLin */

#include <queue>
#include <vector>
#include <utility>

#include "opencv2/opencv.hpp"

#include "utils/utils.hpp"

/* Breadth-First Search Algorithm */
std::pair<int, int> bfs(const cv::Mat& map, int xs, int ys, const int d_thr = 0)
{
    /* Variables */
    int x0, y0;
    std::pair<int, int> xy;

    /* Constants */
    const int H = map.rows, W = map.cols;

    /* Correct xs and ys */
    xs = std::min(std::max(0, xs), W - 1);
    ys = std::min(std::max(0, ys), H - 1);

    /* Initialize queue and arrays */
    std::queue<std::pair<int, int>> queue;
    std::vector<std::vector<bool>> visited(
        H, std::vector<bool>(W, false)
    );
    std::vector<std::vector<int>> dist(
        H, std::vector<int>(W, INF)
    );

    /* Push the first point into the queue */
    queue.push(std::make_pair(xs, ys));
    visited[ys][xs] = true; dist[ys][xs] = 0;

    /* Main loop */
    while(!queue.empty())
    {
        /* Dequeue a point */
        xy = queue.front(); queue.pop();
        x0 = xy.first; y0 = xy.second;

        /* If the target is found */
        if(dist[y0][x0] > d_thr)
            break;
        
        /* Expand the point */
        for(int dx = -1; dx <= 1; dx++)
        {
            int x = x0 + dx;
            if(x < 0 || x >= W) continue;
            for(int dy = -1; dy <= 1; dy++)
            {
                int y = y0 + dy;
                if(y < 0 || y >= H) continue;
                if(!visited[y][x])
                {
                    visited[y][x] = true;
                    queue.push(std::make_pair(x, y));
                    dist[y][x] = map.at<uchar>(y, x)? 0: dist[y0][x0] + 1;
    }   }   }   }

    return xy;
}
