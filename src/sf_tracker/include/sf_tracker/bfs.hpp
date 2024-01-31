/* @author: YueLin */

#include <queue>
#include <vector>
#include <utility>

#include "opencv2/opencv.hpp"

namespace sf_tracker
{
    typedef unsigned int uint;
    typedef std::pair<int, int> int2D;

    const uint INF = 0xFFFFFFFF;

    int2D bfs(const cv::Mat& map, int xs, int ys, int d_thr)
    {
        int2D xy;
        int x0, y0;
        std::queue<int2D> queue;
        int h = map.rows, w = map.cols;
        xs = std::min(std::max(0, xs), w - 1);
        ys = std::min(std::max(0, ys), h - 1);
        std::vector<std::vector<uint>>
        dist(h, std::vector<uint>(w, INF));
        std::vector<std::vector<bool>>
        visited(h, std::vector<bool>(w, false));
        visited[ys][xs] = true; dist[ys][xs] = 0;
        queue.push(std::make_pair(xs, ys));
        while(!queue.empty())
        {
            xy = queue.front();
            x0 = xy.first; y0 = xy.second;
            if(dist[y0][x0] > d_thr)
                break; queue.pop();
            for(int dx = -1; dx <= 1; dx++)
            {
                int x = x0 + dx;
                if(x < 0 || x >= w) continue;
                for(int dy = -1; dy <= 1; dy++)
                {
                    int y = y0 + dy;
                    if(y < 0 || y >= h) continue;
                    if(!visited[y][x])
                    {
                        visited[y][x] = true;
                        queue.push(std::make_pair(x, y));
                        dist[y][x] = map.at<uchar>(y, x)? 0: dist[y0][x0] + 1;
        }   }   }   }
        return xy;
    }
}
