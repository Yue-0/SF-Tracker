/* @author: YueLin */

#include <queue>
#include <vector>
#include <utility>

#include "opencv2/opencv.hpp"

#include "utils/utils.hpp"

namespace dijkstra
{
    /* Dijkstra's Algorithm */
    std::vector<cv::Point> plan
    (const cv::Mat& graph, const std::vector<cv::Point>& points)
    {
        /* Constant */
        const int N = graph.cols;
        assert(graph.cols == graph.rows);

        /* Result */
        std::vector<cv::Point> path;

        /* Initialize arrays and queue */
        std::vector<int> parent(N, -1);
        std::vector<float> dist(N, INF);
        std::vector<bool> visited(N, false);
        std::priority_queue<std::pair<float, int>> queue;

        /* Push the first node into the queue */
        queue.push(std::make_pair(dist[0] = 0, 0));

        /* Main loop */
        while(!queue.empty())
        {
            /* Dequeue a node */
            int u = queue.top().second; queue.pop();

            /* If found a path */
            if(u == N - 1)
            {
                path.push_back(points[N - 1]);
                while((u = parent[u]))
                    path.push_back(points[u]);
                path.push_back(points[0]);
                std::reverse(path.begin(), path.end());
                break;
            }

            /* Expand the node */
            for(int v = u + 1; v < N; v++)
                if(!visited[v] && graph.at<float>(u, v) > 1e-2)
                {
                    float d = dist[u] + graph.at<float>(u, v);
                    if(d < dist[v])
                    {
                        dist[v] = d;
                        parent[v] = u;
                        queue.push(std::make_pair(-d, v));
                    }
                }
            visited[u] = true;
        }

        return path;
    }
}
