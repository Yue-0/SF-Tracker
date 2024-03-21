/* @author: YueLin */

#include <vector>

#include "opencv2/opencv.hpp"

#define pow2(n) (n)*(n)
#define encode(x, y, cols) (y * cols + x)
#define decode(code, x, y, cols) x = code % cols, y = code / cols
#define distance2(x1, y1, x2, y2) (pow2(x1 - x2) + pow2(y1 - y2))
#define F(g, x0, y0, end) (g + std::sqrt(distance2(x0, y0, end.x, end.y)))
#define FIX(map, coordinate) do\
{\
    int2D fixed = bfs(0xFF - map, coordinate.x, coordinate.y, 0xa);\
    coordinate.x = fixed.first; coordinate.y = fixed.second;\
}\
while(false)

namespace sf_tracker
{
    typedef cv::Point2i Point;
    typedef std::vector<Point> Points;
    typedef std::vector<double> Vector;
    typedef std::vector<std::vector<int>> Matrix;

    class PathSearcher
    {
        public:
            PathSearcher(){}
            Points plan(const cv::Mat&, Point, Point);

        private:
            Points keypoint(const Points&);
            Points astar(const cv::Mat&, Point, Point);
            Matrix graph(const cv::Mat&, const Points&);
            Points dijkstra(const Matrix&, const Points&);
    };

    class TrajectoryOptimizer
    {
        public:
            cv::Mat sdf;
            cv::Point2d q[0x400];
            double lambda_d, lambda_f, lambda_s;
            float t, safe, max_vel, max_acc, max_time, scale = 0;

        public:
            TrajectoryOptimizer(){}
            Points plan(const cv::Mat&, const Points&,
                        double, double, double, double);

        private:
            Points samples(const Points&);
            Points optimize(const Points&);
            Points bspline(const Points&, double, double, double, double);
    };

    double objective(const Vector&, Vector&, void*);
}