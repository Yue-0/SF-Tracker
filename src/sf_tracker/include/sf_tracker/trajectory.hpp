/* @author: YueLin */

#include "utils/lbfgs.hpp"
#include "utils/bspline.hpp"
#include "sf_tracker/path.hpp"

namespace sf_tracker
{
    class TrajectoryPlanner
    {
        public:
            cv::Mat sdf;
            double lambda_d, lambda_f, lambda_s;
            double t, safe, max_vel, max_acc, max_time, dist, scale = 0;
        
        private:
            lbfgs::lbfgs_parameter_t params;

        public:
            TrajectoryPlanner(){params.g_epsilon = 0.0;}
            std::vector<cv::Point2d> plan(const cv::Mat&, 
                                          const std::vector<cv::Point>&,
                                          double, double, double, double);
            
            std::vector<cv::Point> search(
                const cv::Mat& map, cv::Point& start, cv::Point& goal){
                return path_searcher::plan(map, start, goal);
            }

        private:
            void optimize(std::vector<cv::Point2d>&);
            std::vector<cv::Point2d> sample(const std::vector<cv::Point>&);

            std::vector<cv::Point2d> bspline(
                const std::vector<cv::Point2d>& points, 
                double vx, double vy, double ax, double ay){
                return bspline::creat2d(points, vx, vy, ax, ay, t);
            }

            static double f(void*, const Eigen::VectorXd&, Eigen::VectorXd&);
    };

    std::vector<cv::Point2d> TrajectoryPlanner::plan(const cv::Mat& map, const
                                                     std::vector<cv::Point>& pi,
                                                     double vel_x, double vel_y,
                                                     double acc_x, double acc_y)
    {
        if(pi.empty()) return std::vector<cv::Point2d>();
        std::vector<cv::Point2d> trajectory = bspline(
            sample(pi), vel_x, vel_y, acc_x, acc_y
        );
        return optimize(trajectory), trajectory; 
    }

    void TrajectoryPlanner::optimize(std::vector<cv::Point2d>& points)
    {
        /* Get the number of points */
        const int N = points.size();
        if(N <= 6) return;

        /* Initilize for optimization */
        const int X = 0, Y = 1;
        Eigen::VectorXd var(N * 2);
        for(int i = 0; i < N; i++)
        {
            var(i * 2 + X) = points[i].x * scale;
            var(i * 2 + Y) = points[i].y * scale;
        }

        /* Optimize */
        double cost;
        lbfgs::lbfgs_optimize(var, cost, f, nullptr, nullptr, this, params);

        /* Update points */
        for(int i = 0; i < N; i++)
        {
            points[i].x = var(i * 2 + X);
            points[i].y = var(i * 2 + Y);
        }
    }

    std::vector<cv::Point2d> TrajectoryPlanner::sample(
        const std::vector<cv::Point>& keypoints
    ){
        int p = 1;
        bool next = true;
        double length = 0;
        cv::Point2d p1, p2;
        const int N = keypoints.size();
        const double STEP = max_vel / t;
        std::vector<cv::Point2d> points;
        points.push_back(p1 = keypoints[0]);
        while(p < N)
        {
            if(next) p2 = keypoints[p];
            double delta = distance(p1.x, p1.y, p2.x, p2.y);
            if(length + delta >= STEP)
            {
                double k = (STEP - length) / delta;
                p1 = k * p2 + (1 - k) * p1;
                points.push_back(p1);
                next = false;
                length = 0;
            }
            else
            {
                length += delta; next = true; ++p;
            }
        }
        points.push_back(keypoints.back());
        return points;
    }
}
