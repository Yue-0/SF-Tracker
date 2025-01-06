/* @author: YueLin */

#include <vector>

#include "utils/lbfgs.hpp"
#include "utils/bspline.hpp"

namespace sf_tracker
{
    class OrientationPlanner
    {
        public:
            double* a;
            double mu_f, mu_s, mu_v;
            float t, max_time, max_alpha, max_omega;
        
        private:
            lbfgs::lbfgs_parameter_t params;

        public:
            OrientationPlanner() = default;
            std::vector<double> plan(const std::vector<cv::Point2d>&, 
                                     const cv::Point2d, double, double);
        
        private:
            void optimize(std::vector<double>&);
            std::vector<double> sample(const std::vector<cv::Point2d>&, 
                                       const cv::Point2d);

            std::vector<double> bspline(
                const std::vector<double>& points, double v, double a){
                return bspline::creat1d(points, v, a, t);
            }

            static double f(void*, const Eigen::VectorXd&, Eigen::VectorXd&);
    };

    std::vector<double> OrientationPlanner::plan(
        const std::vector<cv::Point2d>& path,
        const cv::Point2d target, double vel, double acc
    ){
        if(path.empty())
            return std::vector<double>();
        a = new double[path.size()];
        std::vector<double> yaw = bspline(
            sample(path, target), vel, acc
        );
        optimize(yaw);
        delete[] a;
        return yaw;
    }

    void OrientationPlanner::optimize(std::vector<double>& angles)
    {
        /* Get the number of points */
        const int N = angles.size();
        if(N <= 3) return;

        /* Initilize for optimization */
        Eigen::VectorXd var(N);
        for(int i = 0; i < N; i++)
            var(i) = angles[i];

        /* Optimize */
        double cost;
        lbfgs::lbfgs_optimize(var, cost, f, nullptr, nullptr, this, params);

        /* Update angles */
        for(int i = 0; i < N; i++)
            angles[i] = clip(var(i));
    }

    std::vector<double> OrientationPlanner::sample(
        const std::vector<cv::Point2d>& trajectory, const cv::Point2d target
    ){
        const int N = trajectory.size();
        std::vector<double> yaw(N, 0);
        for(int p = 1; p < N; p++)
            yaw[p] = *(a + p) = std::atan2(
                target.y - trajectory[p].y,
                target.x - trajectory[p].x
            );
        return yaw;
    }
}