/* @author: YueLin */

#include <vector>

#include "utils/bspline.hpp"

namespace sf_tracker
{
    class Controller
    {
        public:
            double dt;
            cv::Point2d target;
            cv::Point3d max_vel, max_acc;

        public:
            Controller() = default;
            cv::Point3d control(const std::vector<cv::Point3d>&,
                                const std::vector<double>&,
                                const cv::Point2d&, double);
            std::vector<double> adjustment(std::vector<double>&,
                                           std::vector<cv::Point2d>&);
    };

    cv::Point3d Controller::control(const std::vector<cv::Point3d>& ctrl,
                                    const std::vector<double>& times,
                                    const cv::Point2d& position, double time)
    {
        if(times.empty() || time > times.back())
            return cv::Point3d(0, 0, max_vel.z * std::tanh(std::atan2(
                target.y - position.y, target.x - position.x
            )));
        return bspline::deboor(ctrl, times, time);
    }

    std::vector<double> Controller::adjustment(
        std::vector<double>& orientation,
        std::vector<cv::Point2d>& trajectory
    ){
        /* Initialize uniform B-spline */
        const int N = trajectory.size();
        std::vector<double> times(N + 4);
        for(int p = 0; p < N + 4; p++)
            times[p] = dt * (p - 3);

        /* Adjust time based on velocity */
        for(int p = 0; p < N - 1; p++)
        {
            double omega = bspline::derivative(orientation, times, p);
            cv::Point2d vel = bspline::derivative(trajectory, times, p);
            
            int index = -1;
            double mu = 1.;
            double ratio[3] = {
                std::fabs(vel.x) / max_vel.x,
                std::fabs(vel.y) / max_vel.y,
                std::fabs(omega) / max_vel.z
            };
            for(int i = 0; i < 3; i++)
                if(ratio[i] > mu)
                    mu = ratio[index = i];
            if(index != -1)
            {
                double time = times[p + 4] - times[p + 1];
                double delta = (mu - 1) * time;
                for(int t = p + 2; t <= p + 4; t++)
                    times[t] += (t - p - 1) * delta / 3;
                for(int t = p + 5; t < N + 4; t++)
                    times[t] += delta;
            }
        }

        // /* Adjusting time based on acceleration */
        // for(int p = 0; p < N - 2; p++)
        // {
        //     double alpha = bspline::derivative2(orientation, times, p);
        //     cv::Point2d acc = bspline::derivative2(trajectory, times, p);

        //     int index = -1;
        //     double mu = 1.;
        //     double ratio[3] = {
        //         std::fabs(acc.x) / max_acc.x,
        //         std::fabs(acc.y) / max_acc.y,
        //         std::fabs(alpha) / max_acc.z
        //     };
        //     for(int i = 0; i < 3; i++)
        //         if(ratio[i] > mu)
        //             mu = ratio[index = i];
        //     if(index != -1)
        //     {
        //         mu = std::sqrt(mu);
        //         double time = times[p + 4] - times[p + 2];
        //         double delta = (mu - 1) * time;
        //         if(p == 1 || p == 2)
        //         {
        //             for(int t = 2; t <= 5; t++)
        //                 times[t] += (t - 1) * delta / 2;
        //             for(int t = 6; t < N + 4; t++)
        //                 times[t] += 2 * delta;
        //         }
        //         else
        //         {
        //             for(int t = p + 3; t <= p + 4; t++)
        //                 times[t] += (t - p - 2) * delta / 2;
        //             for(int t = p + 5; t < N + 4; t++)
        //                 times[t] += delta;
        //         }
        //     }
        // }

        /* Get the derivative of non-uniform B-spline */
        trajectory = bspline::derivative(trajectory, times);
        orientation = bspline::derivative(orientation, times);

        /* Return knot span */
        return times;
    }
}
