/* @author: YueLin */

#pragma once

#include <vector>

#include "Eigen/Eigen"
#include "opencv2/opencv.hpp"

#include "utils/utils.hpp"

namespace bspline
{
    std::vector<double> creat1d(const std::vector<double>& points,
                                double vel, double acc, const double t)
    {
        const int N = points.size();
        std::vector<double> control(N + 2);

        Eigen::VectorXd x(N + 4);
        Eigen::Vector3d p(3), v(3), a(3);
        p << 1, 4, 1; v << -1, 0, 1; a << 1, -2, 1;
        Eigen::MatrixXd m = Eigen::MatrixXd::Zero(N + 4, N + 2);

        for(int i = 0; i < N; i++)
        {
            x(i) = points[i];
            m.block(i, i, 1, 3) = (1.0 / 6) * p.transpose();
        }
        x(N) = vel;
        x(N + 2) = acc;
        x(N + 1) = x(N + 3) = 0;
        m.block(N, 0, 1, 3) = (0.5 / t) * v.transpose();
        m.block(N + 2, 0, 1, 3) = (1 / t / t) * a.transpose();
        m.block(N + 1, N - 1, 1, 3) = (0.5 / t) * v.transpose();
        m.block(N + 3, N - 1, 1, 3) = (1 / t / t) * a.transpose();

        Eigen::VectorXd c = m.colPivHouseholderQr().solve(x);
        for(int i = 0; i < N + 2; i++)
            control[i] = clip(c(i));
        return control;
    }

    std::vector<cv::Point2d> creat2d(const std::vector<cv::Point2d>& points, 
                                     double vel_x, double vel_y,
                                     double acc_x, double acc_y,
                                     const double t)
    {
        const int N = points.size();
        std::vector<cv::Point2d> control(N + 2);

        Eigen::Vector3d p(3), v(3), a(3);
        Eigen::VectorXd x(N + 4), y(N + 4);
        p << 1, 4, 1; v << -1, 0, 1; a << 1, -2, 1;
        Eigen::MatrixXd m = Eigen::MatrixXd::Zero(N + 4, N + 2);

        for(int i = 0; i < N; i++)
        {
            x(i) = points[i].x; y(i) = points[i].y;
            m.block(i, i, 1, 3) = (1.0 / 6) * p.transpose();
        }
        x(N) = vel_x; y(N) = vel_y;
        x(N + 2) = acc_x; y(N + 2) = acc_y;
        x(N + 1) = y(N + 1) = x(N + 3) = y(N + 3) = 0;
        m.block(N, 0, 1, 3) = (0.5 / t) * v.transpose();
        m.block(N + 2, 0, 1, 3) = (1 / t / t) * a.transpose();
        m.block(N + 1, N - 1, 1, 3) = (0.5 / t) * v.transpose();
        m.block(N + 3, N - 1, 1, 3) = (1 / t / t) * a.transpose();

        Eigen::ColPivHouseholderQR<Eigen::MatrixXd> c = m.colPivHouseholderQr();
        Eigen::VectorXd cx = c.solve(x), cy = c.solve(y);
        for(int i = 0; i < N + 2; i++)
        {
            control[i].x = cx(i);
            control[i].y = cy(i);
        }
        return control;
    }

    template <typename type> std::vector<type> trajectory(
        const std::vector<type>& control
    ){
        const int N = control.size() - 2;
        if(N <= 1) return std::vector<type>();
        
        std::vector<type> points(N);
        for(int p = 0; p < N; p++)
            points[p] = (control[p] + 4 * control[p + 1] + control[p + 2]) / 6;
        return points;
    }

    inline cv::Point2d derivative(
        const std::vector<cv::Point2d>& bs,
        const std::vector<double>& knot, const int i){
        return 3 / (knot[i + 4] - knot[i + 1]) * (bs[i + 1] - bs[i]);
    }

    inline double derivative(
        const std::vector<double>& bs,
        const std::vector<double>& knot, const int i){
        return 3 / (knot[i + 4] - knot[i + 1]) * sub(bs[i + 1], bs[i]);
    }

    template <typename type> std::vector<type> derivative(
        const std::vector<type>& bs,
        const std::vector<double>& knot
    ){
        const int N = bs.size() - 1;
        std::vector<type> points(N);
        for(int p = 0; p < N; p++)
            points[p] = derivative(bs, knot, p);
        return points;
    }

    template <typename type> inline type derivative2(
        const std::vector<type>& bs,
        const std::vector<double>& knot, const int i){
        return 2 / (knot[i + 4] - knot[i + 2]) * (
            derivative(bs, knot, i + 1) - derivative(bs, knot, i)
        );
    }

    template <typename type> type deboor(const std::vector<type>& trajectory,
                                         const std::vector<double>& knot,
                                         double index)
    {
        index = std::min(std::max(index, 0.), knot.back());
        
        int k = 3; 
        while(knot[++k] < index);
        --k;

        std::vector<type> points(4);
        for(int left = 0; left <= 3; left++)
            points[left] = trajectory[k + left - 3];
        for(int left = 1; left <= 3; left++)
            for(int right = 3; right >= left; right--)
            {
                double t = (index - knot[k + right - 3]) / (
                    knot[right + k - left + 1] - knot[right + k - 3]
                );
                points[right] = t * points[right] + (1 - t) * points[right - 1];
            }
        return points.back();
    }
}
