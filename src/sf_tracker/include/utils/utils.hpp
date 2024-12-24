/* @author: YueLin */

#pragma once

#include <cmath>

const double O = 1e-6;
const int INF = 1 << 30;
const double PI = std::acos(-1);

inline bool zero(double x) {return std::fabs(x) < O;}

template <typename type> inline type pow2(type x) {return x * x;}

template <typename type> 
inline double distance(type x1, type y1, type x2, type y2)
{
    return std::hypot(x1 - x2, y1 - y2);
}

inline double clip(double rad)
{
    int k;
    if(rad > PI)
    {
        k = (rad - PI) / PI / 2;
        rad -= (k + 1) * PI * 2;
    }
    if(rad <= -PI)
    {
        k = -(rad + PI) / PI / 2;
        rad += (k + 1) * PI * 2;
    }
    return rad;
}

inline double sub(double rad1, double rad2) {return clip(rad1 - rad2);}
