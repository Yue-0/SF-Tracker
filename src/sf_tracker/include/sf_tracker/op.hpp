#include <cmath>
#include <vector>

#define pow2(n) (n)*(n)
#define sign(n) (n > 0? 1: (n < 0? -1: 0))
#define CLIP(v, v0, dt, am) do\
{\
    double a = (v - v0) / dt;\
    if(std::fabs(a) > am && std::fabs(v) > 1e-3)\
        v = v0 + am * dt * sign(a);\
}\
while(false)

namespace sf_tracker
{
    typedef std::vector<double> Vector;

    const double PI = std::acos(-1);

    class OrientationPlanner
    {
        public:
            Vector yaw;
            double p[0x400] = {0};
            double mu_f, mu_s, mu_v;
            float t, max_time, max_alpha, max_omega;

        public:
            OrientationPlanner(){}
            Vector optimize(const Vector&);
    };

    double objective(const Vector&, Vector&, void*);

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

    inline double sub(double rad1, double rad2)
    {
        return clip(rad1 - rad2);
    }
}