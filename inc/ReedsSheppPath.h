#ifndef HYBRID_ASTAR_REEDSSHEPPPATH_H
#define HYBRID_ASTAR_REEDSSHEPPPATH_H

#include <boost/math/constants/constants.hpp>
#include "CollisionDetection.h"
#include "node.h"

namespace HybridAStar
{
    class ReedsShepp
    {
    public:
        class pos
        {
        public:
            float x = 0;
            float y = 0;
            float angle = 0;
            pos(float _x, float _y, float _a):x(_x),y(_y),angle(_a){}
            pos() = default;
        };
        enum ReedsSheppPathSegmentType
        {
            RS_NOP = 0,
            RS_LEFT = 1,
            RS_STRAIGHT = 2,
            RS_RIGHT = 3
        };
        static const ReedsSheppPathSegmentType reedsSheppPathType[18][5];
        class ReedsSheppPath
        {
        public:
            explicit ReedsSheppPath(const ReedsSheppPathSegmentType *type = reedsSheppPathType[0],
                           double t = std::numeric_limits<double>::max(), double u = 0., double v = 0.,
                           double w = 0., double x = 0.);
            double length() const
            {
                return totalLength_;
            }
            /** Path segment types */
            const ReedsSheppPathSegmentType *type_;
            /** Path segment lengths */
            double length_[5];
            /** Total length */
            double totalLength_;
        };
        ReedsShepp(float rho);
        ReedsSheppPath plan(pos start,pos end);
        double distance(pos start,pos end);
        void interpolate(const pos *from, const ReedsSheppPath &path, double t,
                    pos *state) const;
        bool isTraversable(Node3D* start,ReedsShepp::ReedsSheppPath *path,CollisionDetection *map) const;
    private:
        float rho_;

        const double pi = boost::math::constants::pi<double>();
        const double twopi = 2. * pi;
        const double RS_EPS = 1e-6;
        const double ZERO = 10 * std::numeric_limits<double>::epsilon();

        void CCSC(double x, double y, double phi, ReedsSheppPath &path);
        void CCCC(double x, double y, double phi, ReedsSheppPath &path);
        void CCC(double x, double y, double phi, ReedsSheppPath &path);
        void CSC(double x, double y, double phi, ReedsSheppPath &path);
        void CCSCC(double x, double y, double phi, ReedsSheppPath &path);
        ReedsSheppPath plan(double x, double y, double phi);

        ///////////////////////////////////////////////////////////////////////////////
        inline double mod2pi(double x) const
        {
            double v = fmod(x, twopi);
            if (v < -pi)
                v += twopi;
            else if (v > pi)
                v -= twopi;
            return v;
        }
        static inline void polar(double x, double y, double &r, double &theta)
        {
            r = sqrt(x * x + y * y);
            theta = atan2(y, x);
        }
        inline void tauOmega(double u, double v, double xi, double eta, double phi, double &tau, double &omega)
        {
            double delta = mod2pi(u - v), A = sin(u) - sin(delta), B = cos(u) - cos(delta) - 1.;
            double t1 = atan2(eta * A - xi * B, xi * A + eta * B), t2 = 2. * (cos(delta) - cos(v) - cos(u)) + 3;
            tau = (t2 < 0) ? mod2pi(t1 + pi) : mod2pi(t1);
            omega = mod2pi(tau - u + v - phi);
        }
        inline bool LpSpLp(double x, double y, double phi, double &t, double &u, double &v)
        {
            polar(x - sin(phi), y - 1. + cos(phi), u, t);
            if (t >= -ZERO)
            {
                v = mod2pi(phi - t);
                if (v >= -ZERO)
                {
                    assert(fabs(u * cos(t) + sin(phi) - x) < RS_EPS);
                    assert(fabs(u * sin(t) - cos(phi) + 1 - y) < RS_EPS);
                    assert(fabs(mod2pi(t + v - phi)) < RS_EPS);
                    return true;
                }
            }
            return false;
        }
        inline bool LpSpRp(double x, double y, double phi, double &t, double &u, double &v)
        {
            double t1, u1;
            polar(x + sin(phi), y - 1. - cos(phi), u1, t1);
            u1 = u1 * u1;
            if (u1 >= 4.)
            {
                double theta;
                u = sqrt(u1 - 4.);
                theta = atan2(2., u);
                t = mod2pi(t1 + theta);
                v = mod2pi(t - phi);
                assert(fabs(2 * sin(t) + u * cos(t) - sin(phi) - x) < RS_EPS);
                assert(fabs(-2 * cos(t) + u * sin(t) + cos(phi) + 1 - y) < RS_EPS);
                assert(fabs(mod2pi(t - v - phi)) < RS_EPS);
                return t >= -ZERO && v >= -ZERO;
            }
            return false;
        }
        inline bool LpRmL(double x, double y, double phi, double &t, double &u, double &v)
        {
            double xi = x - sin(phi), eta = y - 1. + cos(phi), u1, theta;
            polar(xi, eta, u1, theta);
            if (u1 <= 4.)
            {
                u = -2. * asin(.25 * u1);
                t = mod2pi(theta + .5 * u + pi);
                v = mod2pi(phi - t + u);
                assert(fabs(2 * (sin(t) - sin(t - u)) + sin(phi) - x) < RS_EPS);
                assert(fabs(2 * (-cos(t) + cos(t - u)) - cos(phi) + 1 - y) < RS_EPS);
                assert(fabs(mod2pi(t - u + v - phi)) < RS_EPS);
                return t >= -ZERO && u <= ZERO;
            }
            return false;
        }
        inline bool LpRupLumRm(double x, double y, double phi, double &t, double &u, double &v)
        {
            double xi = x + sin(phi), eta = y - 1. - cos(phi), rho = .25 * (2. + sqrt(xi * xi + eta * eta));
            if (rho <= 1.)
            {
                u = acos(rho);
                tauOmega(u, -u, xi, eta, phi, t, v);
                assert(fabs(2 * (sin(t) - sin(t - u) + sin(t - 2 * u)) - sin(phi) - x) < RS_EPS);
                assert(fabs(2 * (-cos(t) + cos(t - u) - cos(t - 2 * u)) + cos(phi) + 1 - y) < RS_EPS);
                assert(fabs(mod2pi(t - 2 * u - v - phi)) < RS_EPS);
                return t >= -ZERO && v <= ZERO;
            }
            return false;
        }
        inline bool LpRumLumRp(double x, double y, double phi, double &t, double &u, double &v)
        {
            double xi = x + sin(phi), eta = y - 1. - cos(phi), rho = (20. - xi * xi - eta * eta) / 16.;
            if (rho >= 0 && rho <= 1)
            {
                u = -acos(rho);
                if (u >= -.5 * pi)
                {
                    tauOmega(u, u, xi, eta, phi, t, v);
                    assert(fabs(4 * sin(t) - 2 * sin(t - u) - sin(phi) - x) < RS_EPS);
                    assert(fabs(-4 * cos(t) + 2 * cos(t - u) + cos(phi) + 1 - y) < RS_EPS);
                    assert(fabs(mod2pi(t - v - phi)) < RS_EPS);
                    return t >= -ZERO && v >= -ZERO;
                }
            }
            return false;
        }
        inline bool LpRmSmLm(double x, double y, double phi, double &t, double &u, double &v)
        {
            double xi = x - sin(phi), eta = y - 1. + cos(phi), rho, theta;
            polar(xi, eta, rho, theta);
            if (rho >= 2.)
            {
                double r = sqrt(rho * rho - 4.);
                u = 2. - r;
                t = mod2pi(theta + atan2(r, -2.));
                v = mod2pi(phi - .5 * pi - t);
                assert(fabs(2 * (sin(t) - cos(t)) - u * sin(t) + sin(phi) - x) < RS_EPS);
                assert(fabs(-2 * (sin(t) + cos(t)) + u * cos(t) - cos(phi) + 1 - y) < RS_EPS);
                assert(fabs(mod2pi(t + pi / 2 + v - phi)) < RS_EPS);
                return t >= -ZERO && u <= ZERO && v <= ZERO;
            }
            return false;
        }
        inline bool LpRmSmRm(double x, double y, double phi, double &t, double &u, double &v)
        {
            double xi = x + sin(phi), eta = y - 1. - cos(phi), rho, theta;
            polar(-eta, xi, rho, theta);
            if (rho >= 2.)
            {
                t = theta;
                u = 2. - rho;
                v = mod2pi(t + .5 * pi - phi);
                assert(fabs(2 * sin(t) - cos(t - v) - u * sin(t) - x) < RS_EPS);
                assert(fabs(-2 * cos(t) - sin(t - v) + u * cos(t) + 1 - y) < RS_EPS);
                assert(fabs(mod2pi(t + pi / 2 - v - phi)) < RS_EPS);
                return t >= -ZERO && u <= ZERO && v <= ZERO;
            }
            return false;
        }
        inline bool LpRmSLmRp(double x, double y, double phi, double &t, double &u, double &v)
        {
            double xi = x + sin(phi), eta = y - 1. - cos(phi), rho, theta;
            polar(xi, eta, rho, theta);
            if (rho >= 2.)
            {
                u = 4. - sqrt(rho * rho - 4.);
                if (u <= ZERO)
                {
                    t = mod2pi(atan2((4 - u) * xi - 2 * eta, -2 * xi + (u - 4) * eta));
                    v = mod2pi(t - phi);
                    assert(fabs(4 * sin(t) - 2 * cos(t) - u * sin(t) - sin(phi) - x) < RS_EPS);
                    assert(fabs(-4 * cos(t) - 2 * sin(t) + u * cos(t) + cos(phi) + 1 - y) < RS_EPS);
                    assert(fabs(mod2pi(t - v - phi)) < RS_EPS);
                    return t >= -ZERO && v >= -ZERO;
                }
            }
            return false;
        }
    };
}



#endif //HYBRID_ASTAR_REEDSSHEPPPATH_H
