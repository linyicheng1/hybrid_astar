#include "ReedsSheppPath.h"

namespace HybridAStar
{
    const ReedsShepp::ReedsSheppPathSegmentType
            ReedsShepp::reedsSheppPathType[18][5] = {
            {RS_LEFT, RS_RIGHT, RS_LEFT, RS_NOP, RS_NOP},         // 0
            {RS_RIGHT, RS_LEFT, RS_RIGHT, RS_NOP, RS_NOP},        // 1
            {RS_LEFT, RS_RIGHT, RS_LEFT, RS_RIGHT, RS_NOP},       // 2
            {RS_RIGHT, RS_LEFT, RS_RIGHT, RS_LEFT, RS_NOP},       // 3
            {RS_LEFT, RS_RIGHT, RS_STRAIGHT, RS_LEFT, RS_NOP},    // 4
            {RS_RIGHT, RS_LEFT, RS_STRAIGHT, RS_RIGHT, RS_NOP},   // 5
            {RS_LEFT, RS_STRAIGHT, RS_RIGHT, RS_LEFT, RS_NOP},    // 6
            {RS_RIGHT, RS_STRAIGHT, RS_LEFT, RS_RIGHT, RS_NOP},   // 7
            {RS_LEFT, RS_RIGHT, RS_STRAIGHT, RS_RIGHT, RS_NOP},   // 8
            {RS_RIGHT, RS_LEFT, RS_STRAIGHT, RS_LEFT, RS_NOP},    // 9
            {RS_RIGHT, RS_STRAIGHT, RS_RIGHT, RS_LEFT, RS_NOP},   // 10
            {RS_LEFT, RS_STRAIGHT, RS_LEFT, RS_RIGHT, RS_NOP},    // 11
            {RS_LEFT, RS_STRAIGHT, RS_RIGHT, RS_NOP, RS_NOP},     // 12
            {RS_RIGHT, RS_STRAIGHT, RS_LEFT, RS_NOP, RS_NOP},     // 13
            {RS_LEFT, RS_STRAIGHT, RS_LEFT, RS_NOP, RS_NOP},      // 14
            {RS_RIGHT, RS_STRAIGHT, RS_RIGHT, RS_NOP, RS_NOP},    // 15
            {RS_LEFT, RS_RIGHT, RS_STRAIGHT, RS_LEFT, RS_RIGHT},  // 16
            {RS_RIGHT, RS_LEFT, RS_STRAIGHT, RS_RIGHT, RS_LEFT}   // 17
    };

    ReedsShepp::ReedsSheppPath::ReedsSheppPath(const ReedsShepp::ReedsSheppPathSegmentType *type, double t, double u,
                                               double v, double w, double x):type_(type)
    {
        length_[0] = t;
        length_[1] = u;
        length_[2] = v;
        length_[3] = w;
        length_[4] = x;
        totalLength_ = fabs(t) + fabs(u) + fabs(v) + fabs(w) + fabs(x);
    }

    ReedsShepp::ReedsShepp(float rho):rho_(rho)
    {
    }

    ReedsShepp::ReedsSheppPath ReedsShepp::plan(ReedsShepp::pos start, ReedsShepp::pos end)
    {
        double x1 = start.x, y1 = start.y, th1 = start.angle;
        double x2 = end.x, y2 = end.y, th2 = end.angle;
        double dx = x2 - x1, dy = y2 - y1, c = cos(th1), s = sin(th1);
        double x = c * dx + s * dy, y = -s * dx + c * dy, phi = th2 - th1;
        return plan(x / rho_, y / rho_, phi);
    }
    double ReedsShepp::distance(pos start, pos end)
    {
        return rho_ * plan(start,end).length();
    }
    void ReedsShepp::interpolate(const pos *from, const ReedsSheppPath &path, double t, pos *state) const
    {
        double seg = t * path.length(),phi,v;
        pos s(0,0,from->angle);
        for (unsigned int i = 0; i < 5 && seg > 0; ++i)
        {
            if (path.length_[i] < 0)
            {
                v = std::max(-seg, path.length_[i]);
                seg += v;
            }
            else
            {
                v = std::min(seg, path.length_[i]);
                seg -= v;
            }
            phi = s.angle;
            switch (path.type_[i])
            {
                case RS_LEFT:
                    s.x = (float)(s.x + sin(phi + v) - sin(phi));
                    s.y = (float)(s.y - cos(phi + v) + cos(phi));
                    s.angle = (float)(phi + v);
                    break;
                case RS_RIGHT:
                    s.x = (float)(s.x - sin(phi - v) + sin(phi));
                    s.y = (float)(s.y + cos(phi - v) - cos(phi));
                    s.angle = (float)(phi - v);
                    break;
                case RS_STRAIGHT:
                    s.x = (float)(s.x + v * cos(phi));
                    s.y = (float)(s.y + v * sin(phi));
                    break;
                case RS_NOP:
                    break;
            }
        }
        state->x = s.x * rho_ + from->x;
        state->y = s.y * rho_ + from->y;
        state->angle = s.angle;
    }
    ReedsShepp::ReedsSheppPath ReedsShepp::plan(double x, double y, double phi)
    {
        ReedsSheppPath path;
        CSC(x, y, phi, path);
        CCC(x, y, phi, path);
        CCCC(x, y, phi, path);
        CCSC(x, y, phi, path);
        CCSCC(x, y, phi, path);
        return path;
    }

    void ReedsShepp::CCSC(double x, double y, double phi, ReedsShepp::ReedsSheppPath &path)
    {
        double t, u, v, Lmin = path.length() - .5 * pi, L;
        if (LpRmSmLm(x, y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
        {
            path = ReedsSheppPath(reedsSheppPathType[4], t, -.5 * pi, u, v);
            Lmin = L;
        }
        if (LpRmSmLm(-x, y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip
        {
            path = ReedsSheppPath(reedsSheppPathType[4], -t, .5 * pi, -u, -v);
            Lmin = L;
        }
        if (LpRmSmLm(x, -y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // reflect
        {
            path = ReedsSheppPath(reedsSheppPathType[5], t, -.5 * pi, u, v);
            Lmin = L;
        }
        if (LpRmSmLm(-x, -y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip + reflect
        {
            path = ReedsSheppPath(reedsSheppPathType[5], -t, .5 * pi, -u, -v);
            Lmin = L;
        }

        if (LpRmSmRm(x, y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
        {
            path = ReedsSheppPath(reedsSheppPathType[8], t, -.5 * pi, u, v);
            Lmin = L;
        }
        if (LpRmSmRm(-x, y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip
        {
            path = ReedsSheppPath(reedsSheppPathType[8], -t, .5 * pi, -u, -v);
            Lmin = L;
        }
        if (LpRmSmRm(x, -y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // reflect
        {
            path = ReedsSheppPath(reedsSheppPathType[9], t, -.5 * pi, u, v);
            Lmin = L;
        }
        if (LpRmSmRm(-x, -y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip + reflect
        {
            path = ReedsSheppPath(reedsSheppPathType[9], -t, .5 * pi, -u, -v);
            Lmin = L;
        }

        // backwards
        double xb = x * cos(phi) + y * sin(phi), yb = x * sin(phi) - y * cos(phi);
        if (LpRmSmLm(xb, yb, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
        {
            path = ReedsSheppPath(reedsSheppPathType[6], v, u, -.5 * pi, t);
            Lmin = L;
        }
        if (LpRmSmLm(-xb, yb, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip
        {
            path = ReedsSheppPath(reedsSheppPathType[6], -v, -u, .5 * pi, -t);
            Lmin = L;
        }
        if (LpRmSmLm(xb, -yb, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // reflect
        {
            path = ReedsSheppPath(reedsSheppPathType[7], v, u, -.5 * pi, t);
            Lmin = L;
        }
        if (LpRmSmLm(-xb, -yb, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip + reflect
        {
            path = ReedsSheppPath(reedsSheppPathType[7], -v, -u, .5 * pi, -t);
            Lmin = L;
        }

        if (LpRmSmRm(xb, yb, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
        {
            path = ReedsSheppPath(reedsSheppPathType[10], v, u, -.5 * pi, t);
            Lmin = L;
        }
        if (LpRmSmRm(-xb, yb, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip
        {
            path = ReedsSheppPath(reedsSheppPathType[10], -v, -u, .5 * pi, -t);
            Lmin = L;
        }
        if (LpRmSmRm(xb, -yb, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // reflect
        {
            path = ReedsSheppPath(reedsSheppPathType[11], v, u, -.5 * pi, t);
            Lmin = L;
        }
        if (LpRmSmRm(-xb, -yb, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip + reflect
            path = ReedsSheppPath(reedsSheppPathType[11], -v, -u, .5 * pi, -t);
    }

    void ReedsShepp::CCCC(double x, double y, double phi, ReedsShepp::ReedsSheppPath &path)
    {
        double t, u, v, Lmin = path.length(), L;
        if (LpRupLumRm(x, y, phi, t, u, v) && Lmin > (L = fabs(t) + 2. * fabs(u) + fabs(v)))
        {
            path = ReedsSheppPath(reedsSheppPathType[2], t, u, -u, v);
            Lmin = L;
        }
        if (LpRupLumRm(-x, y, -phi, t, u, v) && Lmin > (L = fabs(t) + 2. * fabs(u) + fabs(v)))  // timeflip
        {
            path = ReedsSheppPath(reedsSheppPathType[2], -t, -u, u, -v);
            Lmin = L;
        }
        if (LpRupLumRm(x, -y, -phi, t, u, v) && Lmin > (L = fabs(t) + 2. * fabs(u) + fabs(v)))  // reflect
        {
            path = ReedsSheppPath(reedsSheppPathType[3], t, u, -u, v);
            Lmin = L;
        }
        if (LpRupLumRm(-x, -y, phi, t, u, v) && Lmin > (L = fabs(t) + 2. * fabs(u) + fabs(v)))  // timeflip + reflect
        {
            path = ReedsSheppPath(reedsSheppPathType[3], -t, -u, u, -v);
            Lmin = L;
        }

        if (LpRumLumRp(x, y, phi, t, u, v) && Lmin > (L = fabs(t) + 2. * fabs(u) + fabs(v)))
        {
            path = ReedsSheppPath(reedsSheppPathType[2], t, u, u, v);
            Lmin = L;
        }
        if (LpRumLumRp(-x, y, -phi, t, u, v) && Lmin > (L = fabs(t) + 2. * fabs(u) + fabs(v)))  // timeflip
        {
            path = ReedsSheppPath(reedsSheppPathType[2], -t, -u, -u, -v);
            Lmin = L;
        }
        if (LpRumLumRp(x, -y, -phi, t, u, v) && Lmin > (L = fabs(t) + 2. * fabs(u) + fabs(v)))  // reflect
        {
            path = ReedsSheppPath(reedsSheppPathType[3], t, u, u, v);
            Lmin = L;
        }
        if (LpRumLumRp(-x, -y, phi, t, u, v) && Lmin > (L = fabs(t) + 2. * fabs(u) + fabs(v)))  // timeflip + reflect
            path = ReedsSheppPath(reedsSheppPathType[3], -t, -u, -u, -v);
    }

    void ReedsShepp::CCC(double x, double y, double phi, ReedsShepp::ReedsSheppPath &path)
    {
        double t, u, v, Lmin = path.length(), L;
        if (LpRmL(x, y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
        {
            path = ReedsSheppPath(reedsSheppPathType[0], t, u, v);
            Lmin = L;
        }
        if (LpRmL(-x, y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip
        {
            path = ReedsSheppPath(reedsSheppPathType[0], -t, -u, -v);
            Lmin = L;
        }
        if (LpRmL(x, -y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // reflect
        {
            path = ReedsSheppPath(reedsSheppPathType[1], t, u, v);
            Lmin = L;
        }
        if (LpRmL(-x, -y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip + reflect
        {
            path = ReedsSheppPath(reedsSheppPathType[1], -t, -u, -v);
            Lmin = L;
        }

        // backwards
        double xb = x * cos(phi) + y * sin(phi), yb = x * sin(phi) - y * cos(phi);
        if (LpRmL(xb, yb, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
        {
            path = ReedsSheppPath(reedsSheppPathType[0], v, u, t);
            Lmin = L;
        }
        if (LpRmL(-xb, yb, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip
        {
            path = ReedsSheppPath(reedsSheppPathType[0], -v, -u, -t);
            Lmin = L;
        }
        if (LpRmL(xb, -yb, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // reflect
        {
            path = ReedsSheppPath(reedsSheppPathType[1], v, u, t);
            Lmin = L;
        }
        if (LpRmL(-xb, -yb, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip + reflect
            path = ReedsSheppPath(reedsSheppPathType[1], -v, -u, -t);
    }

    void ReedsShepp::CSC(double x, double y, double phi, ReedsShepp::ReedsSheppPath &path)
    {
        double t, u, v, Lmin = path.length(), L;
        if (LpSpLp(x, y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
        {
            path = ReedsSheppPath(reedsSheppPathType[14], t, u, v);
            Lmin = L;
        }
        if (LpSpLp(-x, y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip
        {
            path = ReedsSheppPath(reedsSheppPathType[14], -t, -u, -v);
            Lmin = L;
        }
        if (LpSpLp(x, -y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // reflect
        {
            path = ReedsSheppPath(reedsSheppPathType[15], t, u, v);
            Lmin = L;
        }
        if (LpSpLp(-x, -y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip + reflect
        {
            path = ReedsSheppPath(reedsSheppPathType[15], -t, -u, -v);
            Lmin = L;
        }
        if (LpSpRp(x, y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
        {
            path = ReedsSheppPath(reedsSheppPathType[12], t, u, v);
            Lmin = L;
        }
        if (LpSpRp(-x, y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip
        {
            path = ReedsSheppPath(reedsSheppPathType[12], -t, -u, -v);
            Lmin = L;
        }
        if (LpSpRp(x, -y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // reflect
        {
            path = ReedsSheppPath(reedsSheppPathType[13], t, u, v);
            Lmin = L;
        }
        if (LpSpRp(-x, -y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip + reflect
            path = ReedsSheppPath(reedsSheppPathType[13], -t, -u, -v);
    }

    void ReedsShepp::CCSCC(double x, double y, double phi, ReedsShepp::ReedsSheppPath &path)
    {
        double t, u, v, Lmin = path.length() - pi, L;
        if (LpRmSLmRp(x, y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
        {
            path = ReedsSheppPath(reedsSheppPathType[16], t, -.5 * pi, u,
                                                        -.5 * pi, v);
            Lmin = L;
        }
        if (LpRmSLmRp(-x, y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip
        {
            path = ReedsSheppPath(reedsSheppPathType[16], -t, .5 * pi, -u,
                                                        .5 * pi, -v);
            Lmin = L;
        }
        if (LpRmSLmRp(x, -y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // reflect
        {
            path = ReedsSheppPath(reedsSheppPathType[17], t, -.5 * pi, u,
                                                        -.5 * pi, v);
            Lmin = L;
        }
        if (LpRmSLmRp(-x, -y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))  // timeflip + reflect
            path = ReedsSheppPath(reedsSheppPathType[17], -t, .5 * pi, -u,
                                                        .5 * pi, -v);
    }

    bool ReedsShepp::isTraversable(Node3D* start,ReedsShepp::ReedsSheppPath *path, CollisionDetection *map) const
    {
        for(int i = 0;i < path->length()*3;i ++)
        {
            float t = (float)(i) / (float)(path->length()*3);
            ReedsShepp::pos p;
            ReedsShepp::pos s(start->getX(),start->getY(),start->getT());
            interpolate(&s,*path,t,&p);
            Node2D pt((int)(p.x),(int)(p.y),0,0, nullptr);
            pt.setIdx(map->getWidthSize());
            if(!map->isTraversable(&pt))
            {
                return false;
            }
        }
        return true;
    }
}


