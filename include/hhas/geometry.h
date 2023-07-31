#ifndef HHAS_GEOMETRY_H
#define HHAS_GEOMETRY_H

#include <vector>
#include <cmath>
#include <limits>

namespace hhas
{
    namespace geometry
    {
        class Vec2
        {
        public:
            float c[2];
            Vec2(const float x, const float y)
            {
                c[0] = x;
                c[1] = y;
            }
            Vec2() { c[0] = c[1] = 0.0; }
            float &operator[](const int &i)
            {
                return c[i];
            }
            const float &operator[](const int &i) const
            {
                return c[i];
            }
            Vec2 operator-(const Vec2 &a) const
            {
                Vec2 out = *this;
                out[0] -= a[0];
                out[1] -= a[1];
                return out;
            }
            float cross(const Vec2 &a) const { return (*this)[0] * a[1] - (*this)[1] * a[0]; }
            float dot(const Vec2 &a) const { return (*this)[0] * a[0] + (*this)[1] * a[1]; }
            float dist(const Vec2 &a) const { return std::hypot((*this)[0] - a[0], (*this)[1] - a[1]); }
            float dist_line(const Vec2 &a, const Vec2 &b) const
            {
                return (b - a).cross((*this) - a) / b.dist(a);
            }
            float dist_linestrip(const Vec2 &a, const Vec2 &b) const
            {
                if ((b - a).dot((*this) - a) <= 0)
                    return this->dist(a);
                if ((a - b).dot((*this) - b) <= 0)
                    return this->dist(b);
                return std::abs(this->dist_line(a, b));
            }
        };

        class Circle
        {
        public:
            Vec2 center;
            float r;
            Circle(const Vec2 &c, const float &r) : center(c), r(r) {}
            Circle() : center(), r(0.0) {}

            bool inside(const Vec2 &a) const { return center.dist(a) < r; }

            void move(const float &x, const float &y)
            {
                center[0] += x;
                center[1] += y;
            }
        };

        class Polygon
        {
        public:
            std::vector<Vec2> v;
            void addVertex(const Vec2 &a) { v.push_back(a); }
            void addVertices(const std::vector<Vec2> &a)
            {
                for (auto &p : a)
                    v.push_back(p);
            }
            void move(const float &x, const float &y, const float &yaw)
            {
                const float cos_v = cosf(yaw);
                const float sin_v = sinf(yaw);
                for (auto &p : v)
                {
                    const auto tmp = p;
                    p[0] = cos_v * tmp[0] - sin_v * tmp[1] + x;
                    p[1] = sin_v * tmp[0] + cos_v * tmp[1] + y;
                }
            }
            bool inside(const Vec2 &a) const
            {
                int cn = 0;
                for (size_t i = 0; i < v.size() - 1; i++)
                {
                    auto &v1 = v[i];
                    auto &v2 = v[i + 1];
                    if ((v1[1] <= a[1] && a[1] < v2[1]) || (v2[1] <= a[1] && a[1] < v1[1]))
                    {
                        float lx;
                        lx = v1[0] + (v2[0] - v1[0]) * (a[1] - v1[1]) / (v2[1] - v1[1]);
                        if (a[0] < lx)
                            cn++;
                    }
                }
                return ((cn & 1) == 1);
            }
            float dist(const Vec2 &a) const
            {
                float dist = std::numeric_limits<float>::max();
                for (size_t i = 0; i < v.size() - 1; i++)
                {
                    auto &v1 = v[i];
                    auto &v2 = v[i + 1];
                    auto d = a.dist_linestrip(v1, v2);
                    if (d < dist)
                        dist = d;
                }
                return dist;
            }
        };
    }
}
#endif // HHAS_POLYGON_H