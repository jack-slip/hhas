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

        class Rectangle
        {
        private:
            Polygon poly_;
            float inscribed_radius_;
            float circumscribed_radius_;

            void computeRadii(const Vec2 &bottom_left, const Vec2 &bottom_right, const Vec2 &top_right, const Vec2 &top_left)
            {
                float width = bottom_left.dist(bottom_right);
                float height = bottom_left.dist(top_left);

                inscribed_radius_ = 0.5 * std::min(width, height);
                circumscribed_radius_ = 0.5 * std::hypot(width, height);
            }

        public:
            // Constructor: Define a rectangle using all four corners
            Rectangle(const Vec2 &bottom_left, const Vec2 &bottom_right, const Vec2 &top_right, const Vec2 &top_left)
            {
                poly_.addVertex(bottom_left);
                poly_.addVertex(bottom_right);
                poly_.addVertex(top_right);
                poly_.addVertex(top_left);
                computeRadii(bottom_left, bottom_right, top_right, top_left);
            }
            Rectangle(const std::vector<Vec2> &vertices)
            {
                if (vertices.size() != 4)
                    throw std::runtime_error("Rectangle must be defined with 4 vertices");
                poly_.addVertices(vertices);
                computeRadii(vertices[0], vertices[1], vertices[2], vertices[3]);
            }

            std::vector<Vec2> getVertices() const { return poly_.v; }

            float getInscribedRadius() const { return inscribed_radius_; }
            float getCircumscribedRadius() const { return circumscribed_radius_; }

            // Move rectangle by x, y and rotate by yaw
            void move(const float &x, const float &y, const float &yaw)
            {
                poly_.move(x, y, yaw);
            }

            // Check if a point is inside the rectangle
            bool inside(const Vec2 &a) const
            {
                return poly_.inside(a);
            }

            // Get the distance between a point and the rectangle
            float dist(const Vec2 &a) const
            {
                return poly_.dist(a);
            }
        };
    }
}
#endif // HHAS_POLYGON_H