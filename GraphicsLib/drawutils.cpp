#include "drawutils.h"

#include "geometryutils.h"
#include "angleutils.h"

void DrawUtils::circle(VertexBufferCreator &buffer, const Eigen::Vector3f &center, const Eigen::Vector3f &normal, float radius, size_t numSlices)
{
    Eigen::Vector3f v1;
    Eigen::Vector3f v2;
    GeometryUtils::orthogonalBasis(normal, v1, v2);

    Eigen::Vector3f *points = new Eigen::Vector3f[numSlices];
    for (size_t i = 0; i < numSlices; i++)
    {
        float theta = AngleUtils::deg2rad(360.0f / numSlices * i);
        points[i] = center + v1 * std::sin(theta) * radius + v2 * std::cos(theta) * radius;
    }

    buffer.normal(normal);
    for (size_t i = 0; i <= numSlices; i++)
    {
        buffer.line(points[i % numSlices], points[(i + 1) % numSlices]);
    }

    delete[] points;
}

void DrawUtils::rect(VertexBufferCreator &buffer, Rect3d rect)
{
    Eigen::Vector3f min = rect.bottomLeft();
    Eigen::Vector3f max = rect.topRight();

    Eigen::Vector3f vertices[] = {
        Eigen::Vector3f(min.x(), min.y(), max.z()),
        Eigen::Vector3f(max.x(), max.y(), min.z()),
        Eigen::Vector3f(max.x(), max.y(), max.z()),
        Eigen::Vector3f(max.x(), min.y(), max.z()),
        Eigen::Vector3f(max.x(), min.y(), min.z()),
        Eigen::Vector3f(min.x(), max.y(), max.z()),
        Eigen::Vector3f(min.x(), max.y(), min.z()),
        Eigen::Vector3f(min.x(), min.y(), min.z())
    };

    buffer.normal(0.0f, 0.0f, 1.0f);
    buffer.quad(vertices[5], vertices[2], vertices[3], vertices[0]);
    buffer.normal(0.0f, 0.0f, -1.0f);
    buffer.quad(vertices[6], vertices[1], vertices[4], vertices[7]);
    buffer.normal(0.0f, 1.0f, 0.0f);
    buffer.quad(vertices[5], vertices[6], vertices[1], vertices[2]);
    buffer.normal(0.0f, -1.0f, 0.0f);
    buffer.quad(vertices[7], vertices[4], vertices[3], vertices[0]);
    buffer.normal(-1.0f, 0.0f, 0.0f);
    buffer.quad(vertices[2], vertices[1], vertices[4], vertices[3]);
    buffer.normal(1.0f, 0.0f, 0.0f);
    buffer.quad(vertices[5], vertices[6], vertices[7], vertices[0]);
}

void DrawUtils::plane(VertexBufferCreator &buffer, const Plane &plane)
{
    Eigen::Vector3f v1 = plane.center() - plane.basisU() - plane.basisV();
    Eigen::Vector3f v2 = plane.center() - plane.basisU() + plane.basisV();
    Eigen::Vector3f v3 = plane.center() + plane.basisU() - plane.basisV();
    Eigen::Vector3f v4 = plane.center() + plane.basisU() + plane.basisV();
    buffer.quad(v1, v2, v4, v3);
}

void DrawUtils::outlineRect(VertexBufferCreator &buffer, Rect3d rect)
{
    Eigen::Vector3f min = rect.bottomLeft();
    Eigen::Vector3f max = rect.topRight();

    Eigen::Vector3f vertices[] = {
        Eigen::Vector3f(min.x(), min.y(), max.z()),
        Eigen::Vector3f(max.x(), max.y(), min.z()),
        Eigen::Vector3f(max.x(), max.y(), max.z()),
        Eigen::Vector3f(max.x(), min.y(), max.z()),
        Eigen::Vector3f(max.x(), min.y(), min.z()),
        Eigen::Vector3f(min.x(), max.y(), max.z()),
        Eigen::Vector3f(min.x(), max.y(), min.z()),
        Eigen::Vector3f(min.x(), min.y(), min.z())
    };

    buffer.line(vertices[0], vertices[3]);
    buffer.line(vertices[3], vertices[2]);
    buffer.line(vertices[2], vertices[5]);
    buffer.line(vertices[5], vertices[0]);
    buffer.line(vertices[0], vertices[7]);
    buffer.line(vertices[7], vertices[4]);
    buffer.line(vertices[4], vertices[1]);
    buffer.line(vertices[1], vertices[6]);
    buffer.line(vertices[6], vertices[7]);
    buffer.line(vertices[4], vertices[3]);
    buffer.line(vertices[1], vertices[2]);
    buffer.line(vertices[6], vertices[5]);
}

void DrawUtils::outlineSphere(VertexBufferCreator &buffer, const Eigen::Vector3f &center, float radius, size_t numCirclesLon, size_t numCirclesLat)
{
    for (size_t i = 0; i < numCirclesLat; i++)
    {
        float d = std::cos(AngleUtils::deg2rad(360.0f / numCirclesLat * i - 180.0f)) * radius;
        float a = std::sqrt(radius * radius - d * d);
        circle(buffer, center + Eigen::Vector3f(0, d, 0), Eigen::Vector3f(0, 1, 0), a);
    }
    for (size_t i = 0; i < numCirclesLon; i++)
    {
        float theta = AngleUtils::deg2rad(360.0f / numCirclesLon * i);
        Eigen::Vector3f normal = Eigen::Vector3f(std::cos(theta), 0, -std::sin(theta)).cross(Eigen::Vector3f(0, 1, 0)).normalized();
        circle(buffer, center, normal, radius);
    }
}

void DrawUtils::outlineCylinder(VertexBufferCreator &buffer, const Cylinder &cylinder, size_t numCircles, size_t numSlices)
{
    Eigen::Vector3f v1;
    Eigen::Vector3f v2;
    GeometryUtils::orthogonalBasis(cylinder.axis(), v1, v2);
    for (size_t i = 0; i < numSlices; i++)
    {
        float theta = AngleUtils::deg2rad(360.0f / numSlices * i);
        Eigen::Vector3f p1 = cylinder.center() - cylinder.axis() * cylinder.height() / 2.0f + v1 * std::sin(theta) * cylinder.radius() + v2 * std::cos(theta) * cylinder.radius();
        Eigen::Vector3f p2 = cylinder.center() + cylinder.axis() * cylinder.height() / 2.0f + v1 * std::sin(theta) * cylinder.radius() + v2 * std::cos(theta) * cylinder.radius();
        buffer.line(p1, p2);
    }
    for (size_t i = 0; i < numCircles; i++)
    {
        Eigen::Vector3f circleCenter = cylinder.center() + cylinder.axis() * (i / static_cast<float>(numCircles - 1) * cylinder.height() - cylinder.height() / 2.0f);
        circle(buffer, circleCenter, cylinder.axis(), cylinder.radius(), numSlices);
    }
}

void DrawUtils::outlinePlane(VertexBufferCreator &buffer, const Plane &plane, size_t numSlices)
{
    for (size_t i = 0; i <= numSlices; i++)
    {
        Eigen::Vector3f _u = plane.basisU() - plane.basisU() * 2 * i / (float)numSlices;
        Eigen::Vector3f _v = plane.basisV() - plane.basisV() * 2 * i / (float)numSlices;
        buffer.line(plane.center() - plane.basisU() + _v, plane.center() + plane.basisU() + _v);
        buffer.line(plane.center() - plane.basisV() + _u, plane.center() + plane.basisV() + _u);
    }
}
