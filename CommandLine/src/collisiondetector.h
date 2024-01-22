#ifndef COLLISIONDETECTOR_H
#define COLLISIONDETECTOR_H

#include "point.h"
#include "line.h"
#include "rect.h"
#include "circle.h"
#include "plane.h"
#include "cylinder.h"

class CollisionDetector
{
public:
    // reference: http://www.wyrmtale.com/blog/2013/115/2d-line-intersection-in-c
    /**
     * @brief Find the intersection between two lines, if it exists
     * @param line1
     *      First line
     * @param line2
     *      Second line
     * @param intersection
     *      The intersection point between the two lines
     * @return
     *      True if the two lines intersect, false otherwise
     */
    static bool rayRayCollision2d(Line2d line1, Line2d line2, Eigen::Vector2f &intersection)
    {
        // Given a line of equation ax + by = c
        // Get (a, b, c) of the first line
        float a1 = line1.p2().y() - line1.p1().y();
        float b1 = line1.p1().x() - line1.p2().x();
        float c1 = a1 * line1.p1().x() + b1 * line1.p1().y();

        // Get (a, b, c) of the second line
        float a2 = line2.p2().y() - line2.p1().y();
        float b2 = line2.p1().x() - line2.p2().x();
        float c2 = a2 * line2.p1().x() + b2 * line2.p1().y();

        // Get delta and check if the lines are parallel
        float delta = a1 * b2 - a2 * b1;
        if (std::abs(delta) < std::numeric_limits<float>::epsilon())
            return false;

        float intersectX = (b2 * c1 - b1 * c2) / delta;
        float intersectY = (a1 * c2 - a2 * c1) / delta;

        intersection = Eigen::Vector2f(intersectX, intersectY);

        return true;
    }

    // reference: http://www.realtimerendering.com/intersections.html#IV74
    /**
     * @brief Find the intersection between two lines, if it exists
     * @param line1
     *      First line
     * @param line2
     *      Second line
     * @param intersection
     *      The intersection point between the two lines
     * @return
     *      True if the two lines intersect, false otherwise
     */
    static bool rayRayCollision3d(Line3d line1, Line3d line2, Eigen::Vector3f &intersection)
    {
        Eigen::Vector3f direction1 = (line1.p2() - line1.p1()).normalized();
        Eigen::Vector3f direction2 = (line2.p2() - line2.p1()).normalized();
        Eigen::Vector3f cross = direction1.cross(direction2);
        float squaredLength = cross.squaredNorm();
        if (squaredLength < std::numeric_limits<float>::epsilon())
            return false;
        Eigen::Matrix3f matrix(3, 3);
        matrix.col(0) = line2.p1() - line1.p1();
        matrix.col(1) = direction2;
        matrix.col(2) = cross;
        float t = matrix.determinant() / squaredLength;
        intersection = line1.p1() + t * direction1;
        if (line2.distanceToPoint(intersection) > 1e-5)
            return false;
        return true;
    }

    // reference: https://tavianator.com/fast-branchless-raybounding-box-intersections/
    static bool rayBoxCollision2d(const Point2d &ray, Rect2d box)
    {
        float tx1 = (box.bottomLeft().x() - ray.position().x()) * ray.invNormal().x();
        float tx2 = (box.topRight().x() - ray.position().x()) * ray.invNormal().x();

        float tmin = std::min(tx1, tx2);
        float tmax = std::max(tx1, tx2);

        float ty1 = (box.bottomLeft().y() - ray.position().y()) * ray.invNormal().y();
        float ty2 = (box.topRight().y() - ray.position().y()) * ray.invNormal().y();

        tmin = std::max(tmin, std::min(ty1, ty2));
        tmax = std::min(tmax, std::max(ty1, ty2));

        return tmax >= tmin;
    }

    static bool rayBoxCollision3d(const Point3d &ray, Rect3d box)
    {
        float tx1 = (box.bottomLeft().x() - ray.position().x()) * ray.invNormal().x();
        float tx2 = (box.topRight().x() - ray.position().x()) * ray.invNormal().x();

        float tmin = std::min(tx1, tx2);
        float tmax = std::max(tx1, tx2);

        float ty1 = (box.bottomLeft().y() - ray.position().y()) * ray.invNormal().y();
        float ty2 = (box.topRight().y() - ray.position().y()) * ray.invNormal().y();

        tmin = std::max(tmin, std::min(ty1, ty2));
        tmax = std::min(tmax, std::max(ty1, ty2));

        float tz1 = (box.bottomLeft().z() - ray.position().z()) * ray.invNormal().z();
        float tz2 = (box.topRight().z() - ray.position().z()) * ray.invNormal().z();

        tmin = std::max(tmin, std::min(tz1, tz2));
        tmax = std::min(tmax, std::max(tz1, tz2));

        return tmax >= tmin;
    }

    static bool circleBoxCollision(const Circle &circle, Rect2d box)
    {
        return (box.closestPointToPoint(circle.center()) - circle.center()).norm() < circle.radius();
    }

    static bool planeBoxCollision(const Plane &plane, Rect3d box)
    {
        std::vector<Eigen::Vector3f> vertices;
        box.getVertices(vertices);
        bool sign = plane.getSignedDistanceFromSurface(vertices[0]) > 0;
        for (size_t i = 1; i < vertices.size(); i++)
        {
            bool anotherSign = plane.getSignedDistanceFromSurface(vertices[i]) > 0;
            if (sign != anotherSign) return true;
        }
        return false;
    }

    static bool cylinderBoxCollision(const Cylinder &cylinder, Rect3d box)
    {
        std::vector<Eigen::Vector3f> vertices;
        box.getVertices(vertices);

        std::vector<Line3d> edges = {
            Line3d(vertices[0], vertices[3]),
            Line3d(vertices[3], vertices[2]),
            Line3d(vertices[2], vertices[5]),
            Line3d(vertices[5], vertices[0]),
            Line3d(vertices[0], vertices[7]),
            Line3d(vertices[7], vertices[4]),
            Line3d(vertices[4], vertices[1]),
            Line3d(vertices[1], vertices[6]),
            Line3d(vertices[6], vertices[7]),
            Line3d(vertices[4], vertices[3]),
            Line3d(vertices[1], vertices[2]),
            Line3d(vertices[6], vertices[5])
        };

        Eigen::Vector3f basisU, basisV;
        GeometryUtils::orthogonalBasis(cylinder.axis(), basisU, basisV);
        Eigen::Vector2f projectedCenter = GeometryUtils::projectOntoOrthogonalBasis(cylinder.center(), basisU, basisV);
        for (size_t i = 0; i < edges.size(); i++)
        {
            Line2d projectedEdge = GeometryUtils::projectOntoOrthogonalBasis(edges[i], basisU, basisV);
            projectedEdge.segment() = true;
            Eigen::Vector2f nearestPointInEdge = projectedEdge.closestPointToPoint(projectedCenter);
            float dist = (nearestPointInEdge - projectedCenter).norm();
            if (dist <= cylinder.radius()) return true;
        }
        return false;
    }

};

#endif // COLLISIONDETECTOR_H
