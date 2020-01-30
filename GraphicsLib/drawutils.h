#ifndef DRAWUTILS_H
#define DRAWUTILS_H

#include "rect.h"
#include "cylinder.h"
#include "plane.h"
#include "vertexbuffercreator.h"

class DrawUtils
{
public:
    static void circle(VertexBufferCreator &buffer, const Eigen::Vector3f &center, const Eigen::Vector3f &normal, float radius, size_t numSlices = 72);

    static void rect(VertexBufferCreator &buffer, Rect3d rect);

    static void plane(VertexBufferCreator &buffer, const Plane &plane);

    static void outlineRect(VertexBufferCreator &buffer, Rect3d rect);

    static void outlineSphere(VertexBufferCreator &buffer, const Eigen::Vector3f &center,
                              float radius, size_t numCirclesLon = 18, size_t numCirclesLat = 18);

    static void outlineCylinder(VertexBufferCreator &buffer, const Cylinder &cylinder,
                         size_t numCircles = 36, size_t numSlices = 72);

    static void outlinePlane(VertexBufferCreator &buffer, const Plane &plane, size_t numSlices = 36);

};

#endif // DRAWUTILS_H
