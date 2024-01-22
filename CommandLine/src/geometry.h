#ifndef GEOMETRY_H
#define GEOMETRY_H

#include "circle.h"
#include "plane.h"
#include "cylinder.h"
#include "connection.h"

class Geometry
{
public:
    Geometry()
    {

    }

    ~Geometry()
    {
        clearPrimitives();
    }

    void addCircle(Circle *circle)
    {
        mCircles.push_back(circle);
    }

    void removeCircle(size_t index)
    {
        delete mCircles[index];
        mCircles.erase(mCircles.begin() + index);
    }

    size_t numCircles() const
    {
        return mCircles.size();
    }

    Circle* circle(size_t index) const
    {
        return mCircles[index];
    }

    void clearCircles()
    {
        for (Circle *circle : mCircles)
        {
            delete circle;
        }
        mCircles.clear();
    }

    void addPlane(Plane *plane)
    {
        mPlanes.push_back(plane);
    }

    void removePlane(size_t index)
    {
        delete mPlanes[index];
        mPlanes.erase(mPlanes.begin() + index);
    }

    size_t numPlanes() const
    {
        return mPlanes.size();
    }

    Plane* plane(size_t index) const
    {
        return mPlanes[index];
    }

    void clearPlanes()
    {
        for (Plane *plane : mPlanes)
        {
            delete plane;
        }
        mPlanes.clear();
    }

    void addCylinder(Cylinder *cylinder)
    {
        mCylinders.push_back(cylinder);
    }

    void removeCylinder(size_t index)
    {
        delete mCylinders[index];
        mCylinders.erase(mCylinders.begin() + index);
    }

    size_t numCylinders() const
    {
        return mCylinders.size();
    }

    Cylinder* cylinder(size_t index) const
    {
        return mCylinders[index];
    }

    void clearCylinders()
    {
        for (Cylinder *cylinder : mCylinders)
        {
            delete cylinder;
        }
        mCylinders.clear();
        clearConnections();
    }

    void addConnection(Connection *connection)
    {
        mConnections.push_back(connection);
    }

    void removeConnection(size_t index)
    {
        delete mConnections[index];
        mConnections.erase(mConnections.begin() + index);
    }

    size_t numConnections() const
    {
        return mConnections.size();
    }

    Connection* connection(size_t index) const
    {
        return mConnections[index];
    }

    void clearConnections()
    {
        for (Connection *connection : mConnections)
        {
            delete connection;
        }
        mConnections.clear();
    }

    void clearPrimitives()
    {
        clearCircles();
        clearPlanes();
        clearCylinders();
    }

private:
    std::vector<Circle*> mCircles;
    std::vector<Plane*> mPlanes;
    std::vector<Cylinder*> mCylinders;
    std::vector<Connection*> mConnections;

};

#endif // GEOMETRY_H
