#ifndef POINTCLOUD_H
#define POINTCLOUD_H

#include <mutex>
#include <vector>
#include <set>

#include "point.h"
#include "rect.h"
#include "geometry.h"
#include "connectivitygraph.h"

template <size_t DIMENSION>
class PointCloud
{
public:
    typedef typename Point<DIMENSION>::Vector Vector;

    enum Mode
    {
        COLOR = 1,
        INTENSITY = 2,
        NORMAL = 4,
        NORMAL_CONFIDENCE = 8,
        CURVATURE = 16,
        ALL = 31
    };

    PointCloud(const std::vector<Point<DIMENSION> > &points, size_t mode = ALL)
        : mPoints(points)
        , mMode(mode)
        , mConnectivity(NULL)
        , mGeometry(new Geometry)
    {
        static_assert(DIMENSION > 0, "Dimension must be greater than zero.");
        update();
    }

    PointCloud(size_t size, size_t mode)
        : mMode(mode)
        , mConnectivity(NULL)
        , mGeometry(new Geometry)
    {
        mPoints.resize(size);
        static_assert(DIMENSION > 0, "Dimension must be greater than zero.");
    }

    PointCloud(size_t mode = ALL)
        : mMode(mode)
        , mConnectivity(NULL)
        , mGeometry(new Geometry)
    {
        static_assert(DIMENSION > 0, "Dimension must be greater than zero.");
    }

    virtual ~PointCloud()
    {
        if (mConnectivity != NULL) delete mConnectivity;
        delete mGeometry;
    }

    size_t mode() const
    {
        return mMode;
    }

    void mode(size_t mode)
    {
        mMode = mode;
    }

    bool hasMode(size_t mode) const
    {
        return mMode & mode;
    }

    Point<DIMENSION>& operator[](size_t index)
    {
        return mPoints[index];
    }

    const Point<DIMENSION>& at(size_t index) const
    {
        return mPoints[index];
    }

    void add(const Point<DIMENSION> &point)
    {
        mMutex.lock();
        mPoints.push_back(point);
        mMutex.unlock();
    }

    void remove(int index)
    {
        mMutex.lock();
        mPoints.erase(mPoints.begin() + index);
        mMutex.unlock();
    }

    size_t size() const
    {
        return mPoints.size();
    }

    Vector center() const
    {
        return mCenter;
    }

    Rect<DIMENSION> extension() const
    {
        return mExtension;
    }

    void clear()
    {
        mMutex.lock();
        mPoints.clear();
        mMutex.unlock();
    }

    void update()
    {
        calculateCenter();
        calculateExtension();
    }

    void calculateCenter()
    {
        if (mPoints.empty()) mCenter = Point<DIMENSION>::Vector::Zero();
        else
        {
            mCenter = Vector::Zero();
            for (const Point<DIMENSION> &point : mPoints)
            {
                mCenter += point.position();
            }
            mCenter /= mPoints.size();
        }
    }

    void calculateExtension()
    {
        Vector min;
        Vector max;
        float maxValue = std::numeric_limits<float>::max();
        min = Vector::Constant(maxValue);
        max = Vector::Constant(-maxValue);
        for (const Point<DIMENSION> &point : mPoints)
        {
            for (unsigned int i = 0; i < DIMENSION; i++)
            {
                min(i) = std::min(min(i), point.position()(i));
                max(i) = std::max(max(i), point.position()(i));
            }
        }
        mExtension = Rect<DIMENSION>(min, max);
    }

    bool hasConnectivity() const
    {
        return mConnectivity != NULL;
    }

    ConnectivityGraph* connectivity() const
    {
        return mConnectivity;
    }

    void connectivity(ConnectivityGraph *connectivity)
    {
        if (mConnectivity != NULL) delete mConnectivity;
        mConnectivity = connectivity;
    }

    Geometry* geometry() const
    {
        return mGeometry;
    }

    void geometry(Geometry *geometry)
    {
        if (mGeometry != NULL) delete mGeometry;
        mGeometry = geometry;
    }

protected:
    std::vector<Point<DIMENSION> > mPoints;
    std::vector<bool> mVisiblePoints;
    std::mutex mMutex;
    size_t mMode;
    Vector mCenter;
    Rect<DIMENSION> mExtension;
    ConnectivityGraph *mConnectivity;
    Geometry *mGeometry;

};
\
template class PointCloud<2>;
template class PointCloud<3>;

typedef PointCloud<2> PointCloud2d;
typedef PointCloud<3> PointCloud3d;

#endif // POINTCLOUD_H
