#include "selectfilter.h"

#include "drawutils.h"

SelectFilter::SelectFilter()
    : mMode(POINT)
{
    clear();
}

void SelectFilter::select(size_t index)
{
    switch (mMode)
    {
    case POINT:
        mSelectedPoints[index] = true;
        break;
    case PLANE:
        mSelectedPlanes[index] = true;
        break;
    case CYLINDER:
        mSelectedCylinders[index] = true;
        break;
    case CONNECTION:
        mSelectedConnections[index] = true;
        break;
    }
}

void SelectFilter::selectAll()
{
    switch (mMode)
    {
    case POINT:
        std::fill(mSelectedPoints.begin(), mSelectedPoints.end(), true);
        break;
    case PLANE:
        std::fill(mSelectedPlanes.begin(), mSelectedPlanes.end(), true);
        break;
    case CYLINDER:
        std::fill(mSelectedCylinders.begin(), mSelectedCylinders.end(), true);
        break;
    case CONNECTION:
        std::fill(mSelectedConnections.begin(), mSelectedConnections.end(), true);
        break;
    }
}

void SelectFilter::deselect(size_t index)
{
    switch (mMode)
    {
    case POINT:
        mSelectedPoints[index] = false;
        break;
    case PLANE:
        mSelectedPlanes[index] = false;
        break;
    case CYLINDER:
        mSelectedCylinders[index] = false;
        break;
    case CONNECTION:
        mSelectedConnections[index] = false;
        break;
    }
}

void SelectFilter::clearSelection()
{
    switch (mMode)
    {
    case POINT:
        std::fill(mSelectedPoints.begin(), mSelectedPoints.end(), false);
        break;
    case PLANE:
        std::fill(mSelectedPlanes.begin(), mSelectedPlanes.end(), false);
        break;
    case CYLINDER:
        std::fill(mSelectedCylinders.begin(), mSelectedCylinders.end(), false);
        break;
    case CONNECTION:
        std::fill(mSelectedConnections.begin(), mSelectedConnections.end(), false);
        break;
    }
}

bool SelectFilter::isSelected(size_t index) const
{
    switch (mMode)
    {
    case POINT:
        return mSelectedPoints[index];
    case PLANE:
        return mSelectedPlanes[index];
    case CYLINDER:
        return mSelectedCylinders[index];
    case CONNECTION:
        return mSelectedConnections[index];
    }
    return false;
}

void SelectFilter::invertSelection()
{
    switch (mMode)
    {
    case POINT:
        for (size_t i = 0; i < mSelectedPoints.size(); i++)
        {
            mSelectedPoints[i] = !mSelectedPoints[i];
        }
        break;
    case PLANE:
        for (size_t i = 0; i < mSelectedPlanes.size(); i++)
        {
            mSelectedPlanes[i] = !mSelectedPlanes[i];
        }
        break;
    case CYLINDER:
        for (size_t i = 0; i < mSelectedCylinders.size(); i++)
        {
            mSelectedCylinders[i] = !mSelectedCylinders[i];
        }
        break;
    case CONNECTION:
        for (size_t i = 0; i < mSelectedConnections.size(); i++)
        {
            mSelectedConnections[i] = !mSelectedConnections[i];
        }
        break;
    }
}

bool SelectFilter::isAnySelected() const
{
    switch (mMode)
    {
    case POINT:
        for (size_t i = 0; i < mSelectedPoints.size(); i++)
        {
            if (mSelectedPoints[i])
            {
                return true;
            }
        }
        return false;
    case PLANE:
        for (size_t i = 0; i < mSelectedPlanes.size(); i++)
        {
            if (mSelectedPlanes[i])
            {
                return true;
            }
        }
        return false;
    case CYLINDER:
        for (size_t i = 0; i < mSelectedCylinders.size(); i++)
        {
            if (mSelectedCylinders[i])
            {
                return true;
            }
        }
        return false;
    case CONNECTION:
        for (size_t i = 0; i < mSelectedConnections.size(); i++)
        {
            if (mSelectedConnections[i])
            {
                return true;
            }
        }
        return false;
    }
    return false;
}

void SelectFilter::update(size_t numPoints, size_t numPlanes, size_t numCylinders, size_t numConnections)
{
    mSelectedPoints = std::vector<bool>(numPoints, false);
    mSelectedPlanes = std::vector<bool>(numPlanes, false);
    mSelectedCylinders = std::vector<bool>(numCylinders, false);
    mSelectedConnections = std::vector<bool>(numConnections, false);
}

void SelectFilter::filter(FrameBuffer *buffer)
{
    if (mFirstPoint == mSecondPoint) return;
    Eigen::Vector3f color;
    if (mIsSelecting)
    {
        color = Eigen::Vector3f(0, 1, 0);
    }
    else
    {
        color = Eigen::Vector3f(1, 0, 0);
    }
    float alpha = 0.5f;
    float minX = std::min(mFirstPoint.x(), mSecondPoint.x());
    float maxX = std::max(mFirstPoint.x(), mSecondPoint.x());
    float minY = std::min(mFirstPoint.y(), mSecondPoint.y());
    float maxY = std::max(mFirstPoint.y(), mSecondPoint.y());
    for (int x = minX; x <= maxX; x++)
    {
        for (int y = minY; y <= maxY; y++)
        {
            float a;
            if (x == minX || x == maxX || y == minY || y == maxY)
            {
                a = 1.0f;
            }
            else
            {
                a = alpha;
            }
            float r, g, b;
            buffer->pixelColor(x, y, r, g, b);
            r = color.x() * a + r * (1 - a);
            g = color.y() * a + g * (1 - a);
            b = color.z() * a + b * (1 - a);
            buffer->setPixelColor(x, y, r, g, b);
        }
    }
}
