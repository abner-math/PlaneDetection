#ifndef SELECTER_H
#define SELECTER_H

#include <Eigen/Core>

#include "filter.h"

class SelectFilter : public Filter
{
public:
    enum SelectMode
    {
        POINT = 0,
        PLANE = 1,
        CYLINDER = 2,
        CONNECTION = 3
    };

    SelectFilter();

    void firstPoint(const Eigen::Vector2f &firstPoint)
    {
        mFirstPoint = firstPoint;
    }

    const Eigen::Vector2f& firstPoint() const
    {
        return mFirstPoint;
    }

    void secondPoint(const Eigen::Vector2f &secondPoint)
    {
        mSecondPoint = secondPoint;
    }

    const Eigen::Vector2f& secondPoint() const
    {
        return mSecondPoint;
    }

    void clear()
    {
        mFirstPoint = mSecondPoint = Eigen::Vector2f::Zero();
        mIsSelecting = true;
    }

    void setSelect()
    {
        mIsSelecting = true;
    }

    void setDeselect()
    {
        mIsSelecting = false;
    }

    bool isSelecting() const
    {
        return mIsSelecting;
    }

    void mode(SelectMode mode)
    {
        mMode = mode;
    }

    SelectMode mode() const
    {
        return mMode;
    }

    void select(size_t index);

    void selectAll();

    void deselect(size_t index);

    void clearSelection();

    bool isSelected(size_t index) const;

    bool isAnySelected() const;

    void invertSelection();

    void update(size_t numPoints, size_t numPlanes, size_t numCylinders, size_t numConnectors);

private:
    Eigen::Vector2f mFirstPoint;
    Eigen::Vector2f mSecondPoint;
    bool mIsSelecting;
    SelectMode mMode;
    std::vector<bool> mSelectedPoints;
    std::vector<bool> mSelectedPlanes;
    std::vector<bool> mSelectedCylinders;
    std::vector<bool> mSelectedConnections;

    void filter(FrameBuffer *buffer) override;

};

#endif // SELECTER_H
