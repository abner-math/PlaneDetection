#include "transformscaleworker.h"

#include <iostream>

TransformScaleWorker::TransformScaleWorker(const SelectFilter *filter)
    : mFilter(filter)
    , mPointCloud(NULL)
    , mSimplifiedPointCloud(NULL)
{

}

Eigen::Vector3f TransformScaleWorker::scale(const Eigen::Vector3f &v)
{
    return ((v - mPointCloud->center()).array() * mScale.array()) + mPointCloud->center().array();
}

void TransformScaleWorker::actions()
{
    if (mPointCloud == NULL) return;
    emit workerStatus("Scaling...");
    if (!mFilter->isAnySelected())
    {
        for (size_t i = 0; i < mPointCloud->size(); i++)
        {
            (*mPointCloud)[i].position(scale((*mPointCloud)[i].position()));
        }
        for (size_t i = 0; i < mPointCloud->geometry()->numPlanes(); i++)
        {
            Plane *plane = mPointCloud->geometry()->plane(i);
            plane->center(scale(plane->center()));
            plane->basisU(plane->basisU().normalized() * (plane->basisU().array() * mScale.array()).matrix().norm());
            plane->basisV(plane->basisV().normalized() * (plane->basisV().array() * mScale.array()).matrix().norm());
        }
        for (size_t i = 0; i < mPointCloud->geometry()->numCylinders(); i++)
        {
            Cylinder *cylinder = mPointCloud->geometry()->cylinder(i);
            cylinder->center(scale(cylinder->center()));
            cylinder->height((cylinder->axis().array() * mScale.array()).matrix().norm() * cylinder->height());
            Eigen::Vector3f basisU, basisV;
            GeometryUtils::orthogonalBasis(cylinder->axis(), basisU, basisV);
            float radius = std::max((basisU.array() * mScale.array()).matrix().norm(),
                                    (basisV.array() * mScale.array()).matrix().norm()) * cylinder->radius();
            cylinder->radius(radius);
        }
    }
    else if (mFilter->mode() == SelectFilter::SelectMode::POINT)
    {
        for (size_t i = 0; i < mPointCloud->size(); i++)
        {
            if (mFilter->isSelected(i))
            {
                (*mPointCloud)[i].position(scale((*mPointCloud)[i].position()));
            }
        }
    }
    else if (mFilter->mode() == SelectFilter::SelectMode::PLANE)
    {
        for (size_t i = 0; i < mPointCloud->geometry()->numPlanes(); i++)
        {
            if (mFilter->isSelected(i))
            {
                Plane *plane = mPointCloud->geometry()->plane(i);
                plane->center(scale(plane->center()));
                plane->basisU(plane->basisU().normalized() * (plane->basisU().array() * mScale.array()).matrix().norm());
                plane->basisV(plane->basisV().normalized() * (plane->basisV().array() * mScale.array()).matrix().norm());
            }
        }
    }
    else if (mFilter->mode() == SelectFilter::SelectMode::CYLINDER)
    {
        for (size_t i = 0; i < mPointCloud->geometry()->numCylinders(); i++)
        {
            if (mFilter->isSelected(i))
            {
                Cylinder *cylinder = mPointCloud->geometry()->cylinder(i);
                cylinder->center(scale(cylinder->center()));
                cylinder->height((cylinder->axis().array() * mScale.array()).matrix().norm() * cylinder->height());
                Eigen::Vector3f basisU, basisV;
                GeometryUtils::orthogonalBasis(cylinder->axis(), basisU, basisV);
                float radius = std::max((basisU.array() * mScale.array()).matrix().norm(),
                                        (basisV.array() * mScale.array()).matrix().norm()) * cylinder->radius();
                cylinder->radius(radius);
            }
        }
    }
    mPointCloud->update();
    emit workerStatus("Updating visualization...");
    mSimplifiedPointCloud->update();
}
