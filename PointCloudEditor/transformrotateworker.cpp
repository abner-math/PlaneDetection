#include "transformrotateworker.h"

#include "angleutils.h"

TransformRotateWorker::TransformRotateWorker(const SelectFilter *filter)
    : mFilter(filter)
    , mPointCloud(NULL)
    , mSimplifiedPointCloud(NULL)
{

}

Eigen::Vector3f TransformRotateWorker::rotate(const Eigen::Vector3f &v)
{
    return mPointCloud->center() + AngleUtils::rotate(v - mPointCloud->center(), mDegrees, mAxis);
}

void TransformRotateWorker::actions()
{
    if (mPointCloud == NULL) return;
    emit workerStatus("Rotating...");
    if (!mFilter->isAnySelected())
    {
        for (size_t i = 0; i < mPointCloud->size(); i++)
        {
            (*mPointCloud)[i].position(rotate((*mPointCloud)[i].position()));
            (*mPointCloud)[i].normal(AngleUtils::rotate((*mPointCloud)[i].normal(), mDegrees, mAxis).normalized());
        }
        for (size_t i = 0; i < mPointCloud->geometry()->numPlanes(); i++)
        {
            Plane *plane = mPointCloud->geometry()->plane(i);
            plane->center(rotate(plane->center()));
            plane->normal(AngleUtils::rotate(plane->normal(), mDegrees, mAxis));
            plane->basisU(AngleUtils::rotate(plane->basisU(), mDegrees, mAxis));
            plane->basisV(AngleUtils::rotate(plane->basisV(), mDegrees, mAxis));
        }
        for (size_t i = 0; i < mPointCloud->geometry()->numCylinders(); i++)
        {
            Cylinder *cylinder = mPointCloud->geometry()->cylinder(i);
            cylinder->center(rotate(cylinder->center()));
            cylinder->axis(AngleUtils::rotate(cylinder->axis(), mDegrees, mAxis));
        }
    }
    else if (mFilter->mode() == SelectFilter::SelectMode::POINT)
    {
        for (size_t i = 0; i < mPointCloud->size(); i++)
        {
            if (mFilter->isSelected(i))
            {
                (*mPointCloud)[i].position(rotate((*mPointCloud)[i].position()));
                (*mPointCloud)[i].normal(AngleUtils::rotate((*mPointCloud)[i].normal(), mDegrees, mAxis).normalized());
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
                plane->center(rotate(plane->center()));
                plane->normal(AngleUtils::rotate(plane->normal(), mDegrees, mAxis));
                plane->basisU(AngleUtils::rotate(plane->basisU(), mDegrees, mAxis));
                plane->basisV(AngleUtils::rotate(plane->basisV(), mDegrees, mAxis));
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
                cylinder->center(rotate(cylinder->center()));
                cylinder->axis(AngleUtils::rotate(cylinder->axis(), mDegrees, mAxis));
            }
        }
    }
    mPointCloud->update();
    emit workerStatus("Updating visualization...");
    mSimplifiedPointCloud->update();
}
