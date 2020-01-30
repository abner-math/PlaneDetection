#include "transformtranslateworker.h"

TransformTranslateWorker::TransformTranslateWorker(const SelectFilter *filter)
    : mFilter(filter)
    , mPointCloud(NULL)
    , mSimplifiedPointCloud(NULL)
{

}

void TransformTranslateWorker::actions()
{
    if (mPointCloud == NULL) return;
    emit workerStatus("Translating...");
    if (!mFilter->isAnySelected())
    {
        for (size_t i = 0; i < mPointCloud->size(); i++)
        {
            (*mPointCloud)[i].position((*mPointCloud)[i].position() + mTranslation);
        }
        for (size_t i = 0; i < mPointCloud->geometry()->numPlanes(); i++)
        {
            Plane *plane = mPointCloud->geometry()->plane(i);
            plane->center(plane->center() + mTranslation);
        }
        for (size_t i = 0; i < mPointCloud->geometry()->numCylinders(); i++)
        {
            Cylinder *cylinder = mPointCloud->geometry()->cylinder(i);
            cylinder->center(cylinder->center() + mTranslation);
        }
    }
    else if (mFilter->mode() == SelectFilter::SelectMode::POINT)
    {
        for (size_t i = 0; i < mPointCloud->size(); i++)
        {
            if (mFilter->isSelected(i))
            {
                (*mPointCloud)[i].position((*mPointCloud)[i].position() + mTranslation);
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
                plane->center(plane->center() + mTranslation);
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
                cylinder->center(cylinder->center() + mTranslation);
            }
        }
    }
    mPointCloud->update();
    emit workerStatus("Updating visualization...");
    mSimplifiedPointCloud->update();
}
