#include "normalestimatorworker.h"

//#include <tbb/parallel_for.h>
//#include <tbb/blocked_range.h>
//#include <tbb/mutex.h>

#include "boundaryvolumehierarchy.h"

NormalEstimatorWorker::NormalEstimatorWorker(PointCloud3d *pointCloud)
    : mPointCloud(pointCloud)
    , mNumNeighbors(21)
    , mSpeed(NormalEstimator3d::QUICK)
{

}

void NormalEstimatorWorker::actions()
{
    if (mPointCloud == NULL) return;

    emit workerStatus("Pre-processing...");

    Octree octree(mPointCloud);
    octree.partition(10, 30);

    emit workerStatus("Estimating normals...");

    ConnectivityGraph *connectivity = new ConnectivityGraph(mPointCloud->size());
    mPointCloud->connectivity(connectivity);

    NormalEstimator3d estimator(&octree, mNumNeighbors, mSpeed);

    //tbb::mutex mutex;
    size_t count = 0;
    //tbb::parallel_for(tbb::blocked_range<size_t>(0, mPointCloud->size()), [&](const tbb::blocked_range<size_t> &x) {
    //    for (size_t i = x.begin(); i != x.end(); ++i)
        for (size_t i = 0; i < mPointCloud->size(); i++)
        {
            if (isRunning())
            {
                NormalEstimator3d::Normal normal = estimator.estimate(i);
                connectivity->addNode(i, normal.neighbors);
                (*mPointCloud)[i].normal(normal.normal);
                (*mPointCloud)[i].normalConfidence(normal.confidence);
                (*mPointCloud)[i].curvature(normal.curvature);
                //mutex.lock();
                ++count;
                if (count % 1000 == 0)
                {
                    emit workerProgress(static_cast<float>(count) / mPointCloud->size());
                }
                //mutex.unlock();
            }
        }
    //});
}
