#include "pointcloudioworker.h"

PointCloudIOWorker::PointCloudIOWorker()
    : mMode(LOAD_POINT_CLOUD)
{
    connect(&mLoader, SIGNAL(loadProgress(float)), this, SLOT(onLoadProgress(float)));
    connect(&mLoader, SIGNAL(load(const QString&)), this, SLOT(onLoad(const QString&)));
    connect(&mLoader, SIGNAL(saveProgress(float)), this, SLOT(onSaveProgress(float)));
    connect(&mLoader, SIGNAL(save(const QString&)), this, SLOT(onSave(const QString&)));
}

void PointCloudIOWorker::actions()
{
    switch (mMode)
    {
    case LOAD_POINT_CLOUD:
        emit workerStatus("Loading point cloud...");
        try
        {
            mPointCloud = mLoader.load(mFilename);
            if (mPointCloud != NULL)
            {
                mPointCloud->update();
                emit workerStatus("Creating point cloud visualization...");
                mSimplified = new SimplifiedPointCloud(mPointCloud);
            }
        }
        catch (const std::string &error)
        {
            emit loadError(QString(error.c_str()));
        }
        break;
    case LOAD_CONNECTIVITY:
        emit workerStatus("Loading connectivity...");
        mConnectivity = mLoader.loadConnectivity(mPointCloud->size(), mFilename);
        break;
    case LOAD_GEOMETRY:
        emit workerStatus("Loading geometry...");
        mGeometry = mLoader.loadGeometry(mFilename);
        break;
    case SAVE_POINT_CLOUD:
        emit workerStatus("Saving point cloud...");
        mLoader.save(mPointCloud, mFilename);
        break;
    case SAVE_CONNECTIVITY:
        emit workerStatus("Saving connectivity...");
        mLoader.saveConnectivity(mConnectivity, mFilename);
        break;
    case SAVE_GEOMETRY:
        mLoader.saveGeometry(mGeometry, mFilename);
        break;
    }

}

void PointCloudIOWorker::onLoadProgress(float progress)
{
    emit workerProgress(progress);
}

void PointCloudIOWorker::onLoad(const QString &message)
{
    emit workerStatus(("Loading " + message.toStdString() + "...").c_str());
}

void PointCloudIOWorker::onSaveProgress(float progress)
{
    emit workerProgress(progress);
}

void PointCloudIOWorker::onSave(const QString &message)
{
    emit workerStatus(("Saving " + message.toStdString() + "...").c_str());
}

