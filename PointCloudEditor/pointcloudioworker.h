#ifndef POINTCLOUDLOADERWORKER_H
#define POINTCLOUDLOADERWORKER_H

#include "worker.h"
#include "pointcloudio.h"
#include "simplifiedpointcloud.h"

class PointCloudIOWorker : public Worker
{
    Q_OBJECT
public:
    enum Mode
    {
        LOAD_POINT_CLOUD = 0,
        LOAD_CONNECTIVITY = 1,
        LOAD_GEOMETRY = 2,
        SAVE_POINT_CLOUD = 3,
        SAVE_CONNECTIVITY = 4,
        SAVE_GEOMETRY = 5
    };

    PointCloudIOWorker();

    void filename(const std::string &filename)
    {
        mFilename = filename;
    }

    Mode mode() const
    {
        return mMode;
    }

    void mode(Mode mode)
    {
        mMode = mode;
    }

    PointCloud3d* pointCloud()
    {
        return mPointCloud;
    }

    void pointCloud(PointCloud3d *pointCloud)
    {
        mPointCloud = pointCloud;
    }

    SimplifiedPointCloud* simplifiedPointCloud()
    {
        return mSimplified;
    }

    ConnectivityGraph* connectivity()
    {
        return mConnectivity;
    }

    void connectivity(ConnectivityGraph *connectivity)
    {
        mConnectivity = connectivity;
    }

    Geometry* geometry()
    {
        return mGeometry;
    }

    void geometry(Geometry *geometry)
    {
        mGeometry = geometry;
    }

signals:
    void loadError(const QString &error);

private slots:
    void onLoadProgress(float progress);
    void onLoad(const QString &message);
    void onSaveProgress(float progress);
    void onSave(const QString &message);

private:
    std::string mFilename;
    Mode mMode;
    PointCloudIO mLoader;
    PointCloud3d *mPointCloud;
    SimplifiedPointCloud *mSimplified;
    ConnectivityGraph *mConnectivity;
    Geometry *mGeometry;

    void actions() override;

};

#endif // POINTCLOUDLOADERWORKER_H
