#ifndef POINTCLOUDINFODIALOG_H
#define POINTCLOUDINFODIALOG_H

#include <QDialog>
#include <QLabel>
#include <QPushButton>

#include "pointcloud.h"

class PointCloudInfoDialog : public QDialog
{
    Q_OBJECT
public:
    PointCloudInfoDialog();

    void setPointCloud(PointCloud3d *pointCloud);

private:
    QLabel *mNumPointsLabel;
    QLabel *mExtensionLabel;
    QLabel *mCenterLabel;
    QPushButton *mDoneButton;

};

#endif // POINTCLOUDINFODIALOG_H
