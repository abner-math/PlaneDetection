#include "pointcloudinfodialog.h"

#include <sstream>

#include <QVBoxLayout>

PointCloudInfoDialog::PointCloudInfoDialog()
{
    QVBoxLayout *layout = new QVBoxLayout(this);

    layout->addWidget(new QLabel("<b>Number of points:</b>", this));
    mNumPointsLabel = new QLabel(this);
    layout->addWidget(mNumPointsLabel);

    layout->addWidget(new QLabel("<b>Extension:</b>", this));
    mExtensionLabel = new QLabel(this);
    layout->addWidget(mExtensionLabel);

    layout->addWidget(new QLabel("<b>Center:</b>", this));
    mCenterLabel = new QLabel(this);
    layout->addWidget(mCenterLabel);

    mDoneButton = new QPushButton("Ok", this);
    connect(mDoneButton, SIGNAL(clicked()), this, SLOT(close()));
    layout->addWidget(mDoneButton);

    setLayout(layout);
    setWindowTitle("Point Cloud Info");
}

void PointCloudInfoDialog::setPointCloud(PointCloud3d *pointCloud)
{
    mNumPointsLabel->setText(std::to_string(pointCloud->size()).c_str());
    Rect3d extension = pointCloud->extension();
    std::stringstream ssExtension;
    ssExtension << "[" << extension.bottomLeft().x() << ", " << extension.bottomLeft().y() << ", " << extension.bottomLeft().z() << "]; " << std::endl
                << "[" << extension.topRight().x() << ", " << extension.topRight().y() << ", " << extension.topRight().z() << "]";
    mExtensionLabel->setText(ssExtension.str().c_str());
    Eigen::Vector3f center = pointCloud->center();
    std::stringstream ssCenter;
    ssCenter << "[" << center.x() << ", " << center.y() << ", " << center.z() << "]";
    mCenterLabel->setText(ssCenter.str().c_str());
}
