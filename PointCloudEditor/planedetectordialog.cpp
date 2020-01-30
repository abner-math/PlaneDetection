#include "planedetectordialog.h"

#include <QVBoxLayout>

#include "angleutils.h"

PlaneDetectorDialog::PlaneDetectorDialog()
{
    QVBoxLayout *layout = new QVBoxLayout(this);
    layout->addWidget(new QLabel("Max normal diff:", this));

    mMinNormalBox = new QSpinBox;
    mMinNormalBox->setRange(0, 180);
    mMinNormalBox->setSingleStep(1);
    mMinNormalBox->setValue(60);
    mMinNormalBox->setSuffix("°");
    layout->addWidget(mMinNormalBox);

    layout->addWidget(new QLabel("Max dist:", this));

    mMaxDistBox = new QSpinBox;
    mMaxDistBox->setRange(0, 180);
    mMaxDistBox->setSingleStep(1);
    mMaxDistBox->setValue(75);
    mMaxDistBox->setSuffix("°");
    layout->addWidget(mMaxDistBox);

    layout->addWidget(new QLabel("Outlier ratio:", this));

    mOutlierRatioBox = new QSpinBox;
    mOutlierRatioBox->setRange(50, 100);
    mOutlierRatioBox->setValue(75);
    mOutlierRatioBox->setSuffix("%");
    layout->addWidget(mOutlierRatioBox);

    mDoneButton = new QPushButton("Ok", this);
    connect(mDoneButton, SIGNAL(clicked()), this, SLOT(done()));
    layout->addWidget(mDoneButton);

    setLayout(layout);
    setWindowTitle("Plane detector options");
}

void PlaneDetectorDialog::done()
{
    emit detectPlaneOptions(std::cos(AngleUtils::deg2rad(mMinNormalBox->value())),
                            std::cos(AngleUtils::deg2rad(mMaxDistBox->value())),
                            mOutlierRatioBox->value() / 100.0f);
    close();
}
