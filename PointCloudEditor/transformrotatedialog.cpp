#include "transformrotatedialog.h"

#include <QVBoxLayout>
#include <QHBoxLayout>

TransformRotateDialog::TransformRotateDialog()
{
    QVBoxLayout *layout = new QVBoxLayout;
    mAxisXButton = new QRadioButton("X axis");
    mAxisYButton = new QRadioButton("Y axis");
    mAxisZButton = new QRadioButton("Z axis");
    mAnotherAxisButton = new QRadioButton("Arbitrary axis");
    layout->addWidget(mAxisXButton);
    layout->addWidget(mAxisYButton);
    layout->addWidget(mAxisZButton);
    layout->addWidget(mAnotherAxisButton);
    QHBoxLayout *axisLayout = new QHBoxLayout;
    QWidget *axis = new QWidget;
    mAnotherAxisX = new QDoubleSpinBox(axis);
    mAnotherAxisX->setRange(0, 1);
    mAnotherAxisX->setSingleStep(0.01f);
    mAnotherAxisY = new QDoubleSpinBox(axis);
    mAnotherAxisY->setRange(0, 1);
    mAnotherAxisY->setSingleStep(0.01f);
    mAnotherAxisZ = new QDoubleSpinBox(axis);
    mAnotherAxisZ->setRange(0, 1);
    mAnotherAxisZ->setSingleStep(0.01f);
    axisLayout->addWidget(mAnotherAxisX);
    axisLayout->addWidget(mAnotherAxisY);
    axisLayout->addWidget(mAnotherAxisZ);
    axis->setLayout(axisLayout);
    layout->addWidget(axis);

    layout->addWidget(new QLabel("Degrees:"));
    mDegrees = new QDoubleSpinBox(this);
    mDegrees->setRange(-360, 360);
    mDegrees->setSingleStep(1.0f);
    mDegrees->setSuffix("°");
    layout->addWidget(mDegrees);

    mDoneButton = new QPushButton("Ok", this);
    connect(mDoneButton, SIGNAL(clicked()), this, SLOT(done()));
    layout->addWidget(mDoneButton);

    setLayout(layout);
    setWindowTitle("Rotate");
}

void TransformRotateDialog::showEvent(QShowEvent *e)
{
    Q_UNUSED(e);
    mAnotherAxisX->setValue(0.0f);
    mAnotherAxisY->setValue(0.0f);
    mAnotherAxisZ->setValue(0.0f);
    mDegrees->setValue(0.0f);
    mAxisXButton->setChecked(true);
}

void TransformRotateDialog::done()
{
    float x, y, z;
    if (mAxisXButton->isChecked())
    {
        x = 1.0f;
        y = 0.0f;
        z = 0.0f;
    }
    else if (mAxisYButton->isChecked())
    {
        x = 0.0f;
        y = 1.0f;
        z = 0.0f;
    }
    else if (mAxisZButton->isChecked())
    {
        x = 0.0f;
        y = 0.0f;
        z = 1.0f;
    }
    else
    {
        x = (float)mAnotherAxisX->value();
        y = (float)mAnotherAxisY->value();
        z = (float)mAnotherAxisZ->value();
    }
    float degrees = (float)mDegrees->value();
    emit rotate(x, y, z, degrees);
    close();
}
