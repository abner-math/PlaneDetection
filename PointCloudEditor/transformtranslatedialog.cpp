#include "transformtranslatedialog.h"

#include <QHBoxLayout>

TransformTranslateDialog::TransformTranslateDialog()
{
    QHBoxLayout *layout = new QHBoxLayout(this);
    layout->addWidget(new QLabel("X:", this));
    mTranslateX = new QDoubleSpinBox;
    layout->addWidget(mTranslateX);
    layout->addWidget(new QLabel("Y:", this));
    mTranslateY = new QDoubleSpinBox;
    layout->addWidget(mTranslateY);
    layout->addWidget(new QLabel("Z:", this));
    mTranslateZ = new QDoubleSpinBox;
    layout->addWidget(mTranslateZ);

    mDoneButton = new QPushButton("Ok", this);
    connect(mDoneButton, SIGNAL(clicked()), this, SLOT(done()));
    layout->addWidget(mDoneButton);

    setLayout(layout);
    setWindowTitle("Translate");
}

void TransformTranslateDialog::setPointCloud(const PointCloud3d *pointCloud)
{
    float size = pointCloud->extension().maxSize();
    mTranslateX->setRange(-size * 10, size * 10);
    mTranslateX->setSingleStep(size / 100);
    mTranslateY->setRange(-size * 10, size * 10);
    mTranslateY->setSingleStep(size / 100);
    mTranslateZ->setRange(-size * 10, size * 10);
    mTranslateZ->setSingleStep(size / 100);
}

void TransformTranslateDialog::showEvent(QShowEvent *e)
{
    Q_UNUSED(e);
    mTranslateX->setValue(0);
    mTranslateY->setValue(0);
    mTranslateZ->setValue(0);
}

void TransformTranslateDialog::done()
{
    emit translate(mTranslateX->value(), mTranslateY->value(), mTranslateZ->value());
    close();
}
