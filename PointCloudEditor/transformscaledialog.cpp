#include "transformscaledialog.h"

#include <QHBoxLayout>

TransformScaleDialog::TransformScaleDialog()
{
    QHBoxLayout *layout = new QHBoxLayout(this);
    layout->addWidget(new QLabel("X:", this));
    mScaleX = new QDoubleSpinBox;
    mScaleX->setRange(0, 10.0f);
    mScaleX->setSingleStep(0.01f);
    layout->addWidget(mScaleX);
    layout->addWidget(new QLabel("Y:", this));
    mScaleY = new QDoubleSpinBox;
    mScaleY->setRange(0, 10.0f);
    mScaleY->setSingleStep(0.01f);
    layout->addWidget(mScaleY);
    layout->addWidget(new QLabel("Z:", this));
    mScaleZ = new QDoubleSpinBox;
    mScaleZ->setRange(0, 10.0f);
    mScaleZ->setSingleStep(0.01f);
    layout->addWidget(mScaleZ);

    mDoneButton = new QPushButton("Ok", this);
    connect(mDoneButton, SIGNAL(clicked()), this, SLOT(done()));
    layout->addWidget(mDoneButton);

    setLayout(layout);
    setWindowTitle("Scale");
}

void TransformScaleDialog::showEvent(QShowEvent *e)
{
    Q_UNUSED(e);
    mScaleX->setValue(1.0f);
    mScaleY->setValue(1.0f);
    mScaleZ->setValue(1.0f);
}

void TransformScaleDialog::done()
{
    emit scale(mScaleX->value(), mScaleY->value(), mScaleZ->value());
    close();
}
