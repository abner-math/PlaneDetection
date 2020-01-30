#include "viewoptionsdialog.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QStyle>
#include <QColorDialog>

ViewOptionsDialog::ViewOptionsDialog()
{
    QVBoxLayout *layout = new QVBoxLayout(this);
    layout->addWidget(new QLabel("Detail level:", this));

    mLevelBox = new QSpinBox;
    mLevelBox->setRange(1, 25);
    mLevelBox->setSingleStep(1);
    mLevelBox->setValue(10);
    layout->addWidget(mLevelBox);

    QWidget *backgroundColor = new QWidget(this);
    mBackgroundColorLabel = new QWidget(backgroundColor);
    mBackgroundColorLabel->setFixedSize(QSize(35, 35));
    mBackgroundColorButton = new QPushButton("Background color");
    connect(mBackgroundColorButton, SIGNAL(clicked()), this, SLOT(setBackgroundColor()));
    QHBoxLayout *backgroundLayout = new QHBoxLayout;
    backgroundLayout->addWidget(mBackgroundColorLabel);
    backgroundLayout->addWidget(mBackgroundColorButton);
    backgroundColor->setLayout(backgroundLayout);
    layout->addWidget(backgroundColor);

    QWidget *pointColor = new QWidget(this);
    mPointColorLabel = new QWidget(pointColor);
    mPointColorLabel->setFixedSize(QSize(35, 35));
    mPointColorButton = new QPushButton("Default point color");
    connect(mPointColorButton, SIGNAL(clicked()), this, SLOT(setPointColor()));
    QHBoxLayout *pointLayout = new QHBoxLayout;
    pointLayout->addWidget(mPointColorLabel);
    pointLayout->addWidget(mPointColorButton);
    pointColor->setLayout(pointLayout);
    layout->addWidget(pointColor);

    layout->addWidget(new QLabel("Point size:", this));
    mPointSizeBox = new QSpinBox;
    mPointSizeBox->setRange(1, 100);
    mPointSizeBox->setSingleStep(1);
    layout->addWidget(mPointSizeBox);

    mDoneButton = new QPushButton("Ok", this);
    connect(mDoneButton, SIGNAL(clicked()), this, SLOT(done()));
    layout->addWidget(mDoneButton);

    setLayout(layout);
    setWindowTitle("View options");
}

void ViewOptionsDialog::backgroundColor(const Eigen::Vector3f &color)
{
    mBackgroundColor = color;
}

void ViewOptionsDialog::defaultPointColor(const Eigen::Vector3f &color)
{
    mDefaultPointColor = color;
}

void ViewOptionsDialog::levels(size_t levels)
{
    mLevels = levels;
}

void ViewOptionsDialog::pointSize(float pointSize)
{
    mPointSize = pointSize;
}

void ViewOptionsDialog::update()
{
    mLevelBox->setValue(mLevels);
    mBackgroundColorLabel->setStyleSheet(("* { border: 1px solid black; margin: 5px; border-radius: 2; background-color: rgb(" +
                                 std::to_string(int(mBackgroundColor.x() * 255)) + "," + std::to_string(int(mBackgroundColor.y() * 255)) + "," + std::to_string(int(mBackgroundColor.z() * 255)) + "); }").c_str());
    mPointColorLabel->setStyleSheet(("* { border: 1px solid black; margin: 5px; border-radius: 2; background-color: rgb(" +
                                 std::to_string(int(mDefaultPointColor.x() * 255)) + "," + std::to_string(int(mDefaultPointColor.y() * 255)) + "," + std::to_string(int(mDefaultPointColor.z() * 255)) + "); }").c_str());
    mPointSizeBox->setValue((int)mPointSize);
}

void ViewOptionsDialog::showEvent(QShowEvent *e)
{
    Q_UNUSED(e);
    update();
}

void ViewOptionsDialog::setBackgroundColor()
{
    QColor color = QColorDialog::getColor(QColor(int(mBackgroundColor.x() * 255),
        int(mBackgroundColor.y() * 255), int(mBackgroundColor.z() * 255)), this);
    if (color.isValid())
    {
        mBackgroundColor.x() = color.red() / 255.0f;
        mBackgroundColor.y() = color.green() / 255.0f;
        mBackgroundColor.z() = color.blue() / 255.0f;
        update();
    }
}

void ViewOptionsDialog::setPointColor()
{
    QColor color = QColorDialog::getColor(QColor(int(mDefaultPointColor.x() * 255),
        int(mDefaultPointColor.y() * 255), int(mDefaultPointColor.z() * 255)), this);
    if (color.isValid())
    {
        mDefaultPointColor.x() = color.red() / 255.0f;
        mDefaultPointColor.y() = color.green() / 255.0f;
        mDefaultPointColor.z() = color.blue() / 255.0f;
        update();
    }
}

void ViewOptionsDialog::done()
{
    emit setViewOptions(mBackgroundColor.x(), mBackgroundColor.y(), mBackgroundColor.z(),
                        mDefaultPointColor.x(), mDefaultPointColor.y(), mDefaultPointColor.z(),
                        mLevelBox->value(), mPointSizeBox->value());
    close();
}
