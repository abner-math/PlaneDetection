#ifndef VIEWOPTIONSDIALOG_H
#define VIEWOPTIONSDIALOG_H

#include <QDialog>
#include <QLabel>
#include <QPushButton>
#include <QSpinBox>

#include "simplifiedpointcloud.h"

class ViewOptionsDialog : public QDialog
{
    Q_OBJECT
public:
    ViewOptionsDialog();

    void backgroundColor(const Eigen::Vector3f &color);

    void defaultPointColor(const Eigen::Vector3f &color);

    void levels(size_t levels);

    void pointSize(float pointSize);

signals:
    void setViewOptions(float bx, float by, float bz, float dx, float dy, float dz, size_t levels, float pointSize);

private:
    QPushButton *mDoneButton;
    QSpinBox *mLevelBox;
    QWidget *mBackgroundColorLabel;
    QPushButton *mBackgroundColorButton;
    QWidget *mPointColorLabel;
    QPushButton *mPointColorButton;
    QSpinBox *mPointSizeBox;
    Eigen::Vector3f mBackgroundColor;
    Eigen::Vector3f mDefaultPointColor;
    size_t mLevels;
    float mPointSize;

    void update();

    void showEvent(QShowEvent *e) override;

private slots:
    void setBackgroundColor();
    void setPointColor();
    void done();

};

#endif // VIEWOPTIONSDIALOG_H
