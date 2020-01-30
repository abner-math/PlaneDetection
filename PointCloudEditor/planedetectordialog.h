#ifndef PLANEDETECTORDIALOG_H
#define PLANEDETECTORDIALOG_H

#include <QDialog>
#include <QLabel>
#include <QPushButton>
#include <QSpinBox>

class PlaneDetectorDialog : public QDialog
{
    Q_OBJECT
public:
    PlaneDetectorDialog();

signals:
    void detectPlaneOptions(float minNormalDiff, float maxDist, float outlierRatio);

private:
    QPushButton *mDoneButton;
    QSpinBox *mMinNormalBox;
    QSpinBox *mMaxDistBox;
    QSpinBox *mOutlierRatioBox;

private slots:
    void done();

};

#endif // PLANEDETECTORDIALOG_H
