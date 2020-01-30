#ifndef TRANSFORMTRANSLATEDIALOG_H
#define TRANSFORMTRANSLATEDIALOG_H

#include <QDialog>
#include <QLabel>
#include <QPushButton>
#include <QDoubleSpinBox>

#include "pointcloud.h"

class TransformTranslateDialog : public QDialog
{
    Q_OBJECT
public:
    TransformTranslateDialog();

    void setPointCloud(const PointCloud3d *pointCloud);

signals:
    void translate(float dx, float dy, float dz);

private:
    QPushButton *mDoneButton;
    QDoubleSpinBox *mTranslateX;
    QDoubleSpinBox *mTranslateY;
    QDoubleSpinBox *mTranslateZ;

    void showEvent(QShowEvent *e) override;

private slots:
    void done();

};

#endif // TRANSFORMTRANSLATEDIALOG_H
