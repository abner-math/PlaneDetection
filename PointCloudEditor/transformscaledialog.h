#ifndef TRANSFORMSCALEDIALOG_H
#define TRANSFORMSCALEDIALOG_H

#include <QDialog>
#include <QLabel>
#include <QPushButton>
#include <QDoubleSpinBox>

class TransformScaleDialog : public QDialog
{
    Q_OBJECT
public:
    TransformScaleDialog();

signals:
    void scale(float dx, float dy, float dz);

private:
    QPushButton *mDoneButton;
    QDoubleSpinBox *mScaleX;
    QDoubleSpinBox *mScaleY;
    QDoubleSpinBox *mScaleZ;

    void showEvent(QShowEvent *e) override;

private slots:
    void done();

};

#endif // TRANSFORMSCALEDIALOG_H
