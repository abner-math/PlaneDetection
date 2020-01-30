#ifndef TRANSFORMROTATEDIALOG_H
#define TRANSFORMROTATEDIALOG_H

#include <QDialog>
#include <QLabel>
#include <QPushButton>
#include <QDoubleSpinBox>
#include <QRadioButton>
#include <QButtonGroup>

class TransformRotateDialog : public QDialog
{
    Q_OBJECT
public:
    TransformRotateDialog();

signals:
    void rotate(float x, float y, float z, float degrees);

private:
    QPushButton *mDoneButton;
    QRadioButton *mAxisXButton;
    QRadioButton *mAxisYButton;
    QRadioButton *mAxisZButton;
    QRadioButton *mAnotherAxisButton;
    QDoubleSpinBox *mAnotherAxisX;
    QDoubleSpinBox *mAnotherAxisY;
    QDoubleSpinBox *mAnotherAxisZ;
    QDoubleSpinBox *mDegrees;

    void showEvent(QShowEvent *e) override;

private slots:
    void done();

};

#endif // TRANSFORMROTATEDIALOG_H
