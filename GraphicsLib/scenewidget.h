#ifndef SCENEWIDGET_H
#define SCENEWIDGET_H

#include <QLabel>
#include <QBasicTimer>

#include "scene.h"

class SceneWidget : public QLabel
{
    Q_OBJECT
public:
    explicit SceneWidget(QWidget *parent = NULL);
    ~SceneWidget();

    void setAnimating(bool animate)
    {
        mAnimate = animate;
    }

    Scene* scene()
    {
        return mScene;
    }

signals:
    void mouseMove(int x, int y);
    void mouseLeftClick();
    void mouseRightClick();
    void cameraUpdated();

protected:
    void mousePressEvent(QMouseEvent *e) override;
    void mouseMoveEvent(QMouseEvent *e) override;
    void mouseReleaseEvent(QMouseEvent *e) override;
    void wheelEvent(QWheelEvent *e) override;
    void timerEvent(QTimerEvent *e) override;
    void resizeEvent(QResizeEvent *e) override;
    void paintEvent(QPaintEvent *e) override;

private:
    QBasicTimer mTimer;

    bool mAnimate;
    QVector2D mMousePosition;
    bool mMouseDown;
    bool mMouseLeftButton;

    Camera mCamera;
    Scene *mScene;

};

#endif // SCENEWIDGET_H
