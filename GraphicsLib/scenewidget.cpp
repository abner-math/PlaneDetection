#include "scenewidget.h"

#include <algorithm>
#include <iostream>

#include <QMouseEvent>
#include <QWheelEvent>
#include <QResizeEvent>
#include <QKeyEvent>
#include <QPaintEvent>
#include <QPainter>
#include <QMessageBox>

SceneWidget::SceneWidget(QWidget *parent)
    : QLabel(parent)
    , mAnimate(false)
    , mMouseDown(false)
    , mScene(new Scene)
{
    setMouseTracking(true);
    mTimer.start(12, this);
    connect(mScene, SIGNAL(cameraUpdated()), this, SIGNAL(cameraUpdated()));
}

SceneWidget::~SceneWidget()
{
    delete mScene;
}

void SceneWidget::mousePressEvent(QMouseEvent *e)
{
    mMousePosition = QVector2D(e->localPos());
    mMouseDown = true;
    mMouseLeftButton = e->button() == Qt::LeftButton;
    if (mMouseLeftButton)
    {
        emit mouseLeftClick();
    }
    else
    {
        emit mouseRightClick();
    }
}

void SceneWidget::mouseMoveEvent(QMouseEvent *e)
{
    int x = static_cast<int>(std::round(e->localPos().x()));
    int y = static_cast<int>(std::round(e->localPos().y()));
    if (rect().contains(x, y))
    {
        emit mouseMove(x, y);
        if (mMouseDown)
        {
            QVector2D newMousePosition = QVector2D(e->localPos());
            QVector2D diff = newMousePosition - mMousePosition;
            mMousePosition = newMousePosition;
            if (mMouseLeftButton)
            {
                mScene->camera().rotate(-diff.x(), Camera::Axis::Y);
                mScene->camera().rotate(-diff.y(), Camera::Axis::X);
            }
            else
            {
                mScene->camera().translate(-diff.x(), -diff.y());
            }
            update();
        }
    }
}

void SceneWidget::mouseReleaseEvent(QMouseEvent *e)
{
    Q_UNUSED(e);
    mMouseDown = false;
}

void SceneWidget::wheelEvent(QWheelEvent *e)
{
    mScene->camera().zoom(e->delta() / std::abs(e->delta()));
    update();
}

void SceneWidget::timerEvent(QTimerEvent *e)
{
    Q_UNUSED(e);
    if (mAnimate)
        update();
}

void SceneWidget::resizeEvent(QResizeEvent *e)
{
    int width = e->size().width();
    int height = e->size().height();
    mScene->camera().perspectiveRatio(width / (float)height);
    mScene->size(width, height);
}

void SceneWidget::paintEvent(QPaintEvent *e)
{
    Q_UNUSED(e);
    mScene->render();
    QPixmap img = QPixmap::fromImage(mScene->frameBuffer()->image());
    QPainter painter;
    painter.begin(this);
    painter.drawPixmap(0, 0, width(), height(), img);
    painter.end();
}
