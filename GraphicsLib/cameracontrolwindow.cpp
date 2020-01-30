#include "cameracontrolwindow.h"

#include <sstream>

#include <QMessageBox>
#include <QInputDialog>

std::vector<std::string> split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    split(s, delim, std::back_inserter(elems));
    return elems;
}

CameraControlWindow::CameraControlWindow(QWidget *parent)
    : QMainWindow(parent)
{
    QWidget *centralWidget = new QWidget(this);
    setCentralWidget(centralWidget);

    mViewMenu = menuBar()->addMenu("View");

    QMenu *cameraTranslateMenu = mViewMenu->addMenu("Translate");
    cameraTranslateMenu->addAction("Up", this, &CameraControlWindow::cameraTranslateUp, QKeySequence(Qt::Key_Up));
    cameraTranslateMenu->addAction("Down", this, &CameraControlWindow::cameraTranslateDown, QKeySequence(Qt::Key_Down));
    cameraTranslateMenu->addAction("Left", this, &CameraControlWindow::cameraTranslateLeft, QKeySequence(Qt::Key_Left));
    cameraTranslateMenu->addAction("Right", this, &CameraControlWindow::cameraTranslateRight, QKeySequence(Qt::Key_Right));

    QMenu *cameraRotateMenu = mViewMenu->addMenu("Rotate");
    cameraRotateMenu->addAction("Upward", this, &CameraControlWindow::cameraRotateUp, QKeySequence(Qt::Key_W));
    cameraRotateMenu->addAction("Downward", this, &CameraControlWindow::cameraRotateDown, QKeySequence(Qt::Key_S));
    cameraRotateMenu->addAction("Leftward", this, &CameraControlWindow::cameraRotateLeft, QKeySequence(Qt::Key_D));
    cameraRotateMenu->addAction("Rightward", this, &CameraControlWindow::cameraRotateRight, QKeySequence(Qt::Key_A));

    mViewMenu->addAction("Zoom in", this, &CameraControlWindow::cameraZoomIn, QKeySequence(Qt::Key_Plus));
    mViewMenu->addAction("Zoom out", this, &CameraControlWindow::cameraZoomOut, QKeySequence(Qt::Key_Minus));

    mViewMenu->addAction("Camera bounds", this, &CameraControlWindow::cameraBounds);

    mViewMenu->addSeparator();

    QMenu *lightMenu = mViewMenu->addMenu("Light position");
    lightMenu->addAction("Up", this, &CameraControlWindow::lightRotateUp, QKeySequence(Qt::SHIFT + Qt::Key_W));
    lightMenu->addAction("Down", this, &CameraControlWindow::lightRotateDown, QKeySequence(Qt::SHIFT + Qt::Key_S));
    lightMenu->addAction("Left", this, &CameraControlWindow::lightRotateLeft, QKeySequence(Qt::SHIFT + Qt::Key_D));
    lightMenu->addAction("Right", this, &CameraControlWindow::lightRotateRight, QKeySequence(Qt::SHIFT + Qt::Key_A));

    mViewMenu->addSeparator();

    mViewMenu->addAction("Reset", this, &CameraControlWindow::cameraReset, QKeySequence(Qt::Key_Escape));

    mLayout = new QHBoxLayout;
    centralWidget->setLayout(mLayout);

    mSceneWidget = new SceneWidget(this);
    mSceneWidget->setFocusPolicy(Qt::FocusPolicy::WheelFocus);
    mSceneWidget->setSizePolicy(QSizePolicy::Policy::Expanding, QSizePolicy::Policy::Preferred);
    mLayout->addWidget(mSceneWidget, 1);

    mScene = mSceneWidget->scene();
}

void CameraControlWindow::cameraTranslateUp()
{
    mScene->camera().translate(0, 1);
    mSceneWidget->update();
}

void CameraControlWindow::cameraTranslateDown()
{
    mScene->camera().translate(0, -1);
    mSceneWidget->update();
}

void CameraControlWindow::cameraTranslateLeft()
{
    mScene->camera().translate(1, 0);
    mSceneWidget->update();
}

void CameraControlWindow::cameraTranslateRight()
{
    mScene->camera().translate(-1, 0);
    mSceneWidget->update();
}

void CameraControlWindow::cameraRotateUp()
{
    mScene->camera().rotate(-1, Camera::Axis::X);
    mSceneWidget->update();
}

void CameraControlWindow::cameraRotateDown()
{
    mScene->camera().rotate(1, Camera::Axis::X);
    mSceneWidget->update();
}

void CameraControlWindow::cameraRotateLeft()
{
    mScene->camera().rotate(-1, Camera::Axis::Y);
    mSceneWidget->update();
}

void CameraControlWindow::cameraRotateRight()
{
    mScene->camera().rotate(1, Camera::Axis::Y);
    mSceneWidget->update();
}

void CameraControlWindow::lightRotateUp()
{
    ++mScene->lightSource().elevation();
    mScene->lightSource().update();
    mSceneWidget->update();
}

void CameraControlWindow::lightRotateDown()
{
    --mScene->lightSource().elevation();
    mScene->lightSource().update();
    mSceneWidget->update();
}

void CameraControlWindow::lightRotateLeft()
{
    ++mScene->lightSource().azimuth();
    mScene->lightSource().update();
    mSceneWidget->update();
}

void CameraControlWindow::lightRotateRight()
{
    --mScene->lightSource().azimuth();
    mScene->lightSource().update();
    mSceneWidget->update();
}

void CameraControlWindow::cameraZoomIn()
{
    mScene->camera().zoom(-1);
    mSceneWidget->update();
}

void CameraControlWindow::cameraZoomOut()
{
    mScene->camera().zoom(1);
    mSceneWidget->update();
}

void CameraControlWindow::cameraReset()
{
    mScene->camera().reset();
    mSceneWidget->update();
}

void CameraControlWindow::cameraBounds()
{
    Eigen::Vector2f translation = mScene->camera().translation();
    Rect3d cameraBounds = mScene->camera().getExtension();
    Eigen::Vector3f bottomLeft = cameraBounds.bottomLeft() + Eigen::Vector3f(translation.x(), translation.y(), 0);
    Eigen::Vector3f topRight = cameraBounds.topRight() + Eigen::Vector3f(translation.x(), translation.y(), 0);
    float zoom = mScene->camera().zoom();
    std::stringstream ss;
    ss << bottomLeft.x() << "," << bottomLeft.y() << "," << bottomLeft.z() << ";" <<
          topRight.x() << "," << topRight.y() << "," << topRight.z() << ";" << zoom;
    bool ok;
    QString input = QInputDialog::getText(this, "Camera bounds", "Bounds:", QLineEdit::Normal, ss.str().c_str(), &ok);
    if (!ok) return;
    std::string str = input.toStdString();
    std::vector<std::string> words = split(str, ';');
    if (words.size() != 3)
    {
        QMessageBox messageBox;
        messageBox.critical(this, "Error", "Invalid camera bounds");
        return;
    }
    std::vector<std::string> bottomLeftWords = split(words[0], ',');
    if (bottomLeftWords.size() != 3)
    {
        QMessageBox messageBox;
        messageBox.critical(this, "Error", "Invalid camera bounds");
        return;
    }
    bottomLeft.x() = atof(bottomLeftWords[0].c_str());
    bottomLeft.y() = atof(bottomLeftWords[1].c_str());
    bottomLeft.z() = atof(bottomLeftWords[2].c_str());
    std::vector<std::string> topRightWords = split(words[1], ',');
    if (topRightWords.size() != 3)
    {
        QMessageBox messageBox;
        messageBox.critical(this, "Error", "Invalid camera bounds");
        return;
    }
    topRight.x() = atof(topRightWords[0].c_str());
    topRight.y() = atof(topRightWords[1].c_str());
    topRight.z() = atof(topRightWords[2].c_str());
    zoom = atof(words[2].c_str());
    cameraBounds = Rect3d(bottomLeft, topRight);
    mScene->camera().setExtension(cameraBounds);
    mScene->camera().reset();
    mScene->camera().zoom(zoom);
}
