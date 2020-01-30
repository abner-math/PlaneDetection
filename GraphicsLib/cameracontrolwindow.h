#ifndef CAMERACONTROLWINDOW_H
#define CAMERACONTROLWINDOW_H

#include <QMainWindow>
#include <QMenuBar>
#include <QMenu>
#include <QHBoxLayout>

#include "scenewidget.h"

class CameraControlWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit CameraControlWindow(QWidget *parent = 0);

protected:
    SceneWidget *mSceneWidget;
    Scene *mScene;
    QMenu *mViewMenu;
    QHBoxLayout *mLayout;

private slots:
    void cameraTranslateUp();
    void cameraTranslateDown();
    void cameraTranslateLeft();
    void cameraTranslateRight();
    void cameraRotateUp();
    void cameraRotateDown();
    void cameraRotateLeft();
    void cameraRotateRight();
    void cameraBounds();
    void lightRotateUp();
    void lightRotateDown();
    void lightRotateLeft();
    void lightRotateRight();
    void cameraZoomIn();
    void cameraZoomOut();
    void cameraReset();

};

template<typename Out>
void split(const std::string &s, char delim, Out result) {
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        *(result++) = item;
    }
}

std::vector<std::string> split(const std::string &s, char delim);

#endif // CAMERACONTROLWINDOW_H
