#ifndef OPENGLCONTEXT_H
#define OPENGLCONTEXT_H

#include <QSurface>
#include <QOpenGLContext>
#include <QOffscreenSurface>
#include <QOpenGLFunctions>
#include <QMatrix4x4>

#include <Eigen/Core>

class OpenGLContext : public QOffscreenSurface, public QOpenGLFunctions
{
    Q_OBJECT
public:
    OpenGLContext();

    OpenGLContext(const OpenGLContext &shader) = delete;

    void makeCurrent();

    void doneCurrent();

protected:
    QOpenGLContext *mContext;
    bool mCurrent;

    void initialize();

};

#endif // OPENGLCONTEXT_H
