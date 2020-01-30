#ifndef SHADER_H
#define SHADER_H

#include <QOpenGLShaderProgram>
#include <QOffscreenSurface>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLBuffer>

#include "sceneobject.h"
#include "openglcontext.h"
#include "framebuffer.h"

class Shader : public QObject
{
    Q_OBJECT
public:
    Shader(OpenGLContext *context);

    virtual ~Shader();

    void bind();

    void release();

    virtual void createObject(SceneObject *object) = 0;

    virtual void drawObject(SceneObject *object) = 0;

    virtual void destroyObject(SceneObject *object) = 0;

    virtual void setTransformationMatrix(const QMatrix4x4 &matrix)
    {
        Q_UNUSED(matrix);
    }

    virtual void setModelMatrix(const QMatrix4x4 &matrix)
    {
        Q_UNUSED(matrix);
    }

    virtual void setInverseTransposeModelMatrix(const QMatrix4x4 &matrix)
    {
        Q_UNUSED(matrix);
    }

    virtual void setInverseTransposeModelViewMatrix(const QMatrix4x4 &matrix)
    {
        Q_UNUSED(matrix);
    }

    virtual void setLightPosition(const QVector3D &position)
    {
        Q_UNUSED(position);
    }

    virtual void setLightColor(const QVector3D &color)
    {
        Q_UNUSED(color);
    }

protected:
    OpenGLContext *mContext;
    QOpenGLShaderProgram *mProgram;

};

#endif // SHADER_H
