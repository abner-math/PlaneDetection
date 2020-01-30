#ifndef SCENE_H
#define SCENE_H

#include <map>

#include <QOpenGLFunctions>
#include <QThread>

#include "sceneobject.h"
#include "camera.h"
#include "framebuffer.h"
#include "openglcontext.h"
#include "lightsource.h"
#include "pointshader.h"
#include "normalshader.h"
#include "phongshader.h"
#include "filter.h"

class Scene : public QObject
{
    Q_OBJECT
public:
    enum ShaderType
    {
        POINT = 0,
        NORMAL = 1,
        PHONG = 2
    };

    Scene(QSize size = QSize(1, 1));

    ~Scene();

    Scene(const Scene &scene) = delete;

    void size(QSize size);

    void size(int width, int height);

    void clearColor(float r, float g, float b);

    void clearColor(const Eigen::Vector3f &color);

    const Eigen::Vector3f& clearColor() const
    {
        return mClearColor;
    }

    void addObject(SceneObject *object, ShaderType shader = POINT);

    void removeObject(SceneObject *object);

    void addFilter(Filter *filter);

    void removeFilter(Filter *filter);

    void render();

    const FrameBuffer* frameBuffer() const
    {
        return mBuffer;
    }

    Camera& camera()
    {
        return mCamera;
    }

    LightSource& lightSource()
    {
        return mLight;
    }

signals:
    void cameraUpdated();

private slots:
    void objectUpdated(SceneObject *object);
    void lightUpdated();

private:
    OpenGLContext *mContext;
    PointShader *mSimpleShader;
    NormalShader *mNormalShader;
    PhongShader *mPhongShader;
    FrameBuffer *mBuffer;
    Camera mCamera;
    LightSource mLight;
    Eigen::Vector3f mClearColor;
    std::map<SceneObject*, Shader*> mObjects;
    std::vector<Filter*> mFilters;

    void initializeGL();

    void drawObjects(SceneObject::Priority priority);

};

#endif // SCENE_H
