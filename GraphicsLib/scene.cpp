#include "scene.h"

Scene::Scene(QSize screenSize)
    : mContext(new OpenGLContext)
    , mSimpleShader(new PointShader(mContext))
    , mNormalShader(new NormalShader(mContext))
    , mPhongShader(new PhongShader(mContext))
    , mBuffer(NULL)
    , mClearColor(Eigen::Vector3f::Zero())
{
    initializeGL();
    size(screenSize);
    addObject(&mLight);
    connect(&mCamera, SIGNAL(cameraUpdated()), this, SIGNAL(cameraUpdated()));
    connect(&mCamera, SIGNAL(cameraUpdated()), this, SLOT(lightUpdated()));
}

Scene::~Scene()
{
    if (mBuffer != NULL)
    {
        delete mSimpleShader;
        delete mNormalShader;
        delete mPhongShader;
        mContext->makeCurrent();
        delete mBuffer;
        mContext->doneCurrent();
    }
    delete mContext;
}

void Scene::initializeGL()
{
    mContext->makeCurrent();
    mContext->glEnable(GL_DEPTH_TEST);
    mContext->glClearColor(0, 0, 0, 1);
    mContext->glEnable(GL_BLEND);
    mContext->glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    mContext->glEnable(GL_PROGRAM_POINT_SIZE);
    mContext->doneCurrent();
}

void Scene::size(int width, int height)
{
    mContext->makeCurrent();
    if (mBuffer != NULL)
    {
        delete mBuffer;
    }
    mBuffer = new FrameBuffer(width, height);
    mContext->glViewport(0, 0, mBuffer->width(), mBuffer->height());
    mContext->doneCurrent();
}

void Scene::size(QSize size)
{
    Scene::size(size.width(), size.height());
}

void Scene::clearColor(float r, float g, float b)
{
    mContext->makeCurrent();
    mContext->glClearColor(r, g, b, 1.0f);
    mClearColor = Eigen::Vector3f(r, g, b);
    mContext->doneCurrent();
}

void Scene::clearColor(const Eigen::Vector3f &color)
{
    clearColor(color.x(), color.y(), color.z());
}

void Scene::addObject(SceneObject *object, ShaderType shader)
{
    if (mObjects.find(object) != mObjects.end()) return;
    switch (shader)
    {
    case POINT:
        mObjects[object] = mSimpleShader;
        break;
    case NORMAL:
        mObjects[object] = mNormalShader;
        break;
    case PHONG:
        mObjects[object] = mPhongShader;
        break;
    }
    mContext->makeCurrent();
    mObjects[object]->createObject(object);
    mContext->doneCurrent();
    connect(object, SIGNAL(updated(SceneObject*)), this, SLOT(objectUpdated(SceneObject*)));
}

void Scene::removeObject(SceneObject *object)
{
    if (mObjects.find(object) == mObjects.end()) return;
    mObjects[object]->destroyObject(object);
    mObjects.erase(mObjects.find(object));
    disconnect(object, SIGNAL(updated(SceneObject*)), this, SLOT(objectUpdated(SceneObject*)));
}

void Scene::objectUpdated(SceneObject *object)
{
    mObjects[object]->destroyObject(object);
    mObjects[object]->createObject(object);
}

void Scene::lightUpdated()
{
    mLight.target() = mCamera.target();
}

void Scene::addFilter(Filter *filter)
{
    auto it = std::find(mFilters.begin(), mFilters.end(), filter);
    if (it != mFilters.end()) return;
    mFilters.push_back(filter);
}

void Scene::removeFilter(Filter *filter)
{
    auto it = std::find(mFilters.begin(), mFilters.end(), filter);
    if (it == mFilters.end()) return;
    mFilters.erase(it);
}

void Scene::render()
{
    if (mBuffer == NULL)
        throw "Frame buffer was not set yet.";

    mContext->makeCurrent();

    mBuffer->bind();

    mContext->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    drawObjects(SceneObject::Priority::HIGH);
    drawObjects(SceneObject::Priority::NORMAL);
    drawObjects(SceneObject::Priority::LOW);

    mContext->glFlush();

    mBuffer->release();

    for (Filter *filter : mFilters)
    {
        filter->filter(mBuffer);
    }
    if (!mFilters.empty())
    {
        mBuffer->update();
    }

    mContext->doneCurrent();
}

void Scene::drawObjects(SceneObject::Priority priority)
{
    Eigen::Matrix4f transformationMatrix = mCamera.transformationMatrix();
    Eigen::Matrix4f viewMatrix = mCamera.viewMatrix();
    Eigen::Vector3f lightPosition = mLight.position();
    Eigen::Vector3f lightColor = mLight.color();

    QVector3D qLightPosition(lightPosition.x(), lightPosition.y(), lightPosition.z());
    QVector3D qLightColor(lightColor.x(), lightColor.y(), lightColor.z());

    for (auto it = mObjects.begin(); it != mObjects.end(); ++it)
    {
        if (it->first->priority() == priority)
        {
            Eigen::Matrix4f modelMatrix = it->first->transform().matrix();
            Eigen::Matrix4f fullTransformationMatrix = transformationMatrix * modelMatrix;
            Eigen::Matrix4f inverseTransposeModel = modelMatrix.inverse().transpose();
            Eigen::Matrix4f inverseTransposeModelView = (viewMatrix * modelMatrix).inverse().transpose();

            QMatrix4x4 qModelMatrix(modelMatrix.data());
            QMatrix4x4 qTransformationMatrix(fullTransformationMatrix.data());
            QMatrix4x4 qInverseTransposeModelMatrix(inverseTransposeModel.data());
            QMatrix4x4 qInverseTransposeModelViewMatrix(inverseTransposeModelView.data());

            it->second->bind();
            it->second->setTransformationMatrix(qTransformationMatrix);
            it->second->setModelMatrix(qModelMatrix);
            it->second->setInverseTransposeModelMatrix(qInverseTransposeModelMatrix);
            it->second->setInverseTransposeModelViewMatrix(qInverseTransposeModelViewMatrix);
            it->second->setLightPosition(qLightPosition);
            it->second->setLightColor(qLightColor);
            it->second->drawObject(it->first);
            it->second->release();
        }
    }
}
