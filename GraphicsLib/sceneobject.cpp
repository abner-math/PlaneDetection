#include "sceneobject.h"

#include <algorithm>

#include <QOpenGLBuffer>

SceneObject::SceneObject(Priority priority)
    : mData(NULL)
    , mPriority(priority)
{

}

SceneObject::~SceneObject()
{
    delete mData;
}

void SceneObject::draw(OpenGLContext *context)
{
    mBufferCreator.draw(context);
}

void SceneObject::update()
{
    if (mData != NULL)
        delete mData;

    mBufferCreator.clear();
    create();
    for (std::function<void(VertexBufferCreator&)> &func : mFunctions)
    {
        func(mBufferCreator);
    }
    mData = mBufferCreator.getVertexBuffer();

    emit updated(this);
}

