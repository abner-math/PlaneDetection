#ifndef SCENEOBJECT_H
#define SCENEOBJECT_H

#include <map>
#include <functional>

#include <QOpenGLFunctions>

#include "affinetransform.h"
#include "vertexbuffer.h"
#include "vertexbuffercreator.h"

class SceneObject : public QObject
{
    Q_OBJECT
public:
    enum Priority
    {
        LOW = 0,
        NORMAL = 1,
        HIGH = 2
    };

    SceneObject(Priority priority = NORMAL);

    virtual ~SceneObject();

    const VertexBuffer* data() const
    {
        return mData;
    }

    Priority priority() const
    {
        return mPriority;
    }

    void priority(Priority priority)
    {
        mPriority = priority;
    }

    AffineTransform& transform()
    {
        return mTransform;
    }

    void draw(OpenGLContext *context);

    void draw(std::function<void(VertexBufferCreator&)> &function)
    {
        mFunctions.push_back(function);
    }

    void update();

signals:
    void updated(SceneObject *object);

protected:
    virtual void create() = 0;

    VertexBufferCreator& buffer()
    {
        return mBufferCreator;
    }

private:
    VertexBuffer *mData;
    Priority mPriority;
    AffineTransform mTransform;
    VertexBufferCreator mBufferCreator;
    std::vector<std::function<void(VertexBufferCreator&)> > mFunctions;

};

#endif // SCENEOBJECT_H
