#include "openglcontext.h"

#include <iostream>

OpenGLContext::OpenGLContext()
    : mContext(NULL)
    , mCurrent(false)
{
    setFormat(QSurfaceFormat::defaultFormat());
    create();
    initialize();
}

void OpenGLContext::initialize()
{
    mContext = new QOpenGLContext(this);
    mContext->setFormat(format());
    if (mContext->create())
    {
        makeCurrent();
        initializeOpenGLFunctions();
        doneCurrent();
    }
    else
    {
        delete mContext;
        mContext = NULL;
        throw "Failed to create OpenGL Context.";
    }
}

void OpenGLContext::makeCurrent()
{
    if (mContext != NULL)
    {
        mContext->makeCurrent(this);
        mCurrent = true;
    }
    else
    {
        throw "Window not yet propertly initialized.";
    }
}

void OpenGLContext::doneCurrent()
{
    if (mContext != NULL)
    {
        mContext->doneCurrent();
        mCurrent = false;
    }
}

