#include "shader.h"

Shader::Shader(OpenGLContext *context)
    : mContext(context)
{
    mContext->makeCurrent();
    mProgram = new QOpenGLShaderProgram(context);
}

Shader::~Shader()
{
    mProgram->release();
    delete mProgram;
}

void Shader::bind()
{
    mProgram->bind();
}

void Shader::release()
{
    mProgram->release();
}
