#include "pointshader.h"

PointShader::PointShader(OpenGLContext *context)
    : Shader(context)
{
    initializeProgram();
}

PointShader::~PointShader()
{
    mContext->makeCurrent();
    for (auto it = mVAOs.begin(); it != mVAOs.end(); ++it)
    {
        it->second->destroy();
        delete it->second;
    }
    mContext->doneCurrent();
}

void PointShader::initializeProgram()
{
    std::string vertexShader = "\
            attribute mediump vec3 positionAttr;\
            attribute lowp vec4 colorAttr;\
            attribute highp float pointSizeAttr;\
            uniform mediump mat4 transformationMatrix;\
            \
            varying lowp vec4 color;\
            \
            void main(void)\
            {\
                color = colorAttr;\
                gl_PointSize = pointSizeAttr;\
                gl_Position = vec4(positionAttr, 1) * transformationMatrix;\
            }";
    std::string fragmentShader = "\
            varying lowp vec4 color;\
            \
            void main(void)\
            {\
                gl_FragColor = color;\
            }";
    if (!mProgram->addShaderFromSourceCode(QOpenGLShader::Vertex, vertexShader.c_str()))
        throw "Could not compile vertex shader.";
    if (!mProgram->addShaderFromSourceCode(QOpenGLShader::Fragment, fragmentShader.c_str()))
        throw "Could not compile fragment shader.";
    if (!mProgram->link()) throw "Could not link shader.";

    mPositionAttr = mProgram->attributeLocation("positionAttr");
    mPointSizeAttr = mProgram->attributeLocation("pointSizeAttr");
    mColorAttr = mProgram->attributeLocation("colorAttr");
    mTransformationMatrixUniform = mProgram->uniformLocation("transformationMatrix");
}

void PointShader::setTransformationMatrix(const QMatrix4x4 &transformationMatrix)
{
    mProgram->setUniformValue(mTransformationMatrixUniform, transformationMatrix);
}

void PointShader::createObject(SceneObject *object)
{
    if (mVAOs.find(object) != mVAOs.end()) return;

    if (object->data() == NULL)
        object->update();

    mContext->makeCurrent();

    QOpenGLVertexArrayObject *vao = new QOpenGLVertexArrayObject;
    vao->create();
    vao->bind();

    mContext->glEnableVertexAttribArray(mPositionAttr);
    QOpenGLBuffer positionBuffer;
    positionBuffer.create();
    positionBuffer.setUsagePattern(QOpenGLBuffer::StaticDraw);
    positionBuffer.bind();
    positionBuffer.allocate(object->data()->positions(), object->data()->numVertices() * 3 * sizeof(float));
    mContext->glVertexAttribPointer(mPositionAttr, 3, GL_FLOAT, GL_FALSE, 0, NULL);

    mContext->glEnableVertexAttribArray(mPointSizeAttr);
    QOpenGLBuffer pointSizeBuffer;
    pointSizeBuffer.create();
    pointSizeBuffer.setUsagePattern(QOpenGLBuffer::StaticDraw);
    pointSizeBuffer.bind();
    pointSizeBuffer.allocate(object->data()->pointSizes(), object->data()->numVertices() * sizeof(float));
    mContext->glVertexAttribPointer(mPointSizeAttr, 1, GL_FLOAT, GL_FALSE, 0, NULL);

    mContext->glEnableVertexAttribArray(mColorAttr);
    QOpenGLBuffer colorBuffer;
    colorBuffer.create();
    colorBuffer.setUsagePattern(QOpenGLBuffer::StaticDraw);
    colorBuffer.bind();
    colorBuffer.allocate(object->data()->colors(), object->data()->numVertices() * 4 * sizeof(float));
    mContext->glVertexAttribPointer(mColorAttr, 4, GL_FLOAT, GL_FALSE, 0, NULL);

    vao->release();

    mContext->doneCurrent();

    mVAOs[object] = vao;
}

void PointShader::drawObject(SceneObject *object)
{
    if (mVAOs.find(object) == mVAOs.end())
        throw "Cannot draw an object that was not created first.";

    mVAOs[object]->bind();
    object->draw(mContext);
    mVAOs[object]->release();
}

void PointShader::destroyObject(SceneObject *object)
{
    if (mVAOs.find(object) == mVAOs.end()) return;
    if (mVAOs.find(object) != mVAOs.end())
    {
        mContext->makeCurrent();

        mVAOs[object]->destroy();
        delete mVAOs[object];
        mVAOs.erase(mVAOs.find(object));

        mContext->doneCurrent();
    }
}
