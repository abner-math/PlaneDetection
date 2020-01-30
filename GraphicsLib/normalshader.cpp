#include "normalshader.h"

NormalShader::NormalShader(OpenGLContext *context)
    : Shader(context)
{
    initializeProgram();
}

NormalShader::~NormalShader()
{
    mContext->makeCurrent();
    for (auto it = mVAOs.begin(); it != mVAOs.end(); ++it)
    {
        it->second->destroy();
        delete it->second;
    }
    mContext->doneCurrent();
}

void NormalShader::initializeProgram()
{
    std::string vertexShader = "\
            attribute mediump vec3 positionAttr;\
            attribute mediump vec3 normalAttr;\
            uniform mediump mat4 transformationMatrix;\
            uniform mediump mat4 inverseTransposeModelMatrix;\
            \
            varying mediump vec3 color;\
            \
            void main(void)\
            {\
                color = (vec4(normalAttr, 1) * inverseTransposeModelMatrix).xyz;\
                gl_Position = vec4(positionAttr, 1) * transformationMatrix;\
            }";
    std::string fragmentShader = "\
            varying mediump vec3 color;\
            \
            void main(void)\
            {\
                gl_FragColor = vec4(normalize(color), 1.0f);\
            }";
    if (!mProgram->addShaderFromSourceCode(QOpenGLShader::Vertex, vertexShader.c_str()))
        throw "Could not compile vertex shader.";
    if (!mProgram->addShaderFromSourceCode(QOpenGLShader::Fragment, fragmentShader.c_str()))
        throw "Could not compile fragment shader.";
    if (!mProgram->link()) throw "Could not link shader.";

    mPositionAttr = mProgram->attributeLocation("positionAttr");
    mNormalAttr = mProgram->attributeLocation("normalAttr");
    mTransformationMatrixUniform = mProgram->uniformLocation("transformationMatrix");
    mInverseTransposeModelMatrixUniform = mProgram->uniformLocation("inverseTransposeModelMatrix");
}

void NormalShader::setTransformationMatrix(const QMatrix4x4 &transformationMatrix)
{
    mProgram->setUniformValue(mTransformationMatrixUniform, transformationMatrix);
}

void NormalShader::setInverseTransposeModelMatrix(const QMatrix4x4 &transformationMatrix)
{
    mProgram->setUniformValue(mInverseTransposeModelMatrixUniform, transformationMatrix);
}

void NormalShader::createObject(SceneObject *object)
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

    mContext->glEnableVertexAttribArray(mNormalAttr);
    QOpenGLBuffer normalBuffer;
    normalBuffer.create();
    normalBuffer.setUsagePattern(QOpenGLBuffer::StaticDraw);
    normalBuffer.bind();
    normalBuffer.allocate(object->data()->normals(), object->data()->numVertices() * 3 * sizeof(float));
    mContext->glVertexAttribPointer(mNormalAttr, 3, GL_FLOAT, GL_FALSE, 0, NULL);

    vao->release();

    mContext->doneCurrent();

    mVAOs[object] = vao;
}

void NormalShader::drawObject(SceneObject *object)
{
    if (mVAOs.find(object) == mVAOs.end())
        throw "Cannot draw an object that was not created first.";
    mVAOs[object]->bind();
    object->draw(mContext);
    mVAOs[object]->release();
}

void NormalShader::destroyObject(SceneObject *object)
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
