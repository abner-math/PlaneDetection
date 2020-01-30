#include "phongshader.h"

const std::string PhongShader::sVertexShader = ":/shaders/phongvertexshader.vsh";
const std::string PhongShader::sFragmentShader = ":/shaders/phongfragmentshader.frag";

PhongShader::PhongShader(OpenGLContext *context)
    : Shader(context)
{
    initializeProgram();
}

PhongShader::~PhongShader()
{
    mContext->makeCurrent();
    for (auto it = mVAOs.begin(); it != mVAOs.end(); ++it)
    {
        it->second->destroy();
        delete it->second;
    }
    mContext->doneCurrent();
}

void PhongShader::initializeProgram()
{
    if (!mProgram->addShaderFromSourceFile(QOpenGLShader::Vertex, sVertexShader.c_str()))
        throw "Could not compile vertex shader.";
    if (!mProgram->addShaderFromSourceFile(QOpenGLShader::Fragment, sFragmentShader.c_str()))
        throw "Could not compile fragment shader.";
    if (!mProgram->link()) throw "Could not link shader.";

    mPositionAttr = mProgram->attributeLocation("positionAttr");
    mNormalAttr = mProgram->attributeLocation("normalAttr");
    mTextureCoordAttr = mProgram->attributeLocation("textureCoordAttr");
    mColorAttr = mProgram->attributeLocation("colorAttr");
    mAmbientColorAttr = mProgram->attributeLocation("ambientColorAttr");
    mSpecularColorAttr = mProgram->attributeLocation("specularColorAttr");
    mSpecularExponentAttr = mProgram->attributeLocation("specularExponentAttr");
    mTransformationMatrixUniform = mProgram->uniformLocation("transformationMatrix");
    mModelMatrixUniform = mProgram->uniformLocation("modelMatrix");
    mInverseTransposeModelMatrixUniform = mProgram->uniformLocation("inverseTransposeModelMatrix");
    mInverseTransposeModelViewMatrixUniform = mProgram->uniformLocation("inverseTransposeModelViewMatrix");
    mLightPositionUniform = mProgram->uniformLocation("lightPosition");
    mLightColorUniform = mProgram->uniformLocation("lightColor");
}

void PhongShader::setTransformationMatrix(const QMatrix4x4 &transformationMatrix)
{
    mProgram->setUniformValue(mTransformationMatrixUniform, transformationMatrix);
}

void PhongShader::setModelMatrix(const QMatrix4x4 &modelMatrix)
{
    mProgram->setUniformValue(mModelMatrixUniform, modelMatrix);
}

void PhongShader::setInverseTransposeModelMatrix(const QMatrix4x4 &inverseModelMatrix)
{
    mProgram->setUniformValue(mInverseTransposeModelMatrixUniform, inverseModelMatrix);
}

void PhongShader::setInverseTransposeModelViewMatrix(const QMatrix4x4 &inverseModelViewMatrix)
{
    mProgram->setUniformValue(mInverseTransposeModelViewMatrixUniform, inverseModelViewMatrix);
}

void PhongShader::setLightPosition(const QVector3D &position)
{
    mProgram->setUniformValue(mLightPositionUniform, position);
}

void PhongShader::setLightColor(const QVector3D &color)
{
    mProgram->setUniformValue(mLightColorUniform, color);
}

void PhongShader::createObject(SceneObject *object)
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

    mContext->glEnableVertexAttribArray(mTextureCoordAttr);
    QOpenGLBuffer textureCoordBuffer;
    textureCoordBuffer.create();
    textureCoordBuffer.setUsagePattern(QOpenGLBuffer::StaticDraw);
    textureCoordBuffer.bind();
    textureCoordBuffer.allocate(object->data()->textureCoords(), object->data()->numVertices() * 2 * sizeof(float));
    mContext->glVertexAttribPointer(mTextureCoordAttr, 2, GL_FLOAT, GL_FALSE, 0, NULL);

    mContext->glEnableVertexAttribArray(mColorAttr);
    QOpenGLBuffer colorBuffer;
    colorBuffer.create();
    colorBuffer.setUsagePattern(QOpenGLBuffer::StaticDraw);
    colorBuffer.bind();
    colorBuffer.allocate(object->data()->colors(), object->data()->numVertices() * 4 * sizeof(float));
    mContext->glVertexAttribPointer(mColorAttr, 4, GL_FLOAT, GL_FALSE, 0, NULL);

    mContext->glEnableVertexAttribArray(mAmbientColorAttr);
    QOpenGLBuffer ambientColorBuffer;
    ambientColorBuffer.create();
    ambientColorBuffer.setUsagePattern(QOpenGLBuffer::StaticDraw);
    ambientColorBuffer.bind();
    ambientColorBuffer.allocate(object->data()->ambientColors(), object->data()->numVertices() * 4 * sizeof(float));
    mContext->glVertexAttribPointer(mAmbientColorAttr, 4, GL_FLOAT, GL_FALSE, 0, NULL);

    mContext->glEnableVertexAttribArray(mSpecularColorAttr);
    QOpenGLBuffer specularColorBuffer;
    specularColorBuffer.create();
    specularColorBuffer.setUsagePattern(QOpenGLBuffer::StaticDraw);
    specularColorBuffer.bind();
    specularColorBuffer.allocate(object->data()->specularColors(), object->data()->numVertices() * 4 * sizeof(float));
    mContext->glVertexAttribPointer(mSpecularColorAttr, 4, GL_FLOAT, GL_FALSE, 0, NULL);

    mContext->glEnableVertexAttribArray(mSpecularExponentAttr);
    QOpenGLBuffer specularExponentBuffer;
    specularExponentBuffer.create();
    specularExponentBuffer.setUsagePattern(QOpenGLBuffer::StaticDraw);
    specularExponentBuffer.bind();
    specularExponentBuffer.allocate(object->data()->specularExponents(), object->data()->numVertices() * sizeof(float));
    mContext->glVertexAttribPointer(mSpecularExponentAttr, 1, GL_FLOAT, GL_FALSE, 0, NULL);

    vao->release();

    mContext->doneCurrent();

    mVAOs[object] = vao;
}

void PhongShader::drawObject(SceneObject *object)
{
    if (mVAOs.find(object) == mVAOs.end())
        throw "Cannot draw an object that was not created first.";

    mVAOs[object]->bind();
    object->draw(mContext);
    mVAOs[object]->release();
}

void PhongShader::destroyObject(SceneObject *object)
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
