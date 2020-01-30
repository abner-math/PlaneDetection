#ifndef PHONGSHADER_H
#define PHONGSHADER_H

#include "shader.h"

class PhongShader : public Shader
{
public:
    PhongShader(OpenGLContext *context);

    ~PhongShader();

    void createObject(SceneObject *object) override;

    void drawObject(SceneObject *object) override;

    void destroyObject(SceneObject *object) override;

    void setTransformationMatrix(const QMatrix4x4 &matrix) override;

    void setModelMatrix(const QMatrix4x4 &matrix) override;

    void setInverseTransposeModelMatrix(const QMatrix4x4 &matrix) override;

    void setInverseTransposeModelViewMatrix(const QMatrix4x4 &matrix) override;

    void setLightPosition(const QVector3D &position) override;

    void setLightColor(const QVector3D &color) override;

private:
    const static std::string sVertexShader;
    const static std::string sFragmentShader;
    int mPositionAttr;
    int mNormalAttr;
    int mTextureCoordAttr;
    int mColorAttr;
    int mAmbientColorAttr;
    int mSpecularColorAttr;
    int mSpecularExponentAttr;
    int mTransformationMatrixUniform;
    int mModelMatrixUniform;
    int mInverseTransposeModelMatrixUniform;
    int mInverseTransposeModelViewMatrixUniform;
    int mLightPositionUniform;
    int mLightColorUniform;
    std::map<const SceneObject*, QOpenGLVertexArrayObject*> mVAOs;

    void initializeProgram();

};

#endif // PHONGSHADER_H
