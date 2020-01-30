#ifndef NORMALSHADER_H
#define NORMALSHADER_H

#include "shader.h"

class NormalShader : public Shader
{
public:
    NormalShader(OpenGLContext *context);

    ~NormalShader();

    void createObject(SceneObject *object) override;

    void drawObject(SceneObject *object) override;

    void destroyObject(SceneObject *object) override;

    void setTransformationMatrix(const QMatrix4x4 &matrix) override;

    void setInverseTransposeModelMatrix(const QMatrix4x4 &matrix) override;

private:
    int mPositionAttr;
    int mNormalAttr;
    int mTransformationMatrixUniform;
    int mInverseTransposeModelMatrixUniform;
    std::map<const SceneObject*, QOpenGLVertexArrayObject*> mVAOs;

    void initializeProgram();

};

#endif // NORMALSHADER_H
