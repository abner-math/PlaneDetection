#ifndef SIMPLESHADER_H
#define SIMPLESHADER_H

#include "shader.h"

class PointShader : public Shader
{
public:
    PointShader(OpenGLContext *context);

    ~PointShader();

    void createObject(SceneObject *object) override;

    void drawObject(SceneObject *object) override;

    void destroyObject(SceneObject *object) override;

    void setTransformationMatrix(const QMatrix4x4 &matrix) override;

private:
    int mPositionAttr;
    int mPointSizeAttr;
    int mColorAttr;
    int mTransformationMatrixUniform;
    std::map<const SceneObject*, QOpenGLVertexArrayObject*> mVAOs;

    void initializeProgram();

};

#endif // SIMPLESHADER_H
