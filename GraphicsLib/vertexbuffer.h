#ifndef VERTEXBUFFER_H
#define VERTEXBUFFER_H

#include "string.h"

class VertexBuffer
{
public:
    VertexBuffer(float *positions, float *normals, float *textureCoords, float *colors,
                 float *ambientColors, float *specularColors, float *specularExponents,
                 float *pointSizes, size_t numVertices);

    ~VertexBuffer();

    const float* positions() const
    {
        return mPositions;
    }

    const float* normals() const
    {
        return mNormals;
    }

    const float* textureCoords() const
    {
        return mTextureCoords;
    }

    const float* colors() const
    {
        return mColors;
    }

    const float* ambientColors() const
    {
        return mAmbientColors;
    }

    const float* specularColors() const
    {
        return mSpecularColors;
    }

    const float* specularExponents() const
    {
        return mSpecularExponents;
    }

    const float* pointSizes() const
    {
        return mPointSizes;
    }

    size_t numVertices() const
    {
        return mNumVertices;
    }

private:
    float *mPositions;
    float *mNormals;
    float *mTextureCoords;
    float *mColors;
    float *mAmbientColors;
    float *mSpecularColors;
    float *mSpecularExponents;
    float *mPointSizes;
    size_t mNumVertices;

};

#endif // VERTEXBUFFER_H
