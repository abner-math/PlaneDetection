#include "vertexbuffer.h"

VertexBuffer::VertexBuffer(float *positions, float *normals, float *textureCoords, float *colors,
                           float *ambientColors, float *specularColors, float *specularExponents,
                           float *pointSizes, size_t numVertices)
    : mPositions(positions)
    , mNormals(normals)
    , mTextureCoords(textureCoords)
    , mColors(colors)
    , mAmbientColors(ambientColors)
    , mSpecularColors(specularColors)
    , mSpecularExponents(specularExponents)
    , mPointSizes(pointSizes)
    , mNumVertices(numVertices)
{

}

VertexBuffer::~VertexBuffer()
{
    delete[] mPositions;
    delete[] mNormals;
    delete[] mTextureCoords;
    delete[] mColors;
    delete[] mAmbientColors;
    delete[] mSpecularColors;
    delete[] mSpecularExponents;
    delete[] mPointSizes;
}
