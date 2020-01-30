#include "vertexbuffercreator.h"
#include "angleutils.h"
#include "geometryutils.h"

VertexBufferCreator::VertexBufferCreator()
    : mCurrentNormal(Eigen::Vector3f::Zero())
    , mCurrentTextureCoord(Eigen::Vector2f::Zero())
    , mCurrentColor(Eigen::Vector4f::Constant(1))
    , mCurrentAmbientColor(Eigen::Vector4f(0.2f, 0.2f, 0.2f, 1.0f))
    , mCurrentSpecularColor(Eigen::Vector4f::Constant(1))
    , mCurrentSpecularExponent(0)
    , mCurrentPointSize(1.0f)
    , mCurrentTexture(NULL)
    , mDepthTestEnabled(true)
{

}

void VertexBufferCreator::color(const Eigen::Vector3f &color)
{
    mCurrentColor = Eigen::Vector4f(color.x(), color.y(), color.z(), 1.0f);
}

void VertexBufferCreator::color(const Eigen::Vector4f &color)
{
    mCurrentColor = color;
}

void VertexBufferCreator::color(float r, float g, float b)
{
    mCurrentColor = Eigen::Vector4f(r, g, b, 1.0f);
}

void VertexBufferCreator::color(float r, float g, float b, float a)
{
    mCurrentColor = Eigen::Vector4f(r, g, b, a);
}

void VertexBufferCreator::ambientColor(const Eigen::Vector3f &color)
{
    mCurrentAmbientColor = Eigen::Vector4f(color.x(), color.y(), color.z(), 1.0f);
}

void VertexBufferCreator::ambientColor(const Eigen::Vector4f &color)
{
    mCurrentAmbientColor = color;
}

void VertexBufferCreator::ambientColor(float r, float g, float b)
{
    mCurrentAmbientColor = Eigen::Vector4f(r, g, b, 1.0f);
}

void VertexBufferCreator::ambientColor(float r, float g, float b, float a)
{
    mCurrentAmbientColor = Eigen::Vector4f(r, g, b, a);
}

void VertexBufferCreator::specularColor(const Eigen::Vector3f &color)
{
    mCurrentSpecularColor = Eigen::Vector4f(color.x(), color.y(), color.z(), 1.0f);
}

void VertexBufferCreator::specularColor(const Eigen::Vector4f &color)
{
    mCurrentSpecularColor = color;
}

void VertexBufferCreator::specularColor(float r, float g, float b)
{
    mCurrentSpecularColor = Eigen::Vector4f(r, g, b, 1.0f);
}

void VertexBufferCreator::specularColor(float r, float g, float b, float a)
{
    mCurrentSpecularColor = Eigen::Vector4f(r, g, b, a);
}

void VertexBufferCreator::specularExponent(float exp)
{
    mCurrentSpecularExponent = exp;
}

void VertexBufferCreator::bindTexture(QOpenGLTexture *texture)
{
    mCurrentTexture = texture;
}

void VertexBufferCreator::normal(const Eigen::Vector3f &normal)
{
    mCurrentNormal = normal;
}

void VertexBufferCreator::normal(float x, float y, float z)
{
    mCurrentNormal = Eigen::Vector3f(x, y, z);
}

void VertexBufferCreator::textureCoord(const Eigen::Vector2f &coord)
{
    mCurrentTextureCoord = coord;
}

void VertexBufferCreator::textureCoord(float x, float y)
{
    mCurrentTextureCoord = Eigen::Vector2f(x, y);
}

void VertexBufferCreator::pointSize(float size)
{
    mCurrentPointSize = size;
}

void VertexBufferCreator::point(const Eigen::Vector3f &position)
{
    addArray(1, GeometryType::POINTS);
    vertex(position);
}

void VertexBufferCreator::point(const Vertex &v)
{
    addArray(1, GeometryType::POINTS);
    vertex(v);
}

void VertexBufferCreator::line(const Eigen::Vector3f &a, const Eigen::Vector3f &b)
{
    addArray(2, GeometryType::LINES);
    vertex(a);
    vertex(b);
}

void VertexBufferCreator::line(const Vertex &v1, const Vertex &v2)
{
    addArray(2, GeometryType::LINES);
    vertex(v1);
    vertex(v2);
}

void VertexBufferCreator::triangle(const Eigen::Vector3f &a, const Eigen::Vector3f &b, const Eigen::Vector3f &c)
{
    addArray(3, GeometryType::TRIANGLES);
    vertex(a);
    vertex(b);
    vertex(c);
}

void VertexBufferCreator::triangle(const Vertex &v1, const Vertex &v2, const Vertex &v3)
{
    addArray(3, GeometryType::TRIANGLES);
    vertex(v1);
    vertex(v2);
    vertex(v3);
}

void VertexBufferCreator::quad(const Eigen::Vector3f &a, const Eigen::Vector3f &b, const Eigen::Vector3f &c, const Eigen::Vector3f &d)
{
    addArray(4, GeometryType::QUADS);
    vertex(a);
    vertex(b);
    vertex(c);
    vertex(d);
}

void VertexBufferCreator::quad(const Vertex &v1, const Vertex &v2, const Vertex &v3, const Vertex &v4)
{
    addArray(4, GeometryType::QUADS);
    vertex(v1);
    vertex(v2);
    vertex(v3);
    vertex(v4);
}

void VertexBufferCreator::polygon(const std::vector<Eigen::Vector3f> &vertices)
{
    addArray(vertices.size(), GeometryType::POLYGONS);
    for (const Eigen::Vector3f &v : vertices)
    {
        vertex(v);
    }
}

void VertexBufferCreator::polygon(const std::vector<Vertex> &vertices)
{
    addArray(vertices.size(), GeometryType::POLYGONS);
    for (const Vertex &v : vertices)
    {
        vertex(v);
    }
}

void VertexBufferCreator::enableDepthTest(bool enabled)
{
    mDepthTestEnabled = enabled;
}

void VertexBufferCreator::addArray(size_t numVertices, GeometryType type)
{
    if (mArrays.empty())
    {
        mArrays.push_back(VertexArray(0, numVertices, type, mCurrentTexture, mDepthTestEnabled));
    }
    else
    {
        GeometryType lastType = mArrays[mArrays.size() - 1].type;
        if (lastType != GeometryType::POLYGONS && lastType == type &&
            mDepthTestEnabled == mArrays[mArrays.size() - 1].depthEnabled &&
            mCurrentTexture == mArrays[mArrays.size() - 1].texture)
        {
            mArrays[mArrays.size() - 1].count += numVertices;
        }
        else
        {
            size_t begin = mArrays[mArrays.size() - 1].begin + mArrays[mArrays.size() - 1].count;
            mArrays.push_back(VertexArray(begin, numVertices, type, mCurrentTexture, mDepthTestEnabled));
        }
    }
}

float* VertexBufferCreator::createCopy(const std::vector<float> &v)
{
    float *copy = new float[v.size()];
    std::memcpy(copy, v.data(), v.size() * sizeof(float));
    return copy;
}

void VertexBufferCreator::vertex(const Eigen::Vector3f &position)
{
    mPositions.push_back(position.x());
    mPositions.push_back(position.y());
    mPositions.push_back(position.z());
    mNormals.push_back(mCurrentNormal.x());
    mNormals.push_back(mCurrentNormal.y());
    mNormals.push_back(mCurrentNormal.z());
    mTextureCoords.push_back(mCurrentTextureCoord.x());
    mTextureCoords.push_back(mCurrentTextureCoord.y());
    mColors.push_back(mCurrentColor.x());
    mColors.push_back(mCurrentColor.y());
    mColors.push_back(mCurrentColor.z());
    mColors.push_back(mCurrentColor.w());
    mAmbientColors.push_back(mCurrentAmbientColor.x());
    mAmbientColors.push_back(mCurrentAmbientColor.y());
    mAmbientColors.push_back(mCurrentAmbientColor.z());
    mAmbientColors.push_back(mCurrentAmbientColor.w());
    mSpecularColors.push_back(mCurrentSpecularColor.x());
    mSpecularColors.push_back(mCurrentSpecularColor.y());
    mSpecularColors.push_back(mCurrentSpecularColor.z());
    mSpecularColors.push_back(mCurrentSpecularColor.w());
    mSpecularExponents.push_back(mCurrentSpecularExponent);
    mPointSizes.push_back(mCurrentPointSize);
}

void VertexBufferCreator::vertex(const Vertex &v)
{
    bindTexture(v.texture);
    specularExponent(v.specularExponent);
    specularColor(v.specularColor);
    ambientColor(v.ambientColor);
    color(v.color);
    normal(v.normal);
    textureCoord(v.textureCoord);
    pointSize(v.pointSize);
    vertex(v.position);
}

VertexBuffer* VertexBufferCreator::getVertexBuffer()
{
    float *positions = createCopy(mPositions);
    float *normals = createCopy(mNormals);
    float *textureCoords = createCopy(mTextureCoords);
    float *colors = createCopy(mColors);
    float *ambientColors = createCopy(mAmbientColors);
    float *specularColors = createCopy(mSpecularColors);
    float *specularExponents = createCopy(mSpecularExponents);
    float *pointSizes = createCopy(mPointSizes);
    return new VertexBuffer(positions, normals, textureCoords, colors, ambientColors,
                            specularColors, specularExponents, pointSizes, numVertices());
}

void VertexBufferCreator::clear()
{
    mPositions.clear();
    mNormals.clear();
    mTextureCoords.clear();
    mColors.clear();
    mAmbientColors.clear();
    mSpecularColors.clear();
    mSpecularExponents.clear();
    mPointSizes.clear();
    mArrays.clear();
}

void VertexBufferCreator::draw(OpenGLContext *context)
{
    for (VertexArray &array : mArrays)
    {
        if (array.depthEnabled)
        {
            context->glEnable(GL_DEPTH_TEST);
        }
        else
        {
            context->glEnable(GL_DEPTH_TEST);
        }
        if (array.texture)
        {
            array.texture->bind();
        }
        switch (array.type)
        {
        case POINTS:
            context->glDrawArrays(GL_POINTS, array.begin, array.count);
            break;
        case LINES:
            context->glDrawArrays(GL_LINES, array.begin, array.count);
            break;
        case TRIANGLES:
            context->glDrawArrays(GL_TRIANGLES, array.begin, array.count);
            break;
        case QUADS:
            context->glDrawArrays(GL_QUADS, array.begin, array.count);
            break;
        case POLYGONS:
            context->glDrawArrays(GL_POLYGON, array.begin, array.count);
            break;
        }
        if (array.texture)
        {
            array.texture->release();
        }
    }
}
