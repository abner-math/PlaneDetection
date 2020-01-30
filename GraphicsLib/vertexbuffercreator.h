#ifndef VERTEXBUFFERCREATOR_H
#define VERTEXBUFFERCREATOR_H

#include <QOpenGLTexture>

#include <Eigen/Core>

#include "vertexbuffer.h"
#include "openglcontext.h"

class VertexBufferCreator
{
public:
    struct Vertex
    {
        Eigen::Vector3f position;
        Eigen::Vector3f normal;
        Eigen::Vector2f textureCoord;
        Eigen::Vector4f color;
        Eigen::Vector4f ambientColor;
        Eigen::Vector4f specularColor;
        float specularExponent;
        float pointSize;
        QOpenGLTexture *texture;
    };

    VertexBufferCreator();

    void color(const Eigen::Vector3f &color);

    void color(const Eigen::Vector4f &color);

    void color(float r, float g, float b);

    void color(float r, float g, float b, float a);

    void ambientColor(const Eigen::Vector3f &color);

    void ambientColor(const Eigen::Vector4f &color);

    void ambientColor(float r, float g, float b);

    void ambientColor(float r, float g, float b, float a);

    void specularColor(const Eigen::Vector3f &specularColor);

    void specularColor(const Eigen::Vector4f &specularColor);

    void specularColor(float r, float g, float b);

    void specularColor(float r, float g, float b, float a);

    void specularExponent(float exp);

    void bindTexture(QOpenGLTexture *texture);

    void normal(const Eigen::Vector3f &normal);

    void normal(float x, float y, float z);

    void textureCoord(const Eigen::Vector2f &coord);

    void textureCoord(float x, float y);

    void pointSize(float size);

    void point(const Eigen::Vector3f &point);

    void point(const Vertex &vertex);

    void line(const Eigen::Vector3f &a, const Eigen::Vector3f &b);

    void line(const Vertex &v1, const Vertex &v2);

    void triangle(const Eigen::Vector3f &a, const Eigen::Vector3f &b, const Eigen::Vector3f &c);

    void triangle(const Vertex &v1, const Vertex &v2, const Vertex &v3);

    void quad(const Eigen::Vector3f &a, const Eigen::Vector3f &b, const Eigen::Vector3f &c, const Eigen::Vector3f &d);

    void quad(const Vertex &v1, const Vertex &v2, const Vertex &v3, const Vertex &v4);

    void polygon(const std::vector<Eigen::Vector3f> &vertices);

    void polygon(const std::vector<Vertex> &vertices);

    void enableDepthTest(bool enabled);

    VertexBuffer* getVertexBuffer();

    void draw(OpenGLContext *functions);

    void clear();

    size_t numVertices() const
    {
        return mPositions.size() / 3;
    }

private:

    enum GeometryType
    {
        POINTS = 0,
        LINES = 1,
        TRIANGLES = 2,
        QUADS = 3,
        POLYGONS = 4
    };

    struct VertexArray
    {
        GeometryType type;
        size_t begin;
        size_t count;
        std::vector<GLuint> indices;
        QOpenGLTexture *texture;
        bool depthEnabled;

        VertexArray(size_t begin, size_t count, GeometryType type, QOpenGLTexture *texture,
                    bool depthEnabled)
            : type(type)
            , begin(begin)
            , count(count)
            , texture(texture)
            , depthEnabled(depthEnabled)
        {

        }
        VertexArray()
            : type(GeometryType::POLYGONS)
        {

        }
    };

    std::vector<float> mPositions;
    std::vector<float> mNormals;
    std::vector<float> mTextureCoords;
    std::vector<float> mColors;
    std::vector<float> mAmbientColors;
    std::vector<float> mSpecularColors;
    std::vector<float> mSpecularExponents;
    std::vector<float> mPointSizes;
    Eigen::Vector3f mCurrentNormal;
    Eigen::Vector2f mCurrentTextureCoord;
    Eigen::Vector4f mCurrentColor;
    Eigen::Vector4f mCurrentAmbientColor;
    Eigen::Vector4f mCurrentSpecularColor;
    float mCurrentSpecularExponent;
    float mCurrentPointSize;
    QOpenGLTexture *mCurrentTexture;
    std::vector<VertexArray> mArrays;
    bool mDepthTestEnabled;

    void vertex(const Eigen::Vector3f &position);

    void vertex(const Vertex &v);

    void addArray(size_t numVertices, GeometryType type);

    float* createCopy(const std::vector<float> &v);

};

#endif // VERTEXBUFFERCREATOR_H
