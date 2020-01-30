#ifndef FRAMEBUFFER_H
#define FRAMEBUFFER_H

#include <QOpenGLFramebufferObject>
#include <QImage>

class FrameBuffer
{
public:
    FrameBuffer(QSize size);

    FrameBuffer(int width, int height);

    FrameBuffer(const FrameBuffer &buffer) = delete;

    ~FrameBuffer();

    void bind();

    void release();

    void pixelColor(int x, int y, float &r, float &g, float &b) const;

    void pixelDepth(int x, int y, float &depth) const;

    void setPixelColor(int x, int y, float r, float g, float b);

    void update();

    void saveAsImage(const std::string &filename) const;

    const QImage& image() const
    {
        return mImage;
    }

    int width() const
    {
        return mFBO->width();
    }

    int height() const
    {
        return mFBO->height();
    }

    const float* colorBuffer() const
    {
        return mColorPixels;
    }

    const float* depthBuffer() const
    {
        return mDepthPixels;
    }

    float minDepth() const;

    float maxDepth() const;

private:
    QOpenGLFramebufferObject *mFBO;
    float *mColorPixels;
    float *mDepthPixels;
    QImage mImage;

};

#endif // FRAMEBUFFER_H
