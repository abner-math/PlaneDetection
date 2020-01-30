#include "framebuffer.h"

#include <QImage>

FrameBuffer::FrameBuffer(int width, int height)
    : mFBO(new QOpenGLFramebufferObject(QSize(width, height)))
    , mColorPixels(new float[width * height * 3])
    , mDepthPixels(new float[width * height])
{
    mFBO->setAttachment(QOpenGLFramebufferObject::Depth);
}

FrameBuffer::FrameBuffer(QSize size)
    : FrameBuffer(size.width(), size.height())
{

}

FrameBuffer::~FrameBuffer()
{
    mFBO->release();
    delete mFBO;
    delete[] mColorPixels;
    delete[] mDepthPixels;
}

void FrameBuffer::bind()
{
    mFBO->bind();
}

void FrameBuffer::release()
{
    glReadPixels(0, 0, mFBO->width(), mFBO->height(), GL_RGB, GL_FLOAT, mColorPixels);
    glReadPixels(0, 0, mFBO->width(), mFBO->height(), GL_DEPTH_COMPONENT, GL_FLOAT, mDepthPixels);
    mImage = mFBO->toImage();
    mFBO->release();
}

void FrameBuffer::pixelColor(int x, int y, float &r, float &g, float &b) const
{
    int pixelPos = (height() - y - 1) * width() * 3 + x * 3;
    r = mColorPixels[pixelPos];
    g = mColorPixels[pixelPos + 1];
    b = mColorPixels[pixelPos + 2];
}

void FrameBuffer::setPixelColor(int x, int y, float r, float g, float b)
{
    int pixelPos = (height() - y - 1) * width() * 3 + x * 3;
    mColorPixels[pixelPos] = r;
    mColorPixels[pixelPos + 1] = g;
    mColorPixels[pixelPos + 2] = b;
}

void FrameBuffer::update()
{
    mFBO->bind();
    glDrawPixels(width(), height(), GL_RGB, GL_FLOAT, mColorPixels);
    mImage = mFBO->toImage();
    mFBO->release();
}

void FrameBuffer::pixelDepth(int x, int y, float &depth) const
{
    depth = mDepthPixels[(height() - y - 1) * width() + x];
}

float FrameBuffer::minDepth() const
{
    float min = std::numeric_limits<float>::max();
    for (size_t i = 0, end = width() * height(); i < end; i++)
    {
        if (mDepthPixels[i] < min) min = mDepthPixels[i];
    }
    return min;
}

float FrameBuffer::maxDepth() const
{
    float max = -std::numeric_limits<float>::max();
    for (size_t i = 0, end = width() * height(); i < end; i++)
    {
        if (mDepthPixels[i] > max) max = mDepthPixels[i];
    }
    return max;
}

void FrameBuffer::saveAsImage(const std::string &filename) const
{
    mImage.save(filename.c_str());
}
