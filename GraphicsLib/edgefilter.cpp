#include "edgefilter.h"

//#include <opencv2/core/core.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/highgui/highgui.hpp>

void EdgeFilter::filter(FrameBuffer *buffer)
{
    /*float minDepth = buffer->minDepth();
    float maxDepth = buffer->maxDepth();
    cv::Mat img = cv::Mat(buffer->height(), buffer->width(), CV_8U);
    for (int y = 0; y < img.rows; y++)
    {
        for (int x = 0; x < img.cols; x++)
        {
            float depth;
            buffer->pixelDepth(x, y, depth);
            img.at<uchar>(y, x) = static_cast<uchar>((depth - minDepth) / (maxDepth - minDepth) * 255);
        }
    }
    cv::Mat gradX, gradY, absGradX, absGradY;
    cv::Sobel(img, gradX, CV_16S, 1, 0);
    cv::Sobel(img, gradY, CV_16S, 0, 1);
    cv::Mat edges;
    cv::Canny(img, edges, 50, 100, 3);
    cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3), cv::Point(1, 1));
    cv::dilate(edges, edges, element);
    const uchar *ptr = (uchar*)edges.data;
    const short *ptrX = (short*)gradX.data;
    const short *ptrY = (short*)gradY.data;
    for (int y = 0; y < img.rows; y++)
    {
        for (int x = 0; x < img.cols; x++)
        {
            float edge = 1.0f - std::min(1.0f, *ptr / 255.0f + std::sqrt(std::pow(*ptrX/255.0f, 2.0f) + std::pow(*ptrY/255.0f, 2.0f)));
            float r, g, b;
            buffer->pixelColor(x, y, r, g, b);
            r *= edge;
            g *= edge;
            b *= edge;
            buffer->setPixelColor(x, y, r, g, b);
            ++ptrX;
            ++ptrY;
            ++ptr;
        }
    }*/
}
