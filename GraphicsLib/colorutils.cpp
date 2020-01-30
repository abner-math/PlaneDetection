#include "colorutils.h"

#include "angleutils.h"

const int ColorUtils::NCOLS;
Eigen::Matrix3Xi ColorUtils::sColorWheel = Eigen::Matrix3Xi(3, NCOLS);
bool ColorUtils::sFirst = true;

float ColorUtils::hue2rgb(float p, float q, float t)
{
    if (t < 0) ++t;
    if (t > 1) --t;
    if (t < 1.0f / 6.0f) return p + (q - p) * 6 * t;
    if (t < 0.5f) return q;
    if (t < 2.0f / 3.0f) return p + (q - p) * (2.0f / 3.0f - t) * 6;
    return p;
}

Eigen::Vector3f ColorUtils::hsl2rgb(float h, float s, float l)
{
    if (s == 0)
        return Eigen::Vector3f(l, l, l);
    float q = l < 0.5f ? l * (1.0f + s) : l + s - l * s;
    float p = 2.0f * l - q;
    float r = hue2rgb(p, q, h + 1.0f / 3.0f);
    float g = hue2rgb(p, q, h);
    float b = hue2rgb(p, q, h - 1.0f / 3.0f);
    return Eigen::Vector3f(r, g, b);
}

Eigen::Vector3f ColorUtils::heatMap(float val, float min, float max)
{
    float range = std::log2(1.0f + max - min);
    float normalizedVal;
    if (range < std::numeric_limits<float>::epsilon())
    {
        normalizedVal = 0;
    }
    else
    {
        normalizedVal = std::log2(1.0f + val - min) / range;
    }
    float h = (1.0f - normalizedVal) * 240.0f / 360.0f;
    return hsl2rgb(h, 1.0f, 0.5f);
}

void ColorUtils::calculateColorWheel()
{
    int k = 0;
    for (int i = 0; i < RY; ++i, ++k)
        sColorWheel.col(k) = Eigen::Vector3i(255, 255 * i / RY, 0);
    for (int i = 0; i < YG; ++i, ++k)
        sColorWheel.col(k) = Eigen::Vector3i(255 - 255 * i / YG, 255, 0);
    for (int i = 0; i < GC; ++i, ++k)
        sColorWheel.col(k) = Eigen::Vector3i(0, 255, 255 * i / GC);
    for (int i = 0; i < CB; ++i, ++k)
        sColorWheel.col(k) = Eigen::Vector3i(0, 255 - 255 * i / CB, 255);
    for (int i = 0; i < BM; ++i, ++k)
        sColorWheel.col(k) = Eigen::Vector3i(255 * i / BM, 0, 255);
    for (int i = 0; i < MR; ++i, ++k)
        sColorWheel.col(k) = Eigen::Vector3i(255, 0, 255 - 255 * i / MR);
}

Eigen::Vector3f ColorUtils::colorWheel(float degrees)
{
    if (sFirst)
    {
        calculateColorWheel();
        sFirst = false;
    }

    float theta = AngleUtils::deg2rad(degrees);
    Eigen::Vector2f d(std::cos(theta), std::sin(theta));

    float rad = d.norm();
    float a = std::atan2(-d.y(), -d.x()) / static_cast<float>(M_PI);

    float fk = (a + 1) / 2 * (NCOLS - 1);
    int k0 = (int)fk;
    int k1 = (k0 + 1) % NCOLS;
    float f = fk - k0;

    Eigen::Vector3f pix;
    for (unsigned int b = 0; b < 3; b++)
    {
        float col0 = sColorWheel(b, k0) / 255.0f;
        float col1 = sColorWheel(b, k1) / 255.0f;
        float col = (1 - f) * col0 + f * col1;
        if (rad <= 1)
            col = 1 - rad * (1 - col); // increase saturation with radius
        else
            col *= 0.75f; // out of range
        pix(2 - b) = col;
    }

    return pix;
}
