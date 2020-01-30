#ifndef COLORUTILS_H
#define COLORUTILS_H

#include <Eigen/Core>

class ColorUtils
{
public:
    static Eigen::Vector3f hsl2rgb(float h, float s, float l);

    static Eigen::Vector3f heatMap(float val, float min, float max);

    static Eigen::Vector3f colorWheel(float degrees);

private:
    static const int RY = 15;
    static const int YG = 6;
    static const int GC = 4;
    static const int CB = 11;
    static const int BM = 13;
    static const int MR = 6;
    static const int NCOLS = RY + YG + GC + CB + BM + MR;
    static Eigen::Matrix3Xi sColorWheel;
    static bool sFirst;

    static void calculateColorWheel();

    static float hue2rgb(float p, float q, float t);

};

#endif // COLORUTILS_H
