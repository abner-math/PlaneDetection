#ifndef STATISTICSUTILS_H
#define STATISTICSUTILS_H

#include <vector>
#include <algorithm>
#include <iostream>
#include <Eigen/Core>
#include "angleutils.h"

class StatisticsUtils
{
public:
    StatisticsUtils(size_t bufferSize)
    {
        mDataBuffer.reserve(bufferSize);
        mTempBuffer.reserve(bufferSize);
    }

    std::vector<float>& dataBuffer()
    {
        return mDataBuffer;
    }

    size_t size() const
    {
        return mSize;
    }

    void size(size_t size)
    {
        mSize = size;
        if (mDataBuffer.size() < size)
        {
            mDataBuffer.resize(size);
            mTempBuffer.resize(size);
        }
    }

    float getMedian()
    {
        std::memcpy(mTempBuffer.data(), mDataBuffer.data(), sizeof(float) * mSize);
        std::nth_element(mTempBuffer.begin(), mTempBuffer.begin() + mSize / 2, mTempBuffer.begin() + mSize);
        return mTempBuffer[mSize / 2];
    }

    float getMAD(float median)
    {
        for (size_t i = 0; i < mSize; i++)
        {
            mTempBuffer[i] = std::abs(mDataBuffer[i] - median);
        }
        std::nth_element(mTempBuffer.begin(), mTempBuffer.begin() + mSize / 2, mTempBuffer.begin() + mSize);
        return 1.4826f * mTempBuffer[mSize / 2];
    }

    inline float getRScore(float value, float median, float mad)
    {
        return std::abs(value - median) / mad;
    }

    void getMinMaxRScore(float &min, float &max, float range)
    {
        float median = getMedian();
        float mad = getMAD(median);
        min = median - range * mad;
        max = median + range * mad;
    }

    float getMean()
    {
        float sum = 0;
        for (size_t i = 0; i < mSize; i++)
        {
            sum += mDataBuffer[i];
        }
        return sum / mSize;
    }

    float getSTD(float mean)
    {
        float sum = 0;
        for (size_t i = 0; i < mSize; i++)
        {
            sum += std::pow(mDataBuffer[i] - mean, 2);
        }
        return std::sqrt(sum / (mSize - 1));
    }

    void getMinMaxZScore(float &min, float &max, float range)
    {
        float mean = getMean();
        float std = getSTD(mean);
        min = mean - range * std;
        max = mean + range * std;
    }

private:
    std::vector<float> mDataBuffer;
    std::vector<float> mTempBuffer;
    size_t mSize;

};

#endif // STATISTICSUTILS_H
