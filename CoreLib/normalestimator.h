#ifndef NORMALESTIMATOR_H
#define NORMALESTIMATOR_H

#include "nearestneighborcalculator.h"
#include "pcacalculator.h"

#include <iostream>

// reference article: Outlier detection and robust normal-curvature estimation in mobile laser scanning 3D point cloud data
// reference article[2]: Fast and Robust Normal Estimation for Point Clouds with Sharp Features
// reference article[3]: Robust statistical approaches for local planar surface fitting in 3D laser scanning data
template <size_t DIMENSION>
class NormalEstimator
{
public:
    typedef Eigen::Matrix<float, DIMENSION, DIMENSION> Matrix;
    typedef typename Point<DIMENSION>::Vector Vector;

    enum Speed
    {
        QUICK = 0,
        SLOW = 1
    };

    struct Normal
    {
        size_t point;
        Vector normal;
        float curvature;
        float confidence;
        std::vector<size_t> neighbors;
    };

    NormalEstimator(const Partitioner<DIMENSION> *partitioner, size_t numNeighbors,
                    Speed speed = QUICK, float cutoffDistance = 3.075f)
        : mPartitioner(partitioner)
        , mNumNeighbors(numNeighbors)
        , mCutoffDistance(cutoffDistance)
        , mSpeed(speed)
    {

    }

    const Partitioner<DIMENSION>* partitioner() const
    {
        return mPartitioner;
    }

    void partitioner(const Partitioner<DIMENSION> *partitioner)
    {
        mPartitioner = partitioner;
    }

    size_t numNeighbors() const
    {
        return mNumNeighbors;
    }

    void numNeighbors(size_t numNeighbors)
    {
        mNumNeighbors = numNeighbors;
    }

    float cutoffDistance() const
    {
        return mCutoffDistance;
    }

    void cutoffDistance(float cutoffDistance)
    {
        mCutoffDistance = cutoffDistance;
    }

    Speed speed() const
    {
        return mSpeed;
    }

    void speed(Speed speed)
    {
        mSpeed = speed;
    }

    void getOutlierFreePoints(const std::vector<size_t> &points, std::vector<size_t> &inliers)
    {
        DataScatter robustScatter;
        calculateRobustMeanAndCovarianceMatrix(points, robustScatter);

        for (const size_t &point : points)
        {
            float robustDistance = calculateMahalanobisDist(point, robustScatter);
            if (robustDistance < mCutoffDistance)
            {
                inliers.push_back(point);
            }
        }
    }

    Eigen::Matrix<float, DIMENSION, 1> getNormalVector(const std::vector<size_t> &points)
    {
        Eigen::Matrix<float, DIMENSION, -1> matrix(DIMENSION, points.size());
        for (size_t i = 0; i < points.size(); i++)
        {
            matrix.col(i) = mPartitioner->pointCloud()->at(points[i]).position();
        }
        Eigen::Matrix<float, DIMENSION, DIMENSION> eigenVectors;
        Eigen::Matrix<float, DIMENSION, 1> eigenValues;
        PCACalculator<DIMENSION>::calculate(matrix, eigenVectors, eigenValues);
        return eigenVectors.col(DIMENSION - 1).normalized();
    }

    // Robust statistical approaches for local planar surface fitting in 3D laser scanning data.
    // link: https://ac.els-cdn.com/S0924271614001762/1-s2.0-S0924271614001762-main.pdf?_tid=d3d92628-d51d-11e7-90c3-00000aacb360&acdnat=1511971163_da7a1046fe0266eb7fe14148ea42b8ee
    Normal estimate(size_t point)
    {
        Normal normal;
        normal.point = point;
        normal.normal = Vector::Zero();
        normal.curvature = 0;
        normal.confidence = 0;
        if (mPartitioner->pointCloud()->hasConnectivity() && mPartitioner->pointCloud()->connectivity()->neighbors(point).size() >= mNumNeighbors)
        {
            const std::vector<size_t> &neighbors = mPartitioner->pointCloud()->connectivity()->neighbors(point);
            normal.neighbors = std::vector<size_t>(neighbors.begin(), neighbors.begin() + mNumNeighbors);
        }
        else
        {
            for (const std::pair<size_t, float> &p : NearestNeighborCalculator<DIMENSION>::kNN(mPartitioner, point, mNumNeighbors))
            {
                normal.neighbors.push_back(p.first);
            }
        }
        switch (mSpeed)
        {
        case QUICK:
            quickEstimation(normal);
            break;
        case SLOW:
            slowEstimation(normal);
            break;
        }
        return normal;
    }

private:
    const Partitioner<DIMENSION> *mPartitioner;
    size_t mNumNeighbors;
    float mCutoffDistance;
    Speed mSpeed;

    struct DataScatter
    {
        Vector mean;
        Matrix cov;
        Matrix invCov;
    };

    void selectRandomSubset(const std::vector<size_t> &points, std::vector<size_t> &subset)
    {
        std::vector<size_t> indices(points.size());
        std::iota(indices.begin(), indices.end(), 0);
        std::random_shuffle(indices.begin(), indices.end());
        for (size_t i = 0; i < subset.size(); i++)
        {
            subset[i] = points[indices[i]];
        }
    }

    DataScatter calculateDataScatter(const std::vector<size_t> &points)
    {
        DataScatter scatter;
        Eigen::Matrix<float, DIMENSION, -1> matrix(DIMENSION, points.size());
        for (size_t i = 0; i < points.size(); i++)
        {
            matrix.col(i) = mPartitioner->pointCloud()->at(points[i]).position();
        }
        scatter.mean = matrix.rowwise().mean();
        Eigen::Matrix<float, DIMENSION, -1> matrixCentered = matrix.colwise() - scatter.mean;
        scatter.cov = (matrixCentered * matrixCentered.transpose()) / (points.size() - 1);
        scatter.invCov = scatter.cov.inverse();
        return scatter;
    }

    inline float calculateMahalanobisDist(size_t point, const DataScatter &scatter)
    {
        Vector positionCentered = mPartitioner->pointCloud()->at(point).position() - scatter.mean;
        return std::sqrt(positionCentered.transpose() * scatter.invCov * positionCentered);
    }

    void calculateSortedMahalanobisDistForAll(const std::vector<size_t> &points, const DataScatter &scatter,
                                        std::vector<std::pair<size_t, float> > &dists)
    {
        for (const size_t &point : points)
        {
            float dist = calculateMahalanobisDist(point, scatter);
            dists.push_back(std::make_pair(point, dist));
        }
        std::sort(dists.begin(), dists.end(), [](const std::pair<size_t, float> &a, const std::pair<size_t, float> &b) {
           return a.second < b.second;
        });
    }

    void cStep(const std::vector<size_t> &points, std::vector<size_t> &hSubset, DataScatter &scatter)
    {
        std::vector<std::pair<size_t, float> > dists;
        calculateSortedMahalanobisDistForAll(points, scatter, dists);
        for (size_t i = 0; i < hSubset.size(); i++)
        {
            hSubset[i] = dists[i].first;
        }
        scatter = calculateDataScatter(hSubset);
    }

    void calculateRobustMeanAndCovarianceMatrix(const std::vector<size_t> &points, DataScatter &robustScatter)
    {
        std::vector<std::pair<std::vector<size_t>, float> > hSubsets;
        for (size_t i = 0; i < 100; i++)
        {
            std::vector<size_t> hSubset(points.size() / 2);
            selectRandomSubset(points, hSubset);
            DataScatter scatter = calculateDataScatter(hSubset);
            cStep(points, hSubset, scatter);
            cStep(points, hSubset, scatter);
            float det = scatter.cov.determinant();
            hSubsets.push_back(std::make_pair(hSubset, det));
        }
        std::sort(hSubsets.begin(), hSubsets.end(), [](const std::pair<std::vector<size_t>, float> &a, const std::pair<std::vector<size_t>, float> &b) {
            return a.second < b.second;
        });
        float minDet = std::numeric_limits<float>::max();
        std::vector<size_t> cleanSubset;
        for (size_t i = 0; i < 10; i++)
        {
            DataScatter scatter = calculateDataScatter(hSubsets[i].first);
            float oldDet, newDet;
            newDet = std::numeric_limits<float>::max();
            do
            {
                oldDet = newDet;
                cStep(points, hSubsets[i].first, scatter);
                newDet = scatter.cov.determinant();
            } while(newDet > std::numeric_limits<float>::epsilon() && (oldDet - newDet) > std::numeric_limits<float>::epsilon());
            if (newDet < minDet)
            {
                minDet = newDet;
                cleanSubset = hSubsets[i].first;
            }
        }
        robustScatter = calculateDataScatter(cleanSubset);
    }

    void slowEstimation(Normal &normal)
    {
        std::vector<size_t> inliers;
        std::vector<size_t> points;
        for (size_t i = 0; i < normal.neighbors.size(); i++)
        {
            points.push_back(normal.neighbors[i]);
        }
        getOutlierFreePoints(points, inliers);

        if (inliers.size() < DIMENSION)
        {
            std::cerr << "WARNING: Could not find estimate normal for point " << normal.point << "..." << std::endl;
            return;
        }

        // fit outlier-free plane
        Eigen::Matrix<float, DIMENSION, -1> matrix(DIMENSION, inliers.size());
        for (size_t i = 0; i < inliers.size(); i++)
        {
            matrix.col(i) = mPartitioner->pointCloud()->at(inliers[i]).position();
        }
        Eigen::Matrix<float, DIMENSION, DIMENSION> eigenVectors;
        Eigen::Matrix<float, DIMENSION, 1> eigenValues;
        PCACalculator<DIMENSION>::calculate(matrix, eigenVectors, eigenValues);

        normal.normal = eigenVectors.col(DIMENSION - 1).normalized();
        normal.curvature = eigenValues(DIMENSION - 1) / eigenValues.array().sum();
        normal.confidence = eigenValues(0) / (eigenValues(DIMENSION - 1) + 1e-4);
    }

    void quickEstimation(Normal &normal)
    {
        if (normal.neighbors.size() < 3)
        {
            std::cerr << "WARNING: Could not find estimate normal for point " << normal.point << "..." << std::endl;
            return;
        }

        Eigen::Matrix<float, DIMENSION, -1> matrix(DIMENSION, normal.neighbors.size());
        for (size_t i = 0; i < normal.neighbors.size(); i++)
        {
            matrix.col(i) = mPartitioner->pointCloud()->at(normal.neighbors[i]).position();
        }
        Eigen::Matrix<float, DIMENSION, DIMENSION> eigenVectors;
        Eigen::Matrix<float, DIMENSION, 1> eigenValues;
        PCACalculator<DIMENSION>::calculate(matrix, eigenVectors, eigenValues);

        normal.normal = eigenVectors.col(DIMENSION - 1).normalized();
        normal.curvature = eigenValues(DIMENSION - 1) / eigenValues.array().sum();
        normal.confidence = eigenValues(0) / (eigenValues(DIMENSION - 1) + 1e-4);
    }

};

template class NormalEstimator<3>;

typedef NormalEstimator<3> NormalEstimator3d;

#endif // NORMALESTIMATOR_H
