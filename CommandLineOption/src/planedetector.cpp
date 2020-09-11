#include "planedetector.h"

#include "pcacalculator.h"
#include "unionfind.h"
#include "angleutils.h"

#include <iostream>
#include <unordered_map>

PlaneDetector::PlaneDetector(const PointCloud3d *pointCloud)
    : PrimitiveDetector<3, Plane>(pointCloud)
    , mMinNormalDiff(std::cos(AngleUtils::deg2rad(60.0f)))
    , mMaxDist(std::cos(AngleUtils::deg2rad(75.0f)))
    , mOutlierRatio(0.75f)
{

}

std::set<Plane*> PlaneDetector::detect()
{
    std::set<Plane*> planes;

    clearRemovedPoints();

    size_t minNumPoints = std::max(size_t(10), size_t(pointCloud()->size() * 0.001f));
    StatisticsUtils statistics(pointCloud()->size());
    Octree octree(pointCloud());
    std::vector<PlanarPatch*> patches;
    detectPlanarPatches(&octree, &statistics, minNumPoints, patches);

    mPatchPoints = std::vector<PlanarPatch*>(pointCloud()->size(), NULL);
    for (PlanarPatch *patch : patches)
    {
        for (const size_t &point : patch->points())
        {
            mPatchPoints[point] = patch;
        }
    }

    bool changed;
    do
    {
        growPatches(patches);
        mergePatches(patches);
        changed = updatePatches(patches);
    } while (changed);

    growPatches(patches, true);
    std::vector<PlanarPatch*> truePositivePatches;
    for (PlanarPatch *patch : patches)
    {
        delimitPlane(patch);
        if (isFalsePositive(patch))
        {
            delete patch;
        }
        else
        {
            truePositivePatches.push_back(patch);
        }
    }
    patches = truePositivePatches;

    for (PlanarPatch *patch : patches)
    {
        Plane *plane = new Plane(patch->plane());
        plane->inliers(patch->points());
        planes.insert(plane);
        delete patch;
    }

    return planes;
}

bool PlaneDetector::detectPlanarPatches(Octree *node, StatisticsUtils *statistics, size_t minNumPoints, std::vector<PlanarPatch*> &patches)
{
    if (node->numPoints() < minNumPoints) return false;
    node->partition(1, minNumPoints);
    bool hasPlanarPatch = false;
    for (size_t i = 0; i < 8; i++)
    {
        if (node->child(i) != NULL && detectPlanarPatches(node->child(i), statistics, minNumPoints, patches))
        {
            hasPlanarPatch = true;
        }
    }
    if (!hasPlanarPatch && node->octreeLevel() > 2)
    {
        PlanarPatch *patch = new PlanarPatch(pointCloud(), statistics, node->points(), mMinNormalDiff, mMaxDist, mOutlierRatio);
        if (patch->isPlanar())
        {
            patches.push_back(patch);
            hasPlanarPatch = true;
        }
        else
        {
            delete patch;
        }
    }
    return hasPlanarPatch;
}

void PlaneDetector::growPatches(std::vector<PlanarPatch*> &patches, bool relaxed)
{
    std::sort(patches.begin(), patches.end(), [](const PlanarPatch *a, const PlanarPatch *b) {
       return a->minNormalDiff() > b->minNormalDiff();
    });
    std::queue<size_t> queue;
    for (PlanarPatch *patch : patches)
    {
        if (patch->stable()) continue;
        for (const size_t &point : patch->points())
        {
            queue.push(point);
        }
        while (!queue.empty())
        {
            size_t point = queue.front();
            queue.pop();
            std::pair<std::vector<size_t>::const_iterator, std::vector<size_t>::const_iterator> neighborsIterators = pointCloud()->connectivity()->neighborsIterator(point);
            for (std::vector<size_t>::const_iterator neighborsIterator = neighborsIterators.first; neighborsIterator != neighborsIterators.second; ++neighborsIterator)
            {
                size_t neighbor = *neighborsIterator;
                if (isRemoved(neighbor) || mPatchPoints[neighbor] != NULL || (!relaxed && patch->isVisited(neighbor))) continue;
                if ((!relaxed && patch->isInlier(neighbor)) || (relaxed && std::abs(patch->plane().getSignedDistanceFromSurface(pointCloud()->at(neighbor).position())) < patch->maxDistPlane()))
                {
                    queue.push(neighbor);
                    patch->addPoint(neighbor);
                    mPatchPoints[neighbor] = patch;
                }
                else
                {
                    patch->visit(neighbor);
                }
            }
        }
    }
}


void PlaneDetector::mergePatches(std::vector<PlanarPatch*> &patches)
{
    size_t n = patches.size();
    for (size_t i = 0; i < n; i++)
    {
        patches[i]->index(i);
    }
    std::vector<bool> graph(n * n, false);
    std::vector<bool> disconnectedPatches(n * n, false);
    for (size_t i = 0; i < patches.size(); i++)
    {
        for (size_t j = i + 1; j < patches.size(); j++)
        {
            float normalThreshold = std::min(patches[i]->minNormalDiff(), patches[j]->minNormalDiff());
            disconnectedPatches[i * n + j] = std::abs(patches[i]->plane().normal().dot(patches[j]->plane().normal())) < normalThreshold;
            disconnectedPatches[j * n + i] = disconnectedPatches[i * n + j];
        }
    }
    for (PlanarPatch *p : patches)
    {
        for (const size_t &point : p->points())
        {
            std::pair<std::vector<size_t>::const_iterator, std::vector<size_t>::const_iterator> neighborsIterators = pointCloud()->connectivity()->neighborsIterator(point);
            for (std::vector<size_t>::const_iterator neighborsIterator = neighborsIterators.first; neighborsIterator != neighborsIterators.second; ++neighborsIterator)
            {
                size_t neighbor = *neighborsIterator;
                PlanarPatch *np = mPatchPoints[neighbor];
                if (p == np || np == NULL || graph[np->index() * n + p->index()] || graph[p->index() * n + np->index()] ||
                    disconnectedPatches[p->index() * n + np->index()] || p->isVisited(neighbor) || np->isVisited(point)) continue;
                p->visit(neighbor);
                np->visit(point);
                const Point3d &p1 = pointCloud()->at(point);
                const Point3d &p2 = pointCloud()->at(neighbor);
                float distThreshold = std::max(p->maxDistPlane(), np->maxDistPlane());
                float normalThreshold = std::min(p->minNormalDiff(), np->minNormalDiff());
                graph[p->index() * n + np->index()] = std::abs(p->plane().normal().dot(p2.normal())) > normalThreshold &&
                        std::abs(np->plane().normal().dot(p1.normal())) > normalThreshold &&
                        std::abs(p->plane().getSignedDistanceFromSurface(p2.position())) < distThreshold &&
                        std::abs(np->plane().getSignedDistanceFromSurface(p1.position())) < distThreshold;
            }
        }
    }
    UnionFind uf(patches.size());
    for (size_t i = 0; i < patches.size(); i++)
    {
        for (size_t j = i + 1; j < patches.size(); j++)
        {
            if (graph[i * n + j] || graph[j * n + i])
            {
                uf.join(i, j);
            }
        }
    }
    std::vector<size_t> largestPatch(patches.size());
    std::iota(largestPatch.begin(), largestPatch.end(), 0);
    for (size_t i = 0; i < patches.size(); i++)
    {
        int root = uf.root(i);
        if (patches[largestPatch[root]]->points().size() < patches[i]->points().size())
        {
            largestPatch[root] = i;
        }
    }
    for (size_t i = 0; i < patches.size(); i++)
    {
        size_t root = largestPatch[uf.root(i)];
        if (root != i)
        {
            for (const size_t &point : patches[i]->points())
            {
                patches[root]->addPoint(point);
                mPatchPoints[point] = patches[root];
            }
            patches[root]->maxDistPlane(std::max(patches[root]->maxDistPlane(), patches[i]->maxDistPlane()));
            patches[root]->minNormalDiff(std::min(patches[root]->minNormalDiff(), patches[i]->minNormalDiff()));
            delete patches[i];
            patches[i] = NULL;
        }
    }
    patches.erase(std::remove_if(patches.begin(), patches.end(), [](PlanarPatch *patch) {
        return patch == NULL;
    }), patches.end());
}

bool PlaneDetector::updatePatches(std::vector<PlanarPatch*> &patches)
{
    bool changed = false;
    for (PlanarPatch *patch : patches)
    {
        if (patch->numNewPoints() > (patch->points().size() - patch->numNewPoints()) / 2)
        {
            patch->updatePlane();
            patch->stable() = false;
            changed = true;
        }
        else
        {
            patch->stable() = true;
        }
    }
    return changed;
}

void PlaneDetector::getPlaneOutlier(const PlanarPatch *patch, std::vector<size_t> &outlier)
{
    Eigen::Vector3f basisU, basisV;
    GeometryUtils::orthogonalBasis(patch->plane().normal(), basisU, basisV);
    std::vector<Eigen::Vector2f> projectedPoints(patch->points().size());
    for (size_t i = 0; i < patch->points().size(); i++)
    {
        Eigen::Vector3f position = pointCloud()->at(patch->points()[i]).position();
        projectedPoints[i] = GeometryUtils::projectOntoOrthogonalBasis(position, basisU, basisV);
    }
    GeometryUtils::convexHull(projectedPoints, outlier);
    for (size_t i = 0; i < outlier.size(); i++)
    {
        outlier[i] = patch->points()[outlier[i]];
    }
}

void PlaneDetector::delimitPlane(PlanarPatch *patch)
{
    std::vector<size_t> outlier;
    getPlaneOutlier(patch, outlier);
    Eigen::Vector3f normal = patch->plane().normal();
    Eigen::Vector3f basisU, basisV;
    GeometryUtils::orthogonalBasis(normal, basisU, basisV);
    Eigen::Matrix3f basis;
    basis << basisU.transpose(), basisV.transpose(), normal.transpose();
    Eigen::Matrix3Xf matrix(3, outlier.size());
    for (size_t i = 0; i < outlier.size(); i++)
    {
        matrix.col(i) = pointCloud()->at(outlier[i]).position();
    }
    float minAngle = 0;
    float maxAngle = 90;
    while (maxAngle - minAngle > 5)
    {
        float mid = (maxAngle + minAngle) / 2;
        float left = (minAngle + mid) / 2;
        float right = (maxAngle + mid) / 2;
        RotatedRect leftRect(matrix, basis, left);
        RotatedRect rightRect(matrix, basis, right);
        if (leftRect.area < rightRect.area)
        {
            maxAngle = mid;
        }
        else
        {
            minAngle = mid;
        }
    }
    patch->rect() = RotatedRect(matrix, basis, (minAngle + maxAngle) / 2);
    Eigen::Vector3f center = patch->plane().center();
    Eigen::Vector3f minBasisU = patch->rect().basis.row(0);
    Eigen::Vector3f minBasisV = patch->rect().basis.row(1);
    center -= minBasisU * minBasisU.dot(center);
    center -= minBasisV * minBasisV.dot(center);
    center += minBasisU * (patch->rect().rect.bottomLeft()(0) + patch->rect().rect.topRight()(0)) / 2;
    center += minBasisV * (patch->rect().rect.bottomLeft()(1) + patch->rect().rect.topRight()(1)) / 2;
    float lengthU = (patch->rect().rect.topRight()(0) - patch->rect().rect.bottomLeft()(0)) / 2;
    float lengthV = (patch->rect().rect.topRight()(1) - patch->rect().rect.bottomLeft()(1)) / 2;
    Plane newPlane(center, patch->plane().normal(), minBasisU * lengthU, minBasisV * lengthV);
    patch->plane(newPlane);
}

void PlaneDetector::delimitPlane(Plane *plane)
{
    StatisticsUtils statistics(pointCloud()->size());
    PlanarPatch patch(pointCloud(), &statistics, plane->inliers(), mMinNormalDiff, mMaxDist, mOutlierRatio);
    patch.updatePlane();
    patch.normal(plane->normal());
    delimitPlane(&patch);
    plane->basisU(patch.plane().basisU());
    plane->basisV(patch.plane().basisV());
    plane->center(patch.plane().center());
}

bool PlaneDetector::isFalsePositive(PlanarPatch *patch)
{
    return patch->numUpdates() == 0 ||
            patch->getSize() / float(pointCloud()->extension().maxSize()) < 0.01f;
}

Plane* PlaneDetector::detectPlane(const std::vector<size_t> &points)
{
    StatisticsUtils statistics(pointCloud()->size());
    PlanarPatch placeholder(pointCloud(), &statistics, points, mMinNormalDiff, mMaxDist, mOutlierRatio);
    mPatchPoints = std::vector<PlanarPatch*>(pointCloud()->size(), NULL);
    for (size_t i = 0; i < pointCloud()->geometry()->numPlanes(); i++)
    {
        const Plane *plane = pointCloud()->geometry()->plane(i);
        for (const size_t &point : plane->inliers())
        {
            mPatchPoints[point] = &placeholder;
        }
    }
    std::vector<size_t> newPoints;
    for (const size_t &point : points)
    {
        if (mPatchPoints[point] == NULL)
        {
            newPoints.push_back(point);
        }
    }
    PlanarPatch patch(pointCloud(), &statistics, newPoints, mMinNormalDiff, mMaxDist, mOutlierRatio);
    patch.updatePlane();
    delimitPlane(&patch);
    Plane *plane = new Plane(patch.plane());
    plane->inliers(newPoints);
    return plane;
}

void PlaneDetector::growRegion(std::vector<size_t> &points)
{
    StatisticsUtils statistics(pointCloud()->size());
    PlanarPatch placeholder(pointCloud(), &statistics, points, mMinNormalDiff, mMaxDist, mOutlierRatio);
    mPatchPoints = std::vector<PlanarPatch*>(pointCloud()->size(), NULL);
    for (size_t i = 0; i < pointCloud()->geometry()->numPlanes(); i++)
    {
        const Plane *plane = pointCloud()->geometry()->plane(i);
        for (const size_t &point : plane->inliers())
        {
            mPatchPoints[point] = &placeholder;
        }
    }
    std::vector<size_t> newPoints;
    for (const size_t &point : points)
    {
        if (mPatchPoints[point] == NULL)
        {
            newPoints.push_back(point);
        }
    }
    PlanarPatch *patch = new PlanarPatch(pointCloud(), &statistics, newPoints, mMinNormalDiff, mMaxDist, mOutlierRatio);
    patch->update();
    std::vector<PlanarPatch*> patches = { patch };
    growPatches(patches);
    points = patch->points();
    delete patch;
}
