#ifndef POINTCLOUDLOADER_H
#define POINTCLOUDLOADER_H

#include <stdio.h>
#include <locale.h>

#include <fstream>
#include <sstream>
#include <iostream>

#include "pointcloud.h"

class PointCloudIO 
{
public:
    void saveGeometry(const Geometry *geometry, const std::string &filename)
    {
        FILE *fp = fopen(filename.c_str(), "wb");
        if (fp == NULL)
            throw "Could not open file: " + filename;

        size_t numCircles = geometry->numCircles();
        fwrite(&numCircles, sizeof(size_t), 1, fp);
        for (size_t i = 0; i < numCircles; i++)
        {
            const Circle *circle = geometry->circle(i);
            Eigen::Vector3f color = circle->color();
            Eigen::Vector2f center = circle->center();
            float radius = circle->radius();
            fwrite(color.data(), sizeof(float), 3, fp);
            fwrite(center.data(), sizeof(float), 2, fp);
            fwrite(&radius, sizeof(float), 1, fp);
            const std::vector<size_t> &inliers = circle->inliers();
            size_t numInliers = inliers.size();
            fwrite(&numInliers, sizeof(size_t), 1, fp);
            fwrite(inliers.data(), sizeof(size_t), numInliers, fp);
        }

        size_t numPlanes = geometry->numPlanes();
        fwrite(&numPlanes, sizeof(size_t), 1, fp);
        for (size_t i = 0; i < numPlanes; i++)
        {
            const Plane *plane = geometry->plane(i);
            Eigen::Vector3f color = plane->color();
            Eigen::Vector3f center = plane->center();
            Eigen::Vector3f normal = plane->normal();
            Eigen::Vector3f basisU = plane->basisU();
            Eigen::Vector3f basisV = plane->basisV();
            fwrite(color.data(), sizeof(float), 3, fp);
            fwrite(center.data(), sizeof(float), 3, fp);
            fwrite(normal.data(), sizeof(float), 3, fp);
            fwrite(basisU.data(), sizeof(float), 3, fp);
            fwrite(basisV.data(), sizeof(float), 3, fp);
            const std::vector<size_t> &inliers = plane->inliers();
            size_t numInliers = inliers.size();
            fwrite(&numInliers, sizeof(size_t), 1, fp);
            fwrite(inliers.data(), sizeof(size_t), numInliers, fp);
        }

        size_t numCylinders = geometry->numCylinders();
        fwrite(&numCylinders, sizeof(size_t), 1, fp);
        for (size_t i = 0; i < numCylinders; i++)
        {
            const Cylinder *cylinder = geometry->cylinder(i);
            Eigen::Vector3f color = cylinder->color();
            Eigen::Vector3f center = cylinder->center();
            Eigen::Vector3f axis = cylinder->axis();
            float radius = cylinder->radius();
            float height = cylinder->height();
            fwrite(color.data(), sizeof(float), 3, fp);
            fwrite(center.data(), sizeof(float), 3, fp);
            fwrite(axis.data(), sizeof(float), 3, fp);
            fwrite(&radius, sizeof(float), 1, fp);
            fwrite(&height, sizeof(float), 1, fp);
            const std::vector<size_t> &inliers = cylinder->inliers();
            size_t numInliers = inliers.size();
            fwrite(&numInliers, sizeof(size_t), 1, fp);
            fwrite(inliers.data(), sizeof(size_t), numInliers, fp);
        }
        fclose(fp);
    }

    void saveConnectivity(const ConnectivityGraph *connectivity, const std::string &filename)
    {
        FILE *fp = fopen(filename.c_str(), "wb");
        if (fp == NULL)
            throw "Could not open file: " + filename;

        size_t size = connectivity->numPoints();
        for (size_t i = 0; i < size; i++)
        {
            size_t numNeighbors = connectivity->neighbors(i).size();
            fwrite(&numNeighbors, sizeof(size_t), 1, fp);
            std::vector<size_t> edges;
            std::vector<float> distances;
            for (const size_t &p : connectivity->neighbors(i))
            {
                edges.push_back(p);
                distances.push_back(0);
            }
            fwrite(edges.data(), sizeof(size_t), numNeighbors, fp);
            fwrite(distances.data(), sizeof(float), numNeighbors, fp);
        }
        fwrite(connectivity->groupIndices().data(), sizeof(size_t), size, fp);

        fclose(fp);
    }

    template <size_t DIMENSION>
    void saveAsPCL(const PointCloud<DIMENSION> *pointCloud, const std::string &filename)
    {
        FILE *fp = fopen(filename.c_str(), "wb");
        if (fp == NULL)
            throw "Could not open file: " + filename;

        size_t size = pointCloud->size();
        fwrite(&size, sizeof(size_t), 1, fp);
        size_t mode = static_cast<size_t>(pointCloud->mode());
        fwrite(&mode, sizeof(size_t), 1, fp);

        for (size_t i = 0; i < size; i++)
        {
            const Point<DIMENSION> &point = pointCloud->at(i);
            float intensity = point.intensity();
            float normalConfidence = point.normalConfidence();
            float curvature = point.curvature();
            fwrite(point.position().data(), sizeof(float), DIMENSION, fp);
            if (pointCloud->hasMode(PointCloud<DIMENSION>::Mode::COLOR))
                fwrite(point.color().data(), sizeof(float), DIMENSION, fp);
            if (pointCloud->hasMode(PointCloud<DIMENSION>::Mode::INTENSITY))
                fwrite(&intensity, sizeof(float), 1, fp);
            if (pointCloud->hasMode(PointCloud<DIMENSION>::Mode::NORMAL))
                fwrite(point.normal().data(), sizeof(float), DIMENSION, fp);
            if (pointCloud->hasMode(PointCloud<DIMENSION>::Mode::NORMAL_CONFIDENCE))
                fwrite(&normalConfidence, sizeof(float), 1, fp);
            if (pointCloud->hasMode(PointCloud<DIMENSION>::Mode::CURVATURE))
                fwrite(&curvature, sizeof(float), 1, fp);
        }

        fclose(fp);

        if (pointCloud->hasConnectivity())
        {
            std::string connectivityFilename = filename.substr(0, filename.find_last_of('.')) + ".con";
            saveConnectivity(pointCloud->connectivity(), connectivityFilename);
        }

        std::string geometryFilename = filename.substr(0, filename.find_last_of('.')) + ".geo";
        saveGeometry(pointCloud->geometry(), geometryFilename);
    }

    void saveAsPoints(const PointCloud3d *pointCloud, const std::string &filename)
    {
        FILE *fp = fopen(filename.c_str(), "wb");
        if (fp == NULL)
            throw "Could not open file: " + filename;

        int size = static_cast<int>(pointCloud->size());
        fwrite(&size, sizeof(int), 1, fp);

        for (int i = 0; i < size; i++)
        {
            fwrite(pointCloud->at(i).position().data(), sizeof(float), 3, fp);
        }

        for (int i = 0; i < size; i++)
        {
            fwrite(pointCloud->at(i).normal().data(), sizeof(float), 3, fp);
        }

        fclose(fp);
    }

    void saveAsXYZ(const PointCloud3d *pointCloud, const std::string &filename)
    {
        FILE *fp = fopen(filename.c_str(), "wb");
        if (fp == NULL)
            throw "Could not open file: " + filename;

        for (size_t i = 0; i < pointCloud->size(); i++)
        {
            Eigen::Vector3f position = pointCloud->at(i).position();
            fprintf(fp, "%f %f %f", position.x(), position.y(), position.z());
            if (pointCloud->hasMode(PointCloud3d::Mode::COLOR))
            {
                Eigen::Vector3f color = pointCloud->at(i).color();
                fprintf(fp, " %d %d %d", (int)(255 * color.x()), (int)(255 * color.y()), (int)(255 * color.z()));
            }
            fprintf(fp, "\n");
        }

        fclose(fp);
    }

    void saveAsPTX(const PointCloud3d *pointCloud, const std::string &filename)
    {
        FILE *fp = fopen(filename.c_str(), "wb");
        if (fp == NULL)
            throw "Could not open file: " + filename;

        for (size_t i = 0; i < pointCloud->size(); i++)
        {
            Eigen::Vector3f position = pointCloud->at(i).position();
            fprintf(fp, "%f %f %f", position.x(), -position.z(), position.y());
            if (pointCloud->hasMode(PointCloud3d::Mode::INTENSITY))
            {
                fprintf(fp, "%f", pointCloud->at(i).intensity());
            }
            else
            {
                fprintf(fp, "%f", 0.0f);
            }
            if (pointCloud->hasMode(PointCloud3d::Mode::COLOR))
            {
                Eigen::Vector3f color = pointCloud->at(i).color();
                fprintf(fp, " %d %d %d", (int)(255 * color.x()), (int)(255 * color.y()), (int)(255 * color.z()));
            }
            fprintf(fp, "\n");
        }

        fclose(fp);
    }

    void save(const PointCloud3d *pointCloud, const std::string &filename)
    {
        std::string extension = filename.substr(filename.find_last_of('.') + 1);
        if (extension == "pcl")
        {
            PointCloudIO::saveAsPCL(pointCloud, filename);
        }
        else if (extension == "points")
        {
            PointCloudIO::saveAsPoints(pointCloud, filename);
        }
        else if (extension == "xyz")
        {
            PointCloudIO::saveAsXYZ(pointCloud, filename);
        }
        else if (extension == "ptx")
        {
            PointCloudIO::saveAsPTX(pointCloud, filename);
        }
        else
        {
            throw "Extension not supported: " + extension;
        }
    }

    Geometry* loadGeometry(const std::string &filename)
    {
        FILE *fp = fopen(filename.c_str(), "rb");
        if (fp == NULL) return NULL;

        Geometry *geometry = new Geometry;

        size_t numCircles;
        fread(&numCircles, sizeof(size_t), 1, fp);
        for (size_t i = 0; i < numCircles; i++)
        {
            float color[3];
            float center[2];
            float radius;
            fread(color, sizeof(float), 3, fp);
            fread(center, sizeof(float), 2, fp);
            fread(&radius, sizeof(float), 1, fp);
            Circle *circle = new Circle(Eigen::Vector2f(center), radius);
            circle->color(Eigen::Vector3f(color));
            size_t numInliers;
            fread(&numInliers, sizeof(size_t), 1, fp);
            std::vector<size_t> inliers(numInliers);
            fread(inliers.data(), sizeof(size_t), numInliers, fp);
            circle->inliers(inliers);
            geometry->addCircle(circle);
        }

        size_t numPlanes;
        fread(&numPlanes, sizeof(size_t), 1, fp);
        std::cout << "Planes=" << numPlanes << std::endl;
        for (size_t i = 0; i < numPlanes; i++)
        {
            float color[3];
            float center[3];
            float normal[3];
            float basisU[3];
            float basisV[3];
            fread(color, sizeof(float), 3, fp);
            fread(center, sizeof(float), 3, fp);
            fread(normal, sizeof(float), 3, fp);
            fread(basisU, sizeof(float), 3, fp);
            fread(basisV, sizeof(float), 3, fp);
            Plane *plane = new Plane(Eigen::Vector3f(center), Eigen::Vector3f(normal),
                                     Eigen::Vector3f(basisU), Eigen::Vector3f(basisV));
            plane->color(Eigen::Vector3f(color));
            size_t numInliers;
            fread(&numInliers, sizeof(size_t), 1, fp);
            std::cout << Eigen::Vector3f(center).transpose() << " " << Eigen::Vector3f(normal).transpose() << " " << numInliers << std::endl;
            std::vector<size_t> inliers(numInliers);
            fread(inliers.data(), sizeof(size_t), numInliers, fp);
            plane->inliers(inliers);
            geometry->addPlane(plane);
        }

        size_t numCylinders;
        fread(&numCylinders, sizeof(size_t), 1, fp);
        std::cout << "Cylinders=" << numCylinders << std::endl;
        for (size_t i = 0; i < numCylinders; i++)
        {
            float color[3];
            float center[3];
            float axis[3];
            float radius;
            float height;
            fread(color, sizeof(float), 3, fp);
            fread(center, sizeof(float), 3, fp);
            fread(axis, sizeof(float), 3, fp);
            fread(&radius, sizeof(float), 1, fp);
            fread(&height, sizeof(float), 1, fp);
            std::cout << "[" << center[0] << " " << center[1] << " " << center[2] << "] [" << axis[0] << " " << axis[1] << " " << axis[2] << "]" << std::endl;
            Cylinder *cylinder = new Cylinder(Eigen::Vector3f(center), Eigen::Vector3f(axis),
                                              radius, height);
            cylinder->color(Eigen::Vector3f(color));
            size_t numInliers;
            fread(&numInliers, sizeof(size_t), 1, fp);
            std::vector<size_t> inliers(numInliers);
            fread(inliers.data(), sizeof(size_t), numInliers, fp);
            cylinder->inliers(inliers);
            geometry->addCylinder(cylinder);
        }
        fclose(fp);

        return geometry;
    }

    ConnectivityGraph* loadConnectivity(size_t size, const std::string &filename)
    {
        FILE *fp = fopen(filename.c_str(), "rb");
        if (fp == NULL) return NULL;

        ConnectivityGraph *connectivity = new ConnectivityGraph(size);

        for (size_t i = 0; i < size; i++)
        {
            size_t numEdges;
            fread(&numEdges, sizeof(size_t), 1, fp);
            size_t *edges = new size_t[numEdges];
            fread(edges, sizeof(size_t), numEdges, fp);
            float *distances = new float[numEdges];
            fread(distances, sizeof(float), numEdges, fp);
            std::vector<size_t> vEdges(numEdges);
            for (size_t i = 0; i < numEdges; i++)
            {
                vEdges[i] = edges[i];
            }
            connectivity->addNode(i, vEdges);
            delete[] edges;
            delete[] distances;
        }

        std::vector<size_t> groupIndices(size);
        fread(groupIndices.data(), sizeof(size_t), size, fp);
        connectivity->setGroupIndices(groupIndices);

        fclose(fp);

        return connectivity;
    }

    template <size_t DIMENSION>
    PointCloud<DIMENSION>* loadFromPCL(const std::string &filename, bool importConnectivity = true,
                                       bool importGeometry = true)
    {
        FILE *fp = fopen(filename.c_str(), "rb");
        if (fp == NULL)
            throw "Could not open file: " + filename;

        size_t size;
        fread(&size, sizeof(size_t), 1, fp);
        size_t mode;
        fread(&mode, sizeof(size_t), 1, fp);

        std::vector<Point<DIMENSION> > points(size);

        for (size_t i = 0; i < size; i++)
        {
            float position[DIMENSION];
            float color[DIMENSION];
            float intensity;
            float normal[DIMENSION];
            float normalConfidence;
            float curvature;
            fread(position, sizeof(float), DIMENSION, fp);
            if (mode & PointCloud<DIMENSION>::COLOR)
                fread(color, sizeof(float), DIMENSION, fp);
            if (mode & PointCloud<DIMENSION>::INTENSITY)
                fread(&intensity, sizeof(float), 1, fp);
            if (mode & PointCloud<DIMENSION>::NORMAL)
                fread(normal, sizeof(float), DIMENSION, fp);
            if (mode & PointCloud<DIMENSION>::NORMAL_CONFIDENCE)
                fread(&normalConfidence, sizeof(float), 1, fp);
            if (mode & PointCloud<DIMENSION>::CURVATURE)
                fread(&curvature, sizeof(float), 1, fp);

            typename Point<DIMENSION>::Vector positionv(position);
            typename Point<DIMENSION>::Vector colorv(color);
            typename Point<DIMENSION>::Vector normalv(normal);
            normalv = normalv.normalized();

            points[i] = Point<DIMENSION>(positionv, colorv, intensity, normalv, normalConfidence, curvature);
        }

        fclose(fp);

        PointCloud<DIMENSION> *pointCloud = new PointCloud<DIMENSION>(points, mode);

        if (importConnectivity)
        {
            std::string connectivityFilename = filename.substr(0, filename.find_last_of('.')) + ".con";
            pointCloud->connectivity(loadConnectivity(points.size(), connectivityFilename));
        }

        if (importGeometry)
        {
            std::string geometryFilename = filename.substr(0, filename.find_last_of('.')) + ".geo";
            Geometry *geometry = loadGeometry(geometryFilename);
            if (geometry != NULL) pointCloud->geometry(geometry);
        }

        return pointCloud;
    }

    PointCloud3d* loadFromPoints(const std::string &filename)
    {
        FILE *fp = fopen(filename.c_str(), "rb");
        if (fp == NULL)
            throw "Could not open file: " + filename;

        int size;
        fread(&size, sizeof(int), 1, fp);

        std::vector<Eigen::Vector3f> positions(size);
        std::vector<Eigen::Vector3f> normals(size);

        for (int i = 0; i < size; i++)
        {
            float position[3];
            fread(position, sizeof(float), 3, fp);
            positions[i] = Eigen::Vector3f(position);
        }

        for (int i = 0; i < size; i++)
        {
            float normal[3];
            fread(normal, sizeof(float), 3, fp);
            normals[i] = Eigen::Vector3f(normal).normalized();
        }

        std::vector<Point3d> points(size);
        for (int i = 0; i < size; i++)
        {
            points[i].position(positions[i]);
            points[i].normal(normals[i]);
        }

        fclose(fp);

        return new PointCloud3d(points, PointCloud3d::NORMAL);
    }

    PointCloud3d* loadFromXYZ(const std::string &filename)
    {
        std::ifstream input(filename.c_str());
        if (!input.good())
            throw "Could not open file: " + filename;

        std::vector<Point3d> points;
        bool hasColor = false;
        std::string line;
        std::setlocale(LC_NUMERIC, "en_US.UTF-8");
        while (std::getline(input, line))
        {
            float x, y, z, r = 0, g = 0, b = 0;
            sscanf(line.c_str(), "%f %f %f %f %f %f", &x, &y, &z, &r, &g, &b);
            r /= 255.0f;
            g /= 255.0f;
            b /= 255.0f;
            if (r > 0 || g > 0 || b > 0) hasColor = true;
            points.push_back(Point3d(Eigen::Vector3f(x, y, z), Eigen::Vector3f(r, g, b)));
        }

        input.close();

        if (hasColor)
            return new PointCloud3d(points, PointCloud3d::Mode::COLOR);
        else
            return new PointCloud3d(points);
    }

    PointCloud3d* loadFromPTX(const std::string &filename)
    {
        std::ifstream input(filename.c_str());
        if (!input.good())
            throw "Could not open file: " + filename;

        //setlocale(LC_ALL, ".OCP");

        std::vector<Point3d> points;
        bool hasIntensity = false;
        bool hasColor = false;
        std::string line;
        //int count = 0;
        std::locale("En_US");
        while (std::getline(input, line))
        {
            float x, y, z, intensity = 0, r = 0, g = 0, b = 0;
            std::stringstream ss;
            ss << line;
            ss >> x >> y >> z;
            if (!ss.eof())
            {
                ss >> intensity;
                if (!ss.eof())
                {
                    ss >> r >> g >> b;
                }
            }
            //sscanf(line.c_str(), "%f %f %f %f %f %f %f", &x, &y, &z, &intensity, &r, &g, &b);
            r /= 255.0f;
            g /= 255.0f;
            b /= 255.0f;
            if (intensity > 0) hasIntensity = true;
            if (r > 0 || g > 0 || b > 0) hasColor = true;
            points.push_back(Point3d(Eigen::Vector3f(x, z, -y), Eigen::Vector3f(r, g, b), intensity));
            //std::cout << x << " " << y << " " << z << " " << line << std::endl;
            //if (count++ > 10) getchar();
        }

        input.close();

        if (hasColor && hasIntensity)
            return new PointCloud3d(points, PointCloud3d::Mode::COLOR | PointCloud3d::Mode::INTENSITY);
        else if (hasColor)
            return new PointCloud3d(points, PointCloud3d::Mode::COLOR);
        else
            return new PointCloud3d(points);
    }

    PointCloud3d* load(const std::string &filename)
    {
        std::string extension = filename.substr(filename.find_last_of('.') + 1);
        PointCloud3d *pointCloud;
        if (extension == "pcl")
        {
            pointCloud = PointCloudIO::loadFromPCL<3>(filename);

        }
        else if (extension == "points")
        {
            pointCloud = PointCloudIO::loadFromPoints(filename);
        }
        else if (extension == "xyz")
        {
            pointCloud = PointCloudIO::loadFromXYZ(filename);
        }
        else if (extension == "ptx")
        {
            pointCloud = PointCloudIO::loadFromPTX(filename);
        }
        else
        {
            throw "Extension not supported: " + extension;
        }
        return pointCloud;
    }

};

#endif // POINTCLOUDLOADER_H
