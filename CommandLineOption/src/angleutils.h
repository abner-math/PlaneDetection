#ifndef ANGLEUTILS_H
#define ANGLEUTILS_H

#define _USE_MATH_DEFINES
#include <cmath>
#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif
#include <Eigen/Dense>

class AngleUtils
{
public:
    inline static float deg2rad(float deg)
    {
        return static_cast<float>(deg * M_PI / 180);
    }

    inline static float rad2deg(float rad)
    {
        return static_cast<float>(rad * 180 / M_PI);
    }

    /**
     * @brief Return a matrix corresponding to a rotation around an arbitrary axis
     * @param degrees
     *      Rotation degrees
     * @param axis
     *      Axis on which the rotation will be perfomed
     * @return
     *      A matrix corresponding to a rotation around an arbitrary axis
     */
    static Eigen::Matrix4f quaternionMatrix(float degrees, const Eigen::Vector3f &axis)
    {
        float radians = deg2rad(degrees);
        float scalar = std::cos(radians / 2);
        Eigen::Vector3f vector = std::sin(radians / 2) * axis.normalized();
        Eigen::Quaternion<float> quaternion(scalar, vector.x(), vector.y(), vector.z());

        Eigen::Matrix4f rotationMatrix;

        float xx = quaternion.x() * quaternion.x();
        float yy = quaternion.y() * quaternion.y();
        float zz = quaternion.z() * quaternion.z();
        float xy = quaternion.x() * quaternion.y();
        float xz = quaternion.x() * quaternion.z();
        float xw = quaternion.x() * quaternion.w();
        float yz = quaternion.y() * quaternion.z();
        float yw = quaternion.y() * quaternion.w();
        float zw = quaternion.z() * quaternion.w();

        rotationMatrix(0) = 1 - 2 * (yy + zz);
        rotationMatrix(1) = 2 * (xy - zw);
        rotationMatrix(2) = 2 * (xz + yw);
        rotationMatrix(3) = 0;

        rotationMatrix(4) = 2 * (xy + zw);
        rotationMatrix(5) = 1 - 2 * (xx + zz);
        rotationMatrix(6) = 2 * (yz - xw);
        rotationMatrix(7) = 0;

        rotationMatrix(8) = 2 * (xz - yw);
        rotationMatrix(9) = 2 * (yz + xw);
        rotationMatrix(10) = 1 - 2 * (xx + yy);
        rotationMatrix(11) = 0;

        rotationMatrix(12) = 0;
        rotationMatrix(13) = 0;
        rotationMatrix(14) = 0;
        rotationMatrix(15) = 1;

        return rotationMatrix;
    }

    static Eigen::Vector3f rotate(const Eigen::Vector3f &v, float degrees, const Eigen::Vector3f &axis)
    {
        Eigen::Affine3f t;
        t = Eigen::AngleAxisf(AngleUtils::deg2rad(degrees), axis);
        return t * v;
    }

    /**
     * @brief Convert spherical coordinates to cartesian
     * @param spherical
     *      A vector composed by (phi, theta, radius), where phi(azimuth) is the angle in x/z plane,
     *      theta(elevation) is the angle in the x/y plane and radius is the distance from origin
     * @return
     *      The vector representation in cartesian coordinates
     */
    static Eigen::Vector3f sphericalToCartesian(const Eigen::Vector3f &spherical)
    {
        float phi = deg2rad(spherical.x());
        float theta = deg2rad(spherical.y());
        float radius = spherical.z();
        float x = radius * std::cos(theta) * std::cos(phi);
        float y = radius * std::sin(theta);
        float z = radius * std::cos(theta) * std::sin(phi);
        return Eigen::Vector3f(x, y, z);
    }

    static Eigen::Vector3f sphericalToCartesian(float azimuth, float elevation, float radius)
    {
        return sphericalToCartesian(Eigen::Vector3f(azimuth, elevation, radius));
    }

    /**
     * @brief Convert cartesian coordinates to spherical
     * @param cartesian
     *      A vector in R3 in cartesian coordinates
     * @return
     *      The vector representation in spherical coordinates
     */
    static Eigen::Vector3f cartesianToSpherical(const Eigen::Vector3f &cartesian)
    {
        float radius = cartesian.norm();
        float phi = rad2deg(std::atan2(cartesian.z(), cartesian.x()));
        float theta = rad2deg(std::asin(cartesian.y() / radius));
        return Eigen::Vector3f(phi, theta, radius);
    }

    static Eigen::Vector3f cartesianToSpherical2(const Eigen::Vector3f &cartesian)
    {
        float radius = cartesian.norm();
        float theta = std::acos(cartesian.z() / radius);
        float phi = std::atan2(cartesian.y(), cartesian.x());
        return Eigen::Vector3f(theta, phi, radius);
    }

    static Eigen::Vector3f cartesianToSpherical(float x, float y, float z)
    {
        return cartesianToSpherical(Eigen::Vector3f(x, y, z));
    }

    /**
     * @brief Return the angle between two n-dimensional vectors, in radians
     */
    static float angleBetween(const Eigen::VectorXf &v1, const Eigen::VectorXf &v2)
    {
        return std::acos(v1.normalized().dot(v2.normalized()));
    }

};

#endif // ANGLEUTILS_H
