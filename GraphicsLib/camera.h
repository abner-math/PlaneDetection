#ifndef CAMERA_H
#define CAMERA_H

#include <QObject>

#include <Eigen/Dense>

#include "rect.h"
#include "angleutils.h"

class Camera : public QObject
{
    Q_OBJECT
public:
    enum Axis
    {
        X = 0,
        Y = 1,
        Z = 2
    };

    Camera(const Eigen::Vector3f &origin = Eigen::Vector3f::Zero(), const Eigen::Vector3f &target = Eigen::Vector3f::Zero());

    const Eigen::Vector3f& origin() const
    {
       return mOrigin;
    }

    Camera& origin(const Eigen::Vector3f &origin)
    {
        mOrigin = origin;
        updateViewMatrix();
        return *this;
    }

    Camera& origin(float x, float y, float z)
    {
        return origin(Eigen::Vector3f(x, y, z));
    }

    const Eigen::Vector3f& target() const
    {
        return mTarget;
    }

    Camera& target(const Eigen::Vector3f &target)
    {
        mTarget = target;
        updateViewMatrix();
        return *this;
    }

    Camera& target(float x, float y, float z)
    {
        return target(Eigen::Vector3f(x, y, z));
    }

    float fieldOfView() const
    {
       return mFieldOfView;
    }

    Camera& fieldOfView(float fov)
    {
       mFieldOfView = fov;
       updateProjectionMatrix();
       return *this;
    }

    float perspectiveRatio() const
    {
        return mPerspectiveRatio;
    }

    Camera& perspectiveRatio(float perspectiveRatio)
    {
        mPerspectiveRatio = perspectiveRatio;
        updateProjectionMatrix();
        return *this;
    }

    float zNear() const
    {
        return mZNear;
    }

    Camera& zNear(float znear)
    {
        mZNear = znear;
        updateProjectionMatrix();
        return *this;
    }

    float zFar() const
    {
        return mZFar;
    }

    Camera& zFar(float zfar)
    {
        mZFar = zfar;
        updateProjectionMatrix();
        return *this;
    }

    /**
     * @brief Return the view-projection matrix
     * @return
     *      The view-projection matrix
     */
    const Eigen::Matrix4f& transformationMatrix() const
    {
        return mTransformationMatrix;
    }

    /**
     * @brief Return the inverse view-projection matrix
     * @return
     *      The inverse view-projection matrix
     */
    const Eigen::Matrix4f& inverseTransformationMatrix() const
    {
        return mInverseTransformationMatrix;
    }

    /**
     * @brief Return the view matrix
     * @return
     *      The view matrix
     */
    const Eigen::Matrix4f& viewMatrix() const
    {
        return mViewMatrix;
    }

    void translate(float x, float y);

    const Eigen::Vector2f& translation() const
    {
        return mTranslation;
    }

    void zoom(float zoom);

    float zoom() const
    {
        return mZoom;
    }

    void rotate(float degrees, Axis axis);

    /**
     * @brief Reset translation, zoom and rotations applied to the camera
     */
    void reset();

    void setExtension(const Rect3d &extension);

    const Rect3d& getExtension() const
    {
        return mExtension;
    }

    /**
     * @brief Transform a pixel back to the world coordinate system
     * @param x
     *      x coordinate on screen
     * @param y
     *      y coordinate on screen
     * @param depth
     *      z value from z-buffer
     * @param width
     *      screen width
     * @param height
     *      screen height
     * @return
     *      The corresponding world coordinate system
     */
    Eigen::Vector3f getWorldPosition(int x, int y, float depth, int width, int height);

signals:
    void cameraUpdated();

private:
    Eigen::Vector3f mOrigin;
    Eigen::Vector3f mTarget;
    float mFieldOfView;
    float mPerspectiveRatio;
    float mZNear;
    float mZFar;
    Rect3d mExtension;

    Eigen::Vector2f mTranslation;
    Eigen::Vector3f mRotation;
    float mZoom;

    Eigen::Matrix4f mViewMatrix;
    Eigen::Matrix4f mProjectionMatrix;
    Eigen::Matrix4f mTransformationMatrix;
    Eigen::Matrix4f mInverseTransformationMatrix;

    Eigen::Matrix4f rotationMatrix() const;

    void updateViewMatrix();

    void updateProjectionMatrix();

    void updateTransformationMatrix();

};

#endif // CAMERA_H
