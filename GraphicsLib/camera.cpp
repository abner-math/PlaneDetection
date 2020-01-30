#include "camera.h"

Camera::Camera(const Eigen::Vector3f &origin, const Eigen::Vector3f &target)
    : mOrigin(origin)
    , mTarget(target)
    , mFieldOfView(60.0f)
    , mPerspectiveRatio(1.0f)
    , mZNear(0.1f)
    , mZFar(10000.0f)
    , mZoom(0.0f)
{
    reset();
}

void Camera::rotate(float degrees, Axis axis)
{
    switch (axis)
    {
    case X:
        mRotation.x() += degrees;
        break;
    case Y:
        mRotation.y() += degrees;
        break;
    case Z:
        mRotation.z() += degrees;
        break;
    }
    updateViewMatrix();
}

void Camera::translate(float x, float y)
{
    float speed = 2.0f * (mOrigin.z() - mTarget.z()) * std::sin(AngleUtils::deg2rad(mFieldOfView / 2.0f)) / 100.0f;
    mTranslation.x() += x * speed;
    mTranslation.y() -= y * speed;
    updateViewMatrix();
}

void Camera::zoom(float zoom)
{
    mZoom += zoom;
    //updateTransformationMatrix();
    updateProjectionMatrix();
}

void Camera::reset()
{
    mTranslation = Eigen::Vector2f::Zero();
    mRotation = Eigen::Vector3f::Zero();
    mZoom = 0.0f;
    updateViewMatrix();
    updateProjectionMatrix();
}

void Camera::setExtension(const Rect3d &extension)
{
    mExtension = extension;
    Eigen::Vector3f cameraTarget = extension.center();
    Eigen::Vector3f cameraOrigin = cameraTarget;
    Eigen::Vector3f max = mExtension.topRight();
    Eigen::Vector3f min = mExtension.bottomLeft();
    float length = std::max(max.x() - min.x(), max.y() - min.y());
    cameraOrigin.z() = mExtension.topRight().z();
    cameraOrigin.z() += length / 2.0f / std::tan(AngleUtils::deg2rad(mFieldOfView / 2.0f)) * 1.5f;
    float zNear = (max.z() - min.z()) / 1000.0f;
    float zFar = zNear + (max.z() - min.z()) * 100.0f;
    mOrigin = cameraOrigin;
    mTarget = cameraTarget;
    mZNear = zNear;
    mZFar = zFar;
    updateViewMatrix();
    updateProjectionMatrix();
}

void Camera::updateViewMatrix()
{
    Eigen::Vector3f origin = mOrigin + Eigen::Vector3f(mTranslation.x(), mTranslation.y(), 0.0f);
    Eigen::Vector3f target = mTarget + Eigen::Vector3f(mTranslation.x(), mTranslation.y(), 0.0f);
    Eigen::Vector3f forward = (target - origin).normalized();
    Eigen::Vector3f pseudoUp(0.0f, 1.0f, 0.0f);
    Eigen::Vector3f side = (forward.cross(pseudoUp)).normalized();
    Eigen::Vector3f up = (side.cross(forward)).normalized();
    mViewMatrix(0, 0) = side.x();       mViewMatrix(0, 1) = side.y();       mViewMatrix(0, 2) = side.z();       mViewMatrix(0, 3) = -side.dot(origin);
    mViewMatrix(1, 0) = up.x();         mViewMatrix(1, 1) = up.y();         mViewMatrix(1, 2) = up.z();         mViewMatrix(1, 3) = -up.dot(origin);
    mViewMatrix(2, 0) = -forward.x();   mViewMatrix(2, 1) = -forward.y();   mViewMatrix(2, 2) = -forward.z();   mViewMatrix(2, 3) = forward.dot(origin);
    mViewMatrix(3, 0) = 0.0f;           mViewMatrix(3, 1) = 0.0f;           mViewMatrix(3, 2) = 0.0f;           mViewMatrix(3, 3) = 1.0f;
    Eigen::Matrix4f translation1 = Eigen::Matrix4f::Identity();
    translation1(3, 0) = -target.x();
    translation1(3, 1) = -target.y();
    translation1(3, 2) = -target.z();
    Eigen::Matrix4f translation2 = Eigen::Matrix4f::Identity();
    translation2(3, 0) = target.x();
    translation2(3, 1) = target.y();
    translation2(3, 2) = target.z();
    mViewMatrix = mViewMatrix * translation2.transpose() * rotationMatrix() * translation1.transpose();
    updateTransformationMatrix();
}

void Camera::updateProjectionMatrix()
{
    float ymax = mZNear * std::tan(AngleUtils::deg2rad(((mFieldOfView + mZoom) / 2.0f)));
    float xmax = ymax * mPerspectiveRatio;
    float n = mZNear;
    float f = mZFar;
    float l = -xmax;
    float r = xmax;
    float b = -ymax;
    float t = ymax;
    mProjectionMatrix(0, 0) = 2*n/(r-l);    mProjectionMatrix(0, 1) = 0;			mProjectionMatrix(0, 2) = (r+l)/(r-l);		mProjectionMatrix(0, 3) = 0;
    mProjectionMatrix(1, 0) = 0;			mProjectionMatrix(1, 1) = 2*n/(t-b);	mProjectionMatrix(1, 2) = (t+b)/(t-b);		mProjectionMatrix(1, 3) = 0;
    mProjectionMatrix(2, 0) = 0;			mProjectionMatrix(2, 1) = 0;			mProjectionMatrix(2, 2) = -(f+n)/(f-n);     mProjectionMatrix(2, 3) = -2*f*n/(f-n);
    mProjectionMatrix(3, 0) = 0;			mProjectionMatrix(3, 1) = 0;			mProjectionMatrix(3, 2) = -1;				mProjectionMatrix(3, 3) = 0;
    updateTransformationMatrix();
}

Eigen::Matrix4f Camera::rotationMatrix() const
{
    return AngleUtils::quaternionMatrix(mRotation.x(), Eigen::Vector3f(-1, 0, 0))
                * AngleUtils::quaternionMatrix(mRotation.y(), Eigen::Vector3f(0, -1, 0))
                * AngleUtils::quaternionMatrix(mRotation.z(), Eigen::Vector3f(0, 0, 1));
}

void Camera::updateTransformationMatrix()
{
    mTransformationMatrix = mProjectionMatrix * mViewMatrix;
    mInverseTransformationMatrix = mTransformationMatrix.inverse();
    emit cameraUpdated();
}

Eigen::Vector3f Camera::getWorldPosition(int x, int y, float depth, int width, int height)
{
    float normalizedX = 2 * (x / (float)width) - 1;
    float normalizedY = 2 * ((height - y - 1) / (float)height) - 1;
    float normalizedZ = 2 * depth - 1;

    Eigen::Vector4f device(normalizedX, normalizedY, normalizedZ, 1.0f);
    Eigen::Vector4f world = mInverseTransformationMatrix * device;
    return Eigen::Vector3f(world.x() / world.w(), world.y() / world.w(), world.z() / world.w());
}
