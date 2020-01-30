#include "mainwindow.h"

#include <iostream>

#include <QApplication>
#include <QFileDialog>
#include <QMessageBox>
#include <QInputDialog>
#include <QElapsedTimer>
#include <QColorDialog>
#include <QtConcurrent/QtConcurrent>

#include "boundaryvolumehierarchy.h"
#include "geometryutils.h"
#include "colorutils.h"
#include "pcacalculator.h"

MainWindow::MainWindow(QWidget *parent)
    : CameraControlWindow(parent)
    , mPointCloud(NULL)
    , mSimplifiedPointCloud(NULL)
    , mPointCloudIOWorker(NULL)
    , mTranslateWorker(NULL)
    , mScaleWorker(NULL)
    , mRotateWorker(NULL)
    , mPlaneDetectorWorker(NULL)
    , mSelectState(DISABLED)
    , mCurrentComponent(std::numeric_limits<size_t>::max())
    , mProjectionPlane(NULL)
{
    std::srand(time_t(0));

    initMenu();

    initStatusBar();

    initGaussianMap();

    initSceneObjects();

    setWindowTitle("Point Cloud Editor");

    showMaximized();

    connect(mSceneWidget, SIGNAL(mouseMove(int, int)), this, SLOT(mouseMove(int, int)));
    connect(mSceneWidget, SIGNAL(mouseLeftClick()), this, SLOT(mouseLeftClick()));
    connect(mSceneWidget, SIGNAL(mouseRightClick()), this, SLOT(mouseRightClick()));
    connect(mSceneWidget, SIGNAL(cameraUpdated()), this, SLOT(cameraUpdated()));

    mSupportedExtensions = { std::make_pair("pcl", "PCL"), std::make_pair("points", "POINTS"), std::make_pair("xyz", "XYZ"), std::make_pair("ptx", "PTX") };
}

MainWindow::~MainWindow()
{

}

void MainWindow::initMenu()
{
    mFileMenu = new QMenu("File");

    mOpenPointCloudMenu = mFileMenu->addAction("Open point cloud...", this, &MainWindow::loadPointCloud, QKeySequence(Qt::CTRL + Qt::Key_O));
    mSavePointCloudMenu = mFileMenu->addAction("Save point cloud...", this, &MainWindow::savePointCloud, QKeySequence(Qt::CTRL + Qt::Key_S));
    mFileMenu->addSeparator();
    mImportConnectivityMenu = mFileMenu->addAction("Import connectivity...", this, &MainWindow::importConnectivity);
    mImportGeometryMenu = mFileMenu->addAction("Import geometry...", this, &MainWindow::importGeometry);
    mFileMenu->addSeparator();
    mExportConnectivityMenu = mFileMenu->addAction("Export connectivity...", this, &MainWindow::exportConnectivity);
    mExportGeometryMenu = mFileMenu->addAction("Export geometry...", this, &MainWindow::exportGeometry);
    mFileMenu->addSeparator();
    mFileMenu->addAction("Exit", this, &MainWindow::exit, QKeySequence(Qt::ALT + Qt::Key_F4));

    mSelectMenu = new QMenu("Select");

    QActionGroup *selectionGroup = new QActionGroup(this);
    mSelectMenu->addSection("Selection mode");
    QAction *selectPoints = mSelectMenu->addAction("Points", this, &MainWindow::selectionModePoint, QKeySequence(Qt::CTRL + Qt::Key_1));
    selectPoints->setCheckable(true);
    selectPoints->setChecked(true);
    selectPoints->setActionGroup(selectionGroup);
    QAction *selectPlanes = mSelectMenu->addAction("Planes", this, &MainWindow::selectionModePlane, QKeySequence(Qt::CTRL + Qt::Key_2));
    selectPlanes->setCheckable(true);
    selectPlanes->setChecked(false);
    selectPlanes->setActionGroup(selectionGroup);
    QAction *selectCylinders = mSelectMenu->addAction("Cylinders", this, &MainWindow::selectionModeCylinder, QKeySequence(Qt::CTRL + Qt::Key_3));
    selectCylinders->setCheckable(true);
    selectCylinders->setChecked(false);
    selectCylinders->setActionGroup(selectionGroup);
    QAction *selectConnectors = mSelectMenu->addAction("Connections", this, &MainWindow::selectionModeConnection, QKeySequence(Qt::CTRL + Qt::Key_4));
    selectConnectors->setCheckable(true);
    selectConnectors->setChecked(false);
    selectConnectors->setActionGroup(selectionGroup);
    mSelectMenu->addSeparator();

    mSelectMenu->addAction("Select region", this, &MainWindow::selectRegion);
    mSelectMenu->addAction("Deselect region", this, &MainWindow::deselectRegion);
    mSelectMenu->addAction("Select all", this, &MainWindow::selectAll, QKeySequence(Qt::CTRL + Qt::Key_A));
    mSelectMenu->addAction("Clear selection", this, &MainWindow::selectNone, QKeySequence(Qt::CTRL + Qt::SHIFT + Qt::Key_Z));
    mSelectMenu->addAction("Invert selection", this, &MainWindow::selectInvert, QKeySequence(Qt::CTRL + Qt::Key_I));
    mSelectMenu->addAction("Select points with high curvature", this, &MainWindow::selectPointsWithHighCurvature);

    mEditMenu = new QMenu("Edit");
    QMenu *transformMenu = new QMenu("Transform");
    transformMenu->addAction("Translate", this, &MainWindow::transformTranslate);
    transformMenu->addAction("Scale", this, &MainWindow::transformScale);
    transformMenu->addAction("Rotate", this, &MainWindow::transformRotate);
    mEditMenu->addMenu(transformMenu);
    mEditMenu->addAction("Set color", this, &MainWindow::editSetColor);
    mEditMenu->addAction("Remove selected", this, &MainWindow::editRemove, QKeySequence(Qt::Key_Delete));
    mEditMenu->addAction("Remove from primitives", this, &MainWindow::editRemoveFromPrimitives, QKeySequence(Qt::SHIFT + Qt::Key_Delete));
    mEditMenu->addSeparator();
    mEditUndo = mEditMenu->addAction("Undo", this, &MainWindow::editUndo, QKeySequence(Qt::CTRL + Qt::Key_Z));
    mEditUndo->setEnabled(false);

    QMenu *primitiveDetectorMenu = new QMenu("Primitive detector");
    QMenu *detectCylindersMenu = new QMenu("Cylinders");
    primitiveDetectorMenu->addMenu(detectCylindersMenu);

    mViewMenu->addSeparator();

    QActionGroup *pointCloudGroup = new QActionGroup(this);

    mPointCloudMenu = new QMenu("Point cloud");

    mDisplayPointCloudMenu = mPointCloudMenu->addAction("Point cloud", this, &MainWindow::displayPointCloud, QKeySequence(Qt::Key_F1));
    mDisplayPointCloudMenu->setCheckable(true);
    mDisplayPointCloudMenu->setChecked(true);
    mDisplayNormalsMenu = mPointCloudMenu->addAction("Normals", this, &MainWindow::displayNormals, QKeySequence(Qt::Key_F2));
    mDisplayNormalsMenu->setCheckable(true);
    mDisplayNormalsMenu->setChecked(false);
    mDisplayGaussianMapMenu = mPointCloudMenu->addAction("Gauss Map", this, &MainWindow::displayGaussianMap, QKeySequence(Qt::Key_F3));
    mDisplayGaussianMapMenu->setCheckable(true);
    mDisplayGaussianMapMenu->setChecked(false);
    mDisplayNextGroupMenu = mPointCloudMenu->addAction("Next connected component", this, &MainWindow::displayNextGroup, QKeySequence(Qt::Key_F4));
    mDisplayNextGroupMenu->setCheckable(false);
    mDisplayNextGroupMenu->setChecked(false);

    mPointCloudMenu->addSection("Color mode");
    QAction *colorDefaultMenu = mPointCloudMenu->addAction("Default", this, &MainWindow::displayColors, QKeySequence(Qt::Key_1));
    colorDefaultMenu->setCheckable(true);
    colorDefaultMenu->setChecked(true);
    colorDefaultMenu->setActionGroup(pointCloudGroup);
    QAction *colorIntensitiesMenu = mPointCloudMenu->addAction("Intensities", this, &MainWindow::displayIntensities, QKeySequence(Qt::Key_2));
    colorIntensitiesMenu->setCheckable(true);
    colorIntensitiesMenu->setChecked(false);
    colorIntensitiesMenu->setActionGroup(pointCloudGroup);
    QAction *colorCurvaturesMenu = mPointCloudMenu->addAction("Curvatures", this, &MainWindow::displayCurvatures, QKeySequence(Qt::Key_3));
    colorCurvaturesMenu->setCheckable(true);
    colorCurvaturesMenu->setChecked(false);
    colorCurvaturesMenu->setActionGroup(pointCloudGroup);
    QAction *colorComponentsMenu = mPointCloudMenu->addAction("Connected components", this, &MainWindow::displayComponents, QKeySequence(Qt::Key_4));
    colorComponentsMenu->setCheckable(true);
    colorComponentsMenu->setChecked(false);
    colorComponentsMenu->setActionGroup(pointCloudGroup);

    QMenu *primitiveMenu = new QMenu("Primitives");
    mDisplayPlanesMenu = primitiveMenu->addAction("Planes", this, &MainWindow::displayPlanes, QKeySequence(Qt::SHIFT + Qt::Key_1));
    mDisplayPlanesMenu->setCheckable(true);
    mDisplayPlanesMenu->setChecked(true);
    mDisplayCylindersMenu = primitiveMenu->addAction("Cylinders", this, &MainWindow::displayCylinders, QKeySequence(Qt::SHIFT + Qt::Key_2));
    mDisplayCylindersMenu->setCheckable(true);
    mDisplayCylindersMenu->setChecked(true);
    mDisplayGeometryMenu = primitiveMenu->addAction("Geometry", this, &MainWindow::displayGeometry, QKeySequence(Qt::SHIFT + Qt::Key_0));
    mDisplayGeometryMenu->setCheckable(true);
    mDisplayGeometryMenu->setChecked(true);

    mViewMenu->addMenu(mPointCloudMenu);
    mViewMenu->addMenu(primitiveMenu);

    mViewMenu->addAction("Projection", this, &MainWindow::displayProjection);

    mDisplayBoundingBoxMenu = mViewMenu->addAction("Bounding box", this, &MainWindow::displayBoundingBox);
    mDisplayBoundingBoxMenu->setCheckable(true);
    mDisplayBoundingBoxMenu->setChecked(false);

    mViewMenu->addSeparator();
    mViewMenu->addAction("Options", this, &MainWindow::displayOptions);

    mToolsMenu = new QMenu("Plane Detector");

    QMenu *detectionMenu = mToolsMenu->addMenu("Detect planes");
    detectionMenu->addAction("Autodetect planes", this, &MainWindow::detectPlanesDialog, QKeySequence(Qt::CTRL + Qt::SHIFT + Qt::Key_P));
    detectionMenu->addSeparator();
    detectionMenu->addAction("Expand region", this, &MainWindow::expandPlaneRegion, QKeySequence(Qt::SHIFT + Qt::Key_P));
    detectionMenu->addAction("Detect plane", this, &MainWindow::detectPlane, QKeySequence(Qt::CTRL + Qt::Key_P));
    detectionMenu->addAction("Merge planes", this, &MainWindow::mergePlanes, QKeySequence(Qt::CTRL + Qt::ALT + Qt::Key_P));

    QMenu *normalsMenu = mToolsMenu->addMenu("Estimate normals");
    normalsMenu->addAction("Slow", this, &MainWindow::estimateNormalsSlow);
    normalsMenu->addAction("Quick", this, &MainWindow::estimateNormalsQuick);

    mToolsMenu->addSeparator();
    mToolsMenu->addAction("Point cloud info", this, &MainWindow::pointCloudInfo);

    menuBar()->addMenu(mSelectMenu);
    menuBar()->insertMenu(mSelectMenu->menuAction(), mFileMenu);
    menuBar()->addMenu(mEditMenu);
    menuBar()->addMenu(mViewMenu);
    menuBar()->addMenu(mToolsMenu);

    mSceneWidget->update();
    enablePointCloudChange(false);
    mOpenPointCloudMenu->setEnabled(true);
}

void MainWindow::initStatusBar()
{
    mProgressBar = new QProgressBar(this);
    mProgressBar->setMinimum(0);
    mProgressBar->setMaximum(100);
    mProgressBar->setVisible(false);

    mStopWorkerButton = new QPushButton("Stop", this);
    connect(mStopWorkerButton, SIGNAL(clicked()), this, SLOT(workerStop()));
    mStopWorkerButton->setVisible(false);

    statusBar()->addPermanentWidget(mProgressBar, 0);
    statusBar()->addPermanentWidget(mStopWorkerButton);
}

void MainWindow::initGaussianMap()
{
    mGaussianMapWidget = new SceneWidget(this);
    mGaussianMapWidget->setVisible(false);
    mGaussianMapWidget->setSizePolicy(QSizePolicy::Policy::Minimum, QSizePolicy::Policy::Preferred);
    mGaussianMapWidget->setFixedWidth(500);

    mGaussianMapDrawer = new GaussianMapDrawer(NULL, 0, 0, 0, 0.5f);

    mGaussianMapWidget->scene()->camera().setExtension(Rect3d(Eigen::Vector3f(-1, -1, -1), Eigen::Vector3f(1, 1, 1)));
    mGaussianMapWidget->scene()->addObject(mGaussianMapDrawer);
}

void MainWindow::initSceneObjects()
{
    mSelectFilter = new SelectFilter;
    mPointCloudDrawer = new PointCloudDrawer(mSelectFilter, NULL, NULL, SceneObject::Priority::NORMAL);
    mScene->addObject(mPointCloudDrawer);
    mBoundingBoxDrawer = new BoundingBoxDrawer(NULL, SceneObject::Priority::NORMAL);
    mPlaneDrawer = new PlaneDrawer(mSelectFilter, NULL);
    mScene->addObject(mPlaneDrawer);
    mCylinderDrawer = new CylinderDrawer(mSelectFilter, NULL);
    mScene->addObject(mCylinderDrawer);
    mNormalDrawer = new NormalDrawer(NULL);
    mScene->addFilter(mSelectFilter);

    mPointCloudIOWorker = new PointCloudIOWorker;
    connect(mPointCloudIOWorker, SIGNAL(workerProgress(float)), this, SLOT(workerProgress(float)));
    connect(mPointCloudIOWorker, SIGNAL(workerStatus(const QString&)), this, SLOT(workerStatus(const QString&)));
    connect(mPointCloudIOWorker, SIGNAL(workerFinish()), this, SLOT(loadPointCloudFinish()));
    connect(mPointCloudIOWorker, SIGNAL(loadError(const QString&)), this, SLOT(loadPointCloudError(const QString&)));
    mNormalEstimatorWorker = new NormalEstimatorWorker(NULL);
    connect(mNormalEstimatorWorker, SIGNAL(workerProgress(float)), this, SLOT(workerProgress(float)));
    connect(mNormalEstimatorWorker, SIGNAL(workerStatus(const QString&)), this, SLOT(workerStatus(const QString&)));
    connect(mNormalEstimatorWorker, SIGNAL(workerFinish()), this, SLOT(normalEstimationFinish()));
    connect(&mTranslateDialog, SIGNAL(translate(float,float,float)), this, SLOT(translatePointCloud(float, float, float)));
    connect(&mScaleDialog, SIGNAL(scale(float,float,float)), this, SLOT(scalePointCloud(float,float,float)));
    connect(&mRotateDialog, SIGNAL(rotate(float,float,float,float)), this, SLOT(rotatePointCloud(float,float,float,float)));
    connect(&mViewOptionsDialog, SIGNAL(setViewOptions(float,float,float,float,float,float,size_t,float)), this, SLOT(setViewOptions(float,float,float,float,float,float,size_t,float)));
    mTranslateWorker = new TransformTranslateWorker(mSelectFilter);
    connect(mTranslateWorker, SIGNAL(workerProgress(float)), this, SLOT(workerProgress(float)));
    connect(mTranslateWorker, SIGNAL(workerStatus(const QString&)), this, SLOT(workerStatus(const QString&)));
    connect(mTranslateWorker, SIGNAL(workerFinish()), this, SLOT(translateFinish()));
    mScaleWorker = new TransformScaleWorker(mSelectFilter);
    connect(mScaleWorker, SIGNAL(workerProgress(float)), this, SLOT(workerProgress(float)));
    connect(mScaleWorker, SIGNAL(workerStatus(const QString&)), this, SLOT(workerStatus(const QString&)));
    connect(mScaleWorker, SIGNAL(workerFinish()), this, SLOT(scaleFinish()));
    mRotateWorker = new TransformRotateWorker(mSelectFilter);
    connect(mRotateWorker, SIGNAL(workerProgress(float)), this, SLOT(workerProgress(float)));
    connect(mRotateWorker, SIGNAL(workerStatus(const QString&)), this, SLOT(workerStatus(const QString&)));
    connect(mRotateWorker, SIGNAL(workerFinish()), this, SLOT(rotateFinish()));
    mPlaneDetectorWorker = new PlaneDetectorWorker(NULL);
    connect(mPlaneDetectorWorker, SIGNAL(workerProgress(float)), this, SLOT(workerProgress(float)));
    connect(mPlaneDetectorWorker, SIGNAL(workerStatus(const QString&)), this, SLOT(workerStatus(const QString&)));
    connect(mPlaneDetectorWorker, SIGNAL(workerFinish()), this, SLOT(planeDetectionFinish()));
    connect(&mPlaneDetectorDialog, SIGNAL(detectPlaneOptions(float, float, float)), this, SLOT(detectPlanes(float, float, float)));
}

void MainWindow::clearPointCloud()
{
    clearPrimitives();
    delete mPointCloud;
    mPointCloud = NULL;
    delete mSimplifiedPointCloud;
    mSimplifiedPointCloud = NULL;
    mCurrentComponent = std::numeric_limits<size_t>::max();
}

void MainWindow::updatePrimitives()
{
    mPlaneDrawer->update();
    mCylinderDrawer->update();
}

void MainWindow::updatePointCloud()
{
    QApplication::setOverrideCursor(Qt::WaitCursor);

    // update scene objects
    mPointCloudDrawer->pointCloud(mPointCloud, mSimplifiedPointCloud);
    mPointCloudDrawer->update();
    mPlaneDrawer->pointCloud(mPointCloud);
    mPlaneDrawer->update();
    mCylinderDrawer->pointCloud(mPointCloud);
    mCylinderDrawer->update();
    mBoundingBoxDrawer->pointCloud(mSimplifiedPointCloud);
    mBoundingBoxDrawer->update();
    mNormalDrawer->pointCloud(mSimplifiedPointCloud);
    mNormalDrawer->update();
    mGaussianMapDrawer->pointCloud(mSimplifiedPointCloud);
    mGaussianMapDrawer->update();
    mGaussianMapWidget->update();
    mSceneWidget->update();

    // update workers
    mNormalEstimatorWorker->pointCloud(mPointCloud);
    mTranslateWorker->pointCloud(mPointCloud, mSimplifiedPointCloud);
    mScaleWorker->pointCloud(mPointCloud, mSimplifiedPointCloud);
    mRotateWorker->pointCloud(mPointCloud, mSimplifiedPointCloud);
    mPlaneDetectorWorker->pointCloud(mPointCloud);

    // update widgets
    mTranslateDialog.setPointCloud(mPointCloud);
    mInfoDialog.setPointCloud(mPointCloud);

    QApplication::setOverrideCursor(Qt::ArrowCursor);
}

void MainWindow::exit()
{
    close();
}

void MainWindow::selectPoints()
{
    int width = mScene->frameBuffer()->width();
    int height = mScene->frameBuffer()->height();
    float minX = std::min(mSelectFilter->firstPoint().x(), mSelectFilter->secondPoint().x()) / (float)width * 2 - 1;
    float maxX = std::max(mSelectFilter->firstPoint().x(), mSelectFilter->secondPoint().x()) / (float)width * 2 - 1;
    float maxY = -(std::min(mSelectFilter->firstPoint().y(), mSelectFilter->secondPoint().y()) / (float)height * 2 - 1);
    float minY = -(std::max(mSelectFilter->firstPoint().y(), mSelectFilter->secondPoint().y()) / (float)height * 2 - 1);
    Eigen::Matrix4Xf points(4, mSimplifiedPointCloud->size());
    for (size_t i = 0; i < mSimplifiedPointCloud->size(); i++)
    {
        Eigen::Vector3f position = mSimplifiedPointCloud->at(i).position();
        points.col(i) = Eigen::Vector4f(position.x(), position.y(), position.z(), 1.0f);
    }
    Eigen::Matrix4Xf transformed = mScene->camera().transformationMatrix() * points;
    transformed = transformed.array().rowwise() / transformed.array().row(3);
    for (size_t i = 0; i < mSimplifiedPointCloud->size(); i++)
    {
        float x = transformed(0, i);
        float y = transformed(1, i);
        float z = transformed(2, i);
        if (z >= -1 && z <= 1 && x >= minX && x <= maxX && y >= minY && y <= maxY
                && mPointCloudDrawer->visible(i))
        {
            if (mSelectFilter->isSelecting())
            {
                for (const size_t &point : mSimplifiedPointCloud->virtual2real(i))
                {
                    mSelectFilter->select(point);
                }
            }
            else
            {
                for (const size_t &point : mSimplifiedPointCloud->virtual2real(i))
                {
                    mSelectFilter->deselect(point);
                }
            }
        }
    }
    mPointCloudDrawer->update();
}

bool MainWindow::isSelected(const Plane *plane, float minX, float maxX, float minY, float maxY)
{
    std::vector<Eigen::Vector3f> points;
    for (size_t i = 0; i <= 36; i++)
    {
        for (size_t j = 0; j <= 36; j++)
        {
            Eigen::Vector3f _u = plane->basisU() - plane->basisU() * 2 * i / 36.0f;
            Eigen::Vector3f _v = plane->basisV() - plane->basisV() * 2 * j / 36.0f;
            points.push_back(plane->center() + _u + _v);
        }
    }
    Eigen::Matrix4Xf matrix(4, points.size());
    for (size_t i = 0; i < points.size(); i++)
    {
        Eigen::Vector3f point = points[i];
        matrix.col(i) = Eigen::Vector4f(point.x(), point.y(), point.z(), 1.0f);
    }
    Eigen::Matrix4Xf transformed = mScene->camera().transformationMatrix() * matrix;
    transformed = transformed.array().rowwise() / transformed.array().row(3);
    for (size_t i = 0; i < points.size(); i++)
    {
        float x = transformed(0, i);
        float y = transformed(1, i);
        float z = transformed(2, i);
        if (z >= -1 && z <= 1 && x >= minX && x <= maxX && y >= minY && y <= maxY)
        {
            return true;
        }
    }
    return false;
}

bool MainWindow::isSelected(const Cylinder *cylinder, float minX, float maxX, float minY, float maxY)
{
    std::vector<Eigen::Vector3f> points;
    Eigen::Vector3f v1;
    Eigen::Vector3f v2;
    GeometryUtils::orthogonalBasis(cylinder->axis(), v1, v2);
    float radius = cylinder->radius();
    float height = cylinder->height();
    for (size_t i = 0; i < 36; i++)
    {
        Eigen::Vector3f circleCenter = cylinder->center() + cylinder->axis() * (i / 35.0f * height - height / 2.0f);
        for (size_t j = 0; j < 36; j++)
        {
            float theta = AngleUtils::deg2rad(10 * j);
            points.push_back(circleCenter + v1 * std::sin(theta) * radius + v2 * std::cos(theta) * radius);
        }
    }
    Eigen::Matrix4Xf matrix(4, points.size());
    for (size_t i = 0; i < points.size(); i++)
    {
        Eigen::Vector3f point = points[i];
        matrix.col(i) = Eigen::Vector4f(point.x(), point.y(), point.z(), 1.0f);
    }
    Eigen::Matrix4Xf transformed = mScene->camera().transformationMatrix() * matrix;
    transformed = transformed.array().rowwise() / transformed.array().row(3);
    for (size_t i = 0; i < points.size(); i++)
    {
        float x = transformed(0, i);
        float y = transformed(1, i);
        float z = transformed(2, i);
        if (z >= -1 && z <= 1 && x >= minX && x <= maxX && y >= minY && y <= maxY)
        {
            return true;
        }
    }
    return false;
}

bool MainWindow::isSelected(const Connection *connection, float minX, float maxX, float minY, float maxY)
{
    std::vector<Eigen::Vector3f> points;
    connection->volume().getVertices(points);
    Eigen::Matrix4Xf matrix(4, points.size());
    for (size_t i = 0; i < points.size(); i++)
    {
        Eigen::Vector3f point = points[i];
        matrix.col(i) = Eigen::Vector4f(point.x(), point.y(), point.z(), 1.0f);
    }
    Eigen::Matrix4Xf transformed = mScene->camera().transformationMatrix() * matrix;
    transformed = transformed.array().rowwise() / transformed.array().row(3);
    for (size_t i = 0; i < points.size(); i++)
    {
        float x = transformed(0, i);
        float y = transformed(1, i);
        float z = transformed(2, i);
        if (z >= -1 && z <= 1 && x >= minX && x <= maxX && y >= minY && y <= maxY)
        {
            return true;
        }
    }
    return false;
}

void MainWindow::selectPlanes()
{
    int width = mScene->frameBuffer()->width();
    int height = mScene->frameBuffer()->height();
    float minX = std::min(mSelectFilter->firstPoint().x(), mSelectFilter->secondPoint().x()) / (float)width * 2 - 1;
    float maxX = std::max(mSelectFilter->firstPoint().x(), mSelectFilter->secondPoint().x()) / (float)width * 2 - 1;
    float maxY = -(std::min(mSelectFilter->firstPoint().y(), mSelectFilter->secondPoint().y()) / (float)height * 2 - 1);
    float minY = -(std::max(mSelectFilter->firstPoint().y(), mSelectFilter->secondPoint().y()) / (float)height * 2 - 1);
    for (size_t i = 0; i < mPointCloud->geometry()->numPlanes(); i++)
    {
        if (isSelected(mPointCloud->geometry()->plane(i), minX, maxX, minY, maxY))
        {
            if (mSelectFilter->isSelecting())
            {
                mSelectFilter->select(i);
            }
            else
            {
                mSelectFilter->deselect(i);
            }
        }
    }
}

void MainWindow::selectCylinders()
{
    int width = mScene->frameBuffer()->width();
    int height = mScene->frameBuffer()->height();
    float minX = std::min(mSelectFilter->firstPoint().x(), mSelectFilter->secondPoint().x()) / (float)width * 2 - 1;
    float maxX = std::max(mSelectFilter->firstPoint().x(), mSelectFilter->secondPoint().x()) / (float)width * 2 - 1;
    float maxY = -(std::min(mSelectFilter->firstPoint().y(), mSelectFilter->secondPoint().y()) / (float)height * 2 - 1);
    float minY = -(std::max(mSelectFilter->firstPoint().y(), mSelectFilter->secondPoint().y()) / (float)height * 2 - 1);
    for (size_t i = 0; i < mPointCloud->geometry()->numCylinders(); i++)
    {
        if (isSelected(mPointCloud->geometry()->cylinder(i), minX, maxX, minY, maxY))
        {
            if (mSelectFilter->isSelecting())
            {
                mSelectFilter->select(i);
            }
            else
            {
                mSelectFilter->deselect(i);
            }
        }
    }
}

void MainWindow::selectConnections()
{
    int width = mScene->frameBuffer()->width();
    int height = mScene->frameBuffer()->height();
    float minX = std::min(mSelectFilter->firstPoint().x(), mSelectFilter->secondPoint().x()) / (float)width * 2 - 1;
    float maxX = std::max(mSelectFilter->firstPoint().x(), mSelectFilter->secondPoint().x()) / (float)width * 2 - 1;
    float maxY = -(std::min(mSelectFilter->firstPoint().y(), mSelectFilter->secondPoint().y()) / (float)height * 2 - 1);
    float minY = -(std::max(mSelectFilter->firstPoint().y(), mSelectFilter->secondPoint().y()) / (float)height * 2 - 1);
    for (size_t i = 0; i < mPointCloud->geometry()->numConnections(); i++)
    {
        if (isSelected(mPointCloud->geometry()->connection(i), minX, maxX, minY, maxY))
        {
            if (mSelectFilter->isSelecting())
            {
                mSelectFilter->select(i);
            }
            else
            {
                mSelectFilter->deselect(i);
            }
        }
    }
}

void MainWindow::mouseLeftClick()
{
    if (mSelectState == FIRST_POINT || (mSelectState == DISABLED && (QApplication::keyboardModifiers().testFlag(Qt::ShiftModifier) ||
                                                                     QApplication::keyboardModifiers().testFlag(Qt::ControlModifier))))
    {
        if (QApplication::keyboardModifiers().testFlag(Qt::ControlModifier) || !mSelectFilter->isSelecting())
        {
            mSelectFilter->setDeselect();
        }
        else
        {
            mSelectFilter->setSelect();
        }
        mSelectFilter->firstPoint(mMousePosition);
        mSelectState = SECOND_POINT;
        QApplication::setOverrideCursor(Qt::CrossCursor);
    }
    else if (mSelectState == SECOND_POINT)
    {
        QApplication::setOverrideCursor(Qt::WaitCursor);
        switch (mSelectFilter->mode())
        {
        case SelectFilter::SelectMode::POINT:
            selectPoints();
            break;
        case SelectFilter::SelectMode::PLANE:
            selectPlanes();
            break;
        case SelectFilter::SelectMode::CYLINDER:
            selectCylinders();
            break;
        case SelectFilter::SelectMode::CONNECTION:
            selectConnections();
            break;
        }
        updatePrimitives();
        mSelectState = DISABLED;
        mSelectFilter->clear();
        mSceneWidget->update();
        QApplication::setOverrideCursor(Qt::ArrowCursor);
    }
}

void MainWindow::mouseRightClick()
{
    QApplication::setOverrideCursor(Qt::ArrowCursor);
    if (mSelectState == FIRST_POINT)
    {
        mSelectState = DISABLED;
    }
    else if (mSelectState == SECOND_POINT)
    {
        mSelectState = DISABLED;
        mSelectFilter->clear();
        mSceneWidget->update();
    }
}

void MainWindow::mouseMove(int x, int y)
{
    if (mPointCloud == NULL) return;
    int width = mScene->frameBuffer()->width();
    int height = mScene->frameBuffer()->height();
    float depth;
    mScene->frameBuffer()->pixelDepth(x, y, depth);
    if (depth < 1)
    {
        mLastValidDepth = depth;
    }
    else
    {
        depth = mLastValidDepth;
    }
    mScreenPosition = mScene->camera().getWorldPosition(x, y, depth, width, height);
    mMousePosition = Eigen::Vector2f(x, y);
    if (mSelectState == SECOND_POINT)
    {
        mSelectFilter->secondPoint(mMousePosition);
        mSceneWidget->update();
    }
}

void MainWindow::cameraUpdated()
{
    if (mPointCloud == NULL) return;
}

void MainWindow::selectRegion()
{
    QApplication::setOverrideCursor(Qt::CrossCursor);
    mSelectState = FIRST_POINT;
    mSelectFilter->clear();
    mSelectFilter->setSelect();
    mSceneWidget->update();
}

void MainWindow::deselectRegion()
{
    QApplication::setOverrideCursor(Qt::CrossCursor);
    mSelectState = FIRST_POINT;
    mSelectFilter->clear();
    mSelectFilter->setDeselect();
    mSceneWidget->update();
}

void MainWindow::selectAll()
{
    if (mPointCloud == NULL) return;
    QApplication::setOverrideCursor(Qt::ArrowCursor);
    mSelectState = DISABLED;
    mSelectFilter->selectAll();
    switch (mSelectFilter->mode())
    {
    case SelectFilter::SelectMode::POINT:
        mPointCloudDrawer->update();
        break;
    }
    updatePrimitives();
    mSceneWidget->update();
}

void MainWindow::selectNone()
{
    if (mPointCloud == NULL) return;
    QApplication::setOverrideCursor(Qt::ArrowCursor);
    mSelectFilter->clearSelection();
    switch (mSelectFilter->mode())
    {
    case SelectFilter::SelectMode::POINT:
        mPointCloudDrawer->update();
        break;
    }
    updatePrimitives();
    mSceneWidget->update();
}

void MainWindow::selectInvert()
{
    if (mPointCloud == NULL) return;
    QApplication::setOverrideCursor(Qt::ArrowCursor);
    mSelectFilter->invertSelection();
    switch (mSelectFilter->mode())
    {
    case SelectFilter::SelectMode::POINT:
        mPointCloudDrawer->update();
        break;
    }
    updatePrimitives();
    mSceneWidget->update();
}

void MainWindow::selectPointsWithHighCurvature()
{
    if (mPointCloud == NULL || !mPointCloud->hasConnectivity() || mSelectFilter->mode() != SelectFilter::SelectMode::POINT) return;
    QApplication::setOverrideCursor(Qt::ArrowCursor);
    /*for (size_t i = 0; i < mPointCloud->size(); i++)
    {
        Eigen::Vector3f basisU, basisV;
        GeometryUtils::orthogonalBasis(mPointCloud->at(i).normal(), basisU, basisV);
        std::vector<size_t> neighbors = mPointCloud->connectivity()->neighbors(i);
        Eigen::Matrix2Xf positions(2, neighbors.size());
        for (size_t j = 0; j < neighbors.size(); j++)
        {
            positions.col(j) = GeometryUtils::projectOntoOrthogonalBasis(mPointCloud->at(neighbors[j]).position(), basisU, basisV);
        }
        Eigen::Vector2f eigenValues;
        PCACalculator<2>::calculate(positions, eigenValues);
        std::cout << eigenValues(1) << " " << eigenValues(0) << std::endl;
        if (std::abs(eigenValues(1)) > 100 * std::abs(eigenValues(0)))
        {
            mSelectFilter->select(i);
        }
    }*/
    std::vector<size_t> points(mPointCloud->size());
    std::iota(points.begin(), points.end(), 0);
    std::sort(points.begin(), points.end(), [&](const size_t &a, const size_t &b) {
        return mPointCloud->at(a).curvature() > mPointCloud->at(b).curvature();
    });
    size_t i = 0;
    for (const size_t &point : points)
    {
        mSelectFilter->select(point);
        ++i;
        if (i >= mPointCloud->size() / 10)
        {
            break;
        }
    }
    mPointCloudDrawer->update();
    mSceneWidget->update();
}

void MainWindow::selectionModePoint()
{
    mSelectFilter->mode(SelectFilter::SelectMode::POINT);
}

void MainWindow::selectionModePlane()
{
    mSelectFilter->mode(SelectFilter::SelectMode::PLANE);
}

void MainWindow::selectionModeCylinder()
{
    mSelectFilter->mode(SelectFilter::SelectMode::CYLINDER);
}

void MainWindow::selectionModeConnection()
{
    mSelectFilter->mode(SelectFilter::SelectMode::CONNECTION);
}

void MainWindow::addState()
{
    return;
    std::vector<Point3d> points(mPointCloud->size());
    for (size_t i = 0; i < mPointCloud->size(); i++)
    {
        points[i] = mPointCloud->at(i);
    }
    PointCloud3d *copy = new PointCloud3d(points, mPointCloud->mode());
    if (mPointCloud->hasConnectivity())
    {
        copy->connectivity(new ConnectivityGraph(*mPointCloud->connectivity()));
    }
    for (size_t i = 0; i < mPointCloud->geometry()->numCircles(); i++)
    {
        copy->geometry()->addCircle(new Circle(*mPointCloud->geometry()->circle(i)));
    }
    for (size_t i = 0; i < mPointCloud->geometry()->numPlanes(); i++)
    {
        copy->geometry()->addPlane(new Plane(*mPointCloud->geometry()->plane(i)));
    }
    for (size_t i = 0; i < mPointCloud->geometry()->numCylinders(); i++)
    {
        copy->geometry()->addCylinder(new Cylinder(*mPointCloud->geometry()->cylinder(i)));
    }
    for (size_t i = 0; i < mPointCloud->geometry()->numConnections(); i++)
    {
        copy->geometry()->addConnection(new Connection(*mPointCloud->geometry()->connection(i)));
    }
    copy->update();
    if (mEditStates.size() >= 10)
    {
        delete mEditStates[0];
        mEditStates.erase(mEditStates.begin());
        mEditStates.push_back(copy);
    }
    else
    {
        mEditStates.push_back(copy);
    }
    mEditUndo->setEnabled(true);
}

void MainWindow::transformTranslate()
{
    mTranslateDialog.show();
}

void MainWindow::translatePointCloud(float dx, float dy, float dz)
{
    addState();
    mTranslateWorker->translation(dx, dy, dz);
    workerStart(mTranslateWorker);
}

void MainWindow::translateFinish()
{
    statusBar()->showMessage(("Done in " + std::to_string(mTranslateWorker->timeElapsedInSec()) + "s.").c_str());

    // update camera
    Rect3d extension(mPointCloud->center().array() - mPointCloud->extension().maxSize() / 2, mPointCloud->center().array() + mPointCloud->extension().maxSize() / 2);
    mScene->camera().setExtension(extension);
    mScene->camera().reset();

    updatePointCloud();

    workerFinish();
}

void MainWindow::scalePointCloud(float dx, float dy, float dz)
{
    addState();
    mScaleWorker->scale(dx, dy, dz);
    workerStart(mScaleWorker);
}

void MainWindow::scaleFinish()
{
    statusBar()->showMessage(("Done in " + std::to_string(mScaleWorker->timeElapsedInSec()) + "s.").c_str());

    // update camera
    Rect3d extension(mPointCloud->center().array() - mPointCloud->extension().maxSize() / 2, mPointCloud->center().array() + mPointCloud->extension().maxSize() / 2);
    mScene->camera().setExtension(extension);
    mScene->camera().reset();

    updatePointCloud();

    workerFinish();
}

void MainWindow::transformScale()
{
    mScaleDialog.show();
}

void MainWindow::rotatePointCloud(float x, float y, float z, float degrees)
{
    addState();
    mRotateWorker->rotate(x, y, z, degrees);
    workerStart(mRotateWorker);
}

void MainWindow::rotateFinish()
{
    statusBar()->showMessage(("Done in " + std::to_string(mRotateWorker->timeElapsedInSec()) + "s.").c_str());

    // update camera
    Rect3d extension(mPointCloud->center().array() - mPointCloud->extension().maxSize() / 2, mPointCloud->center().array() + mPointCloud->extension().maxSize() / 2);
    mScene->camera().setExtension(extension);
    mScene->camera().reset();

    updatePointCloud();

    workerFinish();
}

void MainWindow::transformRotate()
{
    mRotateDialog.show();
}

void MainWindow::editSetColor()
{
    if (mPointCloud == NULL || !mSelectFilter->isAnySelected()) return;
    Eigen::Vector3f color;
    QColor qColor = QColorDialog::getColor(QColor(255, 255, 255), this);
    if (!qColor.isValid()) return;
    color.x() = qColor.red() / 255.0f;
    color.y() = qColor.green() / 255.0f;
    color.z() = qColor.blue() / 255.0f;
    QApplication::setOverrideCursor(Qt::WaitCursor);
    addState();
    if (mSelectFilter->mode() == SelectFilter::SelectMode::POINT)
    {
        for (int i = mPointCloud->size() - 1; i >= 0; i--)
        {
            if (mSelectFilter->isSelected(i))
            {
                (*mPointCloud)[i].color(color);
            }
        }
        mPointCloud->mode(mPointCloud->mode() | PointCloud3d::Mode::COLOR);
        mSimplifiedPointCloud->update();
    }
    else if (mSelectFilter->mode() == SelectFilter::SelectMode::PLANE)
    {
        for (int i = mPointCloud->geometry()->numPlanes() - 1; i >= 0; i--)
        {
            if (mSelectFilter->isSelected(i))
            {
                mPointCloud->geometry()->plane(i)->color(color);
            }
        }
    }
    else if (mSelectFilter->mode() == SelectFilter::SelectMode::CYLINDER)
    {
        for (int i = mPointCloud->geometry()->numCylinders() - 1; i >= 0; i--)
        {
            if (mSelectFilter->isSelected(i))
            {
                mPointCloud->geometry()->cylinder(i)->color(color);
            }
        }
    }
    updatePointCloud();
    QApplication::setOverrideCursor(Qt::ArrowCursor);
}

void MainWindow::editRemove()
{
    if (mPointCloud == NULL || !mSelectFilter->isAnySelected()) return;
    QApplication::setOverrideCursor(Qt::WaitCursor);
    addState();
    if (mSelectFilter->mode() == SelectFilter::SelectMode::POINT)
    {
        for (int i = mPointCloud->size() - 1; i >= 0; i--)
        {
            if (mSelectFilter->isSelected(i))
            {
                mPointCloud->remove(i);
            }
        }
        mSimplifiedPointCloud->update();
    }
    else if (mSelectFilter->mode() == SelectFilter::SelectMode::PLANE)
    {
        for (int i = mPointCloud->geometry()->numPlanes() - 1; i >= 0; i--)
        {
            if (mSelectFilter->isSelected(i))
            {
                mPointCloud->geometry()->removePlane(i);
            }
        }
    }
    else if (mSelectFilter->mode() == SelectFilter::SelectMode::CYLINDER)
    {
        for (int i = mPointCloud->geometry()->numCylinders() - 1; i >= 0; i--)
        {
            if (mSelectFilter->isSelected(i))
            {
                mPointCloud->geometry()->removeCylinder(i);
            }
        }
    }
    else if (mSelectFilter->mode() == SelectFilter::SelectMode::CONNECTION)
    {
        for (int i = mPointCloud->geometry()->numConnections() - 1; i >= 0; i--)
        {
            if (mSelectFilter->isSelected(i))
            {
                mPointCloud->geometry()->removeConnection(i);
            }
        }
    }
    mSelectFilter->update(mPointCloud->size(), mPointCloud->geometry()->numPlanes(), mPointCloud->geometry()->numCylinders(), mPointCloud->geometry()->numConnections());
    updatePointCloud();
    QApplication::setOverrideCursor(Qt::ArrowCursor);
}

void MainWindow::editRemoveFromPrimitives()
{
    if (mPointCloud == NULL || !mSelectFilter->isAnySelected()) return;
    QApplication::setOverrideCursor(Qt::WaitCursor);
    addState();
    if (mSelectFilter->mode() == SelectFilter::SelectMode::POINT)
    {
        std::vector<bool> removed(mPointCloud->size(), false);
        for (size_t i = 0; i < mPointCloud->size(); i++)
        {
            if (mSelectFilter->isSelected(i))
            {
                removed[i] = true;
            }
        }
        PlaneDetector *planeDetector = new PlaneDetector(mPointCloud);
        for (size_t i = 0; i < mPointCloud->geometry()->numPlanes(); i++)
        {
            Plane *plane = mPointCloud->geometry()->plane(i);
            std::vector<size_t> inliers = plane->inliers();
            inliers.erase(std::remove_if(inliers.begin(), inliers.end(), [&](const size_t &inlier) {
                return removed[inlier];
            }), inliers.end());
            if (plane->inliers().size() > inliers.size())
            {
                plane->inliers(inliers);
                planeDetector->delimitPlane(plane);
            }
        }
        delete planeDetector;
        for (size_t i = 0; i < mPointCloud->geometry()->numCylinders(); i++)
        {
            Cylinder *cylinder = mPointCloud->geometry()->cylinder(i);
            std::vector<size_t> inliers = cylinder->inliers();
            inliers.erase(std::remove_if(inliers.begin(), inliers.end(), [&](const size_t &inlier) {
                return removed[inlier];
            }), inliers.end());
            if (cylinder->inliers().size() > inliers.size())
            {
                Eigen::Matrix3Xf points(3, inliers.size());
                for (size_t j = 0; j < inliers.size(); j++)
                {
                    points.col(j) = mPointCloud->at(inliers[j]).position();
                }
                cylinder->inliers(inliers);
                cylinder->leastSquares(points);
            }
        }
        updatePrimitives();
        selectNone();
    }
}

void MainWindow::editUndo()
{
    if (mEditStates.empty()) return;
    QApplication::setOverrideCursor(Qt::WaitCursor);
    size_t levels = mSimplifiedPointCloud->levels();
    delete mPointCloud;
    delete mSimplifiedPointCloud;
    bool updateCamera = !(mPointCloud->extension() == mEditStates[mEditStates.size() - 1]->extension());
    mPointCloud = mEditStates[mEditStates.size() - 1];
    mPointCloud->update();
    mEditStates.erase(mEditStates.begin() + mEditStates.size() - 1);
    mEditUndo->setEnabled(!mEditStates.empty());
    mSimplifiedPointCloud = new SimplifiedPointCloud(mPointCloud, levels);
    mSimplifiedPointCloud->update();
    mSelectFilter->update(mPointCloud->size(), mPointCloud->geometry()->numPlanes(), mPointCloud->geometry()->numCylinders(), mPointCloud->geometry()->numConnections());

    // update camera
    if (updateCamera)
    {
        Rect3d extension(mPointCloud->center().array() - mPointCloud->extension().maxSize() / 2, mPointCloud->center().array() + mPointCloud->extension().maxSize() / 2);
        mScene->camera().setExtension(extension);
        mScene->camera().reset();
    }

    updatePointCloud();
    QApplication::setOverrideCursor(Qt::ArrowCursor);
}

void MainWindow::displayPointCloud()
{
    if (mPointCloud == NULL) return;
    if (mDisplayPointCloudMenu->isChecked())
    {
        mScene->addObject(mPointCloudDrawer);
    }
    else
    {
        mScene->removeObject(mPointCloudDrawer);
    }
    mSceneWidget->update();
}

void MainWindow::displayNormals()
{
    if (mPointCloud == NULL) return;
    if (mDisplayNormalsMenu->isChecked())
    {
        mScene->addObject(mNormalDrawer);
    }
    else
    {
        mScene->removeObject(mNormalDrawer);
    }
    mSceneWidget->update();
}

void MainWindow::displayGaussianMap()
{
    if (mPointCloud == NULL) return;
    if (mDisplayGaussianMapMenu->isChecked())
    {
        mGaussianMapWidget->setVisible(true);
        mGaussianMapWidget->scene()->camera().reset();
        mLayout->addWidget(mGaussianMapWidget);
    }
    else
    {
        mGaussianMapWidget->setVisible(false);
        mLayout->removeWidget(mGaussianMapWidget);
    }
}

void MainWindow::displayProjection()
{
    if (mPointCloud == NULL || !mPointCloud->hasConnectivity()) return;
    bool ok;
    std::string currentPlane = "";
    if (mProjectionPlane != NULL)
    {
        currentPlane += std::to_string(mProjectionPlane->normal().x()) + "," + std::to_string(mProjectionPlane->normal().y()) + "," + std::to_string(mProjectionPlane->normal().z());
    }
    QString input = QInputDialog::getText(this, "Project point cloud", "Projection plane:", QLineEdit::Normal, currentPlane.c_str(), &ok);
    if (!ok) return;
    std::string str = input.toStdString();
    if (str.empty())
    {
        if (mProjectionPlane != NULL)
        {
            delete mProjectionPlane;
        }
        mProjectionPlane = NULL;
    }
    else
    {
        std::vector<std::string> plane = split(str, ',');
        if (plane.size() != 3)
        {
            QMessageBox messageBox;
            messageBox.critical(this, "Error", "Invalid projection plane");
            return;
        }
        float x = atof(plane[0].c_str());
        float y = atof(plane[1].c_str());
        float z = atof(plane[2].c_str());
        if (mProjectionPlane != NULL)
        {
            delete mProjectionPlane;
        }
        mProjectionPlane = new Plane(Eigen::Vector3f::Zero(), Eigen::Vector3f(x, y, z).normalized());
    }

    if (mProjectionPlane != NULL)
    {
        Eigen::Vector3f basisU, basisV;
        GeometryUtils::orthogonalBasis(mProjectionPlane->normal(), basisU, basisV);
        for (size_t i = 0; i < mPointCloud->size(); i++)
        {
            Eigen::Vector3f position = mPointCloud->at(i).position();
            Eigen::Vector2f positionProjected = GeometryUtils::projectOntoOrthogonalBasis(position, basisU, basisV);
            position = Eigen::Vector3f(positionProjected.x(), 1, positionProjected.y());
            (*mPointCloud)[i].position(position);
        }
        delete mSimplifiedPointCloud;
        mSimplifiedPointCloud = new SimplifiedPointCloud(mPointCloud);
        mPointCloud->update();
        updatePointCloud();
    }
}

void MainWindow::displayNextGroup()
{
    if (mPointCloud == NULL || !mPointCloud->hasConnectivity()) return;
    std::set<size_t> groups = mPointCloud->connectivity()->groups();
    if (mCurrentComponent == std::numeric_limits<size_t>::max())
    {
        mCurrentComponent = *groups.begin();
    }
    bool found = false;
    for (auto it = ++groups.find(mCurrentComponent); it != groups.end(); ++it)
    {
        if (mPointCloud->connectivity()->pointsInGroup(*it).size() / (float)mPointCloud->size() > 0.001f)
        {
            mCurrentComponent = *it;
            found = true;
            break;
        }
    }
    if (!found) mCurrentComponent = std::numeric_limits<size_t>::max();
    mPointCloudDrawer->setComponent(mCurrentComponent);
    mPointCloudDrawer->update();
    mNormalDrawer->setComponent(mCurrentComponent);
    mNormalDrawer->update();
    mSceneWidget->update();
    mGaussianMapDrawer->setComponent(mCurrentComponent);
    mGaussianMapDrawer->update();
    mGaussianMapWidget->update();
}

void MainWindow::displayColors()
{
    if (mPointCloud == NULL) return;
    mPointCloudDrawer->colorMode(PointCloudDrawer::ColorMode::DEFAULT);
    mPointCloudDrawer->update();
    mSceneWidget->update();
}

void MainWindow::displayIntensities()
{
    if (mPointCloud == NULL) return;
    mPointCloudDrawer->colorMode(PointCloudDrawer::ColorMode::INTENSITIES);
    mPointCloudDrawer->update();
    mSceneWidget->update();
}

void MainWindow::displayCurvatures()
{
    if (mPointCloud == NULL) return;
    mPointCloudDrawer->colorMode(PointCloudDrawer::ColorMode::CURVATURES);
    mPointCloudDrawer->update();
    mSceneWidget->update();
}

void MainWindow::displayComponents()
{
    if (mPointCloud == NULL) return;
    mPointCloudDrawer->colorMode(PointCloudDrawer::ColorMode::CONNECTED_COMPONENTS);
    mPointCloudDrawer->update();
    mSceneWidget->update();
    if (mPointCloud->hasConnectivity())
    {
        statusBar()->showMessage(("#Connected components: " + std::to_string(mPointCloud->connectivity()->numGroups())).c_str());
    }
}

void MainWindow::displayPlanes()
{
    if (mDisplayPlanesMenu->isChecked())
    {
        mScene->addObject(mPlaneDrawer);
    }
    else
    {
        mScene->removeObject(mPlaneDrawer);
    }
    mSceneWidget->update();
}

void MainWindow::displayCylinders()
{
    if (mDisplayCylindersMenu->isChecked())
    {
        mScene->addObject(mCylinderDrawer);
    }
    else
    {
        mScene->removeObject(mCylinderDrawer);
    }
    mSceneWidget->update();
}

void MainWindow::displayGeometry()
{
    mPlaneDrawer->displayGeometry(mDisplayGeometryMenu->isChecked());
    mCylinderDrawer->displayGeometry(mDisplayGeometryMenu->isChecked());
    updatePrimitives();
    mPointCloudDrawer->displayGeometry(mDisplayGeometryMenu->isChecked());
    mPointCloudDrawer->update();
    mSceneWidget->update();
}

void MainWindow::displayBoundingBox()
{
    if (mDisplayBoundingBoxMenu->isChecked())
    {
        mScene->addObject(mBoundingBoxDrawer);
    }
    else
    {
        mScene->removeObject(mBoundingBoxDrawer);
    }
    mSceneWidget->update();
}

void MainWindow::displayOptions()
{
    if (mPointCloud == NULL) return;
    mViewOptionsDialog.backgroundColor(mScene->clearColor());
    mViewOptionsDialog.defaultPointColor(mPointCloudDrawer->defaultColor());
    mViewOptionsDialog.levels(mSimplifiedPointCloud->levels());
    mViewOptionsDialog.pointSize(mPointCloudDrawer->pointSize());
    mViewOptionsDialog.show();
}

void MainWindow::setViewOptions(float bx, float by, float bz, float dx, float dy, float dz,
                                size_t levels, float pointSize)
{
    bool changed = false;
    if (levels != mSimplifiedPointCloud->levels())
    {
        mSimplifiedPointCloud->levels(levels);
        mSimplifiedPointCloud->update();
        mSelectFilter->update(mPointCloud->size(), mPointCloud->geometry()->numPlanes(), mPointCloud->geometry()->numCylinders(), mPointCloud->geometry()->numConnections());
        changed = true;
    }
    if (changed ||
            mPointCloudDrawer->defaultColor() != Eigen::Vector3f(dx, dy, dz) ||
            mPointCloudDrawer->pointSize() != pointSize)
    {
        mPointCloudDrawer->defaultColor(Eigen::Vector3f(dx, dy, dz));
        mPointCloudDrawer->pointSize(pointSize);
        mPointCloudDrawer->update();
    }
    mScene->clearColor(bx, by, bz);
    mSceneWidget->update();
}

void MainWindow::enablePointCloudChange(bool enabled)
{
    mOpenPointCloudMenu->setEnabled(enabled);
    mSavePointCloudMenu->setEnabled(enabled);
    mImportConnectivityMenu->setEnabled(enabled);
    mImportGeometryMenu->setEnabled(enabled);
    mExportConnectivityMenu->setEnabled(enabled);
    mExportGeometryMenu->setEnabled(enabled);
    mSelectMenu->setEnabled(enabled);
    mEditMenu->setEnabled(enabled);
    mToolsMenu->setEnabled(enabled);
    mPointCloudMenu->setEnabled(enabled);
}

void MainWindow::workerStart(Worker *worker)
{
    enablePointCloudChange(false);
    mProgressBar->setVisible(true);
    mProgressBar->setValue(0);
    mStopWorkerButton->setVisible(true);
    worker->run();
    mRunningWorker = worker;
}

void MainWindow::workerStop()
{
    if (mRunningWorker == NULL) return;
    mRunningWorker->stop();
}

void MainWindow::workerFinish()
{
    mProgressBar->setVisible(false);
    mStopWorkerButton->setVisible(false);
    enablePointCloudChange(true);
    mSceneWidget->update();
    mRunningWorker = NULL;
}

void MainWindow::workerStatus(const QString &status)
{
    mWorkerStatus = status;
    statusBar()->showMessage(status.toStdString().c_str());
}

void MainWindow::workerProgress(float progress)
{
    statusBar()->showMessage(mWorkerStatus.toStdString().c_str());
    mProgressBar->setValue(static_cast<int>(progress * 100));
}

void MainWindow::setPointCloud()
{
    if (mPointCloudIOWorker->pointCloud() == NULL) return;
    mPointCloud = mPointCloudIOWorker->pointCloud();
    mSimplifiedPointCloud = mPointCloudIOWorker->simplifiedPointCloud();
    statusBar()->showMessage(("Done in " + std::to_string(mPointCloudIOWorker->timeElapsedInSec()) + "s. Number of points:" + std::to_string(mPointCloud->size()) + ".").c_str());

    mSimplifiedPointCloud->update();
    // update camera
    Rect3d extension(mPointCloud->center().array() - mPointCloud->extension().maxSize() / 2, mPointCloud->center().array() + mPointCloud->extension().maxSize() / 2);
    mScene->camera().setExtension(extension);
    mScene->camera().reset();

    mSelectFilter->clear();
    mSelectFilter->update(mPointCloud->size(), mPointCloud->geometry()->numPlanes(), mPointCloud->geometry()->numCylinders(), mPointCloud->geometry()->numConnections());

    updatePointCloud();
}

void MainWindow::setConnectivity()
{
    if (mPointCloudIOWorker->connectivity() == NULL) return;
    mPointCloud->connectivity(mPointCloudIOWorker->connectivity());
    updatePointCloud();
}

void MainWindow::setGeometry()
{
    if (mPointCloudIOWorker->geometry() == NULL) return;
    mPointCloud->geometry(mPointCloudIOWorker->geometry());
    for (size_t i = 0; i < mPointCloud->geometry()->numPlanes(); i++)
    {
        std::cout << i << "/" << mPointCloud->geometry()->numPlanes() << std::endl;
        mPointCloud->geometry()->plane(i)->color(ColorUtils::colorWheel(360 / float(mPointCloud->geometry()->numPlanes()) * i));
    }
    for (size_t i = 0; i < mPointCloud->geometry()->numCylinders(); i++)
    {
        mPointCloud->geometry()->cylinder(i)->color(ColorUtils::colorWheel(360 / float(mPointCloud->geometry()->numCylinders()) * i));
    }
    updatePointCloud();
}

void MainWindow::loadPointCloudFinish()
{
    statusBar()->showMessage(("Done in " + std::to_string(mPointCloudIOWorker->timeElapsedInSec()) + "s.").c_str());
    workerFinish();

    switch (mPointCloudIOWorker->mode())
    {
    case PointCloudIOWorker::Mode::LOAD_POINT_CLOUD:
        setPointCloud();
        break;
    case PointCloudIOWorker::Mode::LOAD_CONNECTIVITY:
        setConnectivity();
        break;
    case PointCloudIOWorker::Mode::LOAD_GEOMETRY:
        setGeometry();
        break;
    case PointCloudIOWorker::Mode::SAVE_POINT_CLOUD:
        break;
    case PointCloudIOWorker::Mode::SAVE_CONNECTIVITY:
        break;
    case PointCloudIOWorker::Mode::SAVE_GEOMETRY:
        break;
    }

    mSelectFilter->update(mPointCloud->size(), mPointCloud->geometry()->numPlanes(), mPointCloud->geometry()->numCylinders(), mPointCloud->geometry()->numConnections());
}

void MainWindow::loadPointCloud()
{
    if (mPointCloud != NULL)
        clearPointCloud();

    QString filename = QFileDialog::getOpenFileName(this, "Select file", mLastDirectory.c_str(), getExtensions().c_str());
    if (filename.toStdString().empty()) return;

    updateExtensionsAndDirectory(filename.toStdString());

    mPointCloudIOWorker->filename(filename.toStdString());
    mPointCloudIOWorker->mode(PointCloudIOWorker::Mode::LOAD_POINT_CLOUD);
    workerStart(mPointCloudIOWorker);
}

void MainWindow::loadPointCloudError(const QString &error)
{
    QMessageBox messageBox;
    messageBox.critical(this, "Error", error.toStdString().c_str());
    workerFinish();
}

void MainWindow::importConnectivity()
{
    QString filename = QFileDialog::getOpenFileName(this, "Select file", mLastDirectory.c_str(), "Connectivity (*.con);;");
    if (filename.toStdString().empty()) return;

    updateDirectory(filename.toStdString());

    mPointCloudIOWorker->filename(filename.toStdString());
    mPointCloudIOWorker->mode(PointCloudIOWorker::Mode::LOAD_CONNECTIVITY);
    workerStart(mPointCloudIOWorker);
}

void MainWindow::importGeometry()
{
    QString filename = QFileDialog::getOpenFileName(this, "Select file", mLastDirectory.c_str(), "Geometry (*.geo);;");
    if (filename.toStdString().empty()) return;

    updateDirectory(filename.toStdString());

    mPointCloudIOWorker->filename(filename.toStdString());
    mPointCloudIOWorker->mode(PointCloudIOWorker::Mode::LOAD_GEOMETRY);
    workerStart(mPointCloudIOWorker);
}

void MainWindow::savePointCloud()
{
    QString filename = QFileDialog::getSaveFileName(this, "Save file", mLastDirectory.c_str(), getExtensions().c_str());
    if (filename.toStdString().empty()) return;

    updateExtensionsAndDirectory(filename.toStdString());

    mPointCloudIOWorker->filename(filename.toStdString());
    mPointCloudIOWorker->pointCloud(mPointCloud);
    mPointCloudIOWorker->mode(PointCloudIOWorker::Mode::SAVE_POINT_CLOUD);
    workerStart(mPointCloudIOWorker);
}

void MainWindow::exportConnectivity()
{
    QString filename = QFileDialog::getSaveFileName(this, "Select file", mLastDirectory.c_str(), "Connectivity (*.con);;");
    if (filename.toStdString().empty()) return;

    updateDirectory(filename.toStdString());

    mPointCloudIOWorker->filename(filename.toStdString());
    mPointCloudIOWorker->connectivity(mPointCloud->connectivity());
    mPointCloudIOWorker->mode(PointCloudIOWorker::Mode::SAVE_CONNECTIVITY);
    workerStart(mPointCloudIOWorker);
}

void MainWindow::exportGeometry()
{
    QString filename = QFileDialog::getSaveFileName(this, "Select file", mLastDirectory.c_str(), "Geometry (*.geo);;");
    if (filename.toStdString().empty()) return;

    updateDirectory(filename.toStdString());

    mPointCloudIOWorker->filename(filename.toStdString());
    mPointCloudIOWorker->geometry(mPointCloud->geometry());
    mPointCloudIOWorker->mode(PointCloudIOWorker::Mode::SAVE_GEOMETRY);
    workerStart(mPointCloudIOWorker);
}

void MainWindow::estimateNormals()
{
    if (mPointCloud == NULL) return;

    QInputDialog dialog;
    QString result = dialog.getText(this, "Number of neighbors", "Neighbors:", QLineEdit::Normal, "21");
    size_t numNeighbors = atoi(result.toStdString().c_str());

    mNormalEstimatorWorker->numNeighbors(numNeighbors);

    workerStart(mNormalEstimatorWorker);
}

void MainWindow::estimateNormalsSlow()
{
   mNormalEstimatorWorker->speed(NormalEstimator3d::Speed::SLOW);
   estimateNormals();
}

void MainWindow::estimateNormalsQuick()
{
    mNormalEstimatorWorker->speed(NormalEstimator3d::Speed::QUICK);
    estimateNormals();
}

void MainWindow::normalEstimationFinish()
{
    mPointCloud->mode(mPointCloud->mode() | PointCloud3d::Mode::NORMAL | PointCloud3d::Mode::NORMAL_CONFIDENCE | PointCloud3d::Mode::CURVATURE);
    mSimplifiedPointCloud->update();
    mNormalDrawer->update();
    mPointCloudDrawer->update();
    mGaussianMapDrawer->update();
    mGaussianMapWidget->update();
    statusBar()->showMessage(("Done in " + std::to_string(mNormalEstimatorWorker->timeElapsedInSec()) + "s.").c_str());
    workerFinish();
}

void MainWindow::detectPlanes(float minNormal, float maxDist, float outlierRatio)
{
    clearPlanes();
    mPlaneDetectorWorker->detectorParams(minNormal, maxDist, outlierRatio);
    mPlaneDetectorWorker->mode(PlaneDetectorWorker::Mode::AUTODETECT);
    workerStart(mPlaneDetectorWorker);
}

void MainWindow::detectPlanesDialog()
{
    /*if (!mPointCloud->hasConnectivity())
    {
        QMessageBox messageBox;
        messageBox.critical(this, "Error", "Point cloud has no connectivity.");
        return;
    }*/
    mPlaneDetectorDialog.show();
}

void MainWindow::detectPlane()
{
    if (!mPointCloud->hasConnectivity())
    {
        QMessageBox messageBox;
        messageBox.critical(this, "Error", "Point cloud has no connectivity.");
        return;
    }
    if (mSelectFilter->mode() == SelectFilter::SelectMode::POINT && mSelectFilter->isAnySelected())
    {
        std::vector<size_t> points;
        for (size_t i = 0; i < mPointCloud->size(); i++)
        {
            if (mSelectFilter->isSelected(i))
            {
                points.push_back(i);
            }
        }
        mPlaneDetectorWorker->mode(PlaneDetectorWorker::Mode::DETECT_REGION);
        mPlaneDetectorWorker->region(points);
        workerStart(mPlaneDetectorWorker);
    }
}

void MainWindow::mergePlanes()
{
    if (!mPointCloud->hasConnectivity())
    {
        QMessageBox messageBox;
        messageBox.critical(this, "Error", "Point cloud has no connectivity.");
        return;
    }
    if (mSelectFilter->mode() == SelectFilter::SelectMode::PLANE && mSelectFilter->isAnySelected())
    {
        std::vector<size_t> points;
        for (int i = mPointCloud->geometry()->numPlanes() - 1; i >= 0; i--)
        {
            if (mSelectFilter->isSelected(i))
            {
                const Plane *plane = mPointCloud->geometry()->plane(i);
                points.insert(points.end(), plane->inliers().begin(), plane->inliers().end());
                mPointCloud->geometry()->removePlane(i);
            }
        }
        mPlaneDetectorWorker->mode(PlaneDetectorWorker::Mode::DETECT_REGION);
        mPlaneDetectorWorker->region(points);
        workerStart(mPlaneDetectorWorker);
    }
}

void MainWindow::expandPlaneRegion()
{
    if (!mPointCloud->hasConnectivity())
    {
        QMessageBox messageBox;
        messageBox.critical(this, "Error", "Point cloud has no connectivity.");
        return;
    }
    if (mSelectFilter->mode() == SelectFilter::SelectMode::POINT && mSelectFilter->isAnySelected())
    {
        std::vector<size_t> points;
        for (size_t i = 0; i < mPointCloud->size(); i++)
        {
            if (mSelectFilter->isSelected(i))
            {
                points.push_back(i);
            }
        }
        mPlaneDetectorWorker->mode(PlaneDetectorWorker::Mode::EXPAND_REGION);
        mPlaneDetectorWorker->region(points);
        workerStart(mPlaneDetectorWorker);
    }
}

void MainWindow::planeDetectionFinish()
{
    QApplication::setOverrideCursor(Qt::WaitCursor);
    statusBar()->showMessage(("Done in " + std::to_string(mPlaneDetectorWorker->timeElapsedInSec()) + "s.").c_str());
    //addState();
    switch (mPlaneDetectorWorker->mode())
    {
    case PlaneDetectorWorker::Mode::AUTODETECT:
        std::cout << "Done in " + std::to_string(mPlaneDetectorWorker->timeElapsedInSec()) + "s." << std::endl;
        statusBar()->showMessage(("Done in " + std::to_string(mPlaneDetectorWorker->timeElapsedInSec()) + "s. Number of planes: " + std::to_string(mPlaneDetectorWorker->planes().size()) + ".").c_str());
        mPointCloud->geometry()->clearPlanes();
        for (Plane *plane : mPlaneDetectorWorker->planes())
        {
            plane->color(ColorUtils::colorWheel(rand() % 360));
            mPlaneDetectorWorker->detector()->delimitPlane(plane);
            mPointCloud->geometry()->addPlane(plane);
        }
        mPlaneDrawer->update();
        mPointCloudDrawer->update();
        mSceneWidget->update();
        break;
    case PlaneDetectorWorker::Mode::EXPAND_REGION:
        mSelectFilter->clearSelection();
        for (const size_t &point : mPlaneDetectorWorker->region())
        {
            mSelectFilter->select(point);
        }
        mPointCloudDrawer->update();
        mSceneWidget->update();
        break;
    case PlaneDetectorWorker::Mode::DETECT_REGION:
        Plane *plane = mPlaneDetectorWorker->planes()[0];
        if (plane != NULL)
        {
            mSelectFilter->clearSelection();
            plane->color(ColorUtils::colorWheel(rand() %  360));
            mPointCloud->geometry()->addPlane(plane);
            mPlaneDrawer->update();
            mPointCloudDrawer->update();
            mSceneWidget->update();
        }
        break;
    }
    mSelectFilter->update(mPointCloud->size(), mPointCloud->geometry()->numPlanes(), mPointCloud->geometry()->numCylinders(), mPointCloud->geometry()->numConnections());
    workerFinish();
    QApplication::setOverrideCursor(Qt::ArrowCursor);
}

void MainWindow::clearPrimitives()
{
    clearPlanes();
    clearCylinders();
}

void MainWindow::clearPlanes()
{
    mPointCloud->geometry()->clearPlanes();
    mPlaneDrawer->update();
    mPointCloudDrawer->update();
}

void MainWindow::clearCylinders()
{
    mPointCloud->geometry()->clearCylinders();
    mPlaneDrawer->update();
    mPointCloudDrawer->update();
}

void MainWindow::pointCloudInfo()
{
    if (mPointCloud == NULL) return;
    mInfoDialog.show();
}

void MainWindow::updateDirectory(const std::string &filename)
{
    mLastDirectory = filename.substr(0, filename.find_last_of('/') + 1);
}

void MainWindow::updateExtensionsAndDirectory(const std::string &filename)
{
    mLastDirectory = filename.substr(0, filename.find_last_of('/') + 1);
    mLastFilename = filename.substr(filename.find_last_of('/') + 1);
    setWindowTitle((mLastFilename + " - Point Cloud Editor").c_str());
    std::string extension = filename.substr(filename.find_last_of('.') + 1);
    for (size_t i = 0; i < mSupportedExtensions.size(); i++)
    {
        if (extension == mSupportedExtensions[i].first)
        {
            std::pair<std::string, std::string> temp = mSupportedExtensions[i];
            mSupportedExtensions.erase(mSupportedExtensions.begin() + i);
            mSupportedExtensions.insert(mSupportedExtensions.begin(), temp);
            break;
        }
    }
}

std::string MainWindow::getExtensions()
{
    std::string extensions = "";
    for (size_t i = 0; i < mSupportedExtensions.size(); i++)
    {
        extensions += mSupportedExtensions[i].second + " (*." + mSupportedExtensions[i].first + ");;";
    }
    return extensions;
}
