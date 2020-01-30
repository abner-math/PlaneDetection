#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QStatusBar>
#include <QProgressBar>

#include "cameracontrolwindow.h"
#include "simplifiedpointcloud.h"
#include "pointclouddrawer.h"
#include "pointcloudioworker.h"
#include "planedrawer.h"
#include "planedetectordialog.h"
#include "planedetectorworker.h"
#include "cylinderdrawer.h"
#include "boundingboxdrawer.h"
#include "selectfilter.h"
#include "normaldrawer.h"
#include "normalestimatorworker.h"
#include "gaussianmapdrawer.h"
#include "pointcloudinfodialog.h"
#include "boundaryvolumehierarchy.h"
#include "viewoptionsdialog.h"
#include "transformtranslatedialog.h"
#include "transformtranslateworker.h"
#include "transformscaledialog.h"
#include "transformscaleworker.h"
#include "transformrotatedialog.h"
#include "transformrotateworker.h"

class MainWindow : public CameraControlWindow
{
    Q_OBJECT
public:
    MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void loadPointCloudFinish();
    void loadPointCloudError(const QString &error);
    void loadPointCloud();
    void importConnectivity();
    void importGeometry();
    void savePointCloud();
    void exportConnectivity();
    void exportGeometry();
    void exit();

    void mouseLeftClick();
    void mouseRightClick();
    void mouseMove(int x, int y);
    void cameraUpdated();

    void selectRegion();
    void deselectRegion();
    void selectAll();
    void selectNone();
    void selectInvert();
    void selectPointsWithHighCurvature();
    void selectionModePoint();
    void selectionModePlane();
    void selectionModeCylinder();
    void selectionModeConnection();

    void transformTranslate();
    void transformScale();
    void transformRotate();
    void editSetColor();
    void editRemove();
    void editUndo();
    void editRemoveFromPrimitives();

    void displayPointCloud();
    void displayNormals();
    void displayGaussianMap();
    void displayNextGroup();
    void displayColors();
    void displayIntensities();
    void displayCurvatures();
    void displayComponents();
    void displayProjection();
    void displayPlanes();
    void displayCylinders();
    void displayGeometry();
    void displayBoundingBox();
    void displayOptions();
    void estimateNormals();
    void estimateNormalsSlow();
    void estimateNormalsQuick();
    void normalEstimationFinish();

    void detectPlanes(float minNormal, float maxDist, float outlierRatio);
    void detectPlanesDialog();
    void detectPlane();
    void expandPlaneRegion();
    void mergePlanes();
    void planeDetectionFinish();

    void pointCloudInfo();
    void workerStop();
    void workerProgress(float progress);
    void workerStatus(const QString &status);
    void setViewOptions(float,float,float,float,float,float,size_t,float);
    void translatePointCloud(float dx, float dy, float dz);
    void translateFinish();
    void scalePointCloud(float dx, float dy, float dz);
    void scaleFinish();
    void rotatePointCloud(float x, float y, float z, float degrees);
    void rotateFinish();

private:
    PointCloud3d *mPointCloud;
    SimplifiedPointCloud *mSimplifiedPointCloud;
    PointCloudDrawer *mPointCloudDrawer;
    PlaneDrawer *mPlaneDrawer;
    CylinderDrawer *mCylinderDrawer;
    PointCloudIOWorker *mPointCloudIOWorker;
    BoundingBoxDrawer *mBoundingBoxDrawer;
    SelectFilter *mSelectFilter;
    NormalDrawer *mNormalDrawer;
    NormalEstimatorWorker *mNormalEstimatorWorker;
    TransformTranslateWorker *mTranslateWorker;
    TransformScaleWorker *mScaleWorker;
    TransformRotateWorker *mRotateWorker;
    PlaneDetectorWorker *mPlaneDetectorWorker;

    QMenu *mFileMenu;
    QAction *mOpenPointCloudMenu;
    QAction *mSavePointCloudMenu;
    QAction *mImportConnectivityMenu;
    QAction *mImportGeometryMenu;
    QAction *mExportConnectivityMenu;
    QAction *mExportGeometryMenu;
    std::string mLastDirectory;
    std::string mLastFilename;
    std::vector<std::pair<std::string, std::string> > mSupportedExtensions;

    QMenu *mSelectMenu;
    QMenu *mEditMenu;
    QMenu *mPointCloudMenu;
    QMenu *mToolsMenu;

    QAction *mDisplayPointCloudMenu;
    QAction *mDisplayNormalsMenu;
    QAction *mDisplayGaussianMapMenu;
    QAction *mDisplayNextGroupMenu;

    QAction *mDisplayPlanesMenu;
    QAction *mDisplayCylindersMenu;
    QAction *mDisplayGeometryMenu;

    QAction *mDisplayBoundingBoxMenu;
    QAction *mDisplayPartitionsMenu;
    QAction *mDisplayPCAMenu;

    QAction *mEditUndo;

    QProgressBar *mProgressBar;
    QPushButton *mStopWorkerButton;

    SceneWidget *mGaussianMapWidget;
    GaussianMapDrawer *mGaussianMapDrawer;

    PlaneDetectorDialog mPlaneDetectorDialog;
    TransformTranslateDialog mTranslateDialog;
    TransformScaleDialog mScaleDialog;
    TransformRotateDialog mRotateDialog;
    PointCloudInfoDialog mInfoDialog;
    ViewOptionsDialog mViewOptionsDialog;

    Worker *mRunningWorker;
    QString mWorkerStatus;

    float mLastValidDepth;
    Eigen::Vector2f mMousePosition;
    Eigen::Vector3f mScreenPosition;
    enum SelectState
    {
        DISABLED = 0,
        FIRST_POINT = 1,
        SECOND_POINT = 2
    };
    SelectState mSelectState;

    size_t mCurrentComponent;
    Plane *mProjectionPlane;

    std::vector<PointCloud3d*> mEditStates;

    void enablePointCloudChange(bool enabled);
    void initMenu();
    void initStatusBar();
    void initGaussianMap();
    void initSceneObjects();
    void clearPointCloud();
    void updatePointCloud();
    void updatePrimitives();
    void setPointCloud();
    void setConnectivity();
    void setGeometry();
    void workerStart(Worker *worker);
    void workerFinish();
    void clearPlanes();
    void clearCylinders();
    void clearPrimitives();
    void updateDirectory(const std::string &filename);
    void updateExtensionsAndDirectory(const std::string &filename);
    std::string getExtensions();
    void selectPoints();
    bool isSelected(const Plane *plane, float minX, float maxX, float minY, float maxY);
    bool isSelected(const Cylinder *cylinder, float minX, float maxX, float minY, float maxY);
    bool isSelected(const Connection *connection, float minX, float maxX, float minY, float maxY);
    void selectPlanes();
    void selectCylinders();
    void selectConnections();
    void addState();

};

#endif // MAINWINDOW_H
