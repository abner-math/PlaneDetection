
## About

![](https://www.inf.ufrgs.br/~oliveira/pubs_files/RE/teaser.png)

This is the implementation of the following article:

```
@article{AraujoOliveira_2020a,
    author  = {Abner M. Araujo and Manuel M. Oliveira},
    title   = {A Robust Statistics Approach for Plane Detection in Unorganized Point Clouds},
    journal = {Pattern Recognition},
    volume  = {100},
    DOI     = {10.1016/j.patcog.2019.107115},
    ISSN    = {0031-3203},
    pages   = {107115},
    year    = {2020}
}
```

If case you use it in your research, please cite our work.

Access our homepage to get access to the paper and datasets:

https://www.inf.ufrgs.br/~oliveira/pubs_files/RE/RE.html

## Usage 

There are two interfaces to use our algorithm: a graphical interface and a command line. See the sections below for more details.

### Command Line 

The command line is available in the `CommandLineOption` directory. There are no external dependencies, just call `make` to compile the project.

### Graphical Interface 

#### !! Important !!

**Before estimating a plane (in Plane Detector > Detect planes), you need to estimate normals (Plane Detector > Estimate normals).**

#### Install 

This is a regular Qt project with no external dependencies. We use Eigen, but this is bundled in our code. Once the project is opened in Qt design you should be able to compile it right away.

The actual article implementation is located here:

`DetectionLib/planedetector.cpp`

#### Project structure

This project is structured into four main subprojects:
- CoreLib (contains the main classes such as Point, PointCloud, Plane...)
- DetectionLib (this is where the plane detection algorithm is implemented, see the PlaneDetector class)
- GraphicsLib (OpenGL utilities used by the GUI)
- PointCloudEditor (the GUI. Run it to Load a point cloud, render it, detect planes, and visualize detected planes, among many other possibilities...) 

## Evaluating technique performance 

Besides the Graphical Interface and Command Line, this repository also contains a small cpp file used to calculate some metrics (F1 score, recall, precision, etc.) for a given detection. Please, refer to: 

`compare_plane_detector`

