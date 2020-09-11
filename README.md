
## About

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

Access our homepage to get access to the paper pdf and datasets:

https://www.inf.ufrgs.br/~oliveira/pubs_files/RE/RE.html

## Usage 

There are two options to use our algorithm. We provide one through a Graphic Interface and another through a Command Line. See sections below for more details.

### Command Line 

The command line option is available in the `CommandLineOption` directory. There are no dependencies, just call `make` to compile the program.

### Graphical Interface 

#### !! Important !!

**Before estimating a plane (On Plane Detector > Detect planes), you need to estimate normals first (Plane Detector > Estimate normals).**

#### Install 

This is a regular Qt project with no additional dependencies. We use Eigen, but this is bundled in our code. Open the project on Qt design and you should be able to compile it right away.

The actual article implementation is located here:

`DetectionLib/planedetector.cpp`

#### Project structure

Basically, this project is structured in four main subprojects:
- CoreLib (contains the main classes such as Point, PointCloud, Plane...)
- DetectionLib (this is where the plane detection algorithm is implemented, see class PlaneDetector)
- GraphicsLib (OpenGL utilities used by the UI)
- PointCloudEditor (the UI. Run it to Load a point cloud, visualize it, detect planes and visualize detected planes, among many other possibilities...) 

## Evaluating technique performance 

Besides the Graphical Interface and Command Line options, this repository also contains a small cpp file used to calculate metrics (F1 score, recall, precision, etc.) for a given detection. Please, refer to: 

`compare_plane_detector`

