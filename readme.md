# Brief introduction to Slam2D

## About the project

Slam2D is a project for optimizing a bundle adjustment problem in 2D-SLAM. It includes some source files and a test file named main.cpp in the folder demo.
In the visual odemetry, a camera is restricted to moving in a 2D plane which is parallel to the landmark plane. Therefore, below parameters need to be optimized:
> M landmarks(keypoints): X, Y (total number 2*M)
> N camera pose(frames): x, y (total number 2*N)
> camera intrinsic parameters: focal length, distortion
> camera extrinsic parameters: rotation angle, Z(distance to landmark plane)

## Requirement

> language: c++
> platform: ubuntu 18.04
> IDE: clion
> compile: cmake, gcc
> 3rd-party libraries: ceres(eigen3 needed)

## Structure

### src/include
1. bundle_adjustment.cpp
It includes almost all key codes to accomplish bundle adjustment using ceres.
A class named BundleAdjustment is defined in the file. During the class, below key functions are claimed:
> Add observation
> Compile(build the computational graph)
> Solve(training procedure)
> Report
> Save model
> Save PLY/PCL file to display meshlab/pcl
> etc.

2. model.cpp
It is used to save/read model file and calculate position with new inputs.

### lib
The folder contains shared/static libraries generated after compiling source files.

### demo
The folder contains a demo program which shows how to use Slam2D.

### output
The folder contains all outputs in the demo.

## Using Slam2D
You can almost get started referring to main.cpp in the "demo" folder. Bellow are the main steps:
>1. create an instance
>2. add observations
>3. compile
>4. solve
Optional steps:
>5. print report
>6. save model
>7. save PLY file(meshlab ./output/demo.ply to see the point cloud)

By the way, after compiling source files, you need not copy them but the shared-lib file to any other projects.