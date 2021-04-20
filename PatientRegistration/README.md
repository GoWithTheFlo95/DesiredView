# Patient Registration

The **Patient Registration** is a workflow, determining the position of a patient on the OR's table in the beginning of a surgery, by using a depth image of the Microsoft Kinect, the pre-operatively acquired CT scan, the open-source library PointCloudLibrary (PCL) and the matching algorithm CoherentPointDrift (CPD). It is developed and run in the IDE: Microsoft Visual Studios and coded in C++.



## Content

The project folder **Patient Registration** contains the "build" folder, created by CMake, the "src" folder, containing the soucre code and the CMakeLists.txt file that links all the required libraries, the CMakeLists.txt file, defining the subdirectories, and all the PLY files of the segmented CT scan (skinMesh.ply) and the KinectPointClouds (kinect1.ply, kinect1_2.ply, kinect2.ply, kinect3.ply).





## Getting Started



Download the project folder and place it in a certain directory (- Paths have to be set in the code). Install the software development tool CMake and the open-source libraries KinectforWindows SDK v2.0, PCL v1.8.1, SDL v2.0 and the C++ CPD implementation by P.Gadomski.
Set the required directories in CMake and configure, generate and "Open Project" the project. Build and run the project DesiredView as a StartUp Project. The main.cpp implements the entire workflow.



### Prerequisites

The **PatientRegistration** depends on [CMake](https://cmake.org/), [
KinectforWindowsSDK v2.0](https://www.microsoft.com/en-us/download/details.aspx?id=44561), [SDL2.0](https://www.libsdl.org/download-2.0.php), [PointCloudLibrary 1.8.1](http://unanancyowen.com/en/pcl181/), and the [CPD implemenation by P.Gadomski](https://github.com/gadomski/cpd).

```How to install each library is explained on each website. Links are attached to the libraries above or below.
```







## Built With



* [
KinectforWindowsSDK v2.0](https://www.microsoft.com/en-us/download/details.aspx?id=44561) - Library to access the Kinect's sensor and camera
* [SDL2.0](https://www.libsdl.org/download-2.0.php) - Library to visualize the stream of depth frames

* [PointCloudLibrary 1.8.1](http://unanancyowen.com/en/pcl181/) - Library to process CTscan and Kinect depth image
* [CPD implemenation by P.Gadomski](https://github.com/gadomski/cpd) - Library to match the two datasets







## Author



* **Florian Hermes** - *Bachelor's Thesis at University of Ottawa*







## Acknowledgments



* Thanks to my supervisor Prof. Dr. R. Willenberg of the University of Applied Sciences and my advisor Prof. Dr. P. Fallavollita of the University of Ottawa.

* Thanks to P. Gadomski for providing the C++ implementation of the CPD algorithm and for answering and explaining to me 

* Thanks to the PCL User mailing list. A forum created for PCL users to ask questions and get help.