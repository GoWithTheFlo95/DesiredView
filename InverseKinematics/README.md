# Inverse Kinematics

The project *Inverse Kinematics* computes and delivers joint values for a 6 Degrees of Freedom (DOF) C-arm system, by using the table-to-end-effector Transformation Matrix in order to find close-form solutions, implemented by L. Wang et al.



## Content

The project folder **Inverse Kinematics** contains the "build" folder, created by CMake, the "src" folder, containing the soucre code and the CMakeLists.txt file that links all the required libraries, and the CMakeLists.txt file, defining the subdirectories.





## Getting Started



Download the project folder and place it in a certain directory (- paths have to be set in the code). Install the software development tool CMake and the open-source library Eigen (as it has already been used for other code, the PCL v1.8.1 library is used. It contains the Eigen library as well).
Set the required directories in CMake and configure, generate and "Open Project" the project. Build and run the project InverseKinematics as a StartUp Project. The main.cpp implements the entire workflow.



### Prerequisites

The **Inverse Kinematics** depends on [CMake](https://cmake.org/) and [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). As the PCL library contains the Eigen library the [PointCloudLibrary 1.8.1](http://unanancyowen.com/en/pcl181/) is used to acces the Eigen library.

```How to install each library is explained on each website. Links are attached to the libraries above or below.
```







## Built With

* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page) - Library to process calculations with matrices (linear algebra)
* [PointCloudLibrary 1.8.1](http://unanancyowen.com/en/pcl181/) - Library to process CTscan and Kinect depth image







## Author



* **Florian Hermes** - *Bachelor's Thesis at University of Ottawa*







## Acknowledgments



* Thanks to my supervisor Prof. Dr. R. Willenberg of the University of Applied Sciences and my advisor Prof. Dr. P. Fallavollita of the University of Ottawa.