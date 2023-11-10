

<picture>
  <source media="(prefers-color-scheme: dark)" srcset="https://github.com/Minus-Monster/Qylon/blob/main/Qylon_W.png">
  <source media="(prefers-color-scheme: light)" srcset="https://github.com/Minus-Monster/Qylon/blob/main/Qylon_B.png">
  <img alt="Qylon is Qt ported pylon" src="https://github.com/Minus-Monster/Qylon/blob/main/Qylon_W.png">
</picture>

# **Qylon**

This project is made for better experiences of Basler pylon API with Qt platform.

The purpose of the end is going to make the pylon into an All-Round Image Processing Library.
Consequently, this project will merge with vTools functions and QDC Engine which was created from DATVISION.

The owner of this project is [Minu Park](minu.park@baslerweb.com) from Basler Korea Inc.

If you have any questions or purposes, Please feel free to contact me via e-mail.

### Structure

```
Qylon
├── Acquisition modules
│   ├── Camera (Almost Implemented, started ToF supporting)
│   ├── Frame Grabber (Working, Waiting for enough resources)
│   └── From Image
│
└── Processing modules
    ├── vTools
    └── QDC (OpenCV, AVL, etc)

``` 

### Environment

You can download Basler pylon from the [website](https://baslerweb.com/).
This project was generated by [Qt 5.15.2](https://qt.io/).

---

## How to use

You can add Qylon to your .pro file. 

```c++
include('the/path/of/Qylon/Qylon.pri')
```

Or you can build using CMake.
Adding the below codes in your CMakeLists.txt.
(This is not completely finished. Frame Grabber doesn't work properly at this moment.)
```c++
add_subdirectory("../Qylon" build/Qylon)
target_link_libraries(${PROJECT_NAME} PUBLIC Qylon)
```


In your main function, you can declare this code as below. 

```C++
#include <Qylon.h>
    
Qylon::Qylon qy;
Qylon::Camera *qylonCamera = qy.addCamera();
QObject::connect(qylonCamera, &Qylon::Camera::grabbed, &yourMainwindow, [=](){
    qylonCamera->drawLock();
    QImage resultImage = qylonCamera.getImage();
}
```

- Connect to the camera
```c++
Qylon::Qylon qy;
Qylon::Camera *camera = qy.addCamera();
camera->openCamera("Camera's Friendly Name");
camera->continuousGrab();
```

- Multi-cameras setting
```c++
Qylon::Qylon qy;
Qylon::Camera *camera1 = qy->addCamera();
Qylon::Camera *camera2 = qy->addCamera();

camera1->openCamera("Camera's Friendly Name");
camera2->openCamera("Camera's Friendly Name");
```

- Configurations via GUI
```c++
Qylon::Qylon qy;
Qylon::Camera *camera = qy.addCmaera();
QWidget *widget = camera->getWidget();
widget->show();
```

## How to add Qylon to your project

### Using Git commands

You can add these lines to add Qylon module.

```
// Go to your project directory via Terminal.
git submoudle add git@github.com:MinuParkBasler/Qylon.git
git submodule init
git submodule update
```
Now you can develop your project using Qylon.

### Using Downloading codes

You can download this project into your computer.
You can refer [How to use](https://github.com/Minus-Monster/Qylon#how-to-use).





Or you have anoher option that I provide an example.
You can get it [here](https://github.com/Minus-Monster/QylonTestFlight).

## To use full functions for a Blaze camera

This code should meet the version of PCL(1.13) and VTK(9.2.6).

You will need to install those first and check your dependencies to compile.

- VTK
  
To build VTK Library, you have got an installed Qt 5.

You should configure the Qt 5 part correctly on cmake configuration when you try to build VTK.

You can check the advanced option on cmake or cmake-gui.

- PCL
  
You will be faced an error if your environment hasn't libusb1.0.
```
sudo apt install libusb-1.0-0-dev
```
You can install libusb with the above instruction.

If you checked WITH_OPENMP, there's the possibility to encounter the error couldn't find omp.h.
```
sudo apt install libomp-dev
```
After installing OpenMP Library, you should build PCL from scratch again by revising CMakeLists.txt as below.
```
// message(STATUS "Found OpenMP, spec data ${OpenMP_CXX_SPEC_DATE}")
// you should put this code under line number 308
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
```


If you have trouble while you're installing PCL, this page(https://pcl.readthedocs.io/projects/tutorials/en/latest/compiling_pcl_posix.html#id2) might help you.

