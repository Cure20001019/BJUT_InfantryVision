TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES     +=  main.cpp\
                Src/ArmorDetect.cpp\
                Src/ArmorPose.cpp\
                Src/Camera.cpp\
                Src/Serial.cpp\

INCLUDEPATH +=  D:\OpenCV\opencv\opencv-build_64\install\include\
                D:\NVIDIA\NVIDIA GPU Computing Toolkit\CUDA\v11.2\include
#                /usr/local/cuda/include\
#                /usr/local/cuda-9.0/include\
#                /usr/local/include\
#                /home/dji/flycapture.2.13.3.31_arm64/include

LIBS        +=  D:\OpenCV\opencv\opencv-build_64\lib\libopencv_*.dll.a\
                D:\NVIDIA\NVIDIA GPU Computing Toolkit\CUDA\v11.2\lib\x64
#                /usr/lib/aarch64-linux-gnu/libopencv_*\
#                /usr/lib/libopencv_*\
#                /usr/local/cuda/lib64/lib*\
#                /usr/local/cuda-9.0/lib64/lib*\
#                /usr/lib/libflycapture*\
#                -lpthread


HEADERS     +=  Header.h\
                Inc/ArmorDetect.h\
                Inc/ArmorPose.h\
                Inc/Camera.h\
                Inc/Serial.h\

