/**
  ***********************************************************************************
  * @file   :Camera.h
  * @author :Tinker.Jia
  * @version:1.0
  * @date   :2019.6.28
  * @brief  :Init the PointGrey and get image.
  ***********************************************************************************
  * @attention
  * None
  ***********************************************************************************
  */
#include "Header.h"

#ifndef CAMERA_H
#define CAMERA_H
class PointGreyCamera{
public:


private:
    FlyCapture2::Error error;
public:
    void PointGrey_Init(void);
    void GetImageThread(vector<GpuMat> &Global_Img);
    int Camera_FPS_Detect(void);

private:
    void PrintBuildInfo(void);
    void PrintCameraInfo(CameraInfo *pCamInfo);
    void PrintError(FlyCapture2::Error error);

    int RunSingleCamera(PGRGuid guid);
};

#endif
