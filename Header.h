/**
  ***********************************************************************************
  * @file   :Header.h
  * @author :Tinker.Jia
  * @version:1.0
  * @date   :2019.6.28
  * @brief  :Init the task
  ***********************************************************************************
  * @attention
  * My English is so bad.The purpose of using English for commentary is to exercise
  * one's writing ability.
  ***********************************************************************************
  */
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/cuda.hpp>

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/cudafilters.hpp>
#include <math.h>
#include <iostream>
#include <vector>
#include <thread>
#include "FlyCapture2.h"


#ifndef HEADER_H
#define HEADER_H


#define ATTACK_RED_ARMOR        0x00
#define ATTACK_BLUE_ARMOR       0xff

#define WAIT_SAVE 0
#define WAIT_DEAL 1

using namespace std;
using namespace cv;
using namespace cuda;
using namespace FlyCapture2;


typedef struct
{
    int Status;
    double time;
}Img_Buff_t;


typedef struct
{
    Img_Buff_t CameraBuff[2];
    int Img_ID;
    Img_Buff_t BinaryBuff[2];
    int Binary_ID;

    bool GetBinary;
    bool GetPose;
}TaskFlag_t;


typedef struct {
    int Thres_Red;
    int Thres_Blue;
    int Thres_Gray;
    int Blue_H_Thres;
    int Red_H_Thres;
    int S_Thres;
    int V_Thres;
    bool Debug;
    bool Display_Image;
    bool Display_Pose;
    bool Display_Recognition_Rate;
    float Mono_small_armor_focus;
    float Mono_big_armor_foucs;
    float Camera_Center_x;
    float Camera_Center_y;
    int Distance_Conpensate;
    int Pitch_Conpensate;
}Params_t;




void Deal_Image_Thread(void);
void Parameter_Bsp();
extern TaskFlag_t TaskFlag;
extern Params_t Params;

#endif
