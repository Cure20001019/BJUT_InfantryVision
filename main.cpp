/**
  ***********************************************************************************
  * @file   :main.cpp
  * @author :Tinker.Jia
  * @version:1.0
  * @date   :2019.6.28
  * @brief  :初始化各个进程
  ***********************************************************************************
  * @attention
  *
  ***********************************************************************************
  */
#include "Header.h"
#include "Inc/Serial.h"
#include "Inc/Camera.h"
#include "Inc/ArmorDetect.h"
#include "Inc/ArmorPose.h"

Params_t Params;
TaskFlag_t TaskFlag;
int main()
{
    vector<GpuMat> raw_Img(3);                                                 // 初始化Vector, 容器内元素个数为3
    vector<GpuMat> binary_Img(3);
    PointGreyCamera MonoCamera;
    ArmorDetect ArmorFeature;
    Parameter_Bsp();
    std::thread t1(&PointGreyCamera::GetImageThread, MonoCamera,ref(raw_Img)); // pass by reference
    std::thread t2(&ArmorDetect::DealHSVImageThread,ArmorFeature,ref(raw_Img),ref(binary_Img));//这里暂时没想好用BGR提取特征，还是用HSV提取特征
    std::thread t3(&ArmorDetect::GetArmorThread,ArmorFeature,ref(binary_Img));
    std::thread t4(LinkThread);

    t1.join();
    t2.join();
    t3.join();
    t4.join();

}


void Parameter_Bsp()
{
    FileStorage Config("/home/dji/BJUT-InfantryVision/Config.xml",FileStorage::READ);
    Config["Thres_Red"]          >> Params.Thres_Red;
    Config["Thres_Blue"]         >> Params.Thres_Blue;
    Config["Thres_Gray"]         >> Params.Thres_Gray;
    Config["Blue_H_Thres"]       >> Params.Blue_H_Thres;
    Config["Red_H_Thres"]        >> Params.Red_H_Thres;
    Config["S_Thres"]            >> Params.S_Thres;
    Config["V_Thres"]            >> Params.V_Thres;
    Config["Debug"]              >> Params.Debug;
    Config["Display_Image"]      >> Params.Display_Image;
    Config["Display_Recognition_Rate"] >> Params.Display_Recognition_Rate;
    Config["Display_Pose"]       >> Params.Display_Pose;
    Config["Distance_Conpensate"]>> Params.Distance_Conpensate;
    Config["Pitch_Conpensate"]   >> Params.Pitch_Conpensate;
    Config["Camera_Center_x"]    >> Params.Camera_Center_x;
    Config["Camera_Center_y"]    >> Params.Camera_Center_y;

    Config.release();

    if(Params.Debug == 1)
    {
        namedWindow("Params_Set");
        createTrackbar("Thres_Red","Params_Set",&Params.Thres_Red,255);
        createTrackbar("Thres_Blue","Params_Set",&Params.Thres_Blue,255);
        createTrackbar("Thres_Gray","Params_Set",&Params.Thres_Gray,255);
        createTrackbar("Blue_H_Thres","Params_Set",&Params.Blue_H_Thres,255);
        createTrackbar("Red_H_Thres","Params_Set",&Params.Red_H_Thres,255);
        createTrackbar("S_Thres","Params_Set",&Params.S_Thres,255);
        createTrackbar("V_Thres","Params_Set",&Params.V_Thres,255);
    }
}

