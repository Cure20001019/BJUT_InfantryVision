
/**
  ***********************************************************************************
  * @file   :ArmorPose.cpp
  * @author :Tinker.Jia
  * @version:1.0
  * @date   :2019.6.28
  * @brief  :Use the P4P method to obtain the armor pose in the world.
  ***********************************************************************************
  * @attention
  * None
  ***********************************************************************************
  */
#include "Header.h"
#include "Inc/ArmorPose.h"
#include "Inc/Serial.h"

void PoseSolver::PoseSlover_Init(void)
{
    Set_Camera_Params();
    Set_World_Points();
}


void PoseSolver::Set_Camera_Params(void)
{
    //相机内参矩阵
    CameraMatrix = Mat::eye(3, 3, CV_64F);
    CameraMatrix.at<double>(0, 0) = 1.125496711155745e+03;
    CameraMatrix.at<double>(0, 1) = -0.392652606420607;
    CameraMatrix.at<double>(0, 2) = 7.704915775676064e+02;
    CameraMatrix.at<double>(1, 1) = 1.124862214131361e+03;
    CameraMatrix.at<double>(1, 2) = 5.832778608262751e+02;
    CameraMatrix.at<double>(2, 2) = 1;

    DisCoeffs = Mat::zeros(5, 1, CV_64F);
    DisCoeffs.at<double>(0, 0) = -0.126131919352503;
    DisCoeffs.at<double>(1, 0) = 0.163721422318411;
    DisCoeffs.at<double>(2, 0) = 2.957260432160676e-04;
    DisCoeffs.at<double>(3, 0) = 3.984145323372491e-04;
    DisCoeffs.at<double>(4, 0) = 0;




    /*CameraMatrix.create(3, 3, CV_64FC1);
    CameraMatrix.setTo(cv::Scalar(0));
    CameraMatrix.at<double>(0,0) = 1122.357584085957;
    CameraMatrix.at<double>(0,1) = 0;
    CameraMatrix.at<double>(0,2) = 773.4085273406833;
    CameraMatrix.at<double>(1,1) = 1122.528164053286;
    CameraMatrix.at<double>(1,2) = 583.3659869267061;
    CameraMatrix.at<double>(2,2) = 1;

    DisCoeffs.create(4, 1, CV_64FC1);
    DisCoeffs.setTo(cv::Scalar(0));
    DisCoeffs.at<double>(0,0) = -0.1248;//RadiaDistortion
    DisCoeffs.at<double>(1,0) = 0.1695;//RadiaDistortion
    DisCoeffs.at<double>(2,0) = 0;//TangentialDistortion
    DisCoeffs.at<double>(3,0) = 0;//TangentialDistortion*/


    //建立局部畸变新相机内参矩阵
    NewCameraMatrix.create(3, 3, CV_64FC1);
    NewCameraMatrix = cv::getOptimalNewCameraMatrix(CameraMatrix,DisCoeffs,Size(1600,1200),1,Size(1600,1200),0);
}

void PoseSolver::Set_World_Points(void)
{
    Small_Armor_world_Point.push_back(Point3f(0,0,0));
    Small_Armor_world_Point.push_back(Point3f(0,50.00,0));
    Small_Armor_world_Point.push_back(Point3f(124.71,0,0));
    Small_Armor_world_Point.push_back(Point3f(124.71,50.00,0));

    Big_Armor_world_Point.push_back(Point3f(0,0,0));
    Big_Armor_world_Point.push_back(Point3f(0,52.00,0));
    Big_Armor_world_Point.push_back(Point3f(218.00,0,0));
    Big_Armor_world_Point.push_back(Point3f(218.00,52.00,0));
}


void PoseSolver::Get_Armor_Pose(vector<Point2f> camera_Point,PoseSolver::Pose &ArmorPose)
{
    //相机坐标点局部矫正
    vector<Point2f> undistort_Camera_Point;
    cv::undistortPoints(camera_Point,undistort_Camera_Point,CameraMatrix,DisCoeffs,cv::noArray(),NewCameraMatrix);
    //旋转矩阵和平移矩阵
    cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);
    cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);
    //PNP算法 CV_ITERATIVE(0.8ms) = 0 , CV_EPNP = 1（0.4ms）, CV_P3P(0.12ms) = 2, CV_DLS(0.3ms) = 3
    //这里的运行时间包括了0.04ms的局部去畸变时间
    if(ArmorPose.Hwdiv > 3.5)
        cv::solvePnP(Big_Armor_world_Point,undistort_Camera_Point,CameraMatrix,DisCoeffs, rvec, tvec, false, CV_ITERATIVE);
    else
        cv::solvePnP(Small_Armor_world_Point,undistort_Camera_Point,CameraMatrix,DisCoeffs, rvec, tvec, false, CV_ITERATIVE);
    //cv::solvePnP(world_Point,undistort_Camera_Point,NewCameraMatrix,Mat(), rvec, tvec, false, CV_ITERATIVE);
    //camera_Point.clear();
    //undistort_Camera_Point.clear();//清空容器，防止溢出
    //旋转向量变旋转矩阵
    double rm[9];
    cv::Mat rotM(3, 3, CV_64FC1, rm);
    cv::Rodrigues(rvec, rotM);
    ArmorPose.YawAngle = atan2(-1 * rotM.at<double>(2,0), sqrt(pow(rotM.at<double>(2,1),2) + pow(rotM.at<double>(2,2),2))) / CV_PI * 180;
    //其他两个自由度的偏角，目前尚未使用
    //double thetaz = atan2(rotM.at<double>(1,0), rotM.at<double>(0,0)) / CV_PI * 180;
    //double thetax = atan2(rotM.at<double>(2,1), rotM.at<double>(2,2)) / CV_PI * 180;

    //对PNP解算做一个均值滤波处理
    ArmorPose.YawOffset   = tvec.at<double>(0,0);
    ArmorPose.PitchOffset = tvec.at<double>(1,0);
    ArmorPose.Distance    = tvec.at<double>(2,0);

    ArmorPose.Center.x    = 0.25 *(undistort_Camera_Point[0].x + undistort_Camera_Point[1].x +undistort_Camera_Point[2].x +undistort_Camera_Point[3].x);
    ArmorPose.Center.y    = 0.25 *(undistort_Camera_Point[0].y + undistort_Camera_Point[1].y +undistort_Camera_Point[2].y +undistort_Camera_Point[3].y);
    ArmorPose.YawOffset   = ArmorPose.Distance / 1122.0f * (ArmorPose.Center.x - Params.Camera_Center_x);
    ArmorPose.PitchOffset = ArmorPose.Distance / 1122.0f * (ArmorPose.Center.y - Params.Camera_Center_y);

    //cout << "ArmorPose.HorizonOffset:" << ArmorPose.YawOffset     << endl;
    //cout << "ArmorPose.Distance     :"<<setw(10)<< ArmorPose.Distance      << endl;
    //cout << ":ArmorPose.YawAngle     :" << ArmorPose.YawAngle    <<ArmorPose.Center<< endl;
}


//采用小孔成像原理估算位置
void PoseSolver::Get_Armor_Position(PoseSolver::Pose &ArmorPose)
{
    ArmorPose.Distance    = 56000 / ArmorPose.Armor_Height;
    ArmorPose.YawOffset   = ArmorPose.Distance / 1200.0f * (ArmorPose.Center.x - Params.Camera_Center_x);
    ArmorPose.PitchOffset = ArmorPose.Distance / 1200.0f * (ArmorPose.Center.y - Params.Camera_Center_y);
//    cout << "ArmorPose.PitchOffset   :" << ArmorPose.PitchOffset     << endl;
//    cout << "ArmorPose.YawOffset     :" << ArmorPose.YawOffset       << endl;
//    cout << "ArmorPose.Distance      :" << ArmorPose.Distance        << endl;
}


void PoseSolver::Deal_Armor_Info(PoseSolver::Pose ArmorPose)
{
    static double time[8] = {0};
    time[7] = (double)getTickCount() / getTickFrequency();

    //补偿相机偏差
    YawOffsetConpensate   = ArmorPose.Distance / 1122.0f * (800 - Params.Camera_Center_x);
    cout<<"YawOffsetConpensate : "<<YawOffsetConpensate<<endl;
    PitchOffsetConpensate = ArmorPose.Distance / 1122.0f * (600 - Params.Camera_Center_y);
    //装甲板数据
    USART_Data.Distance    = ArmorPose.Distance + Params.Distance_Conpensate;
    USART_Data.YawOffset[7]= ArmorPose.YawOffset;// + YawOffsetConpensate;
    USART_Data.PitchOffset = ArmorPose.PitchOffset + Params.Pitch_Conpensate;// + PitchOffsetConpensate;
    USART_Data.ArmorAngle  = ArmorPose.YawAngle;
    //中心点数据 中心点距离装甲版距离认为250mm
    USART_Data.CenterYawOffset =  USART_Data.YawOffset[7] + sin(USART_Data.ArmorAngle * CV_PI / 180) * 250;
    USART_Data.CenterDistance  =  USART_Data.Distance + cos(USART_Data.ArmorAngle * CV_PI / 180) * 250;
    USART_Data.CenterYawAngle  =  atan2(USART_Data.CenterYawOffset,USART_Data.CenterDistance) * 4096 / CV_PI;

    if(abs(USART_Data.YawOffset[7] - USART_Data.YawOffset[6]) > 80)
    {
        for(char i = 0;i<8;i++)
        {
            USART_Data.YawOffset[i] = USART_Data.YawOffset[7];
            time[i] = time[7];
        }
        USART_Data.YawSpeed[7] = USART_Data.YawSpeed[6];
    }
    else
    {
        USART_Data.YawSpeed[7]   = (USART_Data.YawOffset[7] - USART_Data.YawOffset[6])   / (time[7] - time[6]);
    }

    int sum;
    for(int i = 0;i < 8;i++)
    {
        sum += USART_Data.YawSpeed[i];
    }
    USART_Data.YawSpeed[8] = sum / 8;

    USART_Data.YawAngle   = atan2(USART_Data.YawOffset[7], USART_Data.Distance)*180.0 / CV_PI;
    USART_Data.PitchAngle = atan2(USART_Data.PitchOffset , USART_Data.Distance)*180.0 / CV_PI;

    if(Params.Display_Pose == 0)
    {
        cout<<"*********************************************************"<<endl;
        cout << "YawSpeed       : " << USART_Data.YawSpeed[8]    << endl;
        cout << "Distance       : " << USART_Data.Distance       << endl;
        cout << "PitchOffset    : " << USART_Data.PitchOffset    << endl;
        cout << "YawOffset      : " << USART_Data.YawOffset[7]   << endl;
        cout << "Yaw_Angle      : " << USART_Data.YawAngle       << endl;
        cout << "Pitch_Angle    : " << USART_Data.PitchAngle     << endl;
        cout << "Armor_Angle    : " << USART_Data.ArmorAngle     << endl;
        cout << "CenterYawOffset: " << USART_Data.CenterYawOffset<< endl;
        cout << "CenterDistance : " << USART_Data.CenterDistance << endl;
        cout << "CenterYawAngle : " << USART_Data.CenterYawAngle << endl;
    }

    for(int i = 0;i<7;i++)
    {
        time[i] = time[i+1];
        USART_Data.YawOffset[i]  = USART_Data.YawOffset[i+1];
        USART_Data.YawSpeed[i]   = USART_Data.YawSpeed[i+1];
    }
    USART_Data.AllowSend = 1;
}
