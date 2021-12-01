/**
  ***********************************************************************************
  * @file   :ArmorPose.h
  * @author :Tinker.Jia
  * @version:1.0
  * @date   :2019.6.28
  * @brief  :Use the P4P method to obtain the armor pose in the world.
  ***********************************************************************************
  * @attention
  * None
  ***********************************************************************************
  */
#ifndef ARMORPOSE_H
#define ARMORPOSE_H
class PoseSolver
{
public:
    typedef struct{
        Point Center;
        float Hwdiv;
        float Armor_Height;
        float Distance;
        float YawOffset;
        float PitchOffset;
        float YawAngle;
    }Pose;
    //Pose ArmorPose;
private:
    Mat CameraMatrix;//IntrinsicMatrix
    Mat NewCameraMatrix;//IntrinsicMatrix
    Mat DisCoeffs;
    vector<Point3f> Big_Armor_world_Point;
    vector<Point3f> Small_Armor_world_Point;
    float PitchOffsetConpensate,YawOffsetConpensate;//因为实际的相机中心和图像的相机中心不在一个坐标上，所以需要对偏差进行补偿

public:
    void Get_Armor_Pose(vector<Point2f> camera_Point, PoseSolver::Pose &ArmorPose);
    void Get_Armor_Position(PoseSolver::Pose &ArmorPose);
    void PoseSlover_Init(void);
    void Deal_Armor_Info(PoseSolver::Pose ArmorPose);
private:
    void Set_Camera_Params(void);
    void Set_World_Points(void);
};

int signal(int a);
#endif
