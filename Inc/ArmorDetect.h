/**
  ***********************************************************************************
  * @file   :ArmorDetecct.h
  * @author :Tinker.Jia
  * @version:1.0
  * @date   :2019.6.28
  * @brief  :Extraction feature for camera recovery image.
  ***********************************************************************************
  * @attention
  * None
  ***********************************************************************************
  */
#include "Header.h"
#include "Inc/ArmorPose.h"
#include "Inc/Camera.h"
#ifndef ARMORDETECT_H
#define ARMORDETECT_H

static inline bool RotateRectSort(RotatedRect a1, RotatedRect a2) {
    return a1.center.x < a2.center.x;
}



class ArmorDetect
{
public:
    int ArmorLostCount = 0;//装甲片丢失计数
    PoseSolver::Pose PerfectArmor,LastPerfectArmor;//最终的装甲片数据
    typedef struct{
        Point Point_tl;
        Point Point_br;
        bool GetROIImage;
        Rect ROIRect;
    }ROI_t;
    ROI_t ROI;
private:
    vector<PoseSolver::Pose> ArmorsPose;//装甲片的位姿，有可能有多组装甲片
    vector<Point2f>ArmorCameraPoint;//装甲片坐标点，相机未矫正前
    PoseSolver PoseArmor;//类声明
    vector<vector<RotatedRect> > Armorlists;//用于暂存刚刚提取的装甲片
    vector<int> CellMaxs;//网格
    vector<RotatedRect> RectfirstResult, RectResults;//提取装甲片过程中的旋转矩形

public:
    void DealHSVImageThread(vector<GpuMat> &input, vector<GpuMat> &output);
    void DealBGRImageThread(vector<GpuMat> &input, vector<GpuMat> &output);
    void GetArmorThread(vector<GpuMat> &input);
private:
    void ChooseArmor(vector<PoseSolver::Pose> &ArmorsPose);
    void DrawCross(Mat &img, Point center, int size, Scalar color, int thickness);
    void GetArmorLights(vector<vector<Point>> &contours);
    void GetArmors(void);
    double GetK(Point2f L1, Point2f L2);
    void Clear(void);
    void DisplayImg(Mat src_Img);
    Mat GetImageROI(Mat &src_Img, ROI_t &ROI);
    void ROI_Range_Limit(Point* value);
    int FPS_Detect(void);


};

#endif


