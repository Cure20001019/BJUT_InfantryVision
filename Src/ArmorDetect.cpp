/**
  ***********************************************************************************
  * @file   :ArmorDetecct.cpp
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
#include "Inc/ArmorDetect.h"
#include "Inc/ArmorPose.h"
#include "Inc/Camera.h"
#include "Inc/Serial.h"
/**
 * @brief ArmorDetect::DealHSVImageThread
 * @param mode
 * @param input
 * @param output
 */
void ArmorDetect::DealHSVImageThread (vector<GpuMat> &input, vector<GpuMat> &output) {
    GpuMat hsv,temp,hsv_InRange;
    vector<GpuMat> splited;
    Mat contourThreadkernel = getStructuringElement(MORPH_ELLIPSE, Size(3, 3));
    Ptr<cuda::Filter> CloseFilter = cuda::createMorphologyFilter(MORPH_CLOSE, output[0].type(), contourThreadkernel);

    cout<<"DealHSVImageThread is start!"<<endl;
    while(1)
    {
        //********************************************************//
        while(TaskFlag.CameraBuff[0].Status == WAIT_SAVE && TaskFlag.CameraBuff[1].Status == WAIT_SAVE);
        if(TaskFlag.CameraBuff[0].Status == WAIT_DEAL)
        {
            TaskFlag.Img_ID = 0;
        }
        else if(TaskFlag.CameraBuff[1].Status == WAIT_DEAL)
        {
            TaskFlag.Img_ID = 1;
        }
        //cout<<"TaskFlag.Img_ID : "<<TaskFlag.Img_ID<<" "<<TaskFlag.CameraBuff[0].Status<<" "<<TaskFlag.CameraBuff[1].Status<<endl;
        //********************************************************//
        cv::cuda::cvtColor(input[TaskFlag.Img_ID],hsv,CV_BGR2HSV);//RGB转换为HSV
        cv::cuda::split(hsv,splited);//HSV通道分离      
        if (USART_Data.Mode == ATTACK_RED_ARMOR)
        {//Red
            cv::cuda::threshold(splited[0],splited[0],Params.Red_H_Thres,255,THRESH_TOZERO_INV);//色调阈值调整
            cv::cuda::threshold(splited[0],splited[0],1,255,THRESH_BINARY);//色调阈值调整
            cv::cuda::threshold(splited[1],splited[1],Params.S_Thres,255,THRESH_BINARY);//饱和度
            cv::cuda::threshold(splited[2],splited[2],Params.V_Thres,255,THRESH_BINARY);//亮度
        }
        else
        {//BLue
            cv::cuda::threshold(splited[0],splited[0],Params.Blue_H_Thres,255,THRESH_BINARY);//同上
            cv::cuda::threshold(splited[1],splited[1],Params.S_Thres,255,THRESH_BINARY);
            cv::cuda::threshold(splited[2],splited[2],Params.V_Thres,255,THRESH_BINARY);
        }
        cv::cuda::bitwise_and(splited[0],splited[1],temp);//三通道转为单通道
        cv::cuda::bitwise_and(splited[2],temp,hsv_InRange);//同上
        //********************************************************//
        while(TaskFlag.BinaryBuff[0].Status != WAIT_SAVE && TaskFlag.BinaryBuff[1].Status != WAIT_SAVE);
        if(TaskFlag.BinaryBuff[0].Status == WAIT_SAVE)
        {
            CloseFilter->apply(hsv_InRange, output[0]);//闭运算
            TaskFlag.BinaryBuff[0].Status = WAIT_DEAL;
        }
        else if(TaskFlag.BinaryBuff[1].Status == WAIT_SAVE)
        {
            CloseFilter->apply(hsv_InRange, output[1]);//闭运算
            TaskFlag.BinaryBuff[1].Status = WAIT_DEAL;
        }
        //********************************************************//
        FPS_Detect();
        /*Mat cpu_Img1,cpu_Img2,cpu_Img3;
        hsv_InRange.download(cpu_Img1);
        //splited[1].download(cpu_Img2);
        //splited[2].download(cpu_Img3);
        imshow("H",cpu_Img1);
        //imshow("S",cpu_Img2);
        //imshow("V",cpu_Img3);
        waitKey(5);*/
        //********************************************************//
        TaskFlag.CameraBuff[TaskFlag.Img_ID].Status = WAIT_SAVE;//图像缓冲区置为可写入状态
        //********************************************************//
    }
}






/**
 * @brief ArmorDetect::DealBGRImageThread
 * @param mode
 * @param input
 * @param output
 */
void ArmorDetect::DealBGRImageThread (vector<GpuMat> &input, vector<GpuMat> &output)
{
    GpuMat thres_whole,temp;
    vector<GpuMat> splited;
    Mat contourThreadkernel = getStructuringElement(MORPH_ELLIPSE, Size(9, 9));
    Ptr<cuda::Filter> dilateFilter = cuda::createMorphologyFilter(MORPH_DILATE, output[0].type(), contourThreadkernel);
        cout<<"DealBGRImageThread is start!"<<endl;
    while(1)
    {
        //********************************************************//
        while(TaskFlag.CameraBuff[0].Status == WAIT_SAVE && TaskFlag.CameraBuff[1].Status == WAIT_SAVE);
        if(TaskFlag.CameraBuff[0].Status == WAIT_DEAL)
        {
            TaskFlag.Img_ID = 0;
        }
        else if(TaskFlag.CameraBuff[1].Status == WAIT_DEAL)
        {
            TaskFlag.Img_ID = 1;
        }
        //cout<<"TaskFlag.Img_ID : "<<TaskFlag.Img_ID<<" "<<TaskFlag.CameraBuff[0].Status<<" "<<TaskFlag.CameraBuff[1].Status<<endl;
        //********************************************************//
        cv::cuda::split(input[TaskFlag.Img_ID], splited);//彩色三通道图分离
        cv::cuda::cvtColor(input[TaskFlag.Img_ID], thres_whole, CV_BGR2GRAY);//彩色图三通道图像转换为灰度单通道图像
        cv::cuda::threshold(thres_whole, thres_whole, Params.Thres_Gray, 255, THRESH_BINARY);//灰度图阈值处理
        if (USART_Data.Mode == ATTACK_RED_ARMOR) {
            cv::cuda::subtract(splited[2], splited[0], temp);//单色通道相减
            cv::cuda::threshold(temp, temp, Params.Thres_Red, 255, THRESH_BINARY);//red，相减后阈值处理
        }
        else {
            cv::cuda::subtract(splited[0], splited[2], temp);
            cv::cuda::threshold(temp, temp, Params.Thres_Blue, 255, THRESH_BINARY);// blue
        }
        dilateFilter->apply(temp, temp);//形态学膨胀


        //********************************************************//
        while(TaskFlag.BinaryBuff[0].Status != WAIT_SAVE && TaskFlag.BinaryBuff[1].Status != WAIT_SAVE);
        if(TaskFlag.BinaryBuff[0].Status == WAIT_SAVE)
        {
            cv::cuda::bitwise_and(temp,thres_whole,output[0]);//灰度图与膨胀图与运算
            TaskFlag.BinaryBuff[0].Status = WAIT_DEAL;
        }
        else if(TaskFlag.BinaryBuff[1].Status == WAIT_SAVE)
        {
            cv::cuda::bitwise_and(temp,thres_whole,output[1]);//灰度图与膨胀图与运算
            TaskFlag.BinaryBuff[1].Status = WAIT_DEAL;
        }
        //********************************************************//

        /*Mat cpu_Img1,cpu_Img2,cpu_Img3;
        input[TaskFlag.Deal_Img].download(cpu_Img1);
        thres_whole.download(cpu_Img2);
        output.download(cpu_Img3);
        imshow("input",cpu_Img1);
        imshow("gray",cpu_Img2);
        imshow("output",cpu_Img3);

        waitKey(1);//如果显示图像必须加入WaitKey*/
        //********************************************************//
        TaskFlag.CameraBuff[TaskFlag.Img_ID].Status = WAIT_SAVE;//图像缓冲区置为可写入状态
        //********************************************************//
    }
}


/**
 * @brief ArmorDetect::GetArmorThread
 * @param input
 * @param ArmorCameraPoint
 */
void ArmorDetect::GetArmorThread(vector<GpuMat> &input)
{
    Mat cpu_Binary,roi_Binary;
    vector<vector<Point>> contours;
    PoseArmor.PoseSlover_Init();
    cout<<"GetArmorThread is start!"<<endl;
   // TaskFlag.GetPose = OK;
    while(1)
    {
        //********************************************************//进程控制
        while(TaskFlag.BinaryBuff[0].Status == WAIT_SAVE && TaskFlag.BinaryBuff[1].Status == WAIT_SAVE);
        if(TaskFlag.BinaryBuff[0].Status == WAIT_DEAL)
        {
            TaskFlag.Binary_ID = 0;
        }
        else if(TaskFlag.BinaryBuff[1].Status == WAIT_DEAL)
        {
            TaskFlag.Binary_ID = 1;
        }
        //********************************************************//

        input[TaskFlag.Binary_ID].download(cpu_Binary);
        //roi_Binary = GetImageROI(cpu_Binary,ROI);
        findContours(cpu_Binary,contours,CV_RETR_TREE,CV_CHAIN_APPROX_SIMPLE,Point(200,500)+ROI.Point_tl);
        GetArmorLights(contours);//获取灯条
        GetArmors();
        ChooseArmor(ArmorsPose);
        DisplayImg(cpu_Binary);
        char key = waitKey(1);
        if(key == 'q' || key == 'Q')
        {
            break;
        }
        //********************************************************//
        TaskFlag.BinaryBuff[TaskFlag.Binary_ID].Status = WAIT_SAVE;//图像缓冲区置为可写入状态
        //********************************************************//
    }
}

/**
* @brief get the final result of the armor lights
* @param none
* @return none
*/
void ArmorDetect::GetArmorLights(vector<vector<Point>> &contours)
{
    Clear();//数据清空
    RotatedRect RRect;
    //先进行角度筛选
    for (int i = 0; i<contours.size(); i++)
    {
        RRect = minAreaRect(contours[i]); //生成包含点集的最小矩形
        if ((fabs(RRect.angle) < 45.0 && RRect.size.height > RRect.size.width)|| (fabs(RRect.angle) > 45.0 && RRect.size.width > RRect.size.height))
        {
            RectfirstResult.push_back(RRect);//如果满足上边这要求 将这个矩形存进来（存的数据有中心点、长、宽、旋转角度）
        }
    }
    contours.clear();
    if (RectfirstResult.size() < 2) { //如果框选的矩形数量小于2（没找到装甲板） 清空装甲片中心位置 返回
        return;
    }
    sort(RectfirstResult.begin(), RectfirstResult.end(), RotateRectSort);//矩阵排序 begin是数组元素首地址 end是数组元素末尾地址 center.x from min to max

    size_t size = RectfirstResult.size();//求第一次框选矩形的数量
    vector<RotatedRect> Groups; //临时存放矩形的数组
    Groups.clear();
    int cellmaxsize;//最大灯条面积
    Groups.push_back(RectfirstResult[0]);//将center.x min矩形传到group里
    cellmaxsize = RectfirstResult[0].size.height * RectfirstResult[0].size.width;//求取面积
    if (cellmaxsize > 2500) cellmaxsize = 0;//如果大于2500 清零
    int maxsize;


    for (int i = 1; i<size; i++)//这个for循环的操作就是把上一步粗提取出的特征，进行归类，中心距离小于10的是一类，中有一个最大灯条面积（应该说网格面积，因为会有强光）
    {
        if (RectfirstResult[i].center.x - RectfirstResult[i - 1].center.x <10)
        {//如果两个最近的矩形x轴坐标小于10  因为一个灯条可能会圈出来多个矩形，灯条的中心可以来说更接近白光，所以中间会有空洞
            maxsize = RectfirstResult[i].size.height * RectfirstResult[i].size.width;//最大面积
            if (maxsize > 2500) continue;//大于2500结束本次循环
            if (maxsize > cellmaxsize) cellmaxsize = maxsize;//最大矩形尺寸
            Groups.push_back(RectfirstResult[i]);//将最大的矩形存到group里
        }
        else {
            Armorlists.push_back(Groups); //将二维数组group里的数据存入到装甲片清单里
            CellMaxs.push_back(cellmaxsize);//将最大的网格值存入到全局变量中
            cellmaxsize = 0;//局部变量值清零
            maxsize = 0;//最大面积清零
            Groups.clear();//group数据清零
            Groups.push_back(RectfirstResult[i]);//将结果存入group里
            cellmaxsize = RectfirstResult[i].size.height * RectfirstResult[i].size.width;//求最大网格值
            }
    }
    Armorlists.push_back(Groups); //把上边最后一次的结果存到装甲片列表里 所以上边的group数据清零的目的就很明显了 为了分类 并且这个二维容器里 对距离进行了归类
    CellMaxs.push_back(cellmaxsize);//把最后一次的最大网格值存进去
    //这里往上的代码 为了把叠在一起的矩形存入一个容器里


    size = Armorlists.size();//求装甲片数量
    for (int i = 0; i<size; i++) {
        int Gsize = Armorlists[i].size();//求这个装甲片里矩形的数量
        int GroupMax = CellMaxs[i];//当前的最大网格尺寸
        if (GroupMax > 5) {
            for (int j = 0; j<Gsize; j++) {
                maxsize = Armorlists[i][j].size.height * Armorlists[i][j].size.width;//比对网格尺寸
                if (maxsize == GroupMax) {
                    RectResults.push_back(Armorlists[i][j]);//让每个装甲片只提取出一个矩形
                }
            }
        }
    }//这里往上的代码 都是为了把多个叠在一起的矩形滤掉 只留下最外边的矩形
    sort(RectResults.begin(), RectResults.end(), RotateRectSort);//对获取到的结果进行排序


    //获取灯条边界的四个点
    Point2f armor_Point[4];
    vector<Point2f> roi_Point;
    for (int i = 0; i<RectResults.size(); i++)
    {
        RectResults[i].points(armor_Point);
        roi_Point.push_back(armor_Point[0]);
        roi_Point.push_back(armor_Point[1]);
        roi_Point.push_back(armor_Point[2]);
        roi_Point.push_back(armor_Point[3]);
    }
    ROI.ROIRect = boundingRect(roi_Point);
    roi_Point.clear();//讲道理局部变量用完应该自己清空，但是遇到了不稳定的情况，所以还是要手动清空一下
}

void ArmorDetect::GetArmors(void)
{//获取装甲片 通过两个灯条将装甲片圈起来
    size_t size = RectResults.size();//获取上一步灯条数量
    if(size < 2)
    {
        return;
    }
    Point2f L1, L2;//两个灯条的中心点
    float K, angleabs = 0.0, angleL1, angleL2;//两个灯条的（中心斜率）（框出的T型上下两条边的斜率）（偏角）
    float divscale, areaL1, areaL2;//两个灯条的（面积比值）（面积）
    float ydis = 0;//y轴偏差值
    float maxangle, xdis, heightmax, hwdiv;
    Point2f _pt[4], pt[4];

    //定义了两个局部函数 auto为自动确定变量返回类型，返回两个点的tan值
    auto ptangle = [](const Point2f &p1, const Point2f &p2) {
        return fabs(atan2(p2.y - p1.y, p2.x - p1.x)*180.0 / CV_PI);
    };



    for (int i = 0; i<size - 1; i++)
    {
        angleL1 = fabs(RectResults[i].angle);//求L1的偏角绝对值
        L1 = RectResults[i].center;//L1的为中心点
        areaL1 = RectResults[i].size.height * RectResults[i].size.width;//灯条的面积
        RectResults[i].points(_pt);//把矩形的四个点赋值给他
        /*pt
        * 0 2
        * 1 3
        * */
        //          1          2
        //       0          3          这是矩形的四个点
        if (angleL1 > 45.0) {//这里没框住灯条 而是框的两个灯条中间黑色的部分，就是有数字的地方
            pt[0] = _pt[3];
            pt[1] = _pt[0];
        }
        else {//偏角不一样，点也不一样
            pt[0] = _pt[2];
            pt[1] = _pt[3];
        }
        //以下计算其他灯条，双重循环，就是每个灯条两两匹配
        for (int j = i + 1; j<size; j++) {
            L2 = RectResults[j].center;//计算灯条中心
            if (L1.x != L2.x) {//如果两个中心不相同
                K = GetK(L1, L2);//获取斜率

                areaL2 = RectResults[j].size.height * RectResults[j].size.width;//获取匹配灯条面积
                if (areaL1 > areaL2) {//获取两个灯条的面积比值
                    divscale = areaL1 / areaL2;
                }
                else {
                    divscale = areaL2 / areaL1;
                }
                angleL2 = fabs(RectResults[j].angle);//匹配灯条的偏角

                RectResults[j].points(_pt);//将匹配灯条的左侧两个点拿出，框出装甲片有数字的地方
                if (angleL2 > 45.0) {
                    pt[2] = _pt[2];
                    pt[3] = _pt[1];
                }
                else {
                    pt[2] = _pt[1];
                    pt[3] = _pt[0];
                }

                maxangle = MAX(ptangle(pt[0], pt[2]), ptangle(pt[1], pt[3]));//求x向最大偏角
                //求y向相对偏角值
                if (angleL1 > 45.0 && angleL2 < 45.0) {
                    angleabs = 90.0 - angleL1 + angleL2;
                }
                else if (angleL1 <= 45.0 && angleL2 >= 45.0) {
                    angleabs = 90.0 - angleL2 + angleL1;
                }
                else {
                    if (angleL1 > angleL2) angleabs = angleL1 - angleL2;
                    else angleabs = angleL2 - angleL1;
                }

                ydis = abs(L1.y - L2.y);//获取y轴偏差
                xdis = fabs(L1.x - L2.x);//获取x轴偏差

                heightmax = MAX(MAX(RectResults[i].size.width, RectResults[j].size.width), MAX(RectResults[i].size.height, RectResults[j].size.height));//最大灯条高度
                hwdiv = xdis / heightmax;//x向距离与灯条高度的比值，用来确定装甲片的种类
                if (fabs(K) < 0.5 && divscale < 4 && maxangle < 20.0 && hwdiv < 7.0 && ydis < 0.35*heightmax)//通过各个条件判断是不是装甲片，有时间实时采集测量一下，它这个数据不太好
                {
                    if (angleabs < 7)
                    {
                        PoseSolver::Pose SingleArmor;
                        SingleArmor.Hwdiv = hwdiv;
                        ArmorCameraPoint.push_back(Point2f(pt[0].x,pt[0].y));
                        ArmorCameraPoint.push_back(Point2f(pt[1].x,pt[1].y));
                        ArmorCameraPoint.push_back(Point2f(pt[2].x,pt[2].y));
                        ArmorCameraPoint.push_back(Point2f(pt[3].x,pt[3].y));
                        PoseArmor.Get_Armor_Pose(ArmorCameraPoint,SingleArmor);
                        //PoseArmor.Get_Armor_Position(SingleArmor);
                        ArmorsPose.push_back(SingleArmor);
                        ArmorCameraPoint.clear();

                    }
                }
            }
        }
    }
}

void ArmorDetect::ChooseArmor(vector<PoseSolver::Pose> &ArmorsPose)
{
    int errorRecognition = 0;//删除错误匹配对象后，容器的size会改变，如果再删除会导致地址错乱
    size_t size = ArmorsPose.size();//未剔除假装甲的备选装甲片数量
    ROI.GetROIImage = 1;
    if(size > 1)//寻找到一个以上可攻击装甲片特征
    {
        for(int i = 0;i < size;i++)
        {
            if(((abs(ArmorsPose[i].YawAngle - LastPerfectArmor.YawAngle) < 80) || (abs(ArmorsPose[i].YawAngle - LastPerfectArmor.YawAngle) > 95)) && abs(LastPerfectArmor.YawOffset - ArmorsPose[i].YawOffset) > 50)
            {
                    ArmorsPose.erase(ArmorsPose.begin()+i-errorRecognition);
                    errorRecognition++;
            }
        }
        size = ArmorsPose.size();//剔除假装甲片后的数量
    }
    if(size == 1)//一个可攻击特征
    {
        if(ArmorLostCount > 9)
        {
                //cout<<"1";
            PerfectArmor = ArmorsPose[0];
            ArmorLostCount = 0;
        }
        else if((abs(ArmorsPose[0].YawAngle - LastPerfectArmor.YawAngle) > 80) && abs(LastPerfectArmor.YawOffset - ArmorsPose[0].YawOffset) > 220)
        {
            //cout<<"1";
            PerfectArmor = ArmorsPose[0];
            ArmorLostCount = 0;
        }
        else if(abs(ArmorsPose[0].YawAngle - LastPerfectArmor.YawAngle) < 10 && abs(LastPerfectArmor.YawOffset - ArmorsPose[0].YawOffset) < 50)
        {
            //cout<<"1";
            PerfectArmor = ArmorsPose[0];
            ArmorLostCount = 0;
        }
        else
        {
           // cout<<"0";
            PerfectArmor = LastPerfectArmor;
            ArmorLostCount ++;
            ROI.GetROIImage = 0;
        }
    }
    else if(size > 1)
    {
        int perfectID = -1;//最优装甲片ID
        float minYawAngle = 180;//多个装甲片与镜头相对角度最小角度
        for(int i = 0;i < size;i++)
        {
            if(ArmorsPose[i].YawAngle < minYawAngle)
            {
                minYawAngle = ArmorsPose[i].YawAngle;
                perfectID = i;
            }
        }
        if(perfectID >= 0)
        {
            PerfectArmor = ArmorsPose[perfectID];
        }
        ArmorLostCount = 0;
        //cout<<"1";
    }
    else//未找到可攻击特征
    {
        //cout<<"0";
        ArmorLostCount++;
        ROI.GetROIImage = 0;
    }
    if(ArmorLostCount < 10)
    {
        LastPerfectArmor = PerfectArmor;
        PoseArmor.Deal_Armor_Info(PerfectArmor);
    }
    else
    {
        USART_Data.AllowSend = 2;
    }
    cout<<"PerfectArmor : "<<PerfectArmor.Center<<endl;
}

Mat ArmorDetect::GetImageROI(Mat &src_Img,ROI_t &ROI)
{
    Mat roi;
    if(ROI.GetROIImage == 0)
    {
        roi = src_Img;
        return roi;
    }
    else
    {
        ROI.Point_tl = ROI.ROIRect.tl() + Point(-200,-100) - Point(200,500);
        ROI.Point_br = ROI.ROIRect.br() + Point(200,100) - Point(200,500);
        ROI_Range_Limit(&ROI.Point_tl);
        ROI_Range_Limit(&ROI.Point_br);
        Rect dst_Range(ROI.Point_tl,ROI.Point_br);
        roi = src_Img(dst_Range);
        return roi;
    }
}

void ArmorDetect::ROI_Range_Limit(Point* value)
{
    if(value->x<0)
        value->x=0;
    else if(value->x>1100)
       value->x=1100;

    if(value->y<0)
       value->y=0;
    else if(value->y>600)
       value->y=600;
}

void ArmorDetect::DisplayImg(Mat src_Img)
{
    //cv::cvtColor(src_Img,src_Img,CV_GRAY2BGR);
    if(RectResults.empty() == 0)
    {
        for (int i = 0; i<RectResults.size(); i++)
        {
            //ellipse(src_Img, RectResults[i], Scalar(255,192,203), 3);//在图像上将装甲片圈起来
        }
    }
    if(ArmorsPose.empty() == 0)
    {
        for(int i = 0;i < ArmorsPose.size();i++)
        {
            DrawCross(src_Img,PerfectArmor.Center-Point(200,500), 25, Scalar(255, 0, 255), 2);
        }
    }
    cv::resize(src_Img,src_Img,Size(480,320));
    imshow("output",src_Img);


}

void ArmorDetect::DrawCross(Mat &img, Point center, int size, Scalar color, int thickness)
{
    Point L1, L2, B1, B2;
    int xL = center.x - size > 0 ? center.x - size : 0;
    int xR = center.x + size;
    int yL = center.y - size > 0 ? center.y - size : 0;
    int yR = center.y + size;
    L1 = Point(xL, center.y);
    L2 = Point(xR, center.y);
    B1 = Point(center.x, yL);
    B2 = Point(center.x, yR);
    line(img, L1, L2, color, thickness);
    line(img, B1, B2, color, thickness);

}


/**
 * @brief PointGreyCamera::FPS_Detect
 * @return FPS
 */
int ArmorDetect::FPS_Detect(void)
{
  static double lasttime = 0,time = 0;
  static int FPS = 0;
  time = (double)getTickCount() / getTickFrequency();
  if(time - lasttime > 1.0)
  {
     lasttime = time;
     cout <<"Img_FPS :    "<< FPS <<"  Time : "<<time<< endl;
     FPS = 0;
  }
  FPS++;
  return FPS;
}

double ArmorDetect::GetK(Point2f L1, Point2f L2)
{
    return (L1.y - L2.y) / (L1.x - L2.x);
}

void ArmorDetect::Clear(void)
{
    CellMaxs.clear();
    Armorlists.clear();
    RectfirstResult.clear();
    RectResults.clear();
    ArmorsPose.clear();
}
