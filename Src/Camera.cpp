/**
  ***********************************************************************************
  * @file   :Camera.cpp
  * @author :Tinker.Jia
  * @version:1.1
  * @date   :2019.7.4
  * @brief  :Init the PointGrey and get image.
  ***********************************************************************************
  * @attention
  * V1.1版本更新：
  * 1、直接对相机获取的图像进行了初步ROI获取，将像素由1600x1200改为1000x500，FPS由60提升至116
  * 2、将获取相机图像由单缓冲变为双缓冲
  *
  ***********************************************************************************
  */
#include "Header.h"
#include "Inc/Camera.h"

Camera cam;

/**
 * @brief PointGreyCamera::GetImageThread
 * @param Global_Img
 * @attention
 *
 * 这里使用了双缓冲读取图像，读取速度相比单缓冲快了10-20%，思路来自北方工业大学余鑫，在此感谢，嘻嘻。
 * 本方法的主要思路为当图像特征在被处理时，保证还在提取相机就图像，设置两个图像缓冲，当处理A图像时，可以存入B图像，
 * 这样可以省去单缓冲中图像复制的时间，事实证明，图像间的复制还是很浪费时间的。
 *
 * @  流程介绍：本方法共有三级缓冲，每一级缓冲包括时间戳与状态。
 *              1.时间戳：获取图像时的系统时间
 *              2.状态：共三种可存储、等待处理、处理中，假设三种状态分别为A、B、C
 * @  逻辑介绍：针对相机进程来说共有2种状态分别为：
 *              1.AAA、AAC、ABC ：直接存储即可
 *              2.BBC：通过对比BB图像的时间戳，将图像存储到时间戳较为靠前的缓冲
 *            针对处理进r程来说共有3种状态分别为：
 *              1.AAA：等待图像存储，一般为刚上电，或者相机掉线
 *              2.AAB：直接处理图像
 *              3.ABB、BBB：需要判断哪个时间戳最靠后
 */
void PointGreyCamera::GetImageThread(vector<GpuMat> &Global_Img)
{
    Image rgbImage,rawImage;
    Mat cpu_Img;
    PointGrey_Init();
    cout<<"GetImageThread is start!"<<endl;
    while(1)
    {
        while(TaskFlag.CameraBuff[0].Status != WAIT_SAVE && TaskFlag.CameraBuff[1].Status != WAIT_SAVE);
        //读取图像
        error = cam.RetrieveBuffer(&rawImage);
        if (error != PGRERROR_OK)//这里是防止相机掉线
        {
            PrintError(error);
            while(error != PGRERROR_OK)
            {
                PointGrey_Init();
                error = cam.RetrieveBuffer(&rawImage);
                std::chrono::milliseconds delay(10);
                std::this_thread::sleep_for(delay);
            }
        }
        error = rawImage.Convert( FlyCapture2::PIXEL_FORMAT_BGR, &rgbImage );//The picture format is B G R.
        if (error != PGRERROR_OK)//这里是防止相机掉线
        {
            PrintError(error);
            while(error != PGRERROR_OK)
            {
                PointGrey_Init();
                error = cam.RetrieveBuffer(&rawImage);
                error = rawImage.Convert( FlyCapture2::PIXEL_FORMAT_BGR, &rgbImage );//The picture format is B G R.
                std::chrono::milliseconds delay(10);
                std::this_thread::sleep_for(delay);
            }
        }
		 
        unsigned int rowBytes = (double)rgbImage.GetReceivedDataSize()/(double)rgbImage.GetRows();
        cpu_Img = cv::Mat(rgbImage.GetRows(), rgbImage.GetCols(), CV_8UC3, rgbImage.GetData(),rowBytes);//this operator need 5m
        if(TaskFlag.CameraBuff[0].Status == WAIT_SAVE)
        {
            Global_Img[0].upload(cpu_Img);
            TaskFlag.CameraBuff[0].Status = WAIT_DEAL;
        }
        else if(TaskFlag.CameraBuff[1].Status == WAIT_SAVE)
        {
            Global_Img[1].upload(cpu_Img);
            TaskFlag.CameraBuff[1].Status = WAIT_DEAL;
        }
        Camera_FPS_Detect();
    }
}


/**

 * @brief PointGreyCamera::PointGrey_Init
 */
void PointGreyCamera::PointGrey_Init(void)
{
    PrintBuildInfo();

    FlyCapture2::Error error;

    PGRGuid guid;
    BusManager busMgr;
    unsigned int numCameras;

    error = busMgr.GetNumOfCameras(&numCameras);
    if (error != PGRERROR_OK)
    {
        PrintError(error);
        return;
    }
    cout << "Number of cameras detected: " << numCameras << endl;


    error = busMgr.GetCameraFromIndex(0, &guid);
    if (error != PGRERROR_OK)
    {
        PrintError(error);
        return ;
    }
    RunSingleCamera(guid);

    std::chrono::milliseconds delay(1000);
    std::this_thread::sleep_for(delay);

    error = cam.Connect(&guid);// Connect to camera
    cout<<"Camera is Connect!"<<endl;



    FC2Config config;
    config.numBuffers = 0;
    config.numImageNotifications = 0;
    config.minNumImageNotifications = 0;
    config.grabTimeout = TIMEOUT_UNSPECIFIED;
    config.grabMode = UNSPECIFIED_GRAB_MODE;
    config.isochBusSpeed = BUSSPEED_ANY;
    config.asyncBusSpeed = BUSSPEED_ANY;
    config.bandwidthAllocation = BANDWIDTH_ALLOCATION_UNSPECIFIED;
    config.registerTimeoutRetries = 0;
    config.registerTimeout = 0;
    config.highPerformanceRetrieveBuffer = false;
    memset( config.reserved, 0, sizeof(config.reserved) );
    cam.SetConfiguration(&config);

    Property property;//Set to shutter time ,write by Tinker
    property.type = SHUTTER;
    property.autoManualMode = 0;
    property.valueA = 73;
    property.valueB = 0;
    property.absValue = 1.01;
    property.present = 1;
    cam.SetProperty(&property);ww

    property.type = AUTO_EXPOSURE;
    property.autoManualMode = 0;
    property.valueA = 96;
    property.valueB = 0;
    property.absValue = -1;
    property.present = 1;
    property.onOff = 1;
    cam.SetProperty(&property);

    Format7ImageSettings format7ImageSettings;
    format7ImageSettings.offsetY = 500;
    format7ImageSettings.offsetX = 200;
    format7ImageSettings.width = 1104;
    format7ImageSettings.height = 600;
    format7ImageSettings.pixelFormat = PIXEL_FORMAT_RAW8;
    float packet = 9660;
    cam.SetFormat7Configuration(&format7ImageSettings,packet);

    error = cam.StartCapture();
    cout<<"Start Capture!"<<endl;
}






/**
 * @brief PointGreyCamera::FPS_Detect
 * @return FPS
 */
int PointGreyCamera::Camera_FPS_Detect(void)
{
  static double lasttime = 0,time = 0;
  static int FPS = 0;
  time = (double)getTickCount() / getTickFrequency();
  if(time - lasttime > 1.0)
  {
     lasttime = time;
     cout <<"Camera_FPS : "<< FPS <<"  Time : "<<time<< endl;
     FPS = 0;
  }
  FPS++;
  return FPS;
}


void PointGreyCamera::PrintBuildInfo(void)
{
    FC2Version fc2Version;
    Utilities::GetLibraryVersion(&fc2Version);

    ostringstream version;
    version << "FlyCapture2 library version: " << fc2Version.major << "."
            << fc2Version.minor << "." << fc2Version.type << "."
            << fc2Version.build;
    cout << version.str() << endl;

    ostringstream timeStamp;
    timeStamp << "Application build date: " << __DATE__ << " " << __TIME__;
    cout << timeStamp.str() << endl << endl;
}

void PointGreyCamera::PrintCameraInfo(CameraInfo *pCamInfo)
{
    cout << endl;
    cout << "*** CAMERA INFORMATION ***" << endl;
    cout << "Serial number - " << pCamInfo->serialNumber << endl;
    cout << "Camera model - " << pCamInfo->modelName << endl;
    cout << "Camera vendor - " << pCamInfo->vendorName << endl;
    cout << "Sensor - " << pCamInfo->sensorInfo << endl;
    cout << "Resolution - " << pCamInfo->sensorResolution << endl;
    cout << "Firmware version - " << pCamInfo->firmwareVersion << endl;
    cout << "Firmware build time - " << pCamInfo->firmwareBuildTime << endl
         << endl;
}

void PointGreyCamera::PrintError(FlyCapture2::Error error) { error.PrintErrorTrace(); }

int PointGreyCamera::RunSingleCamera(PGRGuid guid)
{
    const int k_numImages = 10;

    FlyCapture2::Error error;

    // Connect to a camera
    Camera cam;
    error = cam.Connect(&guid);
    if (error != PGRERROR_OK)
    {
        PrintError(error);
        return -1;
    }

    // Get the camera information
    CameraInfo camInfo;
    error = cam.GetCameraInfo(&camInfo);
    if (error != PGRERROR_OK)
    {
        PrintError(error);
        return -1;
    }

    PrintCameraInfo(&camInfo);

    // Get the camera configuration
    FC2Config config;
    error = cam.GetConfiguration(&config);
    if (error != PGRERROR_OK)
    {
        PrintError(error);
        return -1;
    }

    // Set the number of driver buffers used to 10.
    config.numBuffers = 10;

    // Set the camera configuration
    error = cam.SetConfiguration(&config);
    if (error != PGRERROR_OK)
    {
        PrintError(error);
        return -1;
    }

    // Start capturing images
    error = cam.StartCapture();
    if (error != PGRERROR_OK)
    {
        PrintError(error);
        return -1;
    }

    Image rawImage;
    for (int imageCnt = 0; imageCnt < k_numImages; imageCnt++)
    {
        // Retrieve an image
        error = cam.RetrieveBuffer(&rawImage);
        if (error != PGRERROR_OK)
        {
            PrintError(error);
            continue;
        }

        cout << "Grabbed image " << imageCnt << endl;

        // Create a converted image
        Image convertedImage;

        // Convert the raw image
        error = rawImage.Convert(PIXEL_FORMAT_MONO8, &convertedImage);
        if (error != PGRERROR_OK)
        {
            PrintError(error);
            return -1;
        }
    }

    // Stop capturing images
    error = cam.StopCapture();
    if (error != PGRERROR_OK)
    {
        PrintError(error);
        return -1;
    }

    // Disconnect the camera
    error = cam.Disconnect();
    if (error != PGRERROR_OK)
    {
        PrintError(error);
        return -1;
    }

    return 0;
}


