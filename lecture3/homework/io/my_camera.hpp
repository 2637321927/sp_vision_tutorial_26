#include "hikrobot/include/MvCameraControl.h"
#include <opencv2/opencv.hpp>

class myCamera
{
    public:
    myCamera();
    ~myCamera();
    int read(cv::Mat& img);
    private:
    void * handle_;
    bool is_opened_;
    cv::Mat transfer(MV_FRAME_OUT& raw);
    
};