#include <opencv2/opencv.hpp>
#include <iostream>
#include "tool.hpp"


scale_img scaling(cv::Mat& img)
{
    double wide=img.cols;
    double high=img.rows;
    cv::Mat img_new_pre;
    double new_wide;
    double new_high;
    double scale;
    double x_move=0;
    double y_move=0;
    if(wide>=high)
    {
        scale=wide/SCALE;
        new_high=high/scale;
        new_wide=SCALE;
        y_move=(SCALE-new_high)/2;//我理解的图片偏移距离是图片距离边框的距离
        cv::resize(img, img_new_pre, cv::Size(SCALE, new_high)); 
    }
    else
    {
        scale=high/SCALE;
        new_wide=wide/scale;
        new_high=SCALE;
        x_move=(SCALE-new_wide)/2;
        cv::resize(img, img_new_pre, cv::Size(new_wide, SCALE)); 
    }
    cv::Mat img_new(SCALE, SCALE, img.type(), cv::Scalar(0, 0, 0));
    img_new_pre.copyTo(img_new(cv::Rect(x_move, y_move, new_wide, new_high)));
    scale_img si{scale,x_move,y_move,new_wide,new_high,img_new};
    return si;
}
