#ifndef TOOL_HPP
#define TOOL_HPP
#define SCALE 640 //目标大小，方便修改
#include <opencv2/opencv.hpp>
#include <iostream>
struct scale_img
{
    double scale;
    double x_move;
    double y_move;
    double new_wide;
    double new_high;
    cv::Mat img_new;
};
scale_img scaling(cv::Mat& img);


#endif //TOOL_HPP
