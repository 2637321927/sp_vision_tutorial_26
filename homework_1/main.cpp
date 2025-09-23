#include <opencv2/opencv.hpp>
#include <iostream>
#include "tool.hpp"
#include "fmt/core.h"
int main(int argc, char **argv)
{
 if (argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " <input_image>\n";
        return 1;
    }
    std::string inPath = argv[1];
    // 读取图片
     cv::Mat img = cv::imread(inPath);
    if (img.empty())               
    {
        std::cerr << "imread failed!\n";
        return -1;
    }
    scale_img si=scaling(img);
    fmt::print("scale={:.2f} x_move={:.2f} y_move={:.2f} new_wide={:.2f} new_high={:.2f}\n", si.scale, si.x_move, si.y_move, si.new_wide, si.new_high);
    //都保留两位小数，方索的比例，偏移距离，新图的宽高
    cv::imwrite("./img_new.jpg", si.img_new);//不知道学长的路径，直接放在当前目录下了
    cv::imshow("-new picture-", si.img_new);
    cv::waitKey(0); 
 
    return 0;
}