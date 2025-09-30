#include "io/my_camera.hpp"
#include "tasks/yolo.hpp"
#include "opencv2/opencv.hpp"
#include "tools/img_tools.hpp"

int main()
{
    // 初始化相机、yolo类
    auto_aim::YOLO test_yolo("homework/configs/yolo.yaml", true);
    myCamera test_camera;
    //测试用
    /*std::string inPath="homework/test.jpg";
    cv::Mat img = cv::imread(inPath);
    std::list<auto_aim::Armor> all_armors= test_yolo.detect(img);
        // 处理识别结果，绘图
        for (const auto_aim::Armor& armor : all_armors)
        {
            for(int i=0;i<4;i++){
                tools::draw_point(img, armor.points[i]);
            }
            const std::string text=auto_aim::COLORS[armor.color]+auto_aim::ARMOR_NAMES[armor.name];
            tools::draw_points(img, armor.points);
            tools::draw_text(img, text, armor.center);
        }
        // 显示图像

        cv::resize(img, img , cv::Size(640, 480));
        cv::imshow("img", img);
        cv::waitKey(0);*/
    //测试用end
    while (1) {
        cv::Mat img;
        // 调用相机读取图像
        if(test_camera.read(img)==-1){
            std::cout<<"相机打开失败！"<<std::endl;
            return -1;
        }

        // 调用yolo识别装甲板
        std::list<auto_aim::Armor> all_armors= test_yolo.detect(img);
        // 处理识别结果，绘图
        for (const auto_aim::Armor& armor : all_armors)
        {
            for(int i=0;i<4;i++){
                tools::draw_point(img, armor.points[i]);
            }
            const std::string text=auto_aim::COLORS[armor.color]+auto_aim::ARMOR_NAMES[armor.name];
            tools::draw_points(img, armor.points);
            tools::draw_text(img, text, armor.center);
        }
        // 显示图像

        cv::resize(img, img , cv::Size(640, 480));
        cv::imshow("img", img);
        if (cv::waitKey(0) == 'q') {
        break;
        }
    }

    return 0;
}