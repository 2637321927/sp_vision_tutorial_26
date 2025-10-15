
#include "buff_solver.hpp"
#include "buff_detector.hpp"

namespace auto_buff
{
void Buff_Solver::solvePnP(FanBlade fanblade,cv::Mat& rvec, cv::Mat& tvec,cv::Mat camera_matrix,cv::Mat distort_coeffs)
{

    // 在这里实现 solvePnP 逻辑

    std::vector<cv::Point3f> objectPoints{
        {0.0f, 150.0f, 0.0f}, 
        {-150.0f, 0.0f, 0.0f}, 
        {0.0f, -150.0f, 0.0f}, 
        {150.0f, 0.0f, 0.0f},
        {0.0f, 0.0f, 0.0f}
    }; // 3D点
    std::vector<cv::Point2f> imagePoints{
        fanblade.points[0],
        fanblade.points[1],
        fanblade.points[2],
        fanblade.points[3],
        fanblade.points[4]
    };   // 2D点

        bool success = cv::solvePnP(objectPoints, imagePoints, camera_matrix, distort_coeffs , rvec, tvec);
    if (!success) {
        std::cout << "solvePnP failed!" << std::endl;
        return;
    }
    return ;
    

}

}  // namespace auto_buff