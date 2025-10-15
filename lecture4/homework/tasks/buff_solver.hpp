#ifndef AUTO_BUFF__SOLVER_HPP
#define AUTO_BUFF__SOLVER_HPP
#include <opencv2/opencv.hpp>
#include "buff_detector.hpp"
namespace auto_buff
{
class Buff_Solver
{
public:
    void solvePnP(FanBlade fanblade,cv::Mat& rvec, cv::Mat& tvec,cv::Mat camera_matrix,cv::Mat distort_coeffs);
};
}  // namespace auto_buff
#endif  // SOLVER_HPP