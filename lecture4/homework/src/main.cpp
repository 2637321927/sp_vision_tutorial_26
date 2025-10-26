#include <nlohmann/json.hpp>
#include "io/camera.hpp"
#include "tasks/buff_solver.hpp"
#include <chrono>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include "tools/plotter.hpp"
#include "tasks/buff_detector.hpp"
#include <iostream>
cv::Point3d backProjectToZ_distort(double px_x, double px_y,
                                   const cv::Mat& camera_matrix,
                                   const cv::Mat& distort_coeffs,
                                   const cv::Mat& rvec,
                                   const cv::Mat& tvec,
                                   double target_z)
{
    // 1. 去畸变
    std::vector<cv::Point2d> distorted_pts = {{px_x, px_y}};
    std::vector<cv::Point2d> undistorted_pts;
    cv::undistortPoints(distorted_pts, 
                        undistorted_pts, 
                        camera_matrix, 
                        distort_coeffs, 
                        cv::Mat(),
                        camera_matrix);

    // 提取归一化坐标（u, v）
    double u = undistorted_pts[0].x;
    double v = undistorted_pts[0].y;
    // 旋转向量→旋转矩阵
    cv::Mat R;
    cv::Rodrigues(rvec, R); // R: 3x3旋转矩阵（世界→相机）
    // 反投影公式推导：已知Z=target_z，解X、Y（相机坐标系）
    double R11 = R.at<double>(0, 0), R12 = R.at<double>(0, 1), R13 = R.at<double>(0, 2);
    double R21 = R.at<double>(1, 0), R22 = R.at<double>(1, 1), R23 = R.at<double>(1, 2);
    double R31 = R.at<double>(2, 0), R32 = R.at<double>(2, 1), R33 = R.at<double>(2, 2);

    double tx = tvec.at<double>(0, 0);
    double ty = tvec.at<double>(1, 0);
    double tz = tvec.at<double>(2, 0);

    // 解线性方程得到相机坐标系下的3D点（Xc, Yc, Zc=target_z）
    double denominator = R31 * u + R32 * v + R33;
    if (std::fabs(denominator) < 1e-6)
    {
        std::cerr << "[backProject] Error: Denominator is zero, invalid projection!" << std::endl;
        return cv::Point3d(0, 0, 0);
    }

    double Xc = ( (target_z + tz) * u - R11 * tx - R12 * ty - R13 * tz ) / denominator - (R13 * target_z)/denominator;
    double Yc = ( (target_z + tz) * v - R21 * tx - R22 * ty - R23 * tz ) / denominator - (R23 * target_z)/denominator;

    // 转换为世界坐标系（相机→世界：R^T*(Xc - tx, Yc - ty, Zc - tz)）
    cv::Mat camera_point = (cv::Mat_<double>(3, 1) << Xc, Yc, target_z);
    cv::Mat world_point = R.t() * (camera_point - tvec);

    return cv::Point3d(world_point.at<double>(0,0), 
                       world_point.at<double>(1,0), 
                       world_point.at<double>(2,0));
}

cv::Point2d circleCenter3P(const cv::Point2d& p1,
                           const cv::Point2d& p2,
                           const cv::Point2d& p3
                 )
{
    double A = p2.x - p1.x;
    double B = p2.y - p1.y;
    double C = p3.x - p1.x;
    double D = p3.y - p1.y;
    std::cout<<p1.x<<" "<<p1.y<<std::endl;
    std::cout<<p2.x<<" "<<p2.y<<std::endl;
    std::cout<<p3.x<<" "<<p3.y<<std::endl;
    double E = A*(p1.x + p2.x) + B*(p1.y + p2.y);
    double F = C*(p1.x + p3.x) + D*(p1.y + p3.y);

    double G = 2.0 * (A*(p3.y - p2.y) - B*(p3.x - p2.x));
    if (std::fabs(G) < 1e-8)         
        return cv::Point2d(std::numeric_limits<double>::quiet_NaN(),
                           std::numeric_limits<double>::quiet_NaN());

    double cx = (D*E - B*F) / G;
    double cy = (A*F - C*E) / G;
    return cv::Point2d(cx, cy);
}
int main()
{
    static const cv::Mat camera_matrix =
     (cv::Mat_<double>(3, 3) <<  1286.307063384126 , 0                  , 645.34450819155256, 
                                0                 , 1288.1400736562441 , 483.6163720308021 , 
                                0                 , 0                  , 1                   );// 相机内参
    static const cv::Mat distort_coeffs =
    (cv::Mat_<double>(1, 5) << -0.47562935060124745, 0.21831745829617311, 0.0004957613589406044, -0.00034617769548693592, 0);// 畸变
    /*io::Camera camera(2.5, 16.9, "2bdf:0001");
    std::chrono::steady_clock::time_point timestamp;  */
    auto_buff::Buff_Detector detector;
    cv::VideoCapture cap("homework/assets/test.avi"); 
    int solvepnp_open=1;
    int circle=0;
    std::vector<cv::Point2d> pt2d_Rs(3);
             cv::Point2d pt2d_R;
         cv::Point3d pt3d_R;
         int z=0;
    while(true){
        cv::Mat img1;
        cap >> img1; 
        cv::Mat img=img1.clone();
        //camera.read(img, timestamp);
        std::vector<auto_buff::FanBlade>fanblades = detector.detect(img);
        if(fanblades.size() == 0) {
            continue;
        }
        auto_buff::FanBlade fanblade=fanblades[0];
        cv::Mat rvec, tvec;
        auto_buff::Buff_Solver a;
        cv::Scalar color;
            std::string type_name;
            switch (fanblade.type) {
                case auto_buff::_target:
                    color = cv::Scalar(0, 255, 0);  
                    type_name = "_target";
                    break;
                case auto_buff::_light:
                    color = cv::Scalar(0, 255, 255); 
                    type_name = "_light";
                    break;
                case auto_buff::_unlight:
                    color = cv::Scalar(0, 0, 255);  
                    type_name = "_unlight";
                    break;
            }
        if(solvepnp_open)  {
        auto_buff::Buff_Solver a;   
        a.solvePnP(fanblade,rvec, tvec,camera_matrix,distort_coeffs);
        }
        for (size_t i = 0; i < fanblade.points.size(); ++i) {
            cv::circle(img, fanblade.points[i], 3, color, -1);
            cv::putText(img, std::to_string(i), 
            cv::Point(fanblade.points[i].x + 5, fanblade.points[i].y - 5),
            cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
         }
            // 绘制中心点
        cv::circle(img, fanblade.center, 5, color, -1);
        cv::putText(img, "CENTER", 
        cv::Point(fanblade.center.x + 10, fanblade.center.y - 10),
        cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
            
            // 绘制类型标签
         cv::putText(img, type_name, 
                       cv::Point(fanblade.center.x - 20, fanblade.center.y - 20),
                       cv::FONT_HERSHEY_SIMPLEX, 0.7, color, 2);        
        cv::Point2d pt2d=fanblade.center;
        cv::Point3d pt3d=backProjectToZ_distort(pt2d.x, pt2d.y,camera_matrix, distort_coeffs, rvec, tvec,0);
        int yes=0;
        if(circle<3){
            pt2d_Rs[circle]=pt2d;
            circle++;
        }
        else{
            pt2d_R=circleCenter3P(pt2d_Rs[0],pt2d_Rs[1],pt2d_Rs[2]);
            pt3d_R=backProjectToZ_distort(pt2d_R.x, pt2d_R.y,camera_matrix, distort_coeffs, rvec, tvec,0);
            circle=0;
            pt2d_Rs.clear();
            yes=1;

        }

        cv::resize(img,img,{},0.8,0.8);
        cv::imshow("Detection Results", img);
        if (cv::waitKey(30) == 27) { // 按ESC键退出
            break;
        }
        tools::Plotter plotter;
        nlohmann::json data;
        if(1){
        data["center_x"] = pt3d.x;
        data["center_y"] = pt3d.y;
        data["center_z"] = z;
        if(circle==0&&yes==1){
        data["R_x"] = pt3d_R.x;
        data["R_y"] = pt3d_R.y;
        data["R_z"] = z;
        }
    }
        plotter.plot(data); 
    }

    return 0;
}
