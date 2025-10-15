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
cv::Point3d backProjectToZ_distort(double u, double v,
                               const cv::Mat& K,
                               const cv::Mat& dist,
                               const cv::Mat& rvec, const cv::Mat& tvec,
                               double Z)
{
    std::vector<cv::Point2f> src = {{float(u), float(v)}};
    std::vector<cv::Point2f> dst;
    undistortPoints(src, dst, K, dist);   

    double xn = dst[0].x;
    double yn = dst[0].y;

    cv::Mat xn_mat = (cv::Mat_<double>(3,1) << xn, yn, 1.0);

    cv::Mat R;
    Rodrigues(rvec, R);
    double r3xn = R.row(2).t().dot(xn_mat); 
    double t3   = tvec.at<double>(2);
    double s    = Z / (r3xn + t3);

    cv::Mat Xw = s * xn_mat;
    return cv::Point3d(Xw.at<double>(0),
                   Xw.at<double>(1),
                   Xw.at<double>(2));
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
        data["center_x"] = pt2d.x;
        data["center_y"] = pt2d.y;
        data["center_z"] = z;
        if(circle==0&&yes==1){
        data["R_x"] = pt2d_R.x;
        data["R_y"] = pt2d_R.y;
        data["R_z"] = z;
        }
    }
        plotter.plot(data); 
    }

    return 0;
}