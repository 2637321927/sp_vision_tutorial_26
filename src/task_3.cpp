#include <chrono>
#include <opencv2/opencv.hpp>
#include "tools/trajectory.hpp"
#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"
#include "tools/exiter.hpp"
#include "tasks/auto_aim/target.hpp"
const std::string keys =
  "{help h usage ? | | 输出命令行参数说明}"
  "{@config-path   | | yaml配置文件路径 }";

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>("@config-path");
  if (cli.has("help") || !cli.has("@config-path")) {
    cli.printMessage();
    return 0;
  }

  // 初始化工具类
  tools::Exiter exiter;
  tools::Plotter plotter;

  // 初始化io类
  io::Camera camera(config_path);
  io::Gimbal gimbal(config_path);

  // 初始化auto_aim类
  auto_aim::YOLO yolo(config_path, true);
  auto_aim::Solver solver(config_path);

  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;
     // double target_yaw;
 // tools::xyz2ypd(armor.xyz_in_world);
   
  //double target_yaw = center_ypr[0];
 // auto center_xyz_world = armor.xyz_in_world;
  // double distance =std::sqrt(armor.xyz_in_world[0]*armor.xyz_in_world[0]+
        //armor.xyz_in_world[1]*armor.xyz_in_world[1]);
  //tools::Trajectory trajectory (gimbal.state().bullet_speed, distance, armor.xyz_in_world[2]);//计算弹道
       // double  target_pitch=-trajectory.pitch;
  // 初始化Target（装甲板,协方差）

   
   
  // 初始化云台瞄准靶机中心
  // 解算靶机中心世界系角度
  //std::vector<Eigen::Vector4d> predicted_armors = target.armor_xyza_list();
  //Eigen::Vector3d center_xyz_world = predicted_armors[0].head<3>();
  //Eigen::Vector3d center_ypr = tools::xyz2ypd(center_xyz_world); // 中心yaw/pitch
  //double distance = std::sqrt(
    //center_xyz_world[0] * center_xyz_world[0] + center_xyz_world[1] * center_xyz_world[1]);
  //tools::Trajectory trajectory(gimbal.state().bullet_speed, distance, center_xyz_world[2]);
 // float initial_pitch = -trajectory.pitch;
  //gimbal.send(true, false, 
              //static_cast<float>(center_ypr[0]), 
             // initial_pitch);
 // std::vector<cv::Point3f> center_point_world;
  //center_point_world.emplace_back(center_xyz_world[0],
                                    //  center_xyz_world[1],
                                   //   center_xyz_world[2]);
 std::unique_ptr< auto_aim::Target> target_ptr = nullptr; 
 Eigen::Vector3d  center_ypr;
 int count=0;
 double distance;
 float fitted_pitch;
 double time_threshold;
 Eigen::Vector3d target_center;
  while (!exiter.exit()) {
    // Your code start
    Eigen::VectorXd P0_dig(11);
    P0_dig << 1,  // x位置协方差
              0.1,   // vx速度协方差
              1,  // y位置协方差
              0.1,   // vy速度协方差
              1,  // z位置协方差
              0.1,   // vz速度协方差
              1,  // a角度协方差
              3,   // w角速度协方差（重点：拟合角速度的协方差）
              0.005, // r半径协方差
              0.005, // l协方差
              0.005; // h协方差
    camera.read(img, t);  // 获取图像和时间戳
    std::list<auto_aim::Armor> armors = yolo.detect(img);
    q = gimbal.q(t);
    solver.set_R_gimbal2world(q);
    if(!armors.empty()){
      auto_aim::Armor target_armor = *std::min_element(armors.begin(), armors.end(),
      [](const auto_aim::Armor & a, const auto_aim::Armor & b) {
        return a.ypd_in_world[2] < b.ypd_in_world[2];  // 按距离排序（ypd[2]为距离）
      });
    if (!target_ptr || !target_ptr->checkinit()) {
      target_ptr = std::make_unique<auto_aim::Target>(
        target_armor,          // 初始装甲板数据
        t,          // 初始时间戳
        P0_dig               // EKF初始协方差
      );
      continue;  // 初始化后跳过本次循环，等待下一帧进行预测和更新
    }
     float fitted_pitch;
    if (target_ptr->convergened()) {
    Eigen::VectorXd ekf_state = target_ptr->ekf_x();  // 获取EKF当前状态
    
    target_center[0]=ekf_state[0];
    target_center[1]=ekf_state[2];
    target_center[2]=ekf_state[4];
     // X轴位置
      // Y轴位置
     // Z轴位置
    center_ypr = tools::xyz2ypd(target_center); // 中心yaw/pitch
      distance = std::sqrt(
    target_center[0] * target_center[0] + target_center[1] * target_center[1]);
  count++;
}
      solver.solve(target_armor);  
      // EKF预测：根据时间差更新状态
      target_ptr->predict(t);  // 传入当前时间戳，自动计算dt并预测
      // EKF更新：用当前装甲板姿态修正状态
      target_ptr->update(target_armor);  // 更新EKF，拟合角速度
      // 提取EKF拟合的角速度（w：target.ekf_x()[7]，状态向量第8位）
      double ekf_fitted_w = std::abs(target_ptr->ekf_x()[7]);  // 核心：小陀螺旋转角速度
      std::vector<Eigen::Vector4d> predicted_armors = target_ptr->armor_xyza_list();
      // 遍历预测的装甲板，找即将到达中心的目标
      bool is_armor_in_center = false;
     if(target_ptr->convergened()){
      for (const auto& armor_xyza : predicted_armors) {
        double time_delay;
        if (ekf_fitted_w >= 8.0) {
          time_delay = 0.01;  
        } else if (ekf_fitted_w >= 5.0) {
          time_delay = 0.02;  
        } else {
          time_delay = 0.1;  
        }
        Eigen::Vector3d armor_xyz(armor_xyza[0], armor_xyza[1], armor_xyza[2]);
        double armor_angle = armor_xyza[3];  // 装甲板当前角度 
        double center_angle = center_ypr[0];  // 中心角度 
        tools::Trajectory trajectory(gimbal.state().bullet_speed, distance, target_center[2]);
     fitted_pitch = -trajectory.pitch;
        // 计算装甲板与中心的角度差（最短路径）
       double angle_diff = tools::limit_rad(center_angle - armor_angle);
        // 计算预测到达中心的时间（时间=角度差/角速度，仅当角速度>0时有效）
        if (std::abs(ekf_fitted_w )> 0.1) {  // 角速度足够大，避免除以零
          double predict_arrive_time = std::abs(angle_diff/ ekf_fitted_w)+time_delay;
          double time_threshold = trajectory.fly_time; // 预测到达时间阈值,子弹飞行时间
          // 判定开火时机
          bool is_time_ok = (predict_arrive_time < time_threshold&&predict_arrive_time > time_threshold/2);

        if (is_time_ok) {
          is_armor_in_center = true;  // 满足“Target预测的开火时机”
          break;  // 找到目标装甲板，无需遍历其他
        }
      }

    }
  }
  fitted_pitch-=0.07;
      if(target_ptr->convergened()) {
              if (is_armor_in_center) {
        // 装甲板转到中心：发送命令
        gimbal.send(true, true, 
                    static_cast<float>(center_ypr[0]), 
                    fitted_pitch);
        }
      else {
                gimbal.send(true, false, 
                    static_cast<float>(center_ypr[0]), 
                    static_cast<float>(center_ypr[1]));
      }
             //plotter

    }
  
    nlohmann::json data;
    data["w"] = ekf_fitted_w;
    data["boll_ok"]=is_armor_in_center;
    plotter.plot(data);
    // Your code end

  
}
}
}
