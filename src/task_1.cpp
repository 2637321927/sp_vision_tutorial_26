#include <chrono>
#include <opencv2/opencv.hpp>

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
  Eigen::Quaterniond q;// 四元数
  std::chrono::steady_clock::time_point t;

  while (!exiter.exit()) {
    // Your code start
    camera.read(img,t);//获取图像和时间戳
    std::list<auto_aim::Armor> armors=yolo.detect(img);//识别装甲板
    io::GimbalState gimbal_state = gimbal.state();
    q=gimbal.q(t);//获取云台四元数
    solver.set_R_gimbal2world(q);//云台->world juzhen
    if (!armors.empty()){//若识别到装甲板
        auto_aim::Armor armor=armors.front();
        solver.solve(armor);//求解装甲板位姿
        Eigen::Vector3d  armor_ypd = tools::xyz2ypd(armor.xyz_in_world);
        double target_yaw = armor_ypd[0];
        double target_pitch = armor_ypd[1];
        target_pitch=-target_pitch;
          // 发送控制指令到云台
        gimbal.send(true, false, target_yaw, target_pitch);
        //plotter
        nlohmann::json data;
        data["target_yaw"] = target_yaw;
        data["target_pitch"] = target_pitch;
        plotter.plot(data);
    }
  }
  // Your code end
  


  return 0;
}


  return 0;
}
