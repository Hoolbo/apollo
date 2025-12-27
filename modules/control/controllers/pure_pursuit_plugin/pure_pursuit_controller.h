#pragma once

#include <memory>
#include <string>
#include <vector>

// 1. 【关键修改】引入正确的基类头文件
#include "modules/control/control_component/controller_task_base/control_task.h"
#include "cyber/plugin_manager/plugin_manager.h" // 引入宏注册支持

namespace apollo {
namespace control {

// 2. 【关键修改】继承 ControlTask 而不是 Controller
class PurePursuitController : public ControlTask {
 public:
  PurePursuitController() = default;
  virtual ~PurePursuitController() = default;

  // 3. 【关键修改】Init 函数签名变了 (去掉了 control_conf)
  common::Status Init(std::shared_ptr<DependencyInjector> injector) override;

  // 4. 【关键修改】ComputeControlCommand 函数签名变了 (去掉了 last_command)
  common::Status ComputeControlCommand(
      const localization::LocalizationEstimate *localization,
      const canbus::Chassis *chassis,
      const planning::ADCTrajectory *trajectory,
      ControlCommand *cmd) override;

  common::Status Reset() override;

  void Stop() override;

  std::string Name() const override;

 private:
  int CalcTargetIndex(const double x, const double y, const double v,
                      const planning::ADCTrajectory *trajectory);

 private:
  // 你的 ROS 物理参数
  double L_ = 0.65;     // 轴距
  double K_ = 0.1;      // 前视增益
  double Lfc_ = 3.0;    // 最小前视距离
  double max_steer_angle_ = 0.8; // 最大转角(rad)
};

// 5. 【关键修改】注册为 ControlTask 插件
CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::control::PurePursuitController, ControlTask)

}  // namespace control
}  // namespace apollo