#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <thread>

#define ACCEPT_USE_OF_DEPRECATED_PROJ_API_H
#include "proj_api.h"

#include "modules/common_msgs/localization_msgs/localization.pb.h"
#include "modules/drivers/mycar_pos/proto/mycar_pos_conf.pb.h"

#include "cyber/cyber.h"

namespace apollo {
namespace drivers {
namespace mycar_pos {

class MycarPosComponent : public apollo::cyber::Component<> {
 public:
  bool Init() override;
  ~MycarPosComponent();

 private:
  void Run();
  bool ConfigureSerialPort();
  void ProcessData(const std::string& data);
  bool ParseGPFPD(const std::string& line, double* lat, double* lon,
                  double* heading, double* ve, double* vn, double* vu);

  std::shared_ptr<
      apollo::cyber::Writer<apollo::localization::LocalizationEstimate>>
      writer_;
  MycarPosConf conf_;

  std::atomic<bool> running_ = {false};
  std::unique_ptr<std::thread> thread_;
  int fd_ = -1;

  projPJ wgs84pj_source_ = nullptr;
  projPJ utm_target_ = nullptr;
};

CYBER_REGISTER_COMPONENT(MycarPosComponent)

}  // namespace mycar_pos
}  // namespace drivers
}  // namespace apollo
