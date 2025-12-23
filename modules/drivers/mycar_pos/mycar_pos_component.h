#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <thread>

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
  void GPS_XY(double lat, double lon, double* x, double* y);

  std::shared_ptr<
      apollo::cyber::Writer<apollo::localization::LocalizationEstimate>>
      writer_;
  MycarPosConf conf_;

  std::atomic<bool> running_ = {false};
  std::unique_ptr<std::thread> thread_;
  int fd_ = -1;

  // Reference point for GPS to XY conversion
  double ref_lat_ = 0.0;
  double ref_lon_ = 0.0;
  bool ref_initialized_ = false;

  // Earth radius in meters
  static constexpr int CONSTANTS_RADIUS_OF_EARTH = 6371000;
};

CYBER_REGISTER_COMPONENT(MycarPosComponent)

}  // namespace mycar_pos
}  // namespace drivers
}  // namespace apollo
