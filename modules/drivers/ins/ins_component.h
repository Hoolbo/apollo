#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <thread>

#include "modules/common_msgs/localization_msgs/localization.pb.h"
#include "modules/common_msgs/sensor_msgs/gnss.pb.h"
#include "modules/common_msgs/sensor_msgs/gnss_best_pose.pb.h"
#include "modules/common_msgs/sensor_msgs/heading.pb.h"
#include "modules/common_msgs/sensor_msgs/imu.pb.h"
#include "modules/common_msgs/sensor_msgs/ins.pb.h"
#include "modules/drivers/ins/proto/ins_conf.pb.h"

#include "cyber/cyber.h"

namespace apollo {
namespace drivers {
namespace ins {

// Parsed GPFPD data structure
struct GpfpdData {
  double gps_week = 0.0;
  double gps_time = 0.0;
  double heading = 0.0;
  double pitch = 0.0;
  double roll = 0.0;
  double latitude = 0.0;
  double longitude = 0.0;
  double altitude = 0.0;
  double ve = 0.0;  // East velocity
  double vn = 0.0;  // North velocity
  double vu = 0.0;  // Up velocity
  double baseline = 0.0;
  int nsv1 = 0;  // Satellites on primary antenna
  int nsv2 = 0;  // Satellites on secondary antenna
  int status = 0;
};

class InsComponent : public apollo::cyber::Component<> {
 public:
  bool Init() override;
  ~InsComponent();

 private:
  void Run();
  bool ConfigureSerialPort();
  void ProcessData(const std::string& data);
  bool ParseGPFPD(const std::string& line, GpfpdData* data);
  void GPS_XY(double lat, double lon, double* x, double* y);

  // LocalizationEstimate writer (main output)
  std::shared_ptr<
      apollo::cyber::Writer<apollo::localization::LocalizationEstimate>>
      writer_;

  // GNSS module emulation writers
  std::shared_ptr<apollo::cyber::Writer<apollo::drivers::gnss::GnssBestPose>>
      gnss_best_pose_writer_;
  std::shared_ptr<apollo::cyber::Writer<apollo::drivers::gnss::Gnss>>
      gps_writer_;
  std::shared_ptr<apollo::cyber::Writer<apollo::drivers::gnss::Heading>>
      heading_writer_;
  std::shared_ptr<apollo::cyber::Writer<apollo::drivers::gnss::Imu>>
      imu_writer_;
  std::shared_ptr<apollo::cyber::Writer<apollo::drivers::gnss::InsStat>>
      ins_stat_writer_;

  // Localization module emulation writers
  std::shared_ptr<
      apollo::cyber::Writer<apollo::localization::LocalizationStatus>>
      localization_status_writer_;

  InsConf conf_;

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

CYBER_REGISTER_COMPONENT(InsComponent)

}  // namespace ins
}  // namespace drivers
}  // namespace apollo
