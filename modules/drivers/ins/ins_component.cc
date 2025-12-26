#include "modules/drivers/ins/ins_component.h"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cmath>
#include <sstream>
#include <vector>

#include "Eigen/Geometry"

namespace apollo {
namespace drivers {
namespace ins {

using apollo::drivers::gnss::Gnss;
using apollo::drivers::gnss::GnssBestPose;
using apollo::drivers::gnss::Heading;
using apollo::drivers::gnss::Imu;
using apollo::drivers::gnss::InsStat;
using apollo::drivers::gnss::SolutionType;
using apollo::localization::LocalizationEstimate;
using apollo::localization::LocalizationStatus;
using apollo::localization::MeasureState;

namespace {

constexpr double DEG_TO_RAD = M_PI / 180.0;

std::vector<std::string> split(const std::string& s, char delimiter) {
  std::vector<std::string> tokens;
  std::string token;
  std::istringstream tokenStream(s);
  while (std::getline(tokenStream, token, delimiter)) {
    tokens.push_back(token);
  }
  return tokens;
}
}  // namespace

InsComponent::~InsComponent() {
  running_ = false;
  if (thread_ && thread_->joinable()) {
    thread_->join();
  }
  if (fd_ >= 0) {
    close(fd_);
  }
}

bool InsComponent::Init() {
  if (!GetProtoConfig(&conf_)) {
    AERROR << "Unable to load ins conf file: " << ConfigFilePath();
    return false;
  }

  AINFO << "INS config: " << conf_.DebugString();

  // Create all writers
  localization_estimate_writer_ =
      node_->CreateWriter<LocalizationEstimate>("/apollo/localization/pose");

  // GNSS module emulation writers
  gnss_best_pose_writer_ =
      node_->CreateWriter<GnssBestPose>("/apollo/sensor/gnss/best_pose");
  gps_writer_ = node_->CreateWriter<Gnss>("/apollo/sensor/gnss/odometry");
  heading_writer_ = node_->CreateWriter<Heading>("/apollo/sensor/gnss/heading");
  imu_writer_ = node_->CreateWriter<Imu>("/apollo/sensor/gnss/imu");
  ins_stat_writer_ =
      node_->CreateWriter<InsStat>("/apollo/sensor/gnss/ins_stat");

  // Localization module emulation writers
  localization_status_writer_ = node_->CreateWriter<LocalizationStatus>(
      "/apollo/localization/msf_status");

  if (!ConfigureSerialPort()) {
    return false;
  }

  running_ = true;
  thread_.reset(new std::thread(&InsComponent::Run, this));
  return true;
}

bool InsComponent::ConfigureSerialPort() {
  fd_ = open(conf_.device().c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  if (fd_ == -1) {
    AERROR << "Unable to open port " << conf_.device();
    return false;
  }

  fcntl(fd_, F_SETFL, 0);

  struct termios options;
  tcgetattr(fd_, &options);

  cfsetispeed(&options, B115200);
  cfsetospeed(&options, B115200);

  options.c_cflag |= (CLOCAL | CREAD);
  options.c_cflag &= ~PARENB;
  options.c_cflag &= ~CSTOPB;
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8;
  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  options.c_iflag &= ~(IXON | IXOFF | IXANY);
  options.c_oflag &= ~OPOST;

  tcsetattr(fd_, TCSANOW, &options);

  AINFO << "Serial port configured, waiting for GPFPD data from INS device";
  return true;
}

void InsComponent::Run() {
  char buffer[1024];
  std::string residual = "";

  while (running_ && cyber::OK()) {
    int n = read(fd_, buffer, sizeof(buffer) - 1);
    if (n > 0) {
      buffer[n] = '\0';
      std::string data = residual + std::string(buffer);

      size_t pos = 0;
      size_t found = 0;
      while ((found = data.find('\n', pos)) != std::string::npos) {
        std::string line = data.substr(pos, found - pos);
        // Handle optional \r
        if (!line.empty() && line.back() == '\r') {
          line.pop_back();
        }
        ProcessData(line);
        pos = found + 1;
      }
      residual = data.substr(pos);
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
}

void InsComponent::ProcessData(const std::string& line) {
  if (line.find("$GPFPD") == std::string::npos) return;

  GpfpdData gpfpd;
  if (!ParseGPFPD(line, &gpfpd)) return;

  double timestamp = cyber::Time::Now().ToSecond();

  // Initialize reference point (3D origin)
  if (!ref_initialized_) {
    if (conf_.use_fixed_origin()) {
      ref_lat_ = conf_.origin_lat();
      ref_lon_ = conf_.origin_lon();
      ref_alt_ = conf_.origin_alt();
      AINFO << "MPS: Using FIXED GPS origin: lat=" << ref_lat_
            << ", lon=" << ref_lon_ << ", alt=" << ref_alt_;
    } else {
      ref_lat_ = gpfpd.latitude;
      ref_lon_ = gpfpd.longitude;
      ref_alt_ = gpfpd.altitude;
      AINFO << "MPS: Using DYNAMIC GPS origin (first point): lat=" << ref_lat_
            << ", lon=" << ref_lon_ << ", alt=" << ref_alt_;
    }
    ref_initialized_ = true;
  }

  // Convert GPS to local XYZ coordinates (relative to first point)
  double x = 0.0;
  double y = 0.0;
  double z = gpfpd.altitude - ref_alt_;  // Relative altitude
  GPS_XY(gpfpd.latitude, gpfpd.longitude, &x, &y);

  // Convert heading: GPFPD (North=0, CW) -> Apollo (East=0, CCW)
  // Device configured with headoffset=180 to output North=0 format
  double apollo_theta = (90.0 - gpfpd.heading) * DEG_TO_RAD;

  Eigen::Quaterniond q =
      Eigen::AngleAxisd(gpfpd.roll * DEG_TO_RAD, Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(gpfpd.pitch * DEG_TO_RAD, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(apollo_theta, Eigen::Vector3d::UnitZ());

  // ========== 1. Publish LocalizationEstimate (main output) ==========
  auto loc_msg = std::make_shared<LocalizationEstimate>();
  loc_msg->mutable_header()->set_timestamp_sec(timestamp);
  loc_msg->mutable_header()->set_module_name("ins");
  loc_msg->mutable_header()->set_frame_id("localization");
  loc_msg->set_measurement_time(timestamp);

  loc_msg->mutable_pose()->mutable_position()->set_x(x);
  loc_msg->mutable_pose()->mutable_position()->set_y(y);
  loc_msg->mutable_pose()->mutable_position()->set_z(z);

  loc_msg->mutable_pose()->mutable_orientation()->set_qw(q.w());
  loc_msg->mutable_pose()->mutable_orientation()->set_qx(q.x());
  loc_msg->mutable_pose()->mutable_orientation()->set_qy(q.y());
  loc_msg->mutable_pose()->mutable_orientation()->set_qz(q.z());
  loc_msg->mutable_pose()->set_heading(apollo_theta);

  loc_msg->mutable_pose()->mutable_linear_velocity()->set_x(gpfpd.ve);
  loc_msg->mutable_pose()->mutable_linear_velocity()->set_y(gpfpd.vn);
  loc_msg->mutable_pose()->mutable_linear_velocity()->set_z(gpfpd.vu);

  // Euler angles (roll, pitch, yaw in radians)
  loc_msg->mutable_pose()->mutable_euler_angles()->set_x(gpfpd.roll *
                                                         DEG_TO_RAD);
  loc_msg->mutable_pose()->mutable_euler_angles()->set_y(gpfpd.pitch *
                                                         DEG_TO_RAD);
  loc_msg->mutable_pose()->mutable_euler_angles()->set_z(apollo_theta);

  // Angular velocity in Vehicle Reference Frame (VRF)
  // Required by Control module when FLAGS_enable_map_reference_unify is true
  // GPFPD doesn't provide angular velocity, so we use zeros
  // This is acceptable for low-speed RTK playback scenarios
  loc_msg->mutable_pose()->mutable_angular_velocity_vrf()->set_x(0.0);
  loc_msg->mutable_pose()->mutable_angular_velocity_vrf()->set_y(0.0);
  loc_msg->mutable_pose()->mutable_angular_velocity_vrf()->set_z(0.0);

  localization_estimate_writer_->Write(loc_msg);

  // ========== 2. Publish GnssBestPose (for GpsMonitor) ==========
  auto best_pose_msg = std::make_shared<GnssBestPose>();
  best_pose_msg->mutable_header()->set_timestamp_sec(timestamp);
  best_pose_msg->set_measurement_time(timestamp);
  best_pose_msg->set_latitude(gpfpd.latitude);
  best_pose_msg->set_longitude(gpfpd.longitude);
  best_pose_msg->set_height_msl(gpfpd.altitude);
  best_pose_msg->set_num_sats_tracked(gpfpd.nsv1);
  best_pose_msg->set_num_sats_in_solution(gpfpd.nsv1);
  // Set solution type based on status
  if (gpfpd.status >= 4) {
    best_pose_msg->set_sol_type(SolutionType::NARROW_INT);  // Fixed RTK
  } else if (gpfpd.status >= 2) {
    best_pose_msg->set_sol_type(SolutionType::NARROW_FLOAT);  // Float RTK
  } else {
    best_pose_msg->set_sol_type(SolutionType::SINGLE);  // Single point
  }
  gnss_best_pose_writer_->Write(best_pose_msg);

  // ========== 3. Publish Gnss (GPS odometry) ==========
  auto gps_msg = std::make_shared<Gnss>();
  gps_msg->mutable_header()->set_timestamp_sec(timestamp);
  gps_msg->set_measurement_time(timestamp);
  gps_msg->mutable_position()->set_lon(gpfpd.longitude);
  gps_msg->mutable_position()->set_lat(gpfpd.latitude);
  gps_msg->mutable_position()->set_height(gpfpd.altitude);
  gps_msg->mutable_linear_velocity()->set_x(gpfpd.ve);
  gps_msg->mutable_linear_velocity()->set_y(gpfpd.vn);
  gps_msg->mutable_linear_velocity()->set_z(gpfpd.vu);
  gps_msg->set_num_sats(gpfpd.nsv1);
  if (gpfpd.status >= 4) {
    gps_msg->set_type(Gnss::RTK_INTEGER);
  } else if (gpfpd.status >= 2) {
    gps_msg->set_type(Gnss::RTK_FLOAT);
  } else {
    gps_msg->set_type(Gnss::SINGLE);
  }
  gps_writer_->Write(gps_msg);

  // ========== 4. Publish Heading ==========
  auto heading_msg = std::make_shared<Heading>();
  heading_msg->mutable_header()->set_timestamp_sec(timestamp);
  heading_msg->set_measurement_time(timestamp);
  heading_msg->set_heading(static_cast<float>(gpfpd.heading));
  heading_msg->set_pitch(static_cast<float>(gpfpd.pitch));
  heading_msg->set_baseline_length(static_cast<float>(gpfpd.baseline));
  heading_msg->set_satellite_tracked_number(gpfpd.nsv1);
  heading_msg->set_satellite_soulution_number(gpfpd.nsv2);
  heading_writer_->Write(heading_msg);

  // ========== 5. Publish Imu (with default/zero values for raw data)
  // ==========
  auto imu_msg = std::make_shared<Imu>();
  imu_msg->mutable_header()->set_timestamp_sec(timestamp);
  imu_msg->set_measurement_time(timestamp);
  imu_msg->set_measurement_span(0.02f);  // 50Hz
  // GPFPD doesn't provide raw IMU data, so we use zeros
  // This is acceptable because control modules use CorrectedImu, not raw Imu
  imu_msg->mutable_linear_acceleration()->set_x(0.0);
  imu_msg->mutable_linear_acceleration()->set_y(0.0);
  imu_msg->mutable_linear_acceleration()->set_z(9.81);  // Gravity
  imu_msg->mutable_angular_velocity()->set_x(0.0);
  imu_msg->mutable_angular_velocity()->set_y(0.0);
  imu_msg->mutable_angular_velocity()->set_z(0.0);
  imu_writer_->Write(imu_msg);

  // ========== 6. Publish InsStat ==========
  auto ins_stat_msg = std::make_shared<InsStat>();
  ins_stat_msg->mutable_header()->set_timestamp_sec(timestamp);
  // Map GPFPD status to INS status
  // Typical: 0=No fix, 1=Single, 2=Float, 4=Fixed
  ins_stat_msg->set_ins_status(gpfpd.status >= 4 ? 3 : gpfpd.status);  // 3=GOOD
  ins_stat_msg->set_pos_type(gpfpd.status);
  ins_stat_writer_->Write(ins_stat_msg);

  // ========== 7. Publish LocalizationStatus ==========
  auto status_msg = std::make_shared<LocalizationStatus>();
  status_msg->mutable_header()->set_timestamp_sec(timestamp);
  status_msg->set_fusion_status(MeasureState::OK);
  status_msg->set_state_message("INS running normally");
  status_msg->set_measurement_time(timestamp);
  localization_status_writer_->Write(status_msg);

  ADEBUG << "Published GNSS messages: lat=" << gpfpd.latitude
         << ", lon=" << gpfpd.longitude << ", status=" << gpfpd.status;
}

void InsComponent::GPS_XY(double lat, double lon, double* x, double* y) {
  // Convert degrees to radians
  double lat_rad = lat * M_PI / 180.0;
  double lon_rad = lon * M_PI / 180.0;
  double ref_lat_rad = ref_lat_ * M_PI / 180.0;
  double ref_lon_rad = ref_lon_ * M_PI / 180.0;

  double sin_lat = sin(lat_rad);
  double cos_lat = cos(lat_rad);
  double ref_sin_lat = sin(ref_lat_rad);
  double ref_cos_lat = cos(ref_lat_rad);

  double cos_d_lon = cos(lon_rad - ref_lon_rad);
  double arg = ref_sin_lat * sin_lat + ref_cos_lat * cos_lat * cos_d_lon;

  // Clamp arg to [-1, 1] to avoid numerical errors in acos
  if (arg < -1.0) {
    arg = -1.0;
  } else if (arg > 1.0) {
    arg = 1.0;
  }

  double c = acos(arg);
  double k = 1.0;
  if (fabs(c) > 0) {
    k = (c / sin(c));
  }

  *y = k * (ref_cos_lat * sin_lat - ref_sin_lat * cos_lat * cos_d_lon) *
       CONSTANTS_RADIUS_OF_EARTH;
  *x = k * cos_lat * sin(lon_rad - ref_lon_rad) * CONSTANTS_RADIUS_OF_EARTH;
}

bool InsComponent::ParseGPFPD(const std::string& line, GpfpdData* data) {
  // $GPFPD,GPSWeek,GPSTime,Heading,Pitch,Roll,Latitude,Longitude,Altitude,
  //        Ve,Vn,Vu,Baseline,NSV1,NSV2,Status,Cs*hh
  // Index: 0      1        2       3      4    5        6         7
  //        8  9   10  11       12   13   14
  auto tokens = split(line, ',');
  if (tokens.size() < 15) return false;

  try {
    data->gps_week = std::stod(tokens[1]);
    data->gps_time = std::stod(tokens[2]);
    data->heading = std::stod(tokens[3]);
    data->pitch = std::stod(tokens[4]);
    data->roll = std::stod(tokens[5]);
    data->latitude = std::stod(tokens[6]);
    data->longitude = std::stod(tokens[7]);
    data->altitude = std::stod(tokens[8]);
    data->ve = std::stod(tokens[9]);
    data->vn = std::stod(tokens[10]);
    data->vu = std::stod(tokens[11]);
    data->baseline = std::stod(tokens[12]);
    data->nsv1 = std::stoi(tokens[13]);
    data->nsv2 = std::stoi(tokens[14]);

    // Status field might have checksum attached (e.g., "4*5A")
    std::string status_str = tokens[15];
    size_t asterisk_pos = status_str.find('*');
    if (asterisk_pos != std::string::npos) {
      status_str = status_str.substr(0, asterisk_pos);
    }
    data->status = std::stoi(status_str);

    return true;
  } catch (...) {
    AWARN << "Failed to parse GPFPD line: " << line;
    return false;
  }
}

}  // namespace ins
}  // namespace drivers
}  // namespace apollo
