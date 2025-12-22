#include "modules/drivers/mycar_pos/mycar_pos_component.h"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cmath>
#include <sstream>
#include <vector>

#include "Eigen/Geometry"

namespace apollo {
namespace drivers {
namespace mycar_pos {

using apollo::localization::LocalizationEstimate;

namespace {

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

MycarPosComponent::~MycarPosComponent() {
  running_ = false;
  if (thread_ && thread_->joinable()) {
    thread_->join();
  }
  if (fd_ >= 0) {
    close(fd_);
  }
  if (wgs84pj_source_) pj_free(wgs84pj_source_);
  if (utm_target_) pj_free(utm_target_);
}

bool MycarPosComponent::Init() {
  if (!GetProtoConfig(&conf_)) {
    AERROR << "Unable to load mycar_pos conf file: " << ConfigFilePath();
    return false;
  }

  AINFO << "MycarPos config: " << conf_.DebugString();

  // Init Proj
  wgs84pj_source_ = pj_init_plus("+proj=latlong +ellps=WGS84");
  utm_target_ = pj_init_plus(conf_.proj4_text().c_str());
  if (!wgs84pj_source_ || !utm_target_) {
    AERROR << "Failed to init proj";
    return false;
  }

  writer_ = node_->CreateWriter<LocalizationEstimate>(conf_.topic());

  if (!ConfigureSerialPort()) {
    return false;
  }

  running_ = true;
  thread_.reset(new std::thread(&MycarPosComponent::Run, this));
  return true;
}

bool MycarPosComponent::ConfigureSerialPort() {
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

  // Send init command
  std::string cmd = "$cmd,output,com0,gpfpd,0.05*ff\r\n";
  int n = write(fd_, cmd.c_str(), cmd.size());
  if (n < 0) {
    AERROR << "Failed to write init command";
    return false;
  }
  AINFO << "Sent init command: " << cmd;
  return true;
}

void MycarPosComponent::Run() {
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

void MycarPosComponent::ProcessData(const std::string& line) {
  if (line.find("$GPFPD") == std::string::npos) return;

  double lat, lon, heading, ve, vn, vu;
  if (ParseGPFPD(line, &lat, &lon, &heading, &ve, &vn, &vu)) {
    // Convert to radians
    double x = lon * DEG_TO_RAD;
    double y = lat * DEG_TO_RAD;
    double z = 0;  // Altitude ignored for now or parsed if needed

    pj_transform(wgs84pj_source_, utm_target_, 1, 1, &x, &y, NULL);

    auto msg = std::make_shared<LocalizationEstimate>();
    msg->mutable_header()->set_timestamp_sec(cyber::Time::Now().ToSecond());
    msg->mutable_header()->set_module_name("mycar_pos");
    msg->mutable_header()->set_frame_id("localization");

    // Position (UTM)
    msg->mutable_pose()->mutable_position()->set_x(x);
    msg->mutable_pose()->mutable_position()->set_y(y);
    msg->mutable_pose()->mutable_position()->set_z(z);

    // Orientation
    // GPFPD Heading: North=0, Clockwise.
    // Apollo Heading: East=0, Counter-Clockwise.
    // Apollo = 90 - GPFPD = PI/2 - (Heading * DEG_TO_RAD)
    double apollo_theta = (90.0 - heading) * DEG_TO_RAD;
    // Normalize to [-PI, PI)? Not strictly necessary for Quaternion but good
    // practice.

    Eigen::Quaterniond q =
        Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(apollo_theta, Eigen::Vector3d::UnitZ());

    msg->mutable_pose()->mutable_orientation()->set_qw(q.w());
    msg->mutable_pose()->mutable_orientation()->set_qx(q.x());
    msg->mutable_pose()->mutable_orientation()->set_qy(q.y());
    msg->mutable_pose()->mutable_orientation()->set_qz(q.z());
    msg->mutable_pose()->set_heading(apollo_theta);

    // Velocity (NED -> ENU)
    // GPFPD: Ve (East), Vn (North), Vu (Up)
    // Apollo: Linear Velocity usually in Body Frame or ENU?
    // LocalizationEstimate usually expects ENU linear velocity.
    msg->mutable_pose()->mutable_linear_velocity()->set_x(ve);
    msg->mutable_pose()->mutable_linear_velocity()->set_y(vn);
    msg->mutable_pose()->mutable_linear_velocity()->set_z(vu);

    writer_->Write(msg);
  }
}

bool MycarPosComponent::ParseGPFPD(const std::string& line, double* lat,
                                   double* lon, double* heading, double* ve,
                                   double* vn, double* vu) {
  // $GPFPD,GPSWeek,GPSTime,Heading,Pitch,Roll,Lattitude,Longitude,Altitude,Ve,Vn,Vu,Baseline,NSV1,NSV2,Status,Cs*hh
  // Index:
  // 3: Heading
  // 6: Lat
  // 7: Lon
  // 9: Ve
  // 10: Vn
  // 11: Vu
  auto tokens = split(line, ',');
  if (tokens.size() < 12) return false;

  try {
    *heading = std::stod(tokens[3]);
    *lat = std::stod(tokens[6]);
    *lon = std::stod(tokens[7]);
    *ve = std::stod(tokens[9]);
    *vn = std::stod(tokens[10]);
    *vu = std::stod(tokens[11]);
    return true;
  } catch (...) {
    return false;
  }
}

}  // namespace mycar_pos
}  // namespace drivers
}  // namespace apollo
