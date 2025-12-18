#ifndef VEHICLESTATUS_CORE_H
#define VEHICLESTATUS_CORE_H

#include  <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <vehicleStatus/controlcan.h>

namespace VehicleStatusNS
{

class VehicleStatus
{
public:
  VehicleStatus();
  ~VehicleStatus();
  void MainLoop();
  void DateProcessAndSend(PVCI_CAN_OBJ date, DWORD length);
  void CallbackGetDesiredStatus(const geometry_msgs::TwistStamped::ConstPtr &msgs);
  bool OpenCAN();
  void CloseCAN();
  DWORD ReceiveDate(PVCI_CAN_OBJ date, int size);
  void SendDate(PVCI_CAN_OBJ date, DWORD length);

  ros::NodeHandle nh;
  ros::Publisher pub_vehice_speed;
  ros::Publisher pub_vehice_angle;
  ros::Subscriber sub_desired_status;

  int device_type;//设备类型
  int device_index;//设备号
  int channel_index;//通道号
  int baund_rate;//波特率
  bool remote_flag;//是否为远程帧
  bool extern_flag;//是否为扩展帧
};

}

#endif // VEHICLESTATUS_CORE_H
