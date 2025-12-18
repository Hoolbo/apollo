#include <ros/ros.h>
#include <vehicleStatus/vehicle_status_core.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vehicleStatus");
  ros::NodeHandle nh;

  VehicleStatusNS::VehicleStatus v;
  v.MainLoop();
  return 0;
}
