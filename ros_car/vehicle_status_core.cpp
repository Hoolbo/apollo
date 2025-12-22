#include <vehicle_control/vehicle_status_core.h>
#include <fstream>

namespace VehicleStatusNS
{

  VehicleStatus::VehicleStatus()
  {
     pub_vehice_speed = nh.advertise<geometry_msgs::TwistStamped>("/vehicle_speed", 10);//dipanfankuixinxi
     pub_vehice_angle = nh.advertise<geometry_msgs::TwistStamped>("/vehicle_angle", 10);//fasongdipan
     sub_desired_status = nh.subscribe("/desired_status", 10, &VehicleStatus::CallbackGetDesiredStatus, this);

    ros::NodeHandle pnh("~");
    pnh.param<int>("device_type", device_type, 4);  //设备类型
    pnh.param<int>("device_index", device_index, 0);  //设备号
    pnh.param<int>("channel_index", channel_index, 0);  //通道号
    pnh.param<int>("baund_rate", baund_rate, 500);  //波特率
    pnh.param<bool>("remote_flag", remote_flag, false);  //是否为远程帧
    pnh.param<bool>("extern_flag", extern_flag, false);  //是否为扩展帧
  }

  VehicleStatus::~VehicleStatus()
  {
  }

  void VehicleStatus::DateProcessAndSend(PVCI_CAN_OBJ date, DWORD length)
  {
    geometry_msgs::TwistStamped speed;
    geometry_msgs::TwistStamped angle;
    std::ofstream ofs;
    for (int i = 0; i < length; i++)
    {
      switch (date[i].ID)
      {
      case 0x23c: //速度角度信号接收
        speed.twist.linear.x = (date[i].Data[5] + date[i].Data[6] * 16 * 16) / 10.0;   //车速和前轮转角的编码
        pub_vehice_speed.publish(speed);
        break;
      case 0x22c:
        angle.twist.angular.x = date[i].Data[5] + date[i].Data[6] * 16 * 16 - 1024;
        pub_vehice_angle.publish(angle);
        break;
      default:
        break;
      }
    }
    ofs.close();
  }

  bool VehicleStatus::OpenCAN()  //返回布尔值
  {
    if (VCI_OpenDevice(device_type, device_index, 0) != 1)
    {
      std::cout << "open device failed!" << std::endl;
      return false;
    }

    VCI_INIT_CONFIG vic;
    vic.AccCode = 0x80000008;
    vic.AccMask = 0xFFFFFFFF;
    vic.Filter = 1;
    switch (baund_rate)
    {
    case 10:
      vic.Timing0 = 0x31;
      vic.Timing1 = 0x1c;
      break;
    case 20:
      vic.Timing0 = 0x18;
      vic.Timing1 = 0x1c;
      break;
    case 40:
      vic.Timing0 = 0x87;
      vic.Timing1 = 0xff;
      break;
    case 50:
      vic.Timing0 = 0x09;
      vic.Timing1 = 0x1c;
      break;
    case 80:
      vic.Timing0 = 0x83;
      vic.Timing1 = 0xff;
      break;
    case 100:
      vic.Timing0 = 0x04;
      vic.Timing1 = 0x1c;
      break;
    case 125:
      vic.Timing0 = 0x03;
      vic.Timing1 = 0x1c;
      break;
    case 200:
      vic.Timing0 = 0x81;
      vic.Timing1 = 0xfa;
      break;
    case 250:
      vic.Timing0 = 0x01;
      vic.Timing1 = 0x1c;
      break;
    case 400:
      vic.Timing0 = 0x80;
      vic.Timing1 = 0xfa;
      break;
    case 500:
      vic.Timing0 = 0x00;
      vic.Timing1 = 0x1c;
      break;
    case 666:
      vic.Timing0 = 0x80;
      vic.Timing1 = 0xb6;
      break;
    case 800:
      vic.Timing0 = 0x00;
      vic.Timing1 = 0x16;
      break;
    case 1000:
      vic.Timing0 = 0x00;
      vic.Timing1 = 0x14;
      break;
    case 33:
      vic.Timing0 = 0x09;
      vic.Timing1 = 0x6f;
      break;
    case 66:
      vic.Timing0 = 0x04;
      vic.Timing1 = 0x6f;
      break;
    case 83:
      vic.Timing0 = 0x03;
      vic.Timing1 = 0x6f;
      break;
    default:
      break;
    }
    vic.Mode = 0;
    if (VCI_InitCAN(device_type, device_index, 0, &vic) != 1 || VCI_InitCAN(device_type, device_index, 1, &vic) != 1)
    {
      std::cout << "init can failed!" << std::endl;
      return false;
    }

    if (VCI_ClearBuffer(device_type, 0, 0) != 1 || VCI_ClearBuffer(device_type, 0, 1) != 1)
    {
      std::cout << "clear buffer failed!" << std::endl;
      return false;
    }

    if (VCI_StartCAN(device_type, 0, 0) != 1 || VCI_StartCAN(device_type, 0, 1) != 1)
    {
      std::cout << "start can failed!" << std::endl;
      return false;
    }

    return true;
  }

  void VehicleStatus::CloseCAN()
  {
    VCI_CloseDevice(device_type, device_index);
  }

  DWORD VehicleStatus::ReceiveDate(PVCI_CAN_OBJ date, int size)
  {
    if (VCI_GetReceiveNum(device_type, device_index, channel_index) <= 0)
    {
      return 0;
    }

    return VCI_Receive(device_type, device_index, channel_index, date, size, 0);
  }
  void VehicleStatus::SendDate(PVCI_CAN_OBJ date, DWORD length)
  {
    VCI_Transmit(device_type, device_index, channel_index, date, length);
  }

  void VehicleStatus::CallbackGetDesiredStatus(const geometry_msgs::TwistStamped::ConstPtr &msgs)
  {
    VCI_CAN_OBJ date[2];
    DWORD length = 2;
    date[0].ID = 0x233; //速度报文发送
    date[0].RemoteFlag = remote_flag;
    date[0].ExternFlag = extern_flag;
    date[0].DataLen = 8;
    date[0].Data[0] = 0x00;
    date[0].Data[1] = 0x00;
    date[0].Data[2] = 0x00;
    date[0].Data[3] = 0x00;
    date[0].Data[4] = 0x00;
    date[0].Data[5] = (int)fabs(msgs->twist.linear.x * 10) % (16 * 16);
    date[0].Data[6] = (int)fabs(msgs->twist.linear.x * 10) / (16 * 16);
    if (msgs->twist.linear.x > 0) //车速大于0，为D档位
    {
      date[0].Data[7] = 0x21;
    }
    else if (msgs->twist.linear.x < 0)
    {
      date[0].Data[7] = 0x11;  //R档位
    }
    else
    {
      date[0].Data[7] = 0x01;
    }
    date[1].ID = 0x223; //方向盘转角发送
    date[1].RemoteFlag = remote_flag;
    date[1].ExternFlag = extern_flag;
    date[1].DataLen = 8;
    date[1].Data[0] = 0x00;
    date[1].Data[1] = 0x00;
    date[1].Data[2] = 0x00;
    date[1].Data[3] = 0x00;
    date[1].Data[4] = 0x00;
    date[1].Data[5] = (int)((msgs->twist.angular.x + 1024)) % (16 * 16);
    date[1].Data[6] = (int)((msgs->twist.angular.x + 1024)) / (16 * 16);
    date[1].Data[7] = 0x01;
    SendDate(date, length);
  }

  void VehicleStatus::stop()
  {
      VCI_CAN_OBJ date[2];
      DWORD length = 2;
      date[0].ID = 0x233;
      date[0].RemoteFlag = remote_flag;
      date[0].ExternFlag = extern_flag;
      date[0].DataLen = 8;
      date[0].Data[0] = 0x00;
      date[0].Data[1] = 0x00;
      date[0].Data[2] = 0x00;
      date[0].Data[3] = 0x00;
      date[0].Data[4] = 0x00;
      date[0].Data[5] = 0x00;
      date[0].Data[6] = 0x00;
      date[0].Data[7] = 0x01;
      date[1].ID = 0x223;
      date[1].RemoteFlag = remote_flag;
      date[1].ExternFlag = extern_flag;
      date[1].DataLen = 8;
      date[1].Data[0] = 0x00;
      date[1].Data[1] = 0x00;
      date[1].Data[2] = 0x00;
      date[1].Data[3] = 0x00;
      date[1].Data[4] = 0x00;
      date[1].Data[5] = 1024 % (16 * 16);
      date[1].Data[6] = 1024 / (16 * 16);
      date[1].Data[7] = 0x01;
      SendDate(date, length);
  }

  void VehicleStatus::MainLoop()
  {

    ros::Duration(3).sleep();
    if (!OpenCAN())
    {
      return;
    }

    ros::Rate rate_loop(50);
    while (ros::ok())
    {
      ros::spinOnce();

      VCI_CAN_OBJ date[2500];
      DWORD length = ReceiveDate(date, 2500);
      if (length > 0)
      {
        DateProcessAndSend(date, length);
      }

      rate_loop.sleep();
    }

    stop();

    ros::Duration(0.5).sleep();

    CloseCAN();
  }
}
