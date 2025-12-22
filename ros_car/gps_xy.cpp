#include <iostream>
#include <math.h>
#include <cmath>
#include<vector>
#include<cstring>
#include<fstream>
#include<sstream>
using namespace std;



void GPS_XY (vector<double> &cx,vector<double> &cy,double lat,double lon,double ref_lat,double ref_lon,int CONSTANTS_RADIUS_OF_EARTH)
{
  double lat_rad = lat*M_PI/180; //纬度角度转弧度
  double lon_rad = lon*M_PI/180; //经度
  double ref_lat_rad = ref_lat*M_PI/180;
  double ref_lon_rad = ref_lon*M_PI/180;

  double sin_lat = sin(lat_rad); //
  double cos_lat = cos(lat_rad);
  double ref_sin_lat = sin(ref_lat_rad);
  double ref_cos_lat = cos(ref_lat_rad);

  double cos_d_lon = cos(lon_rad - ref_lon_rad);
  double arg = ref_sin_lat * sin_lat + ref_cos_lat * cos_lat * cos_d_lon;
  if (arg < -1)
  {
    arg = -1;
  }
  else if (arg > 1)
  {
    arg = 1;
  }
  double c = acos(arg);
  int k = 1;
  if (abs(c) > 0)
  {
    k = (c/sin(c));
  }
  double y = k * (ref_cos_lat * sin_lat - ref_sin_lat * cos_lat * cos_d_lon) * CONSTANTS_RADIUS_OF_EARTH;
  double x = (k * cos_lat * sin(lon_rad - ref_lon_rad) * CONSTANTS_RADIUS_OF_EARTH);
  cx.push_back(x);
  cy.push_back(y);
}

int main()
{
  int CONSTANTS_RADIUS_OF_EARTH = 6371000;
  vector<double> cx ={};
  vector<double> cy ={};
  vector<double> clat = {};
  vector<double> clon = {};

  //读取GPS数据中的经纬度信息
  //ifstream fin("/home/sunjiyu234/vehicle/vehicle_control/src/vehicle_control/src/0830_road.txt");
  ifstream fin("/home/sunjiyu234/vehicle/vehicle_control/src/vehicle_control/src/0830_road.txt");
  string line;
  if (fin)
  {
    while (getline(fin,line))
    {
      istringstream sin(line);
      vector<string> lat_lon;  //建立容器收经纬度信息
      string info;

      while(getline(sin,info,','))
      {

        lat_lon.push_back(info);

      }
      if(lat_lon.size()==1)
      {continue;}
      if(lat_lon.size()<15){
        continue;
      }
      string lat_str = lat_lon[6];
      string lon_str = lat_lon[7];
      double lat_num,lon_num;
      stringstream slat,slon;
      slat << lat_str;
      slon << lon_str;
      slat >> lat_num;
      slon >> lon_num;
      clat.push_back(lat_num);
      clon.push_back(lon_num);
    }
  }
  else {
     cout<<"no such file"<<endl;
  }

  //将GPS信息转化成XY坐标
  double ref_lat = clat[0];
  double ref_lon = clon[0];
  for (int i = 0;i < clat.size();i++)
  {
    double lat = clat[i];
    double lon = clon[i];
    GPS_XY(cx,cy,lat,lon,ref_lat,ref_lon,CONSTANTS_RADIUS_OF_EARTH);
  }
  /*for (int i = 0;i <clat.size();i++)
  {
    cout << cx[i]<< " " <<cy[i]<<endl;
  }
  */

  // 将cx,cy坐标输入到一个txt文件中，这之前的部分可以作为直接处理GPS.TXT文件的方式
  ofstream out("0830_xy.txt");
  for (int i = 0;i < clat.size();i++)
  {
    out << cx[i] <<" "<< cy[i]<<endl;
  }
  out.close();
  system("pause");
  return 0 ;
}
