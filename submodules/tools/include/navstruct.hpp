/*
** navstruct.h for tools in /home/fwt/program/myprogram/wsl_program/mscnav/submodules/tools/include
**
** Made by little fang
** Login   <fangwentao>
**
** Started on  Wed Jul 3 14:38:27 2019 little fang
** Last update Thu Aug 21 下午9:29:43 2019 little fang
*/

#ifndef NAVSTRUCT_H_
#define NAVSTRUCT_H_
#include "stdio.h"
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <deque>
#include <Eigen/Dense>
#include <map>
#include "navtime.h"

namespace utiltool
{

inline void navsleep(int _milliseconds)
{
  std::chrono::milliseconds msecond(_milliseconds);
  std::this_thread::sleep_for(msecond);
}

inline void navexit(const std::string &info = "Exitting...")
{
  std::cout << info << std::endl;
  navsleep(1000);
  exit(0);
}

enum DataType
{
  DATAUNKOWN = -1,
  IMUDATA,
  GNSSDATA,
  ODODATA,
  CAMERADATA,
  RESULTDATA
};

enum GnssType
{
  GNSSUNKOWN = -1,
  GNSSSPP,
  GNSSPPP,
  GNSSFIXED,
  GNSSRTD,
  GNSSRAW
};

enum GnssFormatType
{
  GNSSPOS = 0,
  GNSSPOSVEL = 1,
  GNSSVEL = 2 //仅备用,应该不考虑这样的观测值
};

class BaseData
{
public:
  BaseData(const double &time) : t0_(time) {}
  BaseData() {}
  virtual ~BaseData() {}
  using bPtr = std::shared_ptr<BaseData>;

public:
  double get_time() const { return t0_; }
  void set_time(const double &time) { t0_ = time; }
  DataType get_type() const { return data_type_; }
  // virtual std::string to_string() { return "nullptr"; }

protected:
  DataType data_type_ = DATAUNKOWN;
  double t0_;
  double t1_; //bak receive time
};

class GnssData : public BaseData
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = std::shared_ptr<GnssData>;

public:
  GnssData(const double &time) : BaseData(time)
  {
    data_type_ = GNSSDATA;
  }
  GnssData() : BaseData()
  {
    data_type_ = GNSSDATA;
  }
  virtual ~GnssData() {}

  // public:
  //   virtual std::string to_string() override
  //   {
  //     std::ostringstream osstream;
  //     osstream << t0_.Time2String() << "\t";
  //     osstream << std::fixed << std::setprecision(9) << std::setw(13) << pos_.transpose() << "\t";
  //     osstream << std::fixed << std::setprecision(4) << std::setw(7) << vel_.transpose() << "\t";
  //     osstream << std::fixed << std::setprecision(4) << std::setw(7) << pos_std_.transpose() << "\t";
  //     osstream << std::fixed << std::setprecision(4) << std::setw(7) << vel_std_.transpose() << "\t";
  //     osstream << std::fixed << gnss_type_;
  //     return osstream.str();
  //   }

public:
  Eigen::Vector3d pos_{0, 0, 0};
  Eigen::Vector3d vel_{0, 0, 0};
  Eigen::Vector3d pos_std_{0, 0, 0};
  Eigen::Vector3d vel_std_{0, 0, 0};
  GnssType gnss_type_ = GNSSUNKOWN;
  GnssFormatType format_type_ = GNSSPOS;
};

class ImuData : public BaseData
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = std::shared_ptr<ImuData>;

public:
  ImuData(const double &time) : BaseData(time)
  {
    data_type_ = IMUDATA;
  }
  ImuData() : BaseData()
  {
    data_type_ = IMUDATA;
  }
  virtual ~ImuData() {}
  // virtual std::string to_string() override
  // {
  //   std::ostringstream osstream;
  //   osstream << t0_.Time2String() << "\t";
  //   osstream << std::fixed << std::setprecision(6) << std::setw(14) << gyro_.transpose() << "\t";
  //   osstream << std::fixed << std::setprecision(6) << std::setw(14) << acce_.transpose() << "\t";
  //   return osstream.str();
  // }

public:
  Eigen::Vector3d gyro_{0, 0, 0};
  Eigen::Vector3d acce_{0, 0, 0};
};
// for accumulate function
struct imu_accumulate
{
public:
  ImuData::Ptr operator()(const ImuData::Ptr &imu1, const ImuData::Ptr &imu2)
  {
    ImuData::Ptr imu = std::make_shared<ImuData>(*imu2);
    imu->acce_ += imu1->acce_;
    imu->gyro_ += imu1->gyro_;
    return imu;
  }
};
class OdoData : public BaseData
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = std::shared_ptr<OdoData>;

public:
  OdoData(const double &time) : BaseData(time)
  {
    data_type_ = ODODATA;
  }
  OdoData() : BaseData()
  {
    data_type_ = ODODATA;
  }
  virtual ~OdoData() {}

  // public:
  //   virtual std::string to_string() override
  //   {
  //     std::ostringstream osstream;
  //     osstream << t0_.Time2String() << "\t";
  //     osstream << std::fixed << std::setprecision(5) << std::setw(10) << odo_vel.transpose();
  //     return osstream.str();
  //   }

public:
  Eigen::Vector4d odo_vel{0, 0, 0, 0};
};

using GNSSDATAPOOL = std::deque<GnssData::Ptr>;
using IMUDATAPOOL = std::deque<ImuData::Ptr>;

struct NavInfo
{
  double time_;
  // 旋转quat_,rotation_是E系下的, 姿态角att_是N系下的
  Eigen::Quaterniond quat_{1, 0, 0, 0};
  // 旋转quat_,rotation_是E系下的, 姿态角att_是N系下的
  Eigen::Matrix3d rotation_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d pos_{0, 0, 0};
  Eigen::Vector3d vel_{0, 0, 0};
  // 旋转quat_,rotation_是E系下的, 姿态角att_是N系下的
  Eigen::Vector3d att_{0, 0, 0};
  Eigen::Vector3d wibb_{0, 0, 0};
  Eigen::Vector3d fibb_{0, 0, 0};
  Eigen::Vector3d pos_std_{0, 0, 0};
  Eigen::Vector3d vel_std_{0, 0, 0};
  Eigen::Vector3d att_std_{0, 0, 0};
  Eigen::Vector3d gyro_bias_{0, 0, 0};
  Eigen::Vector3d acce_bias_{0, 0, 0};
  Eigen::Vector3d gyro_scale_{0, 0, 0};
  Eigen::Vector3d acce_scale_{0, 0, 0};
  Eigen::Vector3d leverarm_{0, 0, 0};
  Eigen::Vector3d cam_imu_{0, 0, 0};
  long long int result_type_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct StateIndex
{
  int pos_index_ = 0;
  int vel_index_ = 0;
  int att_index_ = 0;
  int gyro_bias_index_ = 0;
  int acce_bias_index_ = 0;
  int gyro_scale_index_ = 0;
  int acce_scale_index_ = 0;
  int odometer_scale_index = 0;
  int camera_delay_index_ = 0;
  int camera_rotation_index_ = 0;
  int camera_translation_index_ = 0;
  int imu_vehicle_rotation_index_ = 0;
  int total_state = 0;  //exclude camera state
  std::map<long long int, int, std::less<long long int>>
      camera_state_index; //fisrt CameraState ID，second CameraState index
  bool is_initialized = false;
};

} // namespace utiltool

#endif /* !NAVSTRUCT_H_ */
