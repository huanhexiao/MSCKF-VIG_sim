#include "data/navimu.h"
#include "navtime.h"
#include "navconfig.hpp"
#include "data/navgnss.h"
#include "navbase.hpp"

namespace mscnav
{
using utiltool::ConfigInfo;
using utiltool::NavTime;

ImuDataCollect::ImuDataCollect() : int_markofcollectdata_(-1)
{
}
ImuDataCollect::~ImuDataCollect()
{
	if (th_collectimudata_.joinable())
	{
		th_collectimudata_.join();
	}
}

bool FileImuData::StartReadData()
{
	if (int_markofcollectdata_ == 0)
	{
		return false;
	}
	ConfigInfo::Ptr config = ConfigInfo::GetInstance();
	auto imu_file_path = config->get<std::string>("imu_data_path");
	ifs_imufile_.open(imu_file_path, std::ifstream::binary);
	if (!ifs_imufile_.good())
	{
		return false;
	}
	int_markofcollectdata_ = 0;
	std::thread th1(&FileImuData::ReadingIMUData, this);
	th_collectimudata_ = std::move(th1);
	return true;
}

ImuData ReadIMUline(std::string &IMUline)
{
  std::stringstream ss(IMUline);

  double t, wx, wy, wz, ax, ay, az;
  ss >> t;
  ss >> wx;
  ss >> wy;
  ss >> wz;
  ss >> ax;
  ss >> ay;
  ss >> az;

  ImuData CurIMUData(t);
  CurIMUData.gyro_ << wx, wy, wz;
  CurIMUData.acce_ << ax, ay, az;

  return CurIMUData;
}

void FileImuData::ReadingIMUData()
{
	/* 获取起止时间 */
	double end_time, start_time;
	GetStartAndEndTime(start_time, end_time);
	if (end_time <= start_time)
	{
		LOG(INFO) << "start time is " << start_time << std::endl;
		LOG(INFO) << "end time is " << end_time << std::endl;
		LOG(FATAL) << "start time is later than end time" << std::endl;
		return;
	}

	std::string linestr;
	ImuData imu_temp;
	ConfigInfo::Ptr config = ConfigInfo::GetInstance();
	/* 循环读取数据 */
	int data_rate = config->get<int>("data_rate");
	while (!ifs_imufile_.eof())
	{
	    getline(ifs_imufile_, linestr);
		imu_temp = ReadIMUline(linestr);
		ImuData::Ptr imu_data = std::make_shared<ImuData>(imu_temp);
		if (imu_data->get_time() - start_time < -1.0 / data_rate || imu_data->get_time() - end_time > 1.0 / data_rate)
		{
			continue;
		}
		mtx_imudata_.lock();
		imudatapool_.emplace_back(imu_data);
		mtx_imudata_.unlock();
		while (imudatapool_.size() > MAX_SIZE_IMUPOOL * data_rate)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(200));
		}
	}
	int_markofcollectdata_ = 1;
	ifs_imufile_.close();
}

bool FileImuData::GetImuData(ImuData::Ptr &currimudata)
{
	if (imudatapool_.size() > 0)
	{
		std::unique_lock<std::mutex> lck(mtx_imudata_);
		currimudata = imudatapool_.front();
		imudatapool_.pop_front();
		return true;
	}
	else if (int_markofcollectdata_ == 0)
	{
		while (imudatapool_.size() == 0)
		{
			LOG_EVERY_N(INFO, 500) << "Imudata thread sleep 10s" << std::endl;
			if (int_markofcollectdata_ == 1)
			{
				return false;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(20));
		}
		std::unique_lock<std::mutex> lck(mtx_imudata_);
		currimudata = imudatapool_.front();
		imudatapool_.pop_front();
		return true;
	}
	else
	{
		return false;
	}
}
} // namespace mscnav
