/*
** navstate.h for mscnav in /media/fwt/Data/program/mscnav/include/process
**
** Made by little fang
** Login   <fangwentao>
**
** Started on  Wed Jul 31 下午8:21:33 2019 little fang
** Last update Sat Aug 23 下午3:49:44 2019 little fang
*/

#ifndef PROCESS_STATE_H_
#define PROCESS_STATE_H_

#include "filter/navfilter.h"
#include "imu/navinitialized.h"
#include "imu/navmech.h"
#include "data/navdataque.h"
#include "gpsprocess.h"
#include "navconfig.hpp"
#include "camera/msckf.hpp"

#include <map>

namespace mscnav
{
using camera::MsckfProcess;

class State
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:
    ~State() {}
    using Ptr = std::shared_ptr<State>;
    static Ptr GetState();
    void ReadNavTrue(const std::string &strNavTrueFile);
    void SaveError(const NavInfo &nav_info);

private:
    State() {}
    State(State &&) = delete;
    State(const State &) = delete;
    State &operator=(State &&) = delete;
    State &operator=(const State &) = delete;
    static Ptr state_;

public:
    void StartProcessing();
    NavInfo GetNavInfo() const;
    void ReviseState(const Eigen::VectorXd &dx);
    std::map<double, Eigen::Matrix<double, 9, 1>> mpNavTrue;

private:
    bool InitializeState();

private:
    DataQueue::Ptr data_queue_;
    GpsProcess::Ptr gps_process_;
    KalmanFilter::Ptr filter_;
    ImuDataCollect::Ptr imu_data_;
    InitializedNav::Ptr initialize_nav_;
    GnssDataCollect::Ptr gnss_data_;
    utiltool::ConfigInfo::Ptr config_;
    utiltool::NavInfo nav_info_;
    utiltool::NavInfo nav_info_bak_;
    double latest_update_time_;  // 初始时刻和GNSS、相机时刻
    Eigen::MatrixXd state_q_;

private:
    MsckfProcess::Ptr msckf_process_;
    CameraDataCollect::Ptr camera_data_ = nullptr;
    bool camera_enable;

private:
    std::ofstream ofs_navinfo_;
    std::ofstream ofs_NavError_;
};
} // namespace mscnav

#endif /* !PROCESS_STATE_H_ */
