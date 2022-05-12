#include "process/navstate.h"
#include "navconfig.hpp"
#include "navattitude.hpp"
#include "navearth.hpp"

using namespace utiltool;
using namespace constant;

namespace mscnav
{

State::Ptr State::state_(new State());

State::Ptr State::GetState()
{
    return state_;
}

void State::ReadNavTrue(const std::string &strNavTrueFile){
  double end_time, start_time;
  GetStartAndEndTime(start_time, end_time);

  std::ifstream ifNav(strNavTrueFile);

  while (!ifNav.eof()) {
    std::string strLine;
    std::getline(ifNav, strLine);
    if (!strLine.empty()) {
      if (strLine.at(0) < '0' || strLine.at(0) > '9') continue;

      double t, Nx, Ny, Nz, v_x, v_y, v_z, roll, pitch, yaw;
      std::stringstream ss(strLine);
      ss >> t;

      if (t < start_time)
        continue;
      else {
        ss >> Nx;
        ss >> Ny;
        ss >> Nz;
        ss >> v_x;
        ss >> v_y;
        ss >> v_z;
        ss >> roll;
        ss >> pitch;
        ss >> yaw;

        // 存储真值
        Eigen::Matrix<double, 9, 1> NavTrue;
        NavTrue << Nx, Ny, Nz, v_x, v_y, v_z, roll, pitch, yaw;
        mpNavTrue[t] = NavTrue;
      }
    }
  }

  ifNav.close();
}

void State::SaveError(const NavInfo &nav_info){
  // 写入导航误差
  double NavEpoch = nav_info.time_;
  Eigen::Vector3d Pos = nav_info.pos_;
  Eigen::Vector3d Vel = nav_info.vel_;
  Eigen::Vector3d AttDeg = nav_info.att_ * constant::rad2deg;
//   if (AttDeg.z() > 180) ErrA.z() -= 360;
  if (AttDeg.z() < -90) AttDeg.z() += 360;

  auto RmpitTrue = mpNavTrue.lower_bound(NavEpoch);
  auto LmpitTrue = RmpitTrue;
  LmpitTrue--;

  if (RmpitTrue != mpNavTrue.end()) {
    auto mpitTrue =
        fabs(NavEpoch - LmpitTrue->first) <= fabs(NavEpoch - RmpitTrue->first)
            ? LmpitTrue
            : RmpitTrue;

    Eigen::Vector3d ErrXYZ = Pos - mpitTrue->second.topRows(3);
    auto blh = earth::WGS84XYZ2BLH(mpitTrue->second.topRows(3));
    Eigen::Vector3d ErrP = earth::CalCe2n(blh(0), blh(1)) * ErrXYZ;
    Eigen::Vector3d ErrV = Vel - mpitTrue->second.middleRows<3>(3);
    Eigen::Vector3d ErrA = AttDeg - mpitTrue->second.bottomRows(3);
    if (ErrA.z() > 180) ErrA.z() -= 360;
    if (ErrA.z() < -180) ErrA.z() += 360;

    ofs_NavError_ << std::setprecision(9) << NavEpoch << std::setw(15) <<  // time
        std::setprecision(5) <<                                 // 5 decimeter
        ErrP(0) << std::setw(11) << ErrP(1) << std::setw(11) << ErrP(2) << std::setw(11)
            <<  // pos
        ErrV(0) << std::setw(11) << ErrV(1) << std::setw(11) << ErrV(2) << std::setw(11)
            <<  // vel
        ErrA(0) << std::setw(11) << ErrA(1) << std::setw(11) << ErrA(2) << std::setw(11)
            <<  // att
        // Std(0) << std::setw(11) << Std(1) << std::setw(11) << Std(2) << std::setw(11) <<
        // Std(3) << std::setw(11) << Std(4) << // std std::setw(11) << Std(5) << std::setw(11)
        // << Std(6) << std::setw(11) << Std(7) << std::setw(11) << Std(8)
        std::endl;
  } else {
    std::cout << "Error when calculating nav error, check your navfile" << std::endl;
    exit(-1);
  }
}

/**
 * @brief  初始化
 * @note   
 * @retval 
 */
bool State::InitializeState()
{
    config_ = ConfigInfo::GetInstance();

    /* 数据相关内容 */
    bool gnss_log_enable = (config_->get<int>("gnsslog_enable") != 0);
    camera_enable = (config_->get<int>("camera_enable") != 0);
    gnss_data_ = std::make_shared<FileGnssData>(gnss_log_enable);

    std::string nav_true_path = config_->get<std::string>("nav_true_path"); 
    ReadNavTrue(nav_true_path);

    if (!gnss_data_->StartReadGnssData())
    {
        navexit();
    }

    imu_data_ = std::make_shared<FileImuData>();
    if (!imu_data_->StartReadData())
    {
        navexit();
    }

    if (camera_enable)
    {
        camera_data_ = std::make_shared<FileCameraData>();
        if (!camera_data_->StartReadCameraData())
        {
            navexit();
        }
    }

    // data_queue_包含了所有传感器的数据（各有一个线程）
    //    但各个线程不会一次性读完所有数据，而是设置了缓存尺寸上限
    data_queue_ = std::make_shared<DataQueue>(gnss_data_, imu_data_, camera_data_);

    /*计算相关内容 */
    bool filter_debug_log = (config_->get<int>("filter_debug_log_enable") != 0);
    filter_ = std::make_shared<KalmanFilter>(filter_debug_log);
    initialize_nav_ = std::make_shared<InitializedNav>(data_queue_);
    gps_process_ = std::make_shared<GpsProcess>(filter_);

    if (camera_enable)
    {
        msckf_process_ = std::make_shared<camera::MsckfProcess>(filter_);
    }

    /*读取相关配置，并做好初始对准 */
    Eigen::VectorXd initial_Pvariance;
    if (initialize_nav_->StartAligning(nav_info_, mpNavTrue)) //获取初始的状态
    {
        LOG(INFO) << "Initialize the navigation information successed" << std::endl;
    }
    else
    {
        LOG(ERROR) << "Initialize the navigation information failed" << std::endl;
        return false;
    }
    // 设置索引、初始方差
    initialize_nav_->SetStateIndex(filter_->GetStateIndex());                                                        //设置状态量对应的索引
    initial_Pvariance = initialize_nav_->SetInitialVariance(initial_Pvariance, nav_info_, filter_->GetStateIndex()); //设置初始方差信息
    filter_->InitialStateCov(initial_Pvariance);
    LOG(INFO) << "Initialized state successed" << std::endl;
    latest_update_time_ = nav_info_.time_;

    /**赋值小q 一定要注意单位 */
    const StateIndex &index = filter_->GetStateIndex();
    Eigen::VectorXd state_q_tmp = Eigen::VectorXd::Zero(filter_->GetStateSize());
    // 位置（哪有。。。都置零吧）、速度、角度随机游走
    std::vector<double> tmp_value = config_->get_array<double>("position_random_walk");
    state_q_tmp.segment<3>(index.pos_index_)
        << tmp_value.at(0),
        tmp_value.at(1),
        tmp_value.at(2);

    tmp_value = config_->get_array<double>("velocity_random_walk");
    state_q_tmp.segment<3>(index.vel_index_)
        << tmp_value.at(0) / 60.0,
        tmp_value.at(1) / 60.0,
        tmp_value.at(2) / 60.0;

    tmp_value = config_->get_array<double>("attitude_random_walk");
    state_q_tmp.segment<3>(index.att_index_)
        << tmp_value.at(0) * deg2rad / 60.0,
        tmp_value.at(1) * deg2rad / 60.0,
        tmp_value.at(2) * deg2rad / 60.0;

    tmp_value = config_->get_array<double>("gyro_bias_std");
    state_q_tmp.segment<3>(index.gyro_bias_index_)
        << tmp_value.at(0) * dh2rs,
        tmp_value.at(1) * dh2rs,
        tmp_value.at(2) * dh2rs;

    tmp_value = config_->get_array<double>("acce_bias_std");
    state_q_tmp.segment<3>(index.acce_bias_index_)
        << tmp_value.at(0) * constant_mGal,
        tmp_value.at(1) * constant_mGal,
        tmp_value.at(2) * constant_mGal;

    if (config_->get<int>("evaluate_imu_scale") != 0)
    {
        tmp_value = config_->get_array<double>("gyro_scale_std");
        state_q_tmp.segment<3>(index.gyro_scale_index_)
            << tmp_value.at(0) * constant_ppm,
            tmp_value.at(1) * constant_ppm,
            tmp_value.at(2) * constant_ppm;

        tmp_value = config_->get_array<double>("acce_scale_std");
        state_q_tmp.segment<3>(index.acce_scale_index_)
            << tmp_value.at(0) * constant_ppm,
            tmp_value.at(1) * constant_ppm,
            tmp_value.at(2) * constant_ppm;
    }
    state_q_tmp = state_q_tmp.array().pow(2);
    double gbtime = config_->get<double>("corr_time_of_gyro_bias") * constant_hour;
    double abtime = config_->get<double>("corr_time_of_acce_bias") * constant_hour;

    state_q_tmp.segment<3>(index.gyro_bias_index_) *= (2.0 / gbtime);
    state_q_tmp.segment<3>(index.acce_bias_index_) *= (2.0 / abtime);

    if (config_->get<double>("evaluate_imu_scale") != 0)
    {
        double gstime = config_->get<double>("corr_time_of_gyro_scale") * constant_hour;
        double astime = config_->get<double>("corr_time_of_acce_scale") * constant_hour;
        state_q_tmp.segment<3>(index.gyro_scale_index_) *= (2.0 / gstime);
        state_q_tmp.segment<3>(index.acce_scale_index_) *= (2.0 / astime);
    }
    state_q_ = state_q_tmp.asDiagonal();

    std::string navinfo_path = config_->get<std::string>("result_output_path");
    std::string NavError_path = config_->get<std::string>("result_output_path");
    navinfo_path.append("/MSC_NavInfo.txt");
    NavError_path.append("/MSC_NavError.txt");
    ofs_navinfo_.open(navinfo_path);
    ofs_navinfo_ << std::fixed;
    ofs_NavError_.open(NavError_path);
    ofs_NavError_ << std::fixed;
    if (!ofs_navinfo_.good() || !ofs_NavError_.good())
    {
        LOG(ERROR) << "Open Result File Failed\t" << std::endl;
    }
    // ofs_navinfo_ << nav_info_ << std::endl;
    nav_info_bak_ = nav_info_;
    return true;
}

void State::ReviseState(const Eigen::VectorXd &dx)
{
    filter_->ReviseState(nav_info_, dx);
}

/**
 * @brief  数据处理入口
 * @note   
 * @retval None
 */
void State::StartProcessing()
{
    using namespace camera;

    /**准备工作**/ 
    // 初始化/初始对准
    if (!InitializeState())
    {
        navexit();
    }
    // 其他一些准备工作
    static int state_count = filter_->GetStateSize();  // 21
    static int output_rate = config_->get<int>("result_output_rate");
    static auto &index = filter_->GetStateIndex();
    GnssData::Ptr ptr_gnss_data = nullptr;
    CameraData::Ptr ptr_camera_data = nullptr;
    ImuData::Ptr ptr_pre_imu_data = std::make_shared<ImuData>(), ptr_curr_imu_data;
    BaseData::bPtr base_data;
    Eigen::MatrixXd PHI = Eigen::MatrixXd::Identity(state_count, state_count);
    Eigen::VectorXd dx = Eigen::VectorXd::Zero(state_count);
    int data_rate = config_->get<int>("data_rate");

    /** 循环要开始了 **/ 
    // 先往后遍历data直到出现IMU，计算从第一个IMU开始
    LOG(INFO) << "Starting Processing Data...." << std::endl;
    while (true)
    {
        base_data = data_queue_->GetData();
        if (base_data->get_type() == IMUDATA)
        {
            ptr_curr_imu_data = std::dynamic_pointer_cast<ImuData>(base_data);
            // 把pre和cur两个IMU数据设为一样，但是pre的时间还是前一个(why?)
            *ptr_pre_imu_data = *ptr_curr_imu_data;
            ptr_pre_imu_data->set_time(nav_info_.time_);
            if (!(ptr_curr_imu_data->get_time() - nav_info_.time_ <= 1.0 / data_rate))
            {
                LOG(ERROR) << "Data queue exists bug(s)" << std::endl;
                navsleep(1000);
            }
            break;
        }
    }
    // 主循环开始
    bool gnss_update = false, bOutput = false;
    while (true)
    {
        // 1. 如果是IMU
        if (ptr_curr_imu_data != nullptr)
        {
            Eigen::MatrixXd phi;
            double dt = ptr_curr_imu_data->get_time() - ptr_pre_imu_data->get_time();
            // 补偿零偏和比例因子
            (*ptr_curr_imu_data) = Compensate(*ptr_curr_imu_data, nav_info_, dt);
            // 机械编排,ECEF坐标系下
            nav_info_ = navmech::MechanicalArrangement(*ptr_pre_imu_data, *ptr_curr_imu_data, nav_info_, phi);
            PHI *= phi;   // Phi阵累乘，别看错了
            ptr_pre_imu_data = ptr_curr_imu_data;  // 编排完就交接数据
            ptr_curr_imu_data = nullptr;
        }
        // 2. 如果不是IMU（GNSS或照片）
        else
        {
            /* time update ，方差更新*/
            // latest_update_time_的最初值是在初始化步骤中设置的
            double dt = base_data->get_time() - latest_update_time_;
            // 小q的最初值是在初始化步骤中设置的
            auto &Rbe = nav_info_.rotation_;
            Eigen::MatrixXd state_q_used = state_q_;
            state_q_used.block<3, 3>(index.pos_index_, index.pos_index_) =
                Rbe * state_q_used.block<3, 3>(index.pos_index_, index.pos_index_) * Rbe.transpose();
            state_q_used.block<3, 3>(index.vel_index_, index.vel_index_) =
                Rbe * state_q_used.block<3, 3>(index.vel_index_, index.vel_index_) * Rbe.transpose();
            state_q_used.block<3, 3>(index.att_index_, index.att_index_) =
                Rbe * state_q_used.block<3, 3>(index.att_index_, index.att_index_) * Rbe.transpose();
            Eigen::MatrixXd Q = (PHI * state_q_used * PHI.transpose() + state_q_used) * 0.5 * dt;
            filter_->TimeUpdate(PHI, Q, base_data->get_time());
            // 重置大PHI
            PHI = Eigen::MatrixXd::Identity(state_count, state_count);
            latest_update_time_ = base_data->get_time();

            /* measure update */
            if (ptr_gnss_data != nullptr)
            {
                gps_process_->processing(ptr_gnss_data, nav_info_, dx);
                ReviseState(dx);
                if (camera_enable)
                {
                    int cam_imu_idx = config_->get<int>("evaluate_camera_imu_rotation") == 0 ? 0 : 3;
                    msckf_process_->ReviseCameraState(dx.tail(dx.size() - filter_->GetStateIndex().total_state + cam_imu_idx));
                }
                ptr_gnss_data = nullptr;
                gnss_update = true;

                bOutput = true;
            }
            else if (ptr_camera_data != nullptr)
            {
                msckf_process_->ProcessImage(ptr_camera_data->ImgTxt, ptr_camera_data->get_time(), nav_info_);
                ptr_camera_data = nullptr;

                bOutput = msckf_process_->GetMSCKFState();  // 看是否滤波更新了
                msckf_process_->CloseMSCKFState();          // 不管是否更新，都设为否
            }
        }

        // 3. 算完就输出吧
        // int idt = int((nav_info_.time_ + 0.00005) * output_rate) - int(nav_info_bak_.time_ * output_rate);
        if (bOutput)
        {
            // double second_of_week = nav_info_.time_;
            // double double_part = (second_of_week * output_rate - int(second_of_week * output_rate)) / output_rate;
            // double inter_time = nav_info_.time_ - double_part;
            // auto output_nav = InterpolateNavInfo(nav_info_bak_, nav_info_, inter_time);
            auto output_nav = nav_info_;
            // output_nav.pos_ = earth::CorrectLeverarmPos(output_nav);   // 换算到GNSS
            auto blh = earth::WGS84XYZ2BLH(output_nav.pos_);
            // output_nav.vel_ = earth::CalCe2n(blh(0), blh(1)) * earth::CorrectLeverarmVel(output_nav);
            output_nav.vel_ = earth::CalCe2n(blh(0), blh(1)) * output_nav.vel_;
            ofs_navinfo_ << output_nav << std::endl;

            SaveError(output_nav);

            if (fabs(int(output_nav.time_) - output_nav.time_ < 0.04))
                LOG(ERROR) << output_nav.time_ << std::endl;
            gnss_update = false;

            bOutput = false;
        }
        nav_info_bak_ = nav_info_;

        /*4. 获取新的数据 */
        base_data = data_queue_->GetData();
        // LOG(INFO) << "base_data time: " << base_data->get_type() << "  "
        //           << std::fixed << std::setprecision(8)
        //           << base_data->get_time() << std::endl;
        if (base_data->get_type() == IMUDATA)
        {
            ptr_curr_imu_data = std::dynamic_pointer_cast<ImuData>(base_data);
        }
        else if (base_data->get_type() == GNSSDATA)
        {
            ptr_gnss_data = std::dynamic_pointer_cast<GnssData>(base_data);
        }
        else if (base_data->get_type() == CAMERADATA)
        {
            ptr_camera_data = std::dynamic_pointer_cast<CameraData>(base_data);
        }
        else if (base_data->get_type() == DATAUNKOWN)
        {
            break;
        }
        else
        {
            LOG(ERROR) << "Bak Data processing thread " << std::endl;
            //BAK for other data type
        }
    }
    LOG(INFO) << "all data processing finish" << std::endl;
}

NavInfo State::GetNavInfo() const
{
    return nav_info_;
}

} // namespace mscnav
