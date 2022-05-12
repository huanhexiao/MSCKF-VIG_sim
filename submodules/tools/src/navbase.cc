
#include "navbase.hpp"
#include "navearth.hpp"
#include "navattitude.hpp"
#include <Eigen/Geometry>

namespace utiltool
{
/**
 * @brief  spilt string 
 * @note   
 * @param  &in: string in which need split
 * @param  &delim: spliter mark
 * @retval 
 */
std::vector<std::string> TextSplit(const std::string &in, const std::string &delim)
{
    std::vector<std::string> ret;
    try
    {
        std::regex re{delim};
        return std::vector<std::string>{std::sregex_token_iterator(in.begin(), in.end(), re, -1), std::sregex_token_iterator()};
    }
    catch (const std::exception &e)
    {
        std::cout << "error:" << e.what() << std::endl;
    }
    return ret;
}

std::string &trim(std::string &s)
{
    if (s.empty())
    {
        return s;
    }
    s.erase(0, s.find_first_not_of(" "));
    s.erase(s.find_last_not_of(" ") + 1);
    return s;
}

// std::ostream &operator<<(std::ostream &output, const double &time)
// {
//     output << time  << "\t";
//     return output;
// }
/**
 * @brief  override the ostream output NavResultInfo
 * @note   
 * @param  &output: ostream& 
 * @param  nav_res_info: 
 * @retval 
 */
std::ostream &operator<<(std::ostream &output, const NavInfo &nav_info)
{
    output << std::setprecision(9) << nav_info.time_ << "   ";
    output << std::fixed << std::left << std::setprecision(4) << nav_info.pos_.transpose() << "   ";
    output << std::fixed << std::left << std::setw(10) << std::setprecision(4) << nav_info.vel_.transpose() << "   ";
    output << std::fixed << std::left << std::setw(10) << std::setprecision(4) << nav_info.att_.transpose() * constant::rad2deg << "   ";
    output << std::fixed << std::left << std::setw(10) << std::setprecision(4) << nav_info.cam_imu_.transpose() * constant::rad2deg << "   ";
    // output <<std::fixed << std::left << std::setprecision(3) << nav_info.pos_std_.transpose() << "   ";
    // output <<std::fixed << std::left << std::setprecision(3) << nav_info.vel_std_.transpose() << "   ";
    // output <<std::fixed << std::left << std::setprecision(3) << nav_info.att_std_.transpose() << "   ";
    output << std::fixed << std::left << std::setw(10) << std::setprecision(3) << nav_info.gyro_bias_.transpose() * constant::rs2dh << "   ";
    output << std::fixed << std::left << std::setw(10) << std::setprecision(3) << nav_info.acce_bias_.transpose() / constant::constant_mGal << "   ";
    output << std::fixed << std::left << std::setw(10) << std::setprecision(3) << nav_info.gyro_scale_.transpose() / constant::constant_ppm << "   ";
    output << std::fixed << std::left << std::setw(10) << std::setprecision(3) << nav_info.acce_scale_.transpose() / constant::constant_ppm;
    return output;
}

/**
 * @brief  override the ostream output ImuData
 * @note   
 * @param  &output: 
 * @param  &imu_data: 
 * @retval 
 */
std::ostream &operator<<(std::ostream &output, const ImuData &imu_data)
{
    output << imu_data.get_time() << "\t";
    output << imu_data.gyro_.transpose() << "\t";
    output << imu_data.acce_.transpose() << "\t";
    return output;
}
/**
 * @brief  override the ostream output ImuData::Ptr
 * @note   
 * @param  &output: 
 * @param  &imu_data: 
 * @retval 
 */
std::ostream &operator<<(std::ostream &output, const ImuData::Ptr &imu_data)
{
    output << imu_data->get_time() << "\t";
    output << imu_data->gyro_.transpose() << "\t";
    output << imu_data->acce_.transpose() << "\t";
    return output;
}
/**
 * @brief  override the ostream output GnssData
 * @note   
 * @param  &output: 
 * @param  &imu_data: 
 * @retval 
 */
std::ostream &operator<<(std::ostream &output, const GnssData &gnss_data)
{
    output << gnss_data.get_time() << "\t";
    output << gnss_data.pos_.transpose() << "\t";
    output << gnss_data.vel_.transpose() << "\t";
    output << gnss_data.pos_std_.transpose() << "\t";
    output << gnss_data.vel_std_.transpose() << "\t";
    output << gnss_data.get_type();
    return output;
}

/**
 * @brief  override the ostream output GnssData::Ptr
 * @note   
 * @param  &output: 
 * @param  &imu_data: 
 * @retval 
 */
std::ostream &operator<<(std::ostream &output, const GnssData::Ptr &gnss_data)
{
    output << *gnss_data;
    return output;
}
/**
 * @brief  skew matrix from vector
 * @note   
 * @param  vector: 
 * @retval 
 */
Eigen::Matrix3d skew(const Eigen::Vector3d &vector)
{
    Eigen::Matrix3d dcm1;
    dcm1(0, 0) = 0;
    dcm1(0, 1) = -vector(2);
    dcm1(0, 2) = vector(1);

    dcm1(1, 0) = vector(2);
    dcm1(1, 1) = 0;
    dcm1(1, 2) = -vector(0);

    dcm1(2, 0) = -vector(1);
    dcm1(2, 1) = vector(0);
    dcm1(2, 2) = 0;

    return dcm1;
}

/**
 * @brief  update the attitude about
 * @note   
 * @param  &nav_info: 
 * @retval 
 */
// 姿态角att_转到N系下，旋转quat_,rotation_仍然在E系下, 保持不变
NavInfo &NormalizeAttitude(NavInfo &nav_info)
{
    nav_info.rotation_ = attitude::Quaternion2RotationMatrix(nav_info.quat_);
    auto BLH = earth::WGS84XYZ2BLH(nav_info.pos_);
    nav_info.att_ = attitude::RotationMartix2Euler(earth::CalCe2n(BLH(0), BLH(1)) * nav_info.rotation_);
    return nav_info;
}

/**
 * @brief  
 * @note   
 * @param  &nav_info1: the early information
 * @param  &nav_info2: the later information
 * @param  &time_: interplation time
 * @retval 
 */
NavInfo InterpolateNavInfo(const NavInfo &nav_info1, const NavInfo &nav_info2, const double &time)
{
    double coeff = (time - nav_info1.time_) / (nav_info2.time_ - nav_info1.time_);
    NavInfo result = nav_info2;
    result.time_ = time;
    result.pos_ = interpolate(nav_info1.pos_, nav_info2.pos_, coeff);
    result.vel_ = interpolate(nav_info1.vel_, nav_info2.vel_, coeff);
    result.quat_ = nav_info1.quat_.slerp(coeff, nav_info2.quat_);
    result.pos_std_ = interpolate(nav_info1.pos_std_, nav_info2.pos_std_, coeff);
    result.vel_std_ = interpolate(nav_info1.vel_std_, nav_info2.vel_std_, coeff);
    result.att_std_ = interpolate(nav_info1.att_std_, nav_info2.att_std_, coeff);
    result.gyro_bias_ = interpolate(nav_info1.gyro_bias_, nav_info2.gyro_bias_, coeff);
    result.acce_bias_ = interpolate(nav_info1.acce_bias_, nav_info2.acce_bias_, coeff);
    result.fibb_ = interpolate(nav_info1.fibb_, nav_info2.fibb_, coeff);
    result.gyro_scale_ = interpolate(nav_info1.gyro_scale_, nav_info2.gyro_scale_, coeff);
    result.wibb_ = interpolate(nav_info1.wibb_, nav_info2.wibb_, coeff);
    NormalizeAttitude(result);
    return result;
}

} // namespace utiltool
