#include "navearth.hpp"

namespace utiltool
{
namespace earth
{

Vector3d CalculateGravity(const Vector3d &pos, bool IsECEF)
{
    if (IsECEF)
    {
        /*--------------------------------------董绪荣, 张守信与华仲春, GPS/INS组合导航定位及其应用. 1998: 国防科技大学出版社.page 78-83--------------------------------*/
        double p = sqrt(pos(0) * pos(0) + pos(1) * pos(1) + pos(2) * pos(2));
        double t = pos(2) / p;
        double a_p = WGS84_A / p;
        double a1 = -WGS84_GM / p / p;
        double a2 = 1 + 1.5 * constant_J2 * a_p * a_p - (15.0 / 8) * constant_J4 * POW3(a_p) * a_p + (35.0 / 16) * constant_J6 * POW3(a_p) * POW3(a_p);
        double a3 = -4.5 * constant_J2 * a_p * a_p + (75.0 / 4) * constant_J4 * POW3(a_p) * a_p - (735.0 / 16) * constant_J6 * POW3(a_p) * POW3(a_p);
        double a4 = -(175.0 / 8) * constant_J4 * POW3(a_p) * a_p + (2205.0 / 16) * constant_J6 * POW3(a_p) * POW3(a_p);
        double a5 = -(1617.0 / 16) * constant_J6 * POW3(a_p) * POW3(a_p);

        double b1 = 3 * constant_J2 * a_p * a_p - (15.0 / 2) * constant_J4 * POW3(a_p) * a_p + (105.0 / 8) * constant_J6 * POW3(a_p) * POW3(a_p);
        double b2 = (35.0 / 2) * constant_J4 * POW3(a_p) * a_p - (945.0 / 12) * constant_J6 * POW3(a_p) * POW3(a_p);
        double b3 = (693.0 / 8) * constant_J6 * POW3(a_p) * POW3(a_p);

        double c1 = a2;
        double c2 = a3 - b1;
        double c3 = a4 - b2;
        double c4 = a5 - b3;
        double d1 = a2 + b1;
        double d2 = c2 + b2;
        double d3 = c3 + b3;
        double d4 = c4;
        Vector3d ge_vec;
        ge_vec(0) = (c1 + c2 * t * t + c3 * POW3(t) * t + c4 * POW3(t) * POW3(t)) * pos(0) * a1 / p + WGS84_AngleRate * WGS84_AngleRate * pos(0);
        ge_vec(1) = (c1 + c2 * t * t + c3 * POW3(t) * t + c4 * POW3(t) * POW3(t)) * pos(1) * a1 / p + WGS84_AngleRate * WGS84_AngleRate * pos(1);
        ge_vec(2) = (d1 + d2 * t * t + d3 * POW3(t) * t + d4 * POW3(t) * POW3(t)) * pos(2) * a1 / p;
        return ge_vec;
    }
    else
    {
        double gn = 9.7803267715 * (1 + 0.0052790414 * sin(pos(0)) * sin(pos(0)) + 0.0000232719 * POW3(sin(pos(0))) * sin(pos(0)));
        gn += (-0.0000030876910891 + 0.0000000043977311 * sin(pos(0)) * sin(pos(0))) * pos(2);
        gn += 0.0000000000007211 * pos(2) * pos(2);
        Vector3d gn_vec{0, 0, gn};
        return gn_vec;
    }
}

/**
  * @brief  BLH coordinate convert to rectangle XYZ in WGS84 coordinate
  * @note   
  * @param  &BLH: [rad rad m]
  * @retval 
  */
Eigen::Vector3d WGS84BLH2XYZ(const Eigen::Vector3d &BLH)
{
    Eigen::Vector3d XYZ{0, 0, 0};
    double W84, N84, m_A84, m_B84, m_E84;

    m_A84 = 6378137.0;
    m_B84 = 6378137.0 * 297.257223563 / 298.257223563;
    m_E84 = (1.0 - m_B84 * m_B84 / m_A84 / m_A84);
    W84 = sqrt(1.00 - m_E84 * sin(BLH(0)) * sin(BLH(0)));
    N84 = m_A84 / W84;

    XYZ(0) = (N84 + BLH(2)) * cos(BLH(0)) * cos(BLH(1));
    XYZ(1) = (N84 + BLH(2)) * cos(BLH(0)) * sin(BLH(1));
    XYZ(2) = (N84 * (1 - m_E84) + BLH(2)) * sin(BLH(0));

    return XYZ;
}

/**
  * @brief  XYZ coordinate convert to BLH in WGS84 coordinate
  * @note   
  * @param  &XYZ: [m,m,m]
  * @retval 
  */
Eigen::Vector3d WGS84XYZ2BLH(const Eigen::Vector3d &XYZ)
{
    Eigen::Vector3d BLH;
    double a = 6378137.0;
    double b = 6378137.0 * 297.257223563 / 298.257223563;
    double e2 = (1.0 - b * b / a / a);
    double N;
    double p;
    double dtmp;
    double sinlat;
    double lat;
    double lon;
    double hgt;

    if (XYZ(0) == 0.0 && XYZ(1) == 0.0)
    {
        lon = 0.0;
        if (XYZ(2) < 0)
        {
            hgt = -XYZ(2) - b;
            lat = -M_PI / 2.0;
        }
        else
        {
            hgt = XYZ(2) - b;
            lat = M_PI / 2.0;
        }
    }
    else
    {
        p = sqrt(XYZ(0) * XYZ(0) + XYZ(1) * XYZ(1));

        lon = 2.0 * atan2(XYZ(1), (XYZ(0) + p));
        lat = atan(XYZ(2) / (p * (1.0 - e2)));
        hgt = 0.0;
        do
        {
            dtmp = hgt;
            sinlat = sin(lat);
            N = a / sqrt(1.0 - e2 * sinlat * sinlat);
            hgt = p / cos(lat) - N;
            lat = atan(XYZ(2) / (p * (1.0 - e2 * N / (N + hgt))));
        } while (fabs(hgt - dtmp) > 0.0001);
    }

    BLH(0) = lat;
    BLH(1) = lon;
    BLH(2) = hgt;
    return BLH;
}

/**
 * @brief  calculate the rotation matrix of n coordinate to ecef
 * @note   
 * @param  B: 
 * @param  L: 
 * @retval 
 */
Eigen::Matrix3d CalCn2e(double B, double L)
{
    //TODO 核实NED转XYZ的公式,此处设定要求为转到NED 对应前-右-下坐标系 F-R-D
    Eigen::Matrix3d Cne;
    Cne(0, 0) = -sin(B) * cos(L);
    Cne(0, 1) = -sin(L);
    Cne(0, 2) = -cos(B) * cos(L);

    Cne(1, 0) = -sin(B) * sin(L);
    Cne(1, 1) = cos(L);
    Cne(1, 2) = -cos(B) * sin(L);

    Cne(2, 0) = cos(B);
    Cne(2, 1) = 0;
    Cne(2, 2) = -sin(B);
    return Cne;
}

/**
 * @brief  calculate the rotation matrix of ecef coordinate to navigation(local coordinate)
 * @note   
 * @param  B: 
 * @param  L: 
 * @retval 
 */
Eigen::Matrix3d CalCe2n(double B, double L)
{
    //TODO 核实XYZ转NED的公式,此处设定要求为转到NED 对应前-右-下坐标系 F-R-D
    Eigen::Matrix3d Cne;
    Cne(0, 0) = -sin(B) * cos(L);
    Cne(0, 1) = -sin(B) * sin(L);
    Cne(0, 2) = cos(B);

    Cne(1, 0) = -sin(L);
    Cne(1, 1) = cos(L);
    Cne(1, 2) = 0;

    Cne(2, 0) = -cos(B) * cos(L);
    Cne(2, 1) = -cos(B) * sin(L);
    Cne(2, 2) = -sin(B);
    return Cne;
}

Eigen::Matrix3d BLH2Dr4NED(const Eigen::Vector3d &CurBLH)
{
  double a = 6378137.0000;  // earth radius in meters
  double b = 6356752.3142;  // earth semiminor in meters
  double e2 = 1 - b * b / a / a;

  double lat = CurBLH(0);
  double lon = CurBLH(1);
  double hei = CurBLH(2);

  if (abs(lat) > (2.0 * M_PI) && abs(lon) > (2.0 * M_PI)) 
  {
    lat = lat * constant::deg2rad;
    lon = lon * constant::deg2rad;
  }
  
  double Rn = a / sqrt(1.0 - e2 * sin(lat) * sin(lat));
  double Rm = a * (1.0 - e2) / pow((1.0 - e2 * sin(lat) * sin(lat)), 1.5);

  Eigen::Matrix3d Dr = -Eigen::Matrix3d::Identity();
  Dr(0, 0) = Rm + hei;
  Dr(1, 1) = (Rn + hei) * cos(lat);

  return Dr;
}

} // namespace earth
} // namespace utiltool
