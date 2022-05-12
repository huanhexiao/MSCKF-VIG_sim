/*
** units.hpp for mscnav in /home/fwt/myprogram/mscnav/submodules/tools/include
**
** Made by little fang
** Login   <fangwentao>
**
** Started on  Wed Jul 10 下午9:46:55 2019 little fang
** Last update Thu Jul 17 下午5:49:08 2019 little fang
*/

#ifndef CONSTANT_H_
#define CONSTANT_H_
#include <cmath>

namespace utiltool
{

namespace constant
{
constexpr double WGS84_A = 6378137.0;
constexpr double WGS84_F = (1.0 / 298.257223563);
constexpr double WGS84_AngleRate(7.2921151467E-5);
constexpr double WGS84_GM(3.986005e14);

constexpr double constant_wie = 7.2921151467e-5;
constexpr double constant_g0 = 9.7803267715;
constexpr double constant_mg = 1e-3 * constant_g0;
constexpr double constant_ug = 1e-3 * constant_mg;
constexpr double constant_mGal = 1e-6 * constant_g0; // milli Gal = 1cm/s^2 ~= 1.0E-6*g0
constexpr double constant_ppm = 1e-6;
constexpr double constant_hour = 3600;
constexpr double constant_e_5(1.0e-5);
constexpr double constant_e_6(1.0e-6);
constexpr double constant_e5(1.0e5);
constexpr double constant_e6(1.0e6);
constexpr double constant_J2(0.00108263);
constexpr double constant_J4(-2.37091222e-6);
constexpr double constant_J6(6.08347e-9);

constexpr double deg2rad = M_PI / 180;
constexpr double rad2deg = 180 / M_PI;
constexpr double dh2rs(M_PI / 180.0 / 3600.0);
constexpr double rs2dh(180.0 / M_PI * 3600.0);

constexpr double operator"" _deg(long double x) { return x / 180 * M_PI; }
constexpr double operator"" _hour(long double x) { return x * constant_hour; }
constexpr double operator"" _mGal(long double x) { return x * constant_mGal; }
constexpr double operator"" _g(long double x) { return x * constant_g0; }
constexpr double operator"" _ppm(long double x) { return x * constant_ppm; }
constexpr double operator"" _dh(long double x) { return x * dh2rs; } //deg/h

} // namespace constant
} // namespace utiltool
#endif /* !CONSTANT_H_ */
