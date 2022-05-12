/*
** navbase.hpp for mscnav in /home/fwt/myprogram/mscnav/submodules/tools/include
**
** Made by little fang
** Login   <fangwentao>
**
** Started on  Thu Jul 11 上午9:11:49 2019 little fang
** Last update Fri Feb 6 下午2:40:57 2020 little fang
*/

#ifndef UTIL_BASE_H_
#define UTIL_BASE_H_

#include <string>
#include <regex>
#include <vector>
#include <exception>
#include <iostream>
#include <Eigen/Dense>

#include "navstruct.hpp"
#include "constant.hpp"

namespace utiltool
{

// // 0.9
// const double chi_squared_test_table[100] =
//     {0.015790774, 0.210721031, 0.584374374, 1.063623217,
//      1.610307987, 2.204130656, 2.833106918, 3.489539126,
//      4.168159008, 4.865182052, 5.57778479, 6.30379606,
//      7.04150458, 7.78953361, 8.546756242, 9.312236354,
//      10.08518633, 10.86493612, 11.65091003, 12.44260921,
//      13.23959798, 14.04149319, 14.8479558, 15.65868405,
//      16.473408, 17.29188499, 18.11389597, 18.93924237,
//      19.76774356, 20.59923461, 21.4335645, 22.27059448,
//      23.11019674, 23.95225327, 24.79665478, 25.64329988,
//      26.49209426, 27.34295004, 28.19578518, 29.05052293,
//      29.90709137, 30.76542301, 31.6254544, 32.48712579,
//      33.35038089, 34.21516651, 35.08143242, 35.94913102,
//      36.81821727, 37.68864839, 38.56038379, 39.43338485,
//      40.30761485, 41.18303878, 42.05962329, 42.93733653,
//      43.8161481, 44.69602892, 45.5769512, 46.4588883,
//      47.34181472, 48.225706, 49.11053868, 49.99629021,
//      50.88293897, 51.77046414, 52.65884569, 53.54806438,
//      54.43810163, 55.32893957, 56.22056097, 57.11294919,
//      58.00608819, 58.89996246, 59.79455705, 60.68985747,
//      61.58584975, 62.48252034, 63.37985614, 64.27784447,
//      65.17647304, 66.07572996, 66.97560368, 67.87608301,
//      68.77715708, 69.67881538, 70.58104766, 71.48384399,
//      72.38719472, 73.29109048, 74.19552215, 75.10048085,
//      76.00595797, 76.91194512, 77.81843412, 78.72541703,
//      79.6328861, 80.54083379, 81.44925275, 82.35813581};
//0.95
const double chi_squared_test_table[100] =
    {0.00393214, 0.102586589, 0.351846318, 0.710723021, 1.145476226, 1.635382894,
     2.167349909, 2.732636793, 3.325112843, 3.940299136, 4.574813079, 5.226029488,
     5.891864338, 6.570631384, 7.260943928, 7.961645572, 8.671760205, 9.390455081,
     10.11701306, 10.85081139, 11.59130521, 12.33801458, 13.09051419, 13.84842503,
     14.61140764, 15.37915658, 16.15139585, 16.92787504, 17.70836618, 18.49266098,
     19.28056856, 20.07191346, 20.86653399, 21.66428071, 22.46501522, 23.26860902,
     24.07494256, 24.88390438, 25.6953904, 26.5093032, 27.32555147, 28.1440495,
     28.96471667, 29.78747708, 30.61225915, 31.43899527, 32.26762153, 33.09807743,
     33.93030562, 34.76425168, 35.59986394, 36.43709324, 37.2758928, 38.11621806,
     38.95802653, 39.80127763, 40.64593263, 41.49195448, 42.33930773, 43.18795845,
     44.03787413, 44.88902356, 45.74137684, 46.59490522, 47.4495811, 48.30537793,
     49.16227018, 50.02023325, 50.87924348, 51.73927805, 52.60031495, 53.46233296,
     54.3253116, 55.18923108, 56.05407229, 56.91981675, 57.78644661, 58.65394456,
     59.52229389, 60.39147839, 61.26148237, 62.13229063, 63.00388842, 63.87626144,
     64.74939583, 65.62327812, 66.49789524, 67.37323449, 68.24928355, 69.12603043,
     70.00346347, 70.88157134, 71.76034302, 72.63976779, 73.51983519, 74.40053508,
     75.28185754, 76.16379294, 77.04633186, 77.92946517};
/**
 * @brief  spilt string 
 * @note   
 * @param  &in: string in which need split
 * @param  &delim: spliter mark
 * @retval 
 */
std::vector<std::string>
TextSplit(const std::string &in, const std::string &delim);

std::string &trim(std::string &s);

std::ostream &operator<<(std::ostream &output, const NavTime &time);

NavInfo InterpolateNavInfo(const NavInfo &nav_info1, const NavInfo &nav_info2, const double &time_);

NavInfo &NormalizeAttitude(NavInfo &nav_info);
/**
 * @brief  override the ostream output NavResultInfo
 * @note   
 * @param  &output: ostream& 
 * @param  nav_res_info: 
 * @retval 
 */
std::ostream &operator<<(std::ostream &output, const NavInfo &nav_info);
std::ostream &operator<<(std::ostream &output, const ImuData &imu_data);
std::ostream &operator<<(std::ostream &output, const ImuData::Ptr &imu_data);
std::ostream &operator<<(std::ostream &output, const GnssData &gnss_data);
std::ostream &operator<<(std::ostream &output, const GnssData::Ptr &gnss_data);
/**
 * @brief  skew matrix from vector
 * @note   
 * @param  vector: 
 * @retval 
 */
Eigen::Matrix3d skew(const Eigen::Vector3d &vector);
/**
  * @brief  calculate the euler from Rotation Matrix 
  * @note   reference the Eigen::Matrix::eulerAngles 
  * @note   roll [-pi/2~pi/2], pitch[-pi/2~pi/2], heading/yaw[-pi~pi]
  * @param  Eigen::Matrix<Derived:  the vector of euler by order roll -> pitch -> heading/yaw
  * @param  &coeff: Rotation Matrix
  * @retval 
  */
template <typename Derived>
inline Eigen::Matrix<Derived, 3, 1> eulerAngles(const Eigen::Matrix<Derived, 3, 3> &coeff)
{
  Eigen::Matrix<Derived, 3, 1> res;

  const size_t i = 2;
  const size_t j = 1;
  const size_t k = 0;
  typedef Eigen::Matrix<Derived, 2, 1> Vector2;

  res[2] = atan2(coeff(j, k), coeff(k, k));
  Derived c2 = Vector2(coeff(i, i), coeff(i, j)).norm();
  // if (res[2] < Derived(0))
  // {
  //     res[2] += Derived(EIGEN_PI) * 2;
  // }
  res[1] = atan2(-coeff(i, k), c2);
  Derived s1 = sin(res[2]);
  Derived c1 = cos(res[2]);
  res[0] = atan2(s1 * coeff(k, i) - c1 * coeff(j, i), c1 * coeff(j, j) - s1 * coeff(k, j));

  return res;
}
/**
  * @brief  override the ostream output vector<T>
  * @note   
  * @param  &output: 
  * @param  &data: output data vector<T>
  * @retval 
  */
template <typename T>
std::ostream &
operator<<(std::ostream &output, const std::vector<T> &data)
{
  for (auto index : data)
  {
    output << index << "\t";
  }
  return output;
}

template <typename T>
T interpolate(const T &t1, const T &t2, double coeff)
{
  T t_res = t1 + (t2 - t1) * coeff;
  return t_res;
}

} // namespace utiltool

#endif /* !UTIL_BASE_H_ */
