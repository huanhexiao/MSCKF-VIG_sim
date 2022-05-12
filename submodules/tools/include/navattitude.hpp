/*
** navattitude.hpp for mscnav in /home/fwt/myprogram/mscnav/submodules/tools/include
**
** Made by little fang
** Login   <fangwentao>
**
** Started on  Wed Jul 10 下午12:39:42 2019 little fang
** Last update Sun Jul 13 上午6:30:36 2019 little fang
*/

#ifndef NAVATTITUDE_H_
#define NAVATTITUDE_H_
#include "stdio.h"
#include "navbase.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace utiltool
{

namespace attitude
{

using RotationVector = Eigen::Vector3d;
using Euler = Eigen::Vector3d; /*roll, pitch, heading*/

/**
 * @brief  quaternion to rotation martrix by Eigen
 * @note   
 * @param  q: quaternion 
 * @retval rotation matrix 
 */
Eigen::Matrix3d Quaternion2RotationMatrix(const Eigen::Quaterniond &q);

/**
  * @brief  rotation martrix to quaternion by Eigen
  * @note   
  * @param  mat: 
  * @retval 
  */
Eigen::Quaterniond RotationMartix2Quaternion(const Eigen::Matrix3d &mat);

/**
 * @brief  
 * @note   
 * @param  &euler: 
 * @retval 
 */
Eigen::Quaterniond Euler2Quaternion(const Euler &euler);

/**
 * @brief  Euler angle to rotation matrix
 * @note   
 * @param  euler: Euler angle
 * @retval 
 */
Eigen::Matrix3d Euler2RotationMatrix(const Euler &euler);

/**
  * @brief  Rotation Matrix to Euler by Eigen
  * @note   note the order of the axis
  * @param  &mat: 
  * @retval 
  */
Euler RotationMartix2Euler(const Eigen::Matrix3d &mat);

/**
 * @brief  quaternion to euler by rotation matirx
 * @note   
 * @param  q: quaternion
 * @retval 
 */
Euler Quaternion2Euler(const Eigen::Quaterniond &q);

/**
 * @brief  according the rotation vector construct the Quaternion
 * @note   
 * @param  rv: Equivalent rotation vector 
 * @retval 
 */
Eigen::Quaterniond RotationVector2Quaternion(const RotationVector &rv);

/**
 * @brief  Rotation Vector to Rotation Matrix
 * @note   
 * @param  &rv: Equivalent rotation vector 
 * @retval 
 */
Eigen::Matrix3d RotationVector2RotationMatrix(const RotationVector &rv);
} // namespace attitude
} // namespace utiltool

#endif /* !NAVATTITUDE */
