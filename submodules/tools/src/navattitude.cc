#include "navattitude.hpp"

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
Eigen::Matrix3d Quaternion2RotationMatrix(const Eigen::Quaterniond &q)
{
    return q.toRotationMatrix();
}

/**
  * @brief  rotation martrix to quaternion by Eigen
  * @note   
  * @param  mat: 
  * @retval 
  */
Eigen::Quaterniond RotationMartix2Quaternion(const Eigen::Matrix3d &mat)
{
    Eigen::Quaterniond q(mat);
    return q;
}

/**
 * @brief  
 * @note   
 * @param  &euler: 
 * @retval 
 */
Eigen::Quaterniond Euler2Quaternion(const Euler &euler)
{
    //DONE check by little fang in 20190712
    return (Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitX()));
}

/**
 * @brief  Euler angle to rotation matrix
 * @note   
 * @param  euler: Euler angle
 * @retval 
 */
Eigen::Matrix3d Euler2RotationMatrix(const Euler &euler)
{
    return Quaternion2RotationMatrix(Euler2Quaternion(euler));
}

/**
  * @brief  Rotation Matrix to Euler by Eigen
  * @note   note the order of the axis
  * @param  &mat: 
  * @retval 
  */
Euler RotationMartix2Euler(const Eigen::Matrix3d &mat)
{
    // DONE check by little fang. writed by ourselies, do not use Eigen.
    return eulerAngles(mat);
}

/**
 * @brief  quaternion to euler by rotation matirx
 * @note   
 * @param  q: quaternion
 * @retval 
 */
Euler Quaternion2Euler(const Eigen::Quaterniond &q)
{
    return RotationMartix2Euler(q.toRotationMatrix());
}

/**
 * @brief  according the rotation vector construct the Quaternion
 * @note   
 * @param  rv: Equivalent rotation vector 
 * @retval 
 */
Eigen::Quaterniond RotationVector2Quaternion(const RotationVector &rv)
{
    Eigen::Quaterniond qfromrv;
    RotationVector rv_2 = rv * 0.5;
    double norm = rv_2.norm();
    qfromrv.w() = cos(norm);
    qfromrv.vec() = norm < 1e-8 ? rv_2 : (sin(norm) / norm) * rv_2;
    return qfromrv;
}

/**
 * @brief  Rotation Vector to Rotation Matrix
 * @note   
 * @param  &rv: Equivalent rotation vector 
 * @retval 
 */
Eigen::Matrix3d RotationVector2RotationMatrix(const RotationVector &rv)
{
    return Quaternion2RotationMatrix(RotationVector2Quaternion(rv));
}
} // namespace attitude
} // namespace utiltool
