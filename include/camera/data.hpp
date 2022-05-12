/*
** data.hpp for mscnav in /media/fwt/Data/program/mscnav/include/camera
**
** Made by little fang
** Login   <fangwentao>
**
** Started on  Wed Aug 21 下午9:24:01 2019 little fang
** Last update Sat Aug 23 下午2:11:44 2019 little fang
*/

#ifndef CAMERA_DATA_H_
#define CAMERA_DATA_H_

#include "navstruct.hpp"
#include "navtime.h"
#include <opencv2/core/core.hpp>

namespace mscnav
{

namespace camera
{

using utiltool::BaseData;
using utiltool::NavTime;

class CameraData : public BaseData
{
public:
    using Ptr = std::shared_ptr<CameraData>;
    CameraData(const double &time) : BaseData(time)
    {
        data_type_ = utiltool::DataType::CAMERADATA;
    };
    CameraData(const double &time,const std::string& stdImgTxt) : BaseData(time)
    {
        ImgTxt = stdImgTxt;
        data_type_ = utiltool::DataType::CAMERADATA;
    };
    CameraData()
    {
        data_type_ = utiltool::DataType::CAMERADATA;
    }
    ~CameraData() {}
    CameraData &operator=(const CameraData &camera)
    {
        this->t0_ = camera.get_time();
        this->data_type_ = camera.get_type();
        this->ImgTxt = camera.ImgTxt;
        return *this;
    }
    CameraData(const CameraData &camera)
    {
        this->t0_ = camera.get_time();
        this->data_type_ = camera.get_type();
        this->ImgTxt = camera.ImgTxt;
    }

public:
    int ImgID;
    std::string ImgTxt;
};
using CAMERADATAPOOL = std::deque<CameraData::Ptr>;

} // namespace camera
} // namespace mscnav

#endif /* !CAMERA_DATA_H_ */
