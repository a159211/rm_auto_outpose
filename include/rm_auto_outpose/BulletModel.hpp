#ifndef AUTO_OUTPOSE__BULLETMODEL_HPP_
#define AUTO_OUTPOSE__BULLETMODEL_HPP_

#include <iostream>
#include "opencv2/opencv.hpp"

namespace rm_auto_aim
{
class Bulletmodel {
public:
    Bulletmodel(double v0,const cv::Point3f center_armor);

    double get_angle();


    double t_;

private:

    double get_BulletModel(double angle);

    float pitch;//补偿的角度
    double z_buchang;
    double v0;
    double y;
    double z;

};
}



#endif