#include "rm_auto_outpose/BulletModel.hpp"

#define g 9.8

namespace rm_auto_aim
{

// Bulletmodel::Bulletmodel(double v0,const cv::Point3f center_armor) {
//     this->v0 = v0;
//     this->y = -center_armor.y;
//     this->z = center_armor.z;
// }

Bulletmodel::Bulletmodel(double v0,const cv::Point3f center_armor) {
    this->v0 = v0;
    this->y = center_armor.y;
    this->z = center_armor.z;
}

// double Bulletmodel::get_BulletModel(double h_act) {
//     double s = sqrt(z*z+h_act*h_act);
//     double angle = asin(h_act/s);

//     double t = z/(v0*cos(angle));
//     t_ = t;
//     double h = v0*sin(angle)*t-g*t*t/2;
//     return h;
// }

double Bulletmodel::get_BulletModel(double h_act) {
    double s = sqrt(y*y+h_act*h_act);
    double angle = asin(h_act/s);

    double t = y/(v0*cos(angle));
    t_ = t;
    double h = v0*sin(angle)*t-g*t*t/2;
    return h;
}

double Bulletmodel::get_angle() {
    double z_temp,z_actual;
    z_temp = z;

    for(int i=0;i<20;i++) {
        double s = sqrt(z_temp*z_temp+y*y);
        pitch = asin(z_temp / s);
        z_actual = get_BulletModel(z_temp);
        z_buchang = z - z_actual;
        z_temp = z_temp + z_buchang;

        if (abs(z_buchang) <= 0.0001) {
            break;
        }

    }
    return pitch;
}

// double Bulletmodel::get_angle() {
//     double y_temp,y_actual;
//     y_temp = y;

//     for(int i=0;i<20;i++) {
//         double s = sqrt(y_temp*y_temp+z*z);
//         pitch = asin(y_temp / s);
//         y_actual = get_BulletModel(y_temp);
//         y_buchang = y - y_actual;
//         y_temp = y_temp + y_buchang;

//         if (abs(y_buchang) <= 0.0001) {
//             break;
//         }

//     }
//     return pitch;
// }


}