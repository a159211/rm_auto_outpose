
#ifndef AUTO_OUTPOSE__AUTO_OUTPOSE_NODE_HPP_
#define AUTO_OUTPOSE__AUTO_OUTPOSE_NODE_HPP_

#include <vector>

#include"rclcpp/rclcpp.hpp"
#include"auto_aim_interfaces/msg/armors.hpp"
#include"auto_aim_interfaces/msg/armor.hpp"
#include <sensor_msgs/msg/image.hpp>
#include"opencv2/opencv.hpp"
#include"auto_aim_interfaces/msg/outpose.hpp"
#include"std_msgs/msg/u_int32.hpp"
#include"rm_auto_outpose/BulletModel.hpp"
#include"auto_aim_interfaces/msg/target.hpp"
#include"auto_aim_interfaces/msg/outpose_receive.hpp"
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include<chrono>

namespace rm_auto_outpose
{

enum class ArmorsNum { NORMAL_4 = 4, BALANCE_2 = 2, OUTPOST_3 = 3 };

#define outpose_R 0.2765

class AutoOutposeNode : public rclcpp::Node
{
public:

    using Armor = auto_aim_interfaces::msg::Armor;

    AutoOutposeNode(const rclcpp::NodeOptions & options);

private:

    bool predicte(const cv::Point3f armor_center,const double yaw_rvec);

    void updateArmorsNum(const Armor & armor);

    void GetTargerAngle(const cv::Point3f armor_center,const double yaw_rvec);

    cv::Point3f xyz_transformation(cv::Point3f input_point);

    void rotation_direation();

    void Init(auto_aim_interfaces::msg::Target & data);

    void OutposeCallback(const auto_aim_interfaces::msg::Armors outpose_info);

    rclcpp::Subscription<auto_aim_interfaces::msg::Armors>::SharedPtr outpose_sub_;
    rclcpp::Subscription<auto_aim_interfaces::msg::OutposeReceive>::SharedPtr serial_sub;

    rclcpp::Publisher<auto_aim_interfaces::msg::Target>::SharedPtr outpose_target_pub;

    std::vector<double> v_armor_x;

    int n = 0;
    int rotation_D_ = -1;  //1是顺  0是逆

    double T = 833.3333 * 3;   //ms

    bool have_find = false;         //在时间内循环中是否找到过装甲板  yes-1  no-0
    bool time_up = false;
    bool Isfind_First = true;
    bool IsShut = false;
    bool first_shot = true;

    std::chrono::steady_clock::time_point start;
    std::chrono::steady_clock::time_point start_temp;
    std::chrono::steady_clock::time_point end;

    Bulletmodel * bulletmodel;
    double next_x_Tocenter;
    double yaw_;
    double yaw_temp_;
    double pitch_;
    double shot_v = 15.0;

    double gun_yaw_ = 0;
    double gun_pitch_ = 0;

    ArmorsNum armors_num_;

};

}

#endif 