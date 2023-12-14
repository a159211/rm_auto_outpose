
#ifndef AUTO_OUTPOSE__AUTO_OUTPOSE_NODE_HPP_
#define AUTO_OUTPOSE__AUTO_OUTPOSE_NODE_HPP_

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

namespace rm_auto_outpose
{

enum class ArmorsNum { NORMAL_4 = 4, BALANCE_2 = 2, OUTPOST_3 = 3 };

class AutoOutposeNode : public rclcpp::Node
{
public:

    using Armor = auto_aim_interfaces::msg::Armor;

    AutoOutposeNode(const rclcpp::NodeOptions & options);

private:

    double get_next_x(double now_x);

    void updateArmorsNum(const Armor & armor);

    void GetTargerAngle(const cv::Point3f armor_center);

    cv::Point3f xyz_transformation(cv::Point3f input_point);

    void OutposeCallback(const auto_aim_interfaces::msg::Armors outpose_info);

    rclcpp::Subscription<auto_aim_interfaces::msg::Armors>::SharedPtr outpose_sub_;
    rclcpp::Subscription<auto_aim_interfaces::msg::OutposeReceive>::SharedPtr serial_sub;

    rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr temp_pub;
    rclcpp::Publisher<auto_aim_interfaces::msg::Target>::SharedPtr outpose_target_pub;

    double angle_last = 1000;
    int n = 0;
    bool have_find = false;         //在时间内循环中是否找到过装甲板  yes-1  no-0
    double T = 833.3333 * 3;   //ms
    bool time_up = false;
    bool Isfind_First = true;
    bool IsShut = false;
    double start;
    double start_temp;
    double end;
    Bulletmodel * bulletmodel;
    double yaw_;
    double pitch_;
    double shot_v = 15.0;

    double gun_yaw_ = 0;
    double gun_pitch_ = 0;

    ArmorsNum armors_num_;

};

}

#endif 