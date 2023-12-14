#include"rm_auto_outpose/auto_outpose_node.hpp"
#include <cv_bridge/cv_bridge.h>

namespace rm_auto_outpose
{

AutoOutposeNode::AutoOutposeNode(const rclcpp::NodeOptions & options)
: Node("auto_outpose_node",options)
{
    RCLCPP_INFO(this->get_logger(), "Starting AutoOutposeNode!");

    temp_pub = this->create_publisher<std_msgs::msg::UInt32>("/outpose_bool",rclcpp::SensorDataQoS());
    outpose_target_pub = this->create_publisher<auto_aim_interfaces::msg::Target>("/tracker/target",rclcpp::SensorDataQoS());

    outpose_sub_ = this->create_subscription<auto_aim_interfaces::msg::Armors>
    ("/detector/armors",rclcpp::SensorDataQoS(),std::bind(&AutoOutposeNode::OutposeCallback, this, std::placeholders::_1));

    serial_sub = this->create_subscription<auto_aim_interfaces::msg::OutposeReceive>
    ("/sertial_info_",rclcpp::SensorDataQoS(),[this](auto_aim_interfaces::msg::OutposeReceive serial_info){
        this->gun_pitch_ = serial_info.pitch;
        this->gun_yaw_ = serial_info.yaw;
        // this->shot_v = serial_info.shot_v;
    });
}

void AutoOutposeNode::GetTargerAngle(const cv::Point3f armor_center){

    bulletmodel = new Bulletmodel(this->shot_v,armor_center);

    double yaw = asin(armor_center.x / sqrt(armor_center.x*armor_center.x+armor_center.y*armor_center.y)) * 180 / CV_PI ;  
    double pitch = bulletmodel->get_angle() * 180 / CV_PI;    //加了弹道补偿
    this->yaw_ = yaw;
    this->pitch_ = pitch;

}

double AutoOutposeNode::get_next_x(double now_x){

    double angle_now = asin(now_x/0.553);  //弧度
    double angle_next;
    if(angle_last - yaw_ <= 0){    //逆时针
        angle_next = angle_now - 0.4 * 2 * CV_PI * bulletmodel->t_;
    }
    else{           //顺时针
        angle_next = angle_now + 0.4 * 2 * CV_PI * bulletmodel->t_;
    }
    angle_last = yaw_;
    
    return angle_next * 0.553;
}

cv::Point3f AutoOutposeNode::xyz_transformation(cv::Point3f input_point){        //坐标系转换  相机坐标系-->绝对坐标系  (shang z  qian y  you x)

    cv::Point3f temppoint(input_point.x,input_point.z,-input_point.y);
    double matrix_A[1][4] = {temppoint.x, temppoint.y, temppoint.z, 1};
    double matrix_T[4][4] = {{1     , 0       , 0    , 0},
                             {0     , 1       , 0    , 0},
                             {0     , 0       , 1    , 0},
                             {0.0868, -0.07905, 0.388, 0}};

    double matrix_target[1][4] = {0};

    for(int i=0;i<4;i++){
        for(int j=0;j<4;j++){
            matrix_target[0][i] += matrix_A[0][j]*matrix_T[j][i];
        }
    }

    return cv::Point3f(matrix_target[0][0],matrix_target[0][1],matrix_target[0][2]);
}

void AutoOutposeNode::OutposeCallback(const auto_aim_interfaces::msg::Armors outpose_info)
{
    auto_aim_interfaces::msg::Target data;
    data.tracking = false;
    data.shootstatus = false;
    double x_next = 1000;

    std_msgs::msg::UInt32 msg_int32;

    for(auto outpose_armor : outpose_info.armors)
    {
        cv::Point3f armor_point_camera(outpose_armor.pose.position.x,outpose_armor.pose.position.y,outpose_armor.pose.position.z);

        cv::Point3f armor_point_tf = xyz_transformation(armor_point_camera);

        GetTargerAngle(armor_point_tf);

        x_next = get_next_x(armor_point_tf.x);

        // std::cout<<"x_next:"<<x_next<<std::endl;
        // std::cout<<"yaw_:"<<yaw_<<std::endl;
        // std::cout<<"pitch_:"<<pitch_<<std::endl;

        data.yaw = yaw_ + gun_yaw_;
        data.pitch = pitch_;

        if( abs(yaw_) <= 2.5 && Isfind_First){         //调枪口
            start = this->now().seconds()*1000;
            Isfind_First = false;

            data.tracking = true;

            msg_int32.data = 1;

            RCLCPP_INFO(this->get_logger(), "第一次找到合适的角度,并调好枪口"); 
        }
    }

    end = this->now().seconds()*1000;
    double elapsed_ms = end - start;  //ms

    if( abs(T-elapsed_ms) <= 150 && ! Isfind_First )
    {
        if( abs(T-elapsed_ms) <= 15 ){
            start_temp = this->now().seconds()*1000;
        }

        time_up = true;


        if( abs(yaw_) <= 4 &&  x_next <= 0.1 && !have_find){  //找到可射击装甲板
            IsShut = true;
            have_find = true;
            n=0;
        }
    }

    if( time_up && abs(T-elapsed_ms) > 150 ){  //判断时间内循环结束
        time_up = false;
        start = start_temp;

        if(!have_find){
            n++;
            IsShut = false;
            RCLCPP_INFO(this->get_logger(), "第%d次过了固定的周期后未找到角度合适的装甲板",n); 
        }
        have_find = false;
    }

    if(n>=2){
        RCLCPP_INFO(this->get_logger(), "累积%d次过了固定周期未找到角度合适装甲板,重新选取开始时间",n); 
        Isfind_First = true;
        IsShut = false;
        n=0;
    }

    if(IsShut){  //可以发射
        IsShut = false;
        have_find = true;
        n=0;

        data.shootstatus = true;

        Isfind_First = false;

        RCLCPP_INFO(this->get_logger(), "data.yaw:%f",data.yaw); 
        RCLCPP_INFO(this->get_logger(), "data.pitch:%f",data.pitch); 

        msg_int32.data = 2;

    }

    if( elapsed_ms >= 4000 && !Isfind_First ){
        Isfind_First = true;
        IsShut = false;
        RCLCPP_INFO(this->get_logger(), "时间未读取到规定周期"); 
    }
  
    temp_pub->publish(msg_int32);
    outpose_target_pub->publish(data);
}

}


#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_outpose::AutoOutposeNode)