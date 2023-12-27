#include"rm_auto_outpose/auto_outpose_node.hpp"
#include <cv_bridge/cv_bridge.h>

namespace rm_auto_outpose
{

AutoOutposeNode::AutoOutposeNode(const rclcpp::NodeOptions & options)
: Node("auto_outpose_node",options)
{
    RCLCPP_INFO(this->get_logger(), "Starting AutoOutposeNode!");

    outpose_target_pub = this->create_publisher<auto_aim_interfaces::msg::Target>("/tracker/target",rclcpp::SensorDataQoS());

    debug_outpose_pub = this->create_publisher<auto_aim_interfaces::msg::DebugOutpose>("/outpose_msg",rclcpp::SensorDataQoS());

    outpose_sub_ = this->create_subscription<auto_aim_interfaces::msg::Armors>
    ("/detector/armors",rclcpp::SensorDataQoS(),std::bind(&AutoOutposeNode::OutposeCallback, this, std::placeholders::_1));

    serial_sub = this->create_subscription<auto_aim_interfaces::msg::OutposeReceive>
    ("/sertial_info_",rclcpp::SensorDataQoS(),[this](auto_aim_interfaces::msg::OutposeReceive serial_info){
        this->gun_pitch_ = serial_info.pitch;
        this->gun_yaw_ = serial_info.yaw;
        // this->shot_v = serial_info.shot_v;
    });
}

void AutoOutposeNode::GetTargerAngle(const cv::Point3f armor_center,const double yaw_rvec){

    cv::Point3f outpose_center(armor_center.x+outpose_R*sin(yaw_rvec),armor_center.y+outpose_R*cos(yaw_rvec),armor_center.z);

    double K = sqrt( outpose_center.x*outpose_center.x+outpose_center.y*outpose_center.y ) / sqrt( outpose_center.x*outpose_center.x+(outpose_center.y-outpose_R)*(outpose_center.y-outpose_R) );

    cv::Point3f hit_center(outpose_center.x/K , outpose_center.y/K , outpose_center.z);

    bulletmodel = new Bulletmodel(this->shot_v,hit_center);

    double yaw_To_armor = -asin(armor_center.x / sqrt(armor_center.x*armor_center.x+armor_center.y*armor_center.y)) * 180 / CV_PI;  
    double pitch = -bulletmodel->get_angle() * 180 / CV_PI;    //加了弹道补偿

    double yaw_To_hit_center = -asin(hit_center.x / sqrt(hit_center.x*hit_center.x+ (hit_center.y+0.023496) * (hit_center.y+0.023496) )) * 180 / CV_PI;

    this->yaw_temp_ = yaw_To_armor;
    this->yaw_ = yaw_To_hit_center;
    this->pitch_ = pitch;

}

void AutoOutposeNode::updateArmorsNum(const Armor & armor)
{
  if (armor.type == "large" && (armor.number == "3" || armor.number == "4" || armor.number == "5")) {
    armors_num_ = ArmorsNum::BALANCE_2;
  } else if (armor.number == "outpost") {
    armors_num_ = ArmorsNum::OUTPOST_3;
  } else {
    armors_num_ = ArmorsNum::NORMAL_4;
  }
}

bool AutoOutposeNode::predicte(const cv::Point3f armor_center,const double yaw_rvec){

    if(rotation_D_ == 0){                   //逆
        next_x_Tocenter = sin( yaw_rvec + 0.8 * CV_PI ) * outpose_R + armor_center.x;
    }
    else if(rotation_D_ == 1){              //顺
        next_x_Tocenter = sin( yaw_rvec - 0.8 * CV_PI ) * outpose_R + armor_center.x;
    }
    else{
        return false;
    }

    return true;
}

void AutoOutposeNode::rotation_direation(){
    double sum_x = 0;
    for(int i=0;i<15;i++){
        sum_x += v_armor_x[v_armor_x.size()-16+i];
    }
    if(v_armor_x[v_armor_x.size()-1]> (sum_x/15) ){
        rotation_D_ = 0;
    }
    else{
        rotation_D_ = 1;
    }
    
}

cv::Point3f AutoOutposeNode::xyz_transformation(cv::Point3f input_point){        //坐标系转换  相机坐标系-->绝对坐标系  (shang z  qian y  you x)

    cv::Point3f temppoint(input_point.x,input_point.z,-input_point.y);
    double matrix_A[1][4] = {temppoint.x, temppoint.y, temppoint.z, 1};
    double matrix_T[4][4] = {{1     , 0       , 0    , 0},
                             {0     , 1       , 0    , 0},
                             {0     , 0       , 1    , 0},
                             {0.08675, -0.10254, 0.2216, 1}};

    double matrix_target[1][4] = {0};

    for(int i=0;i<4;i++){
        for(int j=0;j<4;j++){
            matrix_target[0][i] += matrix_A[0][j]*matrix_T[j][i];
        }
    }

    return cv::Point3f(matrix_target[0][0],matrix_target[0][1],matrix_target[0][2]);
}

void AutoOutposeNode::Init(auto_aim_interfaces::msg::Target & data){
    data.tracking = false;
    data.shootstatus = false;
    yaw_ = 1000;
    yaw_temp_ = 1000;
    next_x_Tocenter = 1000;
}

void AutoOutposeNode::OutposeCallback(const auto_aim_interfaces::msg::Armors outpose_info)
{
    auto_aim_interfaces::msg::Target data;
    auto_aim_interfaces::msg::DebugOutpose debug_outpose_msg;

    Init(data);

    for(auto outpose_armor : outpose_info.armors)
    {
        updateArmorsNum(outpose_armor);

        cv::Point3f armor_point_camera(outpose_armor.pose.position.x,outpose_armor.pose.position.y,outpose_armor.pose.position.z);

        cv::Point3f armor_point_tf = xyz_transformation(armor_point_camera);

        GetTargerAngle(armor_point_tf,outpose_armor.yaw_r);

        predicte(armor_point_tf,outpose_armor.yaw_r);

        // std::cout<<"next_x_Tocenter:"<<next_x_Tocenter<<std::endl;
        // if(rotation_D_==1){
        //     std::cout<<"顺时针"<<std::endl;
        // }
        // else if(rotation_D_==0){
        //     std::cout<<"逆时针"<<std::endl;
        // }

        v_armor_x.push_back(armor_point_tf.x);

        data.id = outpose_armor.number;
        data.armors_num = static_cast<int>(armors_num_);
        data.yaw =  yaw_ + gun_yaw_;
        data.pitch = pitch_;

        if(Isfind_First){         //调枪口
            Isfind_First = false;
            rotation_D_ = -1;

            data.tracking = true;

            RCLCPP_INFO(this->get_logger(), "第一次找到合适的角度,并调好枪口"); 
        }
    }

    if( abs(yaw_temp_) <= 6 && !Isfind_First && first_shot ){  //first shot
        RCLCPP_INFO(this->get_logger(), "first shot"); 
        start = std::chrono::steady_clock::now();
        IsShut = true;
        first_shot = false;

        rotation_direation();
        v_armor_x.clear();
    }

    end = std::chrono::steady_clock::now();
    double elapsed_ms = std::chrono::duration_cast<std::chrono::microseconds>(end-start).count()/1000;  //ms
    // std::cout<<"elapsed_ms:"<<elapsed_ms<<std::endl;

    if( abs(T-elapsed_ms) <= 100 && !Isfind_First && !first_shot)
    {
        if( abs(T-elapsed_ms) <= 20 ){
            start_temp = std::chrono::steady_clock::now();
        }

        time_up = true;

        if( abs(yaw_temp_) <= 8 && !have_find  && abs(next_x_Tocenter) <= 0.1){  //找到可射击装甲板
            IsShut = true;
            have_find = true;
            n=0;
        }
    }

    if( time_up && abs(T-elapsed_ms) > 100 ){  //判断时间内循环结束
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
        first_shot = true;
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

    }

    if( elapsed_ms >= 4000 && !Isfind_First && !first_shot){
        first_shot = true;
        Isfind_First = true;
        IsShut = false;
        RCLCPP_INFO(this->get_logger(), "时间未读取到规定周期"); 
    }

    debug_outpose_msg.yaw_to_hit_center = yaw_;
    debug_outpose_msg.yaw_to_armor = yaw_temp_;
    debug_outpose_msg.next_x_tocenter = next_x_Tocenter;
  
    outpose_target_pub->publish(data);
    debug_outpose_pub->publish(debug_outpose_msg);
}

}


#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_outpose::AutoOutposeNode)