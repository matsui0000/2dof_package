#pragma once

#include <stdio.h>
#include <fstream>
#include <cmath>
#include <iomanip>
#include <termios.h>
#include <math.h>
#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <iostream>
#include <cmath>
#include <string>
#include <vector>
#include <chrono>
#include <sstream>  
#include <sys/stat.h> 
#include "inflatable/DataStreamClient.h"
#include "inflatable/msg/voltage_output.hpp"
#include "inflatable/msg/voltage_input.hpp"



class ArmControlNode : public rclcpp::Node {
public:
    ArmControlNode();

private:
    //クラス定義
    Time timer;
    RobotPosition pos;
    RobotParameter param;
    RobotOrientation orientation;
    Pressure pressure;
    ImpactState impactState[DEGREE_OF_FREEDOM];

    rclcpp::Publisher<inflatable::msg::VoltageOutput>::SharedPtr ros_pub_;
    rclcpp::Subscription<inflatable::msg::VoltageInput>::SharedPtr ros_sub2_;
    rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr ros_sub3_;
    rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr ros_sub4_;
    void msgCallback2(const inflatable::msg::VoltageInput::SharedPtr msg);
    void msgCallback_hand(const geometry_msgs::msg::TransformStamped::SharedPtr msg);
    void msgCallback_base(const geometry_msgs::msg::TransformStamped::SharedPtr msg);

    bool setup_flag = false;
    bool InputReady_flag = false;
    bool SquareInput_flag = false;
    bool CombinationInput_flag = false;

    bool ImpactDrivenControl_flag = false; //インパクト駆動制御
    bool ImpactInput_flag = false;   //インパクト駆動入力 矩形入力後にoffになり，次の矩形入力までonで待機する

    bool PressureSinWave_flag =false;
    bool AngleSinWave_flag = false;

    bool PressureFeedback_flag = false;
    bool VisualFeedback_flag = false;

    double orientationCurrent_buf;  //繰り返し入力で立ち下がりを行った際の角度[rad]
    double orientation_w_buf;
    double orientation_delta;
    double Pdf;
    double now_time = 0.0;
    double push_time = 0.0;         //ボタンを押した時間[s]
    
    double inputstart_time = 5.0;   //立ち上がり入力開始時間[s]
    double inputdown_time = 0.0;    //立ち下がり入力開始時間[s]

    double sinwave_frequency = 0.5; //sin波の周波数[Hz]
    double sinwave_amplitude = 20;  //sin波の振幅[kPa]

    unsigned int cycle_buf = 0; //サンプリング周期のバッファ
    unsigned int start_cycle = 0;
    unsigned int inputdown_cycle = 0;
    unsigned int interval_cycle = 0;
    unsigned int interval_cycle_buf = 0;
    
    bool is_first = true;

    std::ofstream ofs_;
    void open_log_file();
    void control_loop_P();
    void viconUpdateLoop();

    geometry_msgs::msg::TransformStamped base_pose;
    bool msgCallback_base_flag;

    rclcpp::TimerBase::SharedPtr timer_;
    inflatable::msg::VoltageOutput pub_msg_;
    ViconDataStreamSDK::CPP::Client vicon_client_;

    LowPassFilter LowPassFilter_D1;
    LowPassFilter LowPassFilter_D2;
    LowPassFilter LowPassFilter_D3;
    LowPassFilter LowPassFilter_Pressure1;
    LowPassFilter LowPassFilter_Pressure2;
    LowPassFilter LowPassFilter_Angle0;
    LowPassFilter LowPassFilter_Angle1;
    LowPassFilter LowPassFilter_Angle2;

};