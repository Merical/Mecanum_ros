/***********************************************************************************************************************
* Copyright (c) Hands Free Team. All rights reserved.
* FileName: sc_link.cpp
* Contact:  QQ Exchange Group -- 521037187
* Version:  V2.0
*
* LICENSING TERMS:
* The Hands Free is licensed generally under a permissive 3-clause BSD license.
* Contributions are required to be made under the same license.
*
* History:
* <author>      <time>      <version>      <desc>
* luke liao       2016.4.1   V1.0           creat this file
*
* Description: handsfree ros ros_control framework
***********************************************************************************************************************/
#include <sc_hw/sc_hw_ros.h>
#include <unistd.h>

namespace smartcar_hw {

SC_HW_ros::SC_HW_ros(ros::NodeHandle &nh, std::string url, std::string config_addr) :
    sc_hw_(url, config_addr),
    nh_(nh)
{
    //get the parameter
    nh_.setCallbackQueue(&queue_);
    base_mode_ = "4omni-wheel";
    with_arm_ = false;
    controller_freq_ = 100;
    odom_angle_scale_correction = 1.0;
    odom_linear_scale_correction = 1.0;
    nh_.getParam("base_mode", base_mode_);
    nh_.getParam("with_arm", with_arm_);
    nh_.getParam("freq", controller_freq_);
    nh_.getParam("odom_linear_scale_correction", odom_linear_scale_correction);
    nh_.getParam("odom_angle_scale_correction", odom_angle_scale_correction);

    intf_state_ros_sub.robot_mode = 0;
    intf_state_ros_sub.robot_intf = 2;

    robot_state_publisher_ = nh_.advertise<sc_msgs::robot_state>("robot_state", 10);
    imu_data_publisher_ = nh_.advertise<sensor_msgs::Imu>("imu_data", 10);

    sonar_front_1_  = nh_.advertise<sensor_msgs::Range>("/sensor/sonar_front_1_", 10);
    sonar_front_5_  = nh_.advertise<sensor_msgs::Range>("/sensor/sonar_front_5_", 10);
    sonar_right_2_  = nh_.advertise<sensor_msgs::Range>("/sensor/sonar_right_2_", 10);
    sonar_right_6_  = nh_.advertise<sensor_msgs::Range>("/sensor/sonar_right_6_", 10);
    sonar_back_3_   = nh_.advertise<sensor_msgs::Range>("/sensor/sonar_back_3_", 10);
    sonar_back_7_   = nh_.advertise<sensor_msgs::Range>("/sensor/sonar_back_7_", 10);
    sonar_left_4_   = nh_.advertise<sensor_msgs::Range>("/sensor/sonar_left_4_", 10);
    sonar_left_8_   = nh_.advertise<sensor_msgs::Range>("/sensor/sonar_left_8_", 10);

    robot_intf_subscriber_ = nh_.subscribe("/sc/intf_state", 1, &SC_HW_ros::expectIntfGrab, this);
    ROS_INFO("System Ros publishers and subscribers are ready.");

    x_ = y_ = theta_ = x_cmd_ = y_cmd_ = theta_cmd_ = 0.0;
    x_vel_ = y_vel_ = theta_vel_ = 0.0;
//    last_x_raw = last_y_raw = last_theta_raw = delta_x = delta_y = delta_theta = 0.0;
    last_x_processed = last_y_processed = last_theta_processed = delta_x = delta_y = delta_theta = 0.0;
    distance = distance_corrected = x_raw = y_raw = theta_raw = 0.0;
    head_servo1_cmd_ = head_servo2_cmd_  =  0.0;
    head_servo1_pos_ = head_servo1_vel_ = head_servo1_eff_ = 0;
    head_servo2_pos_ = head_servo2_vel_ = head_servo2_eff_ = 0;

    //register the hardware interface on the robothw
    hardware_interface::BaseStateHandle base_state_handle("mobile_base", &x_, &y_, &theta_, &x_vel_, &y_vel_, &theta_vel_);
    base_state_interface_.registerHandle(base_state_handle);
    registerInterface(&base_state_interface_);
    hardware_interface::BaseVelocityHandle base_handle(base_state_handle, &x_cmd_, &y_cmd_, &theta_cmd_);
    base_velocity_interface_.registerHandle(base_handle);
    registerInterface(&base_velocity_interface_);
    ROS_INFO("Hardware interface registered.");

    if (base_mode_ == "3omni-wheel")
    {
        wheel_pos_.resize(3,0);
        wheel_vel_.resize(3.0);
        wheel_eff_.resize(3,0);
        wheel_cmd_.resize(3,0);

        hardware_interface::JointStateHandle wheel1_state_handle("wheel_1", &wheel_pos_[0], &wheel_vel_[0], &wheel_eff_[0]);
        jnt_state_interface_.registerHandle(wheel1_state_handle);
        hardware_interface::JointHandle wheel1_handle(wheel1_state_handle, &wheel_cmd_[0]);
        base_vel_interface_.registerHandle(wheel1_handle);

        hardware_interface::JointStateHandle wheel2_state_handle("wheel_2", &wheel_pos_[1], &wheel_vel_[1], &wheel_eff_[1]);
        jnt_state_interface_.registerHandle(wheel2_state_handle);
        hardware_interface::JointHandle wheel2_handle(wheel2_state_handle, &wheel_cmd_[1]);
        base_vel_interface_.registerHandle(wheel2_handle);

        hardware_interface::JointStateHandle wheel3_state_handle("wheel_3", &wheel_pos_[2], &wheel_vel_[2], &wheel_eff_[2]);
        jnt_state_interface_.registerHandle(wheel3_state_handle);
        hardware_interface::JointHandle wheel3_handle(wheel3_state_handle, &wheel_cmd_[2]);
        base_vel_interface_.registerHandle(wheel3_handle);

        registerInterface(&jnt_state_interface_);
        registerInterface(&base_vel_interface_);
    } else if (base_mode_ == "2diff-wheel")
    {
        wheel_pos_.resize(2,0);
        wheel_vel_.resize(2.0);
        wheel_eff_.resize(2,0);
        wheel_cmd_.resize(2,0);

        hardware_interface::JointStateHandle wheel1_state_handle("wheel1", &wheel_pos_[0], &wheel_vel_[0], &wheel_eff_[0]);
        jnt_state_interface_.registerHandle(wheel1_state_handle);
        hardware_interface::JointHandle wheel1_handle(wheel1_state_handle, &wheel_cmd_[0]);
        base_vel_interface_.registerHandle(wheel1_handle);

        hardware_interface::JointStateHandle wheel2_state_handle("wheel2", &wheel_pos_[1], &wheel_vel_[1], &wheel_eff_[1]);
        jnt_state_interface_.registerHandle(wheel2_state_handle);
        hardware_interface::JointHandle wheel2_handle(wheel2_state_handle, &wheel_cmd_[1]);
        base_vel_interface_.registerHandle(wheel2_handle);

        registerInterface(&jnt_state_interface_);
        registerInterface(&base_vel_interface_);

    } else if (base_mode_ == "4omni-wheel")
    {
        wheel_pos_.resize(4,0);
        wheel_vel_.resize(4.0);
        wheel_eff_.resize(4,0);
        wheel_cmd_.resize(4,0);

        hardware_interface::JointStateHandle wheel1_state_handle("wheel1", &wheel_pos_[0], &wheel_vel_[0], &wheel_eff_[0]);
        jnt_state_interface_.registerHandle(wheel1_state_handle);
        hardware_interface::JointHandle wheel1_handle(wheel1_state_handle, &wheel_cmd_[0]);
        base_vel_interface_.registerHandle(wheel1_handle);

        hardware_interface::JointStateHandle wheel2_state_handle("wheel2", &wheel_pos_[1], &wheel_vel_[1], &wheel_eff_[1]);
        jnt_state_interface_.registerHandle(wheel2_state_handle);
        hardware_interface::JointHandle wheel2_handle(wheel2_state_handle, &wheel_cmd_[1]);
        base_vel_interface_.registerHandle(wheel2_handle);

        hardware_interface::JointStateHandle wheel3_state_handle("wheel3", &wheel_pos_[2], &wheel_vel_[2], &wheel_eff_[2]);
        jnt_state_interface_.registerHandle(wheel3_state_handle);
        hardware_interface::JointHandle wheel3_handle(wheel3_state_handle, &wheel_cmd_[2]);
        base_vel_interface_.registerHandle(wheel3_handle);

        hardware_interface::JointStateHandle wheel4_state_handle("wheel4", &wheel_pos_[3], &wheel_vel_[3], &wheel_eff_[3]);
        jnt_state_interface_.registerHandle(wheel4_state_handle);
        hardware_interface::JointHandle wheel4_handle(wheel4_state_handle, &wheel_cmd_[3]);
        base_vel_interface_.registerHandle(wheel4_handle);

        registerInterface(&jnt_state_interface_);
        registerInterface(&base_vel_interface_);
    }
    ROS_INFO("System base mode set.");

    if (with_arm_)
    {
        for (int i = 0;i < 6;i++)
        {
            //get the joint name
            std::stringstream ss;
            ss << "arm" << (i + 1)<<std::endl;
            hardware_interface::JointStateHandle arm_state_handle(ss.str(), &arm_pos_[i], &arm_pos_[i], &arm_pos_[i]);
            jnt_state_interface_.registerHandle(arm_state_handle);
            hardware_interface::JointHandle arm_handle(arm_state_handle , &arm_cmd_[i]);
            servo_pos_interface_.registerHandle(arm_handle);
        }
    }

    hardware_interface::JointStateHandle head_servo1_state_handle("servo_1", &head_servo1_pos_, &head_servo1_vel_, &head_servo1_eff_);
    jnt_state_interface_.registerHandle(head_servo1_state_handle);
    hardware_interface::JointHandle head_servo1_handle(head_servo1_state_handle, &head_servo1_cmd_);
    servo_pos_interface_.registerHandle(head_servo1_handle);

    hardware_interface::JointStateHandle head_servo2_state_handle("servo_2", &head_servo2_pos_, &head_servo2_vel_, &head_servo2_eff_);
    jnt_state_interface_.registerHandle(head_servo2_state_handle);
    hardware_interface::JointHandle head_servo2_handle(head_servo2_state_handle, &head_servo2_cmd_);
    servo_pos_interface_.registerHandle(head_servo2_handle);

    registerInterface(&jnt_state_interface_);
    registerInterface(&servo_pos_interface_);

    if (sc_hw_.initialize_ok())
    {
        ROS_INFO("system initialized succeed, ready for communication");
    } else
    {
        ROS_ERROR("sc link initialized failed, please check the hardware");
    }
}

void SC_HW_ros::mainloop()
{
    ros::CallbackQueue cm_callback_queue;
    ros::NodeHandle cm_nh("mobile_base");
    cm_nh.setCallbackQueue(&cm_callback_queue);
    controller_manager::ControllerManager cm(this, cm_nh);

    ros::AsyncSpinner cm_spinner(1, &cm_callback_queue);
    ros::AsyncSpinner hw_spinner(1, &queue_);
    ros::Rate loop(controller_freq_);
    cm_spinner.start();
    hw_spinner.start();

    int count = 0;
    ros::Time currentTime = ros::Time::now();

    while (ros::ok())
    {
        sc_hw_.checkHandshake();
//        if (count == 0) {
//            sc_hw_.updateCommand(READ_MODULE_CONFIG, count);
//            cout << "LCH: the model config: plate_type "<< plate_type_ <<", imu_num "<< imu_num_ <<", track_num "<< track_num_ <<", ultra_num " << ultra_num_ << ", arm_num " << arm_num_ << ", head_num " << head_num_ << ", serv_num " << serv_num_ << endl;
//        }
//        if (sc_hw_.updateCommand(READ_SYSTEM_INFO, count))
//        {
//            std::cout<< "spend time is  "<< (ros::Time::now() - currentTime).toSec()<<std::endl;
//            currentTime = ros::Time::now();
//            robot_state_publisher_.publish(robot_state);
//        }
//        if (sc_hw_.updateCommand(READ_IMU_FUSION_DATA, count) && (count>0))
//        {
//            imu_data_publisher_.publish(imu_data);
//        }

        if (intf_state_ros_sub.robot_intf == 2 and intf_state_ros_sub.robot_mode == 0){
            sc_hw_.updateCommand(READ_GLOBAL_COORDINATE, count);
//            cout << "read coordinage " << endl;
            sc_hw_.updateCommand(READ_ROBOT_SPEED, count);
//            cout << "read speed " << endl;
            sc_hw_.updateCommand(READ_INTF_MODE, count);

            if (sc_hw_.updateCommand(READ_IMU_FUSION_DATA, count) && (count>0))
            {
                imu_data_publisher_.publish(imu_data);
            }
        }
//        sc_hw_.updateCommand(READ_SONAR_DATA, count);
//        sc_hw_.updateCommand(READ_HEAD_STATE, count);
        if (count == 0) readBufferUpdateFirst();
        else readBufferUpdate();
//        cout << "\nLCH: loop count " << count << " with format x, y, theta" << endl;
//        cout << "the raw data is " << x_raw << " " << y_raw << " " << theta_raw << endl;
//        cout << "the processed data is " << x_ << " " << y_ << " " << theta_ << endl;
//        cout << "\n";


        cm.update(ros::Time::now(), ros::Duration(1 / controller_freq_));

        writeBufferUpdate();
        if (count == 0 or intf_state_robot.robot_intf != intf_state_ros_sub.robot_intf or intf_state_robot.robot_mode != intf_state_ros_sub.robot_mode){
            sc_hw_.updateCommand(SET_INTF_MODE, count);
        }
        if (intf_state_ros_sub.robot_intf == 2 and intf_state_ros_sub.robot_mode == 0){
            sc_hw_.updateCommand(SET_ROBOT_SPEED, count);
        }
        loop.sleep();
        count++;

//        sc_hw_.checkHandshake();
//        sc_hw_.updateCommand(READ_GLOBAL_COORDINATE, count);
//        sleep(0.005);
//        sc_hw_.updateCommand(READ_ROBOT_SPEED, count);
//        sleep(0.005);
//        if (count == 0) readBufferUpdateFirst();
//        else readBufferUpdate();
//
//        cm.update(ros::Time::now(), ros::Duration(1 / controller_freq_));
//
//        writeBufferUpdate();
//        sc_hw_.updateCommand(SET_ROBOT_SPEED, count);
//        sleep(0.005);
//
//        loop.sleep();
//        count++;

    }

    cm_spinner.stop();
    hw_spinner.stop();
}

}
