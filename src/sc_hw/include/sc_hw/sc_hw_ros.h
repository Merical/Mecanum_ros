#ifndef SC_HW_ROS_
#define SC_HW_ROS_

#include <vector>
#include <math.h>

#include <sc_hw/base_cmd_interface.h>
#include <sc_hw/base_state_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_publisher.h>
#include <sc_msgs/robot_state.h>
#include <sc_msgs/intf_state.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>

#include <controller_manager/controller_manager.h>
// for ros headers
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <tf/transform_datatypes.h>

// for sc link and transport
#include <sc_hw/transport.h>
#include <sc_hw/transport_serial.h>
#include <sc_link.h>
#include <sc_hw/sc_hw.h>

using std::cout;
using std::endl;

namespace smartcar_hw {

class SC_HW_ros : public  hardware_interface::RobotHW{

public:
    SC_HW_ros(ros::NodeHandle &nh, std::string url, std::string config_addr);

    double getFreq()const
    {
        return controller_freq_;
    }

    void mainloop();
    void expectIntfGrab(const sc_msgs::intf_state& state)
    {
        intf_state_ros_sub.robot_intf = state.robot_intf;
        intf_state_ros_sub.robot_mode = state.robot_mode;
        ROS_INFO("LCH: INTF Mode Changed!!!");
    }

private:
    //communication with embeded system
    SC_HW sc_hw_;
    ros::NodeHandle nh_;
    ros::CallbackQueue queue_;
    // publish the robot state for diagnose system
    ros::Publisher robot_state_publisher_;
    ros::Publisher imu_data_publisher_;
    ros::Publisher sonar_front_1_, sonar_front_5_, sonar_right_2_, sonar_right_6_,
                   sonar_back_3_, sonar_back_7_, sonar_left_4_, sonar_left_8_;
    ros::Subscriber robot_intf_subscriber_;
    ros::ServiceServer getparam_srv_;
    ros::ServiceServer setparam_srv_;

    //parameter list
    std::string base_mode_;
    bool with_arm_;
    double controller_freq_;

    //hardware resource
    sc_msgs::robot_state robot_state;
    sc_msgs::intf_state intf_state_robot, intf_state_ros_sub;
    sensor_msgs::Imu imu_data;

    tf::Quaternion imu_angular_quat;

    std::vector<double> wheel_pos_, wheel_vel_, wheel_eff_, wheel_cmd_;
    std::vector<double> arm_pos_  , arm_vel_  , arm_eff_, arm_cmd_;
    double x_, y_, theta_, x_cmd_, y_cmd_, theta_cmd_;
    double tran_x, tran_y, rot_theta;
    double yaw_imu_init_raw, pitch_imu_init_raw, roll_imu_init_raw;
    double yaw_imu_current_raw, pitch_imu_current_raw, roll_imu_current_raw;
    double yaw_imu, pitch_imu, roll_imu;
    double last_x_processed, last_y_processed, last_theta_processed, delta_x, delta_y, delta_theta;
    double x_processed, y_processed, theta_processed;
    double distance, distance_corrected, x_raw, y_raw, theta_raw, sin_alpha, cos_alpha;
    double x_vel_, y_vel_, theta_vel_;
    double x_vel_top = 0.96, y_vel_top = 0.96, theta_vel_top = 86;
    double odom_linear_scale_correction, odom_angle_scale_correction;

    double head_servo1_pos_, head_servo1_vel_, head_servo1_eff_;
    double head_servo2_pos_, head_servo2_vel_, head_servo2_eff_;
    double head_servo1_cmd_, head_servo2_cmd_;

    int plate_type_, imu_num_, track_num_, ultra_num_, arm_num_, head_num_, serv_num_;

    hardware_interface::JointStateInterface jnt_state_interface_;
    hardware_interface::PositionJointInterface servo_pos_interface_;
    hardware_interface::VelocityJointInterface base_vel_interface_;

    hardware_interface::BaseStateInterface base_state_interface_;
    hardware_interface::BaseVelocityInterface base_velocity_interface_;

    inline void writeBufferUpdate()
    {
        sc_hw_.getRobotAbstract()->expect_motor_speed.servo1 = wheel_cmd_[0];
        sc_hw_.getRobotAbstract()->expect_motor_speed.servo2 = wheel_cmd_[1];
        sc_hw_.getRobotAbstract()->expect_motor_speed.servo3 = wheel_cmd_[2];
        sc_hw_.getRobotAbstract()->expect_motor_speed.servo4 = wheel_cmd_[3];

        sc_hw_.getRobotAbstract()->expect_robot_speed.x = x_cmd_*100/x_vel_top;
//        sc_hw_.getRobotAbstract()->expect_robot_speed.x = 20;
        sc_hw_.getRobotAbstract()->expect_robot_speed.y = y_cmd_*100/y_vel_top;
        sc_hw_.getRobotAbstract()->expect_robot_speed.z = theta_cmd_/M_PI*180/theta_vel_top*100;

        sc_hw_.getRobotAbstract()->expect_intf_mode.intf = intf_state_ros_sub.robot_intf;
        sc_hw_.getRobotAbstract()->expect_intf_mode.mode = intf_state_ros_sub.robot_mode;

        if (with_arm_)
        {
            sc_hw_.getRobotAbstract()->expect_arm_state.servo1 = arm_cmd_[0];
            sc_hw_.getRobotAbstract()->expect_arm_state.servo2 = arm_cmd_[1];
            sc_hw_.getRobotAbstract()->expect_arm_state.servo3 = arm_cmd_[2];
            sc_hw_.getRobotAbstract()->expect_arm_state.servo4 = arm_cmd_[3];
            sc_hw_.getRobotAbstract()->expect_arm_state.servo5 = arm_cmd_[4];
            sc_hw_.getRobotAbstract()->expect_arm_state.servo6 = arm_cmd_[5];
        }
        // the servo num is different
        sc_hw_.getRobotAbstract()->expect_head_state.pitch  = head_servo1_cmd_;
        sc_hw_.getRobotAbstract()->expect_head_state.yaw  = head_servo2_cmd_;
    }

    inline void readBufferUpdateFirst()
    {
        plate_type_ = sc_hw_.getRobotAbstract()->module_config.plate_type;
        imu_num_    = sc_hw_.getRobotAbstract()->module_config.imu_num;
        track_num_  = sc_hw_.getRobotAbstract()->module_config.track_num;
        ultra_num_  = sc_hw_.getRobotAbstract()->module_config.ultra_num;
        arm_num_    = sc_hw_.getRobotAbstract()->module_config.arm_num;
        head_num_   = sc_hw_.getRobotAbstract()->module_config.head_num;
        serv_num_   = sc_hw_.getRobotAbstract()->module_config.serv_num;

        x_vel_      = sc_hw_.getRobotAbstract()->measure_robot_speed.y;
        y_vel_      = sc_hw_.getRobotAbstract()->measure_robot_speed.x;
        theta_vel_  =  sc_hw_.getRobotAbstract()->measure_robot_speed.z/180*M_PI;

        x_raw     = sc_hw_.getRobotAbstract()->measure_global_coordinate.x;
        y_raw     = sc_hw_.getRobotAbstract()->measure_global_coordinate.y;
        theta_raw = sc_hw_.getRobotAbstract()->measure_global_coordinate.z/180*M_PI;

//        last_x_raw      = x_raw;  // m
//        last_y_raw      = y_raw;
//        last_theta_raw  = theta_raw;

        tran_x = x_raw;
        tran_y = y_raw;
        rot_theta = theta_raw;

        last_x_processed = 0;
        last_y_processed = 0;
        last_theta_processed = 0;

        x_ = 0.;
        y_ = 0.;
        theta_ = 0.;

        yaw_imu_init_raw     = sc_hw_.getRobotAbstract()->magnetic_fusion.yaw/180*M_PI;
        pitch_imu_init_raw   = sc_hw_.getRobotAbstract()->magnetic_fusion.pitch/180*M_PI;
        roll_imu_init_raw    = sc_hw_.getRobotAbstract()->magnetic_fusion.roll/180*M_PI;

        if (with_arm_)
        {
            arm_pos_[0] = sc_hw_.getRobotAbstract()->measure_arm_state.servo1;
            arm_pos_[1] = sc_hw_.getRobotAbstract()->measure_arm_state.servo2;
            arm_pos_[2] = sc_hw_.getRobotAbstract()->measure_arm_state.servo3;
            arm_pos_[3] = sc_hw_.getRobotAbstract()->measure_arm_state.servo4;
            arm_pos_[4] = sc_hw_.getRobotAbstract()->measure_arm_state.servo5;
            arm_pos_[5] = sc_hw_.getRobotAbstract()->measure_arm_state.servo6;
        }

        wheel_vel_[0] = sc_hw_.getRobotAbstract()->measure_motor_speed.servo1;
        wheel_vel_[1] = sc_hw_.getRobotAbstract()->measure_motor_speed.servo2;
        wheel_vel_[2] = sc_hw_.getRobotAbstract()->measure_motor_speed.servo3;

        head_servo1_pos_ = sc_hw_.getRobotAbstract()->measure_head_state.pitch ;
        head_servo1_vel_ = 0 ;
        head_servo1_eff_ = 0 ;

        head_servo2_pos_ = sc_hw_.getRobotAbstract()->measure_head_state.yaw ;
        head_servo2_vel_ = 0 ;
        head_servo2_eff_ = 0 ;

    }
        inline void readBufferUpdate()
    {
        x_vel_      = sc_hw_.getRobotAbstract()->measure_robot_speed.y;
        y_vel_      = sc_hw_.getRobotAbstract()->measure_robot_speed.x;
        theta_vel_  =  sc_hw_.getRobotAbstract()->measure_robot_speed.z/180*M_PI;

//        theta_ = sc_hw_.getRobotAbstract()->measure_global_coordinate.z/180*M_PI;
        theta_raw  = sc_hw_.getRobotAbstract()->measure_global_coordinate.z/180*M_PI;
        x_raw  = sc_hw_.getRobotAbstract()->measure_global_coordinate.x;
        y_raw  = sc_hw_.getRobotAbstract()->measure_global_coordinate.y;

        x_processed = cos(rot_theta) * (x_raw - tran_x) + sin(rot_theta) * (y_raw - tran_y);
        y_processed = -sin(rot_theta) * (x_raw - tran_x) + cos(rot_theta) * (y_raw - tran_y);
        theta_processed = theta_raw - rot_theta;

        delta_theta  = theta_processed - last_theta_processed;
        delta_x = x_processed - last_x_processed;
        delta_y = y_processed - last_y_processed;

        theta_ = theta_ + delta_theta * odom_angle_scale_correction;
        x_ = x_ + delta_x * odom_linear_scale_correction;
        y_ = y_ + delta_y * odom_linear_scale_correction;

        last_x_processed = x_processed;
        last_y_processed = y_processed;
        last_theta_processed  = theta_processed;

//        x_     = sc_hw_.getRobotAbstract()->measure_global_coordinate.x;
//        y_     = sc_hw_.getRobotAbstract()->measure_global_coordinate.y;

/*  compute distance
        x_raw     = sc_hw_.getRobotAbstract()->measure_global_coordinate.x;
        y_raw     = sc_hw_.getRobotAbstract()->measure_global_coordinate.y;

        delta_x = x_raw - last_x_raw;
        delta_y = y_raw - last_y_raw;

        distance = sqrt(delta_x*delta_x + delta_y*delta_y);
        distance_corrected = distance * odom_linear_scale_correction;

        x_ = x_ + (distance_corrected * delta_y / distance);
        y_ = y_ + (distance_corrected * delta_y / distance);

        cout << "LCH: x_: " << x_ << " y_: " << y_ << " theta_:" << theta_ << " delta_x:" << delta_x << " delta_y:" << delta_y << " delta_theta:" << delta_theta << " distance:" << distance << endl;
        cout << "LCH: x_raw: " << x_raw << "y_raw: " << y_raw << "theta_raw: " << theta_raw << endl;
        cout << "LCH: last_x_raw: " << last_x_raw << " last_y_raw: " << last_y_raw << "last_theta_raw: " << last_theta_raw << "\r\n"<< endl;

        last_x_raw = x_raw;
        last_y_raw = y_raw;
        last_theta_raw = theta_raw;
*/
        wheel_pos_[0] = sc_hw_.getRobotAbstract()->measure_motor_mileage.servo1;
        wheel_pos_[1] = sc_hw_.getRobotAbstract()->measure_motor_mileage.servo2;
        wheel_pos_[2] = sc_hw_.getRobotAbstract()->measure_motor_mileage.servo3;

        robot_state.battery_voltage = sc_hw_.getRobotAbstract()->system_info.battery_voltage;
        robot_state.cpu_temperature = sc_hw_.getRobotAbstract()->system_info.cpu_temperature;
        robot_state.cpu_usage = sc_hw_.getRobotAbstract()->system_info.cpu_usage;
        robot_state.system_time = sc_hw_.getRobotAbstract()->system_info.system_time;

        intf_state_robot.robot_intf = sc_hw_.getRobotAbstract()->measure_intf_mode.intf;
        intf_state_robot.robot_mode = sc_hw_.getRobotAbstract()->measure_intf_mode.mode;

        /*****************************************************imu_quat********************************************/
        yaw_imu_current_raw     = sc_hw_.getRobotAbstract()->magnetic_fusion.yaw/180*M_PI;
        pitch_imu_current_raw   = sc_hw_.getRobotAbstract()->magnetic_fusion.pitch/180*M_PI;
        roll_imu_current_raw    = sc_hw_.getRobotAbstract()->magnetic_fusion.roll/180*M_PI;

        yaw_imu = yaw_imu_current_raw - yaw_imu_init_raw;
        pitch_imu = pitch_imu_current_raw - pitch_imu_init_raw;
        roll_imu = roll_imu_current_raw - roll_imu_init_raw;

        if (yaw_imu < 0) yaw_imu += M_PI;       else yaw_imu -= M_PI;       // normalize angle
        if (pitch_imu < 0) pitch_imu += M_PI;   else pitch_imu -= M_PI;
        if (roll_imu < 0) roll_imu += M_PI;     else roll_imu -= M_PI;

        imu_angular_quat.setEuler(yaw_imu, pitch_imu, roll_imu);
        imu_data.header.stamp = ros::Time::now();
        imu_data.header.frame_id = "base_link";
        imu_data.orientation.x = imu_angular_quat.x();
        imu_data.orientation.y = imu_angular_quat.y();
        imu_data.orientation.z = imu_angular_quat.z();
        imu_data.orientation.w = imu_angular_quat.w();
        imu_data.linear_acceleration.x = sc_hw_.getRobotAbstract()->magnetic_fusion.linear_acc_x;
        imu_data.linear_acceleration.y = sc_hw_.getRobotAbstract()->magnetic_fusion.linear_acc_y;
        imu_data.linear_acceleration.z = sc_hw_.getRobotAbstract()->magnetic_fusion.linear_acc_z;
        imu_data.angular_velocity.x = sc_hw_.getRobotAbstract()->magnetic_fusion.angular_vel_x;
        imu_data.angular_velocity.y = sc_hw_.getRobotAbstract()->magnetic_fusion.angular_vel_y;
        imu_data.angular_velocity.z = sc_hw_.getRobotAbstract()->magnetic_fusion.angular_vel_z;


        if (with_arm_)
        {
            arm_pos_[0] = sc_hw_.getRobotAbstract()->measure_arm_state.servo1;
            arm_pos_[1] = sc_hw_.getRobotAbstract()->measure_arm_state.servo2;
            arm_pos_[2] = sc_hw_.getRobotAbstract()->measure_arm_state.servo3;
            arm_pos_[3] = sc_hw_.getRobotAbstract()->measure_arm_state.servo4;
            arm_pos_[4] = sc_hw_.getRobotAbstract()->measure_arm_state.servo5;
            arm_pos_[5] = sc_hw_.getRobotAbstract()->measure_arm_state.servo6;
        }

        wheel_vel_[0] = sc_hw_.getRobotAbstract()->measure_motor_speed.servo1;
        wheel_vel_[1] = sc_hw_.getRobotAbstract()->measure_motor_speed.servo2;
        wheel_vel_[2] = sc_hw_.getRobotAbstract()->measure_motor_speed.servo3;

        head_servo1_pos_ = sc_hw_.getRobotAbstract()->measure_head_state.pitch ;
        head_servo1_vel_ = 0 ;
        head_servo1_eff_ = 0 ;

        head_servo2_pos_ = sc_hw_.getRobotAbstract()->measure_head_state.yaw ;
        head_servo2_vel_ = 0 ;
        head_servo2_eff_ = 0 ;

    }
};

}


#endif
