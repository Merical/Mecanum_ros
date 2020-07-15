#ifndef SC_LINK_H
#define SC_LINK_H

#include "robot_abstract.h"
#include "sc_link_state_machine.h"

//comand type
enum Command{
    SHAKING_HANDS,
    READ_SYSTEM_INFO,
    SET_MOTOR_PARAMETERS,
    SAVE_MOTOR_PARAMETERS,
    SET_CHASSIS_PARAMETERS,
    SAVE_CHASSIS_PARAMETERS,
    SET_HEAD_PARAMETERS,
    SAVE_HEAD_PARAMETERS,
    SET_ARM_PARAMETERS,
    SAVE_ARM_PARAMETERS,
    SET_GLOBAL_SPEED,
    READ_GLOBAL_SPEED,
    SET_ROBOT_SPEED, // 12
    READ_ROBOT_SPEED, // 13
    SET_MOTOR_SPEED,
    READ_MOTOR_SPEED,
    READ_MOTOR_MILEAGE,
    READ_GLOBAL_COORDINATE, // 17
    READ_ROBOT_COORDINATE,
    CLEAR_COORDINATE_DATA,
    SET_HEAD_STATE,
    READ_HEAD_STATE,
    SET_ARM_STATE,
    READ_ARM_STATE,
    READ_IMU_BASE_DATA, // 24
    READ_IMU_FUSION_DATA, // 25
    READ_GPS_DATA,
    READ_INTF_MODE, // 27
    SET_INTF_MODE, // 28
    READ_MODULE_CONFIG, // 29
    READ_SONAR_DATA, // 30
    SET_SONAR_STATE,
    CLEAR_ODOMETER_DATA, // 32
    LAST_COMMAND_FLAG,
}; // 32

class SCLink : public StateMachine
{
public:
    SCLink(RobotAbstract* robot_  , unsigned char my_id_= 0x11 , unsigned char friend_id_= 0x01 , unsigned char port_num_ = 1) :
        StateMachine(my_id_ , friend_id_ , port_num_)
    {
        sc_link_node_model = SC_LINK_NODE_MODEL ;
        //enable sclink ack , generally, master disable and slave enable
        //and slave also can disable to reduce communication burden
        sc_link_ack_en = 0;
        if(sc_link_node_model == 0) sc_link_ack_en = 1;

        robot=robot_;
        shaking_hands_state = 0;
        analysis_package_count  = 0;
        command_state_ = SHAKING_HANDS;
    }

public:  
    //only for master
    //the master can use masterSendCommand function to send data to slave
    //like SET_GLOBAL_SPEED , READ_ROBOT_SYSTEM_INFO, READ_ROBOT_SPEED...
    unsigned char masterSendCommand(const Command command_state);
    inline unsigned char getReceiveRenewFlag(const Command command_state) const
    {
        return receive_package_renew[command_state];
    }

public: 
    //only for slave
    //command updata flag , the robot need to traverse These flag to decide update his own behavior
    unsigned char receive_package_renew[LAST_COMMAND_FLAG];

public:  
    //common
    unsigned char byteAnalysisCall(const unsigned char rx_byte);
    inline void enable_ack(void){if(sc_link_ack_en != 1) sc_link_ack_en=1;}
    inline void disable_ack(void){sc_link_ack_en=0;}

private:
    unsigned char sc_link_node_model;      // 0 slave , 1 master
    unsigned char sc_link_ack_en;                //enable sclink ack
    unsigned char shaking_hands_state;     //1 Success   0 Failed
    float analysis_package_count;
    RobotAbstract* robot;      //robot abstract pointer to sclink
    Command    command_state_;

    unsigned char packageAnalysis(void);
    unsigned char readCommandAnalysis(const Command command_state , unsigned char* p , const unsigned short int len);
    unsigned char setCommandAnalysis(const Command command_state , unsigned char* p , const unsigned short int len);
    void sendStruct(const Command command_state , unsigned char* p , const unsigned short int len);
};

#endif  // #ifndef SC_LINK_H

