#ifndef SC_HW_H_
#define SC_HW_H_

#include <fstream>
#include <sc_hw/transport_serial.h>
#include <sc_link.h>
#include <cstdlib>

namespace smartcar_hw {

class SC_HW{
public:
    SC_HW(std::string url, std::string config_addr);

    bool updateCommand(const Command &command, int count);

    void updateRobot();

    inline RobotAbstract* getRobotAbstract()
    {
        return &my_robot_;
    }

    inline boost::shared_ptr<boost::asio::io_service> getIOinstace()
    {
        return port_->getIOinstace();
    }

    bool reconfig()
    {

    }

    inline bool initialize_ok () const
    {
        return initialize_ok_;
    }

    inline void checkHandshake()
    {
        if (sclink_->getReceiveRenewFlag(SHAKING_HANDS)==1)
        {
            sendCommand(SHAKING_HANDS);
            std::cout<<"send shake hands"<<std::endl;
        }
    }

private:
    boost::shared_ptr<Transport> port_;
    boost::shared_ptr<SCLink> sclink_;
    boost::shared_ptr<boost::asio::deadline_timer> timer_;

    //for reading config file
    std::fstream file_;
    bool initialize_ok_;
    //for updating data
    int sclink_command_set_[LAST_COMMAND_FLAG];
    int sclink_freq_[LAST_COMMAND_FLAG];
    int sclink_count_[LAST_COMMAND_FLAG];
    int sclink_command_set_current_[LAST_COMMAND_FLAG];

    int time_out_;
    bool time_out_flag_;
    boost::mutex wait_mutex_;
    bool ack_ready_;
    void timeoutHandler(const boost::system::error_code &ec);

    inline uint8_t checkUpdate(const Command command_state)
    {
        if (sclink_command_set_current_[command_state] & sclink_->getReceiveRenewFlag(command_state))
        {
            return 1;
        }
        if (sclink_command_set_current_[command_state] == 0 ) return 1;
        return 0;
    }

    inline void sendCommand(const Command command_state)
    {
//        std::cout<<"LCH: send message  "<<command_state <<std::endl;
        sclink_->masterSendCommand(command_state);
        Buffer data(sclink_->getSerializedData(), sclink_->getSerializedLength() + sclink_->getSerializedData());
        port_->writeBuffer(data);
    }

    // a single object for robot
    RobotAbstract my_robot_;
};

}


#endif /* SC_HW_H_ */
