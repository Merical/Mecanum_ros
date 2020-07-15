/***********************************************************************************************************************
* Copyright (c) Hands Free Team. All rights reserved.
* FileName: sc_link.cpp
* Contact:  QQ Exchange Group -- 521037187
* Version:  V2.0
*
* LICENSING TERMS:
* The Hands Free is licensed generally under a permissive 3-clause BSD license.
* Contributions are requiredto be made under the same license.
*
* History:
* <author>      <time>      <version>      <desc>
* luke liao       2016.4.1   V1.0           creat this file
*
* Description: define the handfree pc software interface
***********************************************************************************************************************/


#include <sc_hw/sc_hw.h>

namespace smartcar_hw {

SC_HW::SC_HW(std::string url, std::string config_addr)
{
    std::string transport_method = url.substr(0, url.find("://"));
    if (transport_method == "serial")
    {
        port_ = boost::make_shared<TransportSerial>(url);
        time_out_ = 500;
        sclink_ = boost::make_shared<SCLink>(&my_robot_  , 0x01 , 0x11);
        timer_.reset(new boost::asio::deadline_timer(*(port_->getIOinstace()),
                                                   boost::posix_time::milliseconds(time_out_)));
    }else if (transport_method == "udp")
    {
    }else if (transport_method == "tcp")
    {
    }

    //process the config file
    file_.open(config_addr.c_str(), std::fstream::in);
    if (file_.is_open())
    {
        for (int i = 0; i < LAST_COMMAND_FLAG; i++)
        {
            std::string temp;
            file_ >> temp >> sclink_command_set_[i] >> sclink_freq_[i];
            std::cout<< temp << sclink_command_set_[i] << sclink_freq_[i]<<std::endl;
        }
        file_.close();
        initialize_ok_ = port_->initialize_ok();
    } else
    {
        std::cerr << "config file can't be opened, check your system" <<std::endl;
        initialize_ok_ = false;
    }
}

void SC_HW::timeoutHandler(const boost::system::error_code &ec)
{
    if (!ec)
    {
        std::cerr << "Time Out" <<std::endl;
        boost::mutex::scoped_lock lock(wait_mutex_);
        time_out_flag_ = true;
    }
}

// for QT client , not for ros update
void SC_HW::updateRobot()
{
    boost::asio::deadline_timer cicle_timer_(*(port_->getIOinstace()));

    memset(sclink_count_, 0 ,sizeof sclink_count_);
    int count = 0;
    while (true)
    {
        cicle_timer_.expires_from_now(boost::posix_time::millisec(10)); // 100 hz
        //std::cout<< "start a write" <<std::endl;
        //  check hand shake
        checkHandshake();
        //  uodate normal data
        memset(sclink_command_set_current_, 0, sizeof sclink_command_set_current_);
        for (int i = 1; i < LAST_COMMAND_FLAG; i++)
        {
            if (sclink_command_set_[i] != 0)
            {
                int cnt = count % 100;
                if (cnt %  (100 / sclink_freq_[i]) == 0)
                {
                    sendCommand((Command)i);
                    sclink_command_set_current_[i] = 1;
                }
            }
        }

        //std::cout<< "start a read" <<std::endl;
        Buffer data = port_->readBuffer();
        ack_ready_ = false;
        while (!ack_ready_)
        {
            for (int i = 0; i < data.size(); i++)
            {
                //std::cout<<"get byte   :"<< data[i]<< std::endl;
                if (sclink_->byteAnalysisCall(data[i]))
                {
                    // all receive package ack arrived
                    uint8_t temp = 1;
                    for (int i = 1; i < LAST_COMMAND_FLAG; i++)
                        temp = temp & checkUpdate((Command)i);
                    std::cout<< temp <<std::endl;
                    if (temp)
                    {
                        ack_ready_ = true;
                        std::cout<< "all package received" <<std::endl;
                    }
                }
            }
            if (cicle_timer_.expires_from_now().is_negative())
            {
                //std::cout<<"Timeout continue next circle"<<std::endl;
                break;
            }
            data = port_->readBuffer();
        }
        count++;
        cicle_timer_.wait();
    }
}

bool SC_HW::updateCommand(const Command &command, int count)
{
    boost::asio::deadline_timer cicle_timer_(*(port_->getIOinstace()));
    cicle_timer_.expires_from_now(boost::posix_time::millisec(time_out_));
    // update command set  data from embedded system
    if (sclink_command_set_[command] != 0)
    {
        int cnt = count % 100;
        if (cnt %  (100 / sclink_freq_[command]) == 0)
        {
            sendCommand(command);
        } else
        {
            // skip this package
            return false;
        }
    }
    Buffer data = port_->readBuffer();
    ack_ready_ = false;
//    std::cout << "LCH: the pc received ";
    while (!ack_ready_)
    {
//        for (int i = 0; i < data.size(); i++)
//        {   std::cout << std::setfill('0') << std::setw(2);
//            if (i == 0)
//            {
//                std::cout << "LCH: the received buffer is " << std::hex << (unsigned int)(unsigned char)data.at(i);
//                continue;
//            }
//            if (i == data.size()-1)
//            {
//                std::cout << std::hex << (unsigned int)(unsigned char)data.at(i) << std::endl;
//                continue;
//            }
//            std::cout << std::hex << (unsigned int)(unsigned char)data.at(i);
//        }
        for (int i = 0; i < data.size(); i++)
        {
            if (sclink_->byteAnalysisCall(data[i]))
            {
                // one package ack arrived
                ack_ready_ = true;
            }
        }
        data = port_->readBuffer();
        if (cicle_timer_.expires_from_now().is_negative())
        {
            std::cerr<<"Timeout continue skip this package " << command <<std::endl;
            return false;
        }
    }
    return true;
}
}


