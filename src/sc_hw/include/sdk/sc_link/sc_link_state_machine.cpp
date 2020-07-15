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
* mawenke       2015.10.1   V1.0           creat this file
*
* Description: This file defined hands_free_robot simple communications protocol
*              please read Hands Free Link Manua.doc for detail
***********************************************************************************************************************/

#include "sc_link_state_machine.h"

unsigned char StateMachine::receiveStates(const unsigned char rx_data)
{

    switch (receive_state_)
    {
    case WAITING_FF1:
        if (rx_data == 0xff)
        {
            receive_state_ = WAITING_FF2;
            receive_check_sum_ =0;
            receive_message_length_ = 0;
            byte_count_=0;
            receive_check_sum_ += rx_data;
//            printf("LCH: the rx_data is: %02x", rx_data);
        }
        break;

    case WAITING_FF2:
        if (rx_data == 0xff)
        {
            receive_state_ = SENDER_ID;
            receive_check_sum_ += rx_data;
//            printf("%02x", rx_data);
        }
        else
            receive_state_ = WAITING_FF1;
        break;

    case SENDER_ID:
        rx_message.sender_id = rx_data ;
        if (rx_message.sender_id == friend_id)  //id check
        {
            receive_check_sum_ += rx_data;
            receive_state_ = RECEIVER_ID;
//            printf("%02x", rx_data);
        }
        else
        {
#if SC_LINK_NODE_MODEL == 1
            printf("error , the sender_ID is not my friend \n");
#endif
            receive_state_ = WAITING_FF1;
        }
        break;

    case RECEIVER_ID:
        rx_message.receiver_id = rx_data ;
        if (rx_message.receiver_id == my_id)  //id check
        {
            receive_check_sum_ += rx_data;
            receive_state_ = RECEIVE_LEN_H;
//            printf("%02x", rx_data);
        }
        else
        {
#if SC_LINK_NODE_MODEL == 1
            printf("error , the reciver_ID is not my_ID \n");
#endif
            receive_state_ = WAITING_FF1;
        }
        break;

    case RECEIVE_LEN_H:
        receive_check_sum_ += rx_data;
        receive_message_length_ |= rx_data<<8;
        receive_state_ = RECEIVE_LEN_L;
//        printf("%02x", rx_data);
        break;

    case RECEIVE_LEN_L:
        receive_check_sum_ += rx_data;
//        printf("%02x", rx_data);
        receive_message_length_ |= rx_data;
        rx_message.length = receive_message_length_;
        receive_state_ = RECEIVE_PACKAGE;
        break;

    case RECEIVE_PACKAGE:
        receive_check_sum_ += rx_data;
//        printf("%02x", rx_data);
        rx_message.data[byte_count_++] = rx_data;
        if(byte_count_ >= receive_message_length_)
        {
            receive_state_ = RECEIVE_CHECK;
            receive_check_sum_=receive_check_sum_ % 255;
        }
        break;

    case RECEIVE_CHECK:
//        printf("%02x \n", rx_data);
//        printf("LCH: check_sum \n");
        if(rx_data == (unsigned char)receive_check_sum_)
        {
            receive_check_sum_=0;
            receive_state_ = WAITING_FF1;
#if SC_LINK_NODE_MODEL == 1
//            printf("receive a message \n");
#endif
            receive_message_count ++ ;
            return 1 ;
        }
        else
        {
#if SC_LINK_NODE_MODEL == 1
            printf("check sum error \n");
#endif
            receive_state_ = WAITING_FF1;
        }
        break;
    default:
        receive_state_ = WAITING_FF1;
    }

    return 0;
}

/***********************************************************************************************************************
* Function:    void StateMachine::sendMessage(void)
*
* Scope:
*
* Description:  send a message to sc_link node
*
* Arguments:
*
* Return:
*
* Cpu_Time:    stm32f4+fpu(1 us)
*
* History:
***********************************************************************************************************************/
void StateMachine::sendMessage(const SCMessage* tx_message_)
{

    unsigned int check_sum_=0;

    tx_buffer[0]=0xff;
    check_sum_ += 0xff;
//    printf("LCH: the tx buffer is %02x", tx_buffer[0]);

    tx_buffer[1]=0xff;
    check_sum_ += 0xff;
//    printf("%02x", tx_buffer[1]);

    tx_buffer[2]=tx_message_->sender_id;
    check_sum_ += tx_buffer[2];
//    printf("%02x", tx_buffer[2]);

    tx_buffer[3]=tx_message_->receiver_id;
    check_sum_ += tx_buffer[3];
//    printf("%02x", tx_buffer[3]);

    tx_buffer[4]=(unsigned char)( tx_message_->length >> 8);  //LEN_H
    check_sum_ += tx_buffer[4];
//    printf("%02x", tx_buffer[4]);

    tx_buffer[5]=(unsigned char)tx_message_->length;   //LEN_L
    check_sum_ += tx_buffer[5];
//    printf("%02x", tx_buffer[5]);

    unsigned short int tx_i  = 0;
    for(tx_i = 0 ;  tx_i < tx_message_->length ; tx_i++)   //package
    {
        tx_buffer[ 6 + tx_i] = tx_message_->data[ tx_i ];
        check_sum_ += tx_buffer[6+tx_i];
//        printf("%02x", tx_buffer[6+tx_i]);
    }

    check_sum_=check_sum_%255;
    tx_buffer[6+tx_i] = check_sum_;
//    printf("%02x \n", tx_buffer[6+tx_i]);

    tx_buffer_length = 6 + tx_message_->length + 1;

#if SC_LINK_NODE_MODEL==0
    SCLinkSendBuffer(port_num , tx_buffer , tx_buffer_length);
#endif

    send_message_count++;
}
