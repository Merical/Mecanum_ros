#ifndef SC_LINK_PLATFORM_H
#define SC_LINK_PLATFORM_H

#define SC_LINK_NODE_MODEL  1    //1:master(PC)  0 :slave(MCU)

#if SC_LINK_NODE_MODEL==0

#include "board.h"

inline unsigned char SCLinkSendBuffer(unsigned char port_num , unsigned char* buffer, unsigned short int size)
{
    if(port_num == 1){
        while(size--) SC_USART_Put_Char(1 , *buffer++);
        return 1;
    }
    else if(port_num == 2){
        while(size--) SC_USART_Put_Char(2 , *buffer++);
        return 1;
    }
    else if(port_num == 3){
        while(size--) SC_USART_Put_Char(3 , *buffer++);
        return 1;
    }
    else if(port_num == 4){
        while(size--) SC_USART_Put_Char(4 , *buffer++);
        return 1;
    }
    return 0;
}

#endif

#endif // SC_LINK_PLATFORM_H

