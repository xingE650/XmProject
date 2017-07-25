#ifndef HF_LINK_PORT_H
#define HF_LINK_PORT_H

unsigned char hFLinkSendBuffer(unsigned char port_num , unsigned char* buffer, unsigned short int size);
void get_master(void);
void send_signal(unsigned char signal);

#endif // HF_LINK_PORT_H

