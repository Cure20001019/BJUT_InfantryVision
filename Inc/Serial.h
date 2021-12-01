/**
  ***********************************************************************************
  * @file   :Serial.h
  * @author :Tinker.Jia
  * @version:1.0
  * @date   :2019.6.28
  * @brief  :Init the UART1 to connetion the STM32.
  ***********************************************************************************
  * @attention
  * None
  ***********************************************************************************
  */
#include "Header.h"
#ifndef SERIAL_H
#define SERIAL_H

#include <sys/types.h>
#include <unistd.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/ioctl.h>


#define TRUE 1
#define FALSE 0

#define BAUDRATE        115200
#define UART_DEVICE     "/dev/ttyTHS2"

#define ECHOFLAGS (ECHO | ECHOE | ECHOK | ECHONL)


typedef struct
{
    int Header;
    int  Mode;
    int  AllowSend;
    float ArmorAngle;
    float YawAngle;
    float PitchAngle;
    float Distance;
    float CenterYawOffset;
    float CenterDistance;
    float CenterYawAngle;
    float YawOffset[8];
    float YawSpeed[9];
    float PitchOffset;
}USART_Data_t;

void LinkThread(void);
void GetMode(int &fd,USART_Data_t &data);
void ArmorTrans2STM32(int &fd,USART_Data_t &data);


void Serial_Init(int &fd);
void set_speed(int fd, int speed);
int set_disp_mode(int fd,int option);
int set_Parity(int fd,int databits,int stopbits,int parity);


unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8);
unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC);
uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
void Append_CRC16_Check_Sum(uint8_t * pchMessage,uint32_t dwLength);

extern USART_Data_t USART_Data;

#endif
