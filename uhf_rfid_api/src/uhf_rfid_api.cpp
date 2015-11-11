/*********************************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, Tasbot, Inc.
 *  All rights reserved.
 *
 *FileName:  uhf_inventory_single_tag.cpp
 *Author:  feixiao5566 feixiao5566@126.com
 *Version:  0.3
 *Date:  2015-06-01
 *Description:  //用于主要说明此程序文件完成的主要功能
                //与其他模块或函数的接口、输出值、取值范围、
                //含义及参数间的控制、顺序、独立及依赖关系
 *Others:  //其他内容说明
 *Function List:  //主要函数列表，每条记录应包含函数名及功能简要说明
     1.…………
     2.…………
 *History:  //修改历史记录列表，每条修改记录应包含修改日期、修改者及修改内容简介
     1.Date:2015-05-06
       Author:Tian Bo
       Modification:Jiuray RFID reader api
     2.Date:2015-5-21
       Author:feixiao
       Modification:将函数功能模块化，读出EPC发布订阅,读取RSSI
     3.Date:2015-5-25
       Author:feixiao
       Modification:修改内存泄露问题
**********************************************************************************/

#include "uhf_rfid_api.h"
#include <stdio.h>
#include <iostream>
#include <boost/property_tree/ptree.hpp>

const char UhfRfidReader::CMD_SINGLE[7] = {0xAA,0x00,0x22,0x00,0x00,0x22,0x8e};//single
UhfRfidReader::UhfRfidReader()
{
  serial_port[0] = "/dev/rfid";
  ret_select = 0;
  bytes = 0;
  write_num = 0;
  read_num = 0;
}

UhfRfidReader::~UhfRfidReader()
{
  close(fd);
}

void UhfRfidReader::SerialConnect()
{
  fd = serial_open(0);//open serial interface
  if(-1 == fd)
  {
    perror("UART open error!Please check.\n");
    exit (-1);
  }
  serial_config(fd, B115200, SERIAL_8N1);
  FD_ZERO(&set_input);//将指定的文件描述符集清空，在对文件描述符集合进行设置前，必须对其进行初始化，如果不清空，由于在系统分配内存空间后，通常并不作清空处理，所以结果是不可知的。
  FD_SET(fd, &set_input);//用于在文件描述符集合中增加一个新的文件描述符。
}

int UhfRfidReader::ReadRfid()
{
  int i = 0, n = 0;
  int len = 0;
  unsigned char* p = NULL;
  unsigned char chepc = '\0';
  tcflush(fd, TCIOFLUSH);
/*write command to rfid*/
  write_num = write(fd, CMD_SINGLE, 7);
  timeout.tv_sec = 1;
  timeout.tv_usec = 0;
  ret_select = select(fd + 1, &set_input, NULL, NULL, &timeout);
  if(ret_select < 0)
    perror("select failed");
  else
    if(ret_select == 0)
      printf("timeout\n");
    else
    {
      if(FD_ISSET(fd, &set_input))
      {
        bytes=0;
        usleep(10000);
        read_num = read(fd, buff, 24);
        if(24 > read_num)
        {
          return -1;
        }

        for(i = 0; i<24; i++)
        {
          mybuff[i] = buff[i];
        }
        mybuff[24] = '\0';
        for(i = 0, len = 8; i<12; i++, len++)
        {
          itagid[i] = buff[len];
        }
        itagid[12] = '\0';
        n = resolution_epc(buff);
        if(0 != n)
        {
          perror("resolution_epc error\n");
          return -1;
        }
      }//end of if
    }//end of else
  return 0;
}//end of function UhfRfidReader.

/*open the rfid*/
int UhfRfidReader::serial_open(int port)
{
  int fd = open(serial_port[port], O_RDWR|O_NONBLOCK);
  if (-1 == fd)
  {
     perror("Can't Open Serial Port");
     exit (-1);
  }
  else
  {
     fcntl(fd, F_SETFL, 0);
  }
  return fd;
}
/*serial format set*/
void UhfRfidReader::serial_format_set(int fd, serial_format format)
{
  int status = 0;
  struct termios options;
  if(tcgetattr( fd,&options) != 0)
  {
    perror("serial fomat abnormal");
    exit (-1);
  }
  options.c_cflag &= ~CSIZE;
  switch (format)
  {
    case SERIAL_8N1:
      options.c_cflag &= ~PARENB;
      options.c_cflag &= ~CSTOPB;
      options.c_cflag &= ~CSIZE;
      options.c_cflag |= CS8;
      options.c_iflag &= ~(INPCK | ISTRIP);
      break;
    case SERIAL_7E1:
      options.c_cflag |= PARENB;
      options.c_cflag &= ~PARODD;
      options.c_cflag &= ~CSTOPB;
      options.c_cflag &= ~CSIZE;
      options.c_cflag |= CS7;
      options.c_iflag |= (INPCK | ISTRIP);

      break;
    case SERIAL_7O1:
      options.c_cflag |= PARENB;
      options.c_cflag |= PARODD;
      options.c_cflag &= ~CSTOPB;
      options.c_cflag &= ~CSIZE;
      options.c_cflag |= CS7;
      options.c_iflag |= (INPCK | ISTRIP);

      break;
    case SERIAL_7S1:
      options.c_cflag &= ~PARENB;
      options.c_cflag &= ~CSTOPB;
      options.c_cflag &= ~CSIZE;
      options.c_cflag |= CS8;
      options.c_iflag &= ~INPCK;
      options.c_iflag |= ISTRIP;
      break;
    default:
      perror("serial format abnormal");
      exit (-1);
  }
  tcflush(fd, TCIOFLUSH);
  status = tcsetattr(fd, TCSANOW, &options);
  if(status != 0)
  {
    perror("tcsetattr format abnormal");
    exit(-1);
  }
}
/*serial speed set*/
void UhfRfidReader::serial_speed_set(int fd, int baudrate)
{
  int status;
  struct termios options;
  tcgetattr(fd, &options);
  cfsetispeed(&options, baudrate);
  cfsetospeed(&options, baudrate);
  options.c_cflag |= (CLOCAL | CREAD);
  tcflush(fd, TCIOFLUSH);
  status=tcsetattr(fd, TCSANOW, &options);
  if(status != 0)
  {
    perror("tcsetattr speed abnormal");
    exit(-1);
  }
}



void UhfRfidReader::serial_etc_config(int fd)
{
  int status=0;
  struct termios options;
  if(tcgetattr(fd, &options)  !=  0)
  {
    perror("serial etc abnormal");
    exit (-1);
  }
  options.c_cflag &= ~CRTSCTS;
  options.c_iflag &= ~(IXON | IXOFF | IXANY);
  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  options.c_oflag &= ~OPOST;
  options.c_cc[VMIN]  = 1;
  options.c_cc[VTIME] = 15;
  tcflush(fd, TCIOFLUSH);
  status=tcsetattr(fd, TCSANOW, &options);
  if(status != 0)
  {
    perror("tcsetattr etc abnormal");
    exit(-1);
  }
}

void UhfRfidReader::serial_config(int fd, int baudrate, serial_format format)
{
  serial_speed_set(fd, baudrate);
  serial_format_set(fd, format);
  serial_etc_config(fd);
}
/*turn the Hexadecimal tag to a char[],checkout error*/
int UhfRfidReader::resolution_epc(unsigned char* buff)
{
	int i = 0;
  int n = 0;
	unsigned int checktmp = 0;
	unsigned char epcbuff[13] = {'\0'};
	unsigned char chepc = '\0';

  if(0xaa != buff[0])
	{
		perror("header error\n");
		return -1;
	}

	if(0x02 != buff[1])
	{
		perror("type error\n");
		return -1;
	}
	if(0x22 != buff[2])
	{
		perror("command error\n");
		return -1;
	}

	for(i = 1; i < 22; i++)
	{
		checktmp += buff[i];
	}
	checktmp = checktmp & 0x0ff;

  if(checktmp != buff[22])
	{
		perror("checksum error\n");
		return -1;
	}

	if(0x8e != buff[23])
	{
		perror("end error\n");
		return -1;
	}
	i = 0;
	for(n = 8; n <= 19; n++)
  {
    epcbuff[i] = buff[n];
    i++;
  }//end of for

	n = 0;
	for(i = 0; i<12; i++)
	{
		chepc = (epcbuff[i]/16);
		if(chepc < 10)
		{
			chepc += 48;
		}
		else
		{
			switch(chepc)
			{
				case 10: chepc = 'a';break;
				case 11: chepc = 'b';break;
				case 12: chepc = 'c';break;
				case 13: chepc = 'd';break;
				case 14: chepc = 'e';break;
				case 15: chepc = 'f';break;
				default: return 0;
			}
		}//if else end of else
		epcbuf24[n] = chepc;
		n++;
		chepc = (epcbuff[i] % 16);
		if(chepc < 10)
		{
			chepc += 48;
		}
		else
		{
			switch(chepc)
			{
				case 10: chepc = 'a';break;
				case 11: chepc = 'b';break;
				case 12: chepc = 'c';break;
				case 13: chepc = 'd';break;
				case 14: chepc = 'e';break;
				case 15: chepc = 'f';break;
			}
		}
		epcbuf24[n] = chepc;
		n++;
  }//end of for i
  epcbuf24[n] = '\0';


  return 0;
}



