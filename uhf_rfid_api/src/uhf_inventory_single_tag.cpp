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
     2.Date:2015-05-11
       Author:feixiao
       Modification:发布话题数据，建立数组，临时存储读到的数据。
                    统计扫描到的次数发布标签值，次数，接收信号强度RSSI
     3.Date:2015-07-15
       Author:feixiao5566
       Modification:增加boost资产树读写修改配置文件功能
     4.Date:2015-07-22
       Author:feixiao5566
       Modification:删除配置文件写功能，删除判断标签信号
                    增加自定义消息包，加给数据header
                    输出数据包
     5.Date:2015-08-06
       Author:feixiao5566
       Modification:配合Ratslam修改了数据包
     6.Date:2015-11_11
       Author:feixiao5566
       Modification:修正了一下rfid,下一步把文件中字符串类型的处理用C++
**********************************************************************************/
# include "ros/ros.h"
#include "ros/time.h"
#include "uhf_rfid_api.h"

#include <boost/property_tree/ptree.hpp>  
#include <boost/property_tree/ini_parser.hpp>

#include "std_msgs/String.h"
#include <sstream>
#include <string>
#include "uhf_rfid_api/UhfRfid.h"
#include <unistd.h>
#include <time.h> 

#include<iostream>
#include<math.h>
#include<vector>
#include<fcntl.h>
#include<fstream>
using namespace std;

int g_id = 0;
unsigned  char g_testarry[25][28] = {'\0'};// I tried everything else, but I really need this global variable

vector<long int> time_nu;
vector<int> rssi_nu;

int mystrcmp(const char*str1, const char*str2)//str1 is array，str2 is epc
{
  int j = 0;
  while(*str1 == *str2)
  {
    if(j==24)
    {
        return 0;
    }
    str1++;
    str2++;
    j++;
  }
  return *str1 - *str2;
}

int main(int argc, char **argv)
{
  ROS_INFO_STREAM(argv[0] << " - iRabbit_RFID Copyright (C) 2015 Tarsbot");
  ROS_INFO_STREAM("Distributed under the GNU GPL v3, see the included license file.");
  boost::property_tree::ptree m_pt, tag_setting;
  time_t t;  
  long int number = 0;
  int i = 0, k = 0, flag = 0, f = 0, bagnum = 0;
  unsigned int count = 0;
  int tagname = 0;
  int isec = 0;
  char dou = ',', fen = ';';
  unsigned char *test = NULL;
   
//  ofstream fp("./src/iRabbit/uhf_rfid_api/config/template_config.ini",ofstream::out|ofstream::app);
  ofstream fp(argv[1],ofstream::out|ofstream::app);
  if(fp == NULL)
  {
    perror("file open error");
    return -1;
  }

  if (!ros::isInitialized())
  {
    ros::init(argc, argv, "uhf_inventory_single_tag");
  }
  ros::NodeHandle n;

  UhfRfidReader uhf_reader;
  uhf_reader.SerialConnect();
  uhf_rfid_api::UhfRfid msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "1";// 1 global
  msg.header.seq = 0;

  ros::Publisher rfid_pub = n.advertise<uhf_rfid_api::UhfRfid>("messageepc", 25);

  ros::Rate rate(100);// sleep 0.01s

  while (n.ok())
  {
    f = uhf_reader.ReadRfid();//return 0 means all correct
    if(!f)//uhf_reader.ReadRfid successed
    {
      for(count = 0; count < 25; count++)
      {
        if(flag == 0)//unequal
        {
          if(mystrcmp((const char*)g_testarry[count], (const char*)uhf_reader.epcbuf24)==0)
          {
            flag = 1;
            break;
          }
        }
      }//end of for
      if(!flag)
      {
        if(k > 24)//assert the max num is 24 in a moment
          k = 0;
        for(i = 0; i < 24; i++)
        {
          g_testarry[k][i] = uhf_reader.epcbuf24[i];
        }
        g_testarry[k][25] = uhf_reader.mybuff[5];//RSSI

        g_testarry[k][26] = 1;//count
        g_testarry[k][27] = '\0';//end character
        printf("  RSSI: %d", g_testarry[k][25]-256);
        printf("  count: %d\n", g_testarry[k][26]);
        bagnum = k;
        k++;
      }//end of !flag
      else
      {

        g_testarry[count][26]++;
        g_testarry[count][25] = uhf_reader.mybuff[5];
        g_testarry[k][27] = '\0';
        printf("  RSSI: %d", g_testarry[count][25]-256);
        printf("  count: %d\n", g_testarry[count][26]);
        bagnum = count;        
      }//end of else
      flag = 0;
      msg.header.stamp = ros::Time::now();
      msg.header.seq++;
      msg.data = (char*)uhf_reader.epcbuf24;
      printf("%s\n", msg.data.c_str());
      msg.RSSI = g_testarry[bagnum][25]-256;//remember if U want to get the real RSSI,subtract 256
      msg.count = g_testarry[bagnum][26];  
      number++;
      t=time(NULL);
      //fp<<msg.RSSI<<","<<number<<endl;
      time_nu.insert(time_nu.end(),number);
      rssi_nu.insert(rssi_nu.end(),msg.RSSI);
      if (number >= 10)
      {
        //erchengfa(number);
      }
    }//end of !f
    else
    {
      msg.header.stamp = ros::Time::now();
      msg.header.seq++;
      msg.data = '\0';
      msg.pos_x = 0;
      msg.pos_y = 0;
      msg.RSSI = 0;
      msg.count = 0;
    }
      rfid_pub.publish(msg);
      msg.data.clear();

      if(!rfid_pub)
      {
        continue;
      }
      ros::spinOnce();
  }//end of while
  fp.close();
  //close(fd);
  return 0;
} // end main()
