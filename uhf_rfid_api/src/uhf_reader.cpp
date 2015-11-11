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
 *Description:  //收到18次数据，停止5s发送STOPSTOPSTOP模拟机器人停止前进
                //与其他模块或函数的接口、输出值、取值范围、
                //含义及参数间的控制、顺序、独立及依赖关系
 *Others:  此文件仅为接收话题的测试文件
 *Function List:  //主要函数列表，每条记录应包含函数名及功能简要说明
     1.…………
     2.…………
 *History:  //修改历史记录列表，每条修改记录应包含修改日期、修改者及修改内容简介
     1.Date:2015-5-13
       Author:feixiao
       Modification:接收话题消息，
     2.…………
**********************************************************************************/
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "uhf_rfid_api/UhfRfid.h"

int uh_sum_ = 0;
int uh_i_ = 0;
float uh_avg_ = 0;
float uh_prev1_avg_ = -500;
float uh_prev2_avg_ = -500;
float uh_prev3_avg_ = -500;
float uh_prev4_avg_ = -500;
float uh_prev5_avg_ = -500;
float uh_prev6_avg_ = -500;
std::string current_epc_("\0");

void chatterCallback(const uhf_rfid_api::UhfRfid::ConstPtr& rfid_msgs)
{
  if(*(rfid_msgs->data.c_str()) != 0)
  {
    //printf("rfid_msgs->data: %s \n", rfid_msgs->data.c_str());
    //std::cout << rfid_msgs->data <<std::endl;
    uh_i_++;
    uh_sum_ += rfid_msgs->RSSI;
  }

  if (uh_i_ == 15)
  {
    uh_avg_ = uh_sum_ / 15.0;
    uh_i_ = 0;
    uh_sum_ = 0;
    ROS_INFO("avg: %f", uh_avg_);
    if(-55 < uh_avg_)
    {
      if (uh_prev1_avg_ <= uh_avg_ && uh_prev2_avg_ <= uh_prev1_avg_ && uh_prev3_avg_ <= uh_prev2_avg_ && uh_prev4_avg_ <= uh_prev3_avg_ && uh_prev5_avg_ <= uh_prev4_avg_ && uh_prev6_avg_ <= uh_prev5_avg_)
      {
        uh_prev1_avg_ = uh_avg_;
        uh_prev2_avg_ = uh_prev1_avg_;
        uh_prev3_avg_ = uh_prev2_avg_;
        uh_prev4_avg_ = uh_prev3_avg_;
        uh_prev5_avg_ = uh_prev4_avg_;
        uh_prev6_avg_ = uh_prev5_avg_;
      }
      else
      {
        current_epc_ = rfid_msgs->data;
        ROS_INFO("current_epc_: %s", current_epc_.c_str());
        uh_prev1_avg_ = -500;
        uh_prev2_avg_ = -500;
        uh_prev3_avg_ = -500;
        uh_prev4_avg_ = -500;
        uh_prev5_avg_ = -500;
        uh_prev6_avg_ = -500;
      }
    }
  }
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "listener");

  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("messageepc", 1, chatterCallback);

  ros::spin();
  return 0;
}
