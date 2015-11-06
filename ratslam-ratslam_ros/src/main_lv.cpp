/*
 * openRatSLAM
 *
 * main_lv - ROS interface bindings for the local view cells
 *
 * Copyright (C) 2012
 * David Ball (david.ball@qut.edu.au) (1), Scott Heath (scott.heath@uqconnect.edu.au) (2)
 *
 * modify by feixiao (feixiao5566@gmail.com) 5 Aug 2015 V1.0
 * Tarsbot Xi`An
 * RatSLAM algorithm by:
 * Michael Milford (1) and Gordon Wyeth (1) ([michael.milford, gordon.wyeth]@qut.edu.au)
 *
 * 1. Queensland University of Technology, Australia
 * 2. The University of Queensland, Australia
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <iostream>
using namespace std;

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "utils/utils.h"

#include <boost/property_tree/ini_parser.hpp>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <ratslam_ros/ViewTemplate.h>
#include <ros/console.h>

#include <image_transport/image_transport.h>

#include "ratslam/local_view_match.h"
#include "uhf_rfid_api/UhfRfid.h"

#if HAVE_IRRLICHT
#include "graphics/local_view_scene.h"
ratslam::LocalViewScene *lvs = NULL;
bool use_graphics;
#endif


using namespace ratslam;
ratslam::LocalViewMatch * lv = NULL;
ros::Publisher g_pub_vt;


void rfid_callback( uhf_rfid_api::UhfRfid rfid)
{
  ROS_DEBUG_STREAM("LV:rfid_callback{" << ros::Time::now() << "} seq=" << rfid.header.seq<<"data"<<rfid.data);
  static ratslam_ros::ViewTemplate vt_output;
  // FIXME: intelligent handling of image encoding
  if(rfid.RSSI)
  {
    lv->on_image((const unsigned char *)rfid.data.c_str());
    lv->find_my((const unsigned char *)"e2004000780c002212709894");
    vt_output.header.stamp = ros::Time::now();
    vt_output.header.seq++;
    vt_output.current_id = lv->get_current_vt()+1;    
    vt_output.stop = lv->get_state();
  // TODO: RTABMAP... here it goes out to pose cell
  }
  else
  {
    vt_output.header.stamp = ros::Time::now();
    vt_output.header.seq++;
    vt_output.current_id = 0;
  }
  g_pub_vt.publish(vt_output);
#ifdef HAVE_IRRLICHT
  if (use_graphics)
  {
    lvs->draw_all();
  }
#endif
}

int main(int argc, char * argv[])
{
  ROS_INFO_STREAM(argv[0] << " - openRatSLAM Copyright (C) 2012 David Ball and Scott Heath");
  ROS_INFO_STREAM("RatSLAM algorithm by Michael Milford and Gordon Wyeth");
  ROS_INFO_STREAM("Distributed under the GNU GPL v3, see the included license file.");
  ROS_INFO_STREAM(argv[3] << " - feixiao ceshi.");
  str = argv[3];
  if (argc < 2)

  {
    ROS_FATAL_STREAM("USAGE: " << argv[0] << " <config_file>");
    exit(-1);
  }
  std::string topic_root = "";

  boost::property_tree::ptree settings, ratslam_settings, general_settings;
  read_ini(argv[1], settings);
  get_setting_child(general_settings, settings, "general", true);
  get_setting_from_ptree(topic_root, general_settings, "topic_root", (std::string)"");
  get_setting_child(ratslam_settings, settings, "ratslam", true);
  lv = new ratslam::LocalViewMatch(ratslam_settings);

  if (!ros::isInitialized())
  {
    ros::init(argc, argv, "RatSLAMViewTemplate");
  }
  ros::NodeHandle node;

  g_pub_vt = node.advertise<ratslam_ros::ViewTemplate>(topic_root + "/LocalView/Template", 0);

  ros::Subscriber sub = node.subscribe<uhf_rfid_api::UhfRfid>("messageepc", 0, rfid_callback) ;



/*
#ifdef HAVE_IRRLICHT
    boost::property_tree::ptree draw_settings;
    get_setting_child(draw_settings, settings, "draw", true);
    get_setting_from_ptree(use_graphics, draw_settings, "enable", true);
    if (use_graphics)
      lvs = new ratslam::LocalViewScene(draw_settings, lv);
#endif
*/
  ros::spin();

  return 0;
}
