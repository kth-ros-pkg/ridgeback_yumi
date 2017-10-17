/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, OptoForce, Ltd.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the OptoForce nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/** 
 * Simple stand-alone ROS node that takes data from EtherDAQ sensor and
 * Publishes it ROS topic
 */

#include "ros/ros.h"
#include "etherdaq_driver/etherdaq_driver.h"
#include "geometry_msgs/WrenchStamped.h"
#include "diagnostic_msgs/DiagnosticArray.h"
#include "diagnostic_updater/DiagnosticStatusWrapper.h"
#include "std_msgs/Bool.h"
#include <unistd.h>
#include <iostream>
#include <memory>
#include <boost/program_options.hpp>

namespace po = boost::program_options;
using namespace std;



optoforce_etherdaq_driver::EtherDAQDriver * etherdaq = NULL;

void zeroFunction(const std_msgs::Bool &msg)
{
	if (etherdaq == NULL)
	{
		return;
	}
	
	bool zeroing = msg.data;
	if (zeroing)
	{
		etherdaq->doZero();
		return;
	}
	etherdaq->doUnzero();
}


int main(int argc, char **argv)
{ 
	string sensor_location = "left_arm_";
	string node_name = sensor_location + "etherdaq_node";

	ros::init(argc, argv, node_name);
	ros::NodeHandle nh("~");

	float pub_rate_hz;
	int filter; 
	string ip;
	string frame_id;
	bool publish_wrench = false;
	bool cancel_bias = false;

	nh.param("ip", ip, std::string("192.168.125.3") );
	nh.param("pub_rate_hz", pub_rate_hz, 1000.0f);
	nh.param("filter", filter, 4);
	nh.param("frame_id", frame_id, std::string("optodaq_l_sensor_link") );
	nh.param("publish_wrench", publish_wrench, false );
	nh.param("cancel_bias", cancel_bias, false );

	po::options_description desc("Options");
	desc.add_options()
	("help", "display help")
	("pub_rate_hz", po::value<float>(&pub_rate_hz)->default_value(100.0), "set publish rate and Ethernet DAQ speed (in hertz)")
	("filter", po::value<int>(&filter)->default_value(4), "set filtering (0 = No filter; 1 = 500 Hz; 2 = 150 Hz; 3 = 50 Hz; 4 = 15 Hz; 5 = 5 Hz; 6 = 1.5 Hz)") 
	("publish_wrench", "publish older Wrench message type instead of WrenchStamped")
	("ip", po::value<string>(&ip), "IP address of EthernetDAQ box")
	("frame_id", po::value<string>(&frame_id)->default_value("base_link"), "Frame ID for Wrench data")  
	;

	if(filter < 0 || filter > 6) 
	{
		cout << desc << endl;
		cerr<<"Please specify a valid filtering value instead of " << filter << endl;
		exit(EXIT_FAILURE);
	}

	if(publish_wrench)
	{
		publish_wrench = true;
		ROS_WARN("Publishing EthernetDAQ data as geometry_msgs::Wrench is deprecated");
	}

	etherdaq = new optoforce_etherdaq_driver::EtherDAQDriver(ip, pub_rate_hz, filter);

	if(cancel_bias)
	{
		etherdaq->doZero();
	}

	std::string topic_name = sensor_location + "ethdaq_data";
	bool is_raw_data = etherdaq->isRawData();
	if (is_raw_data) 
	{
		topic_name += "_raw";
	}

	ros::Publisher pub;
	ros::Subscriber sub = nh.subscribe(sensor_location + "ethdaq_zero", 1000, zeroFunction);
	if (publish_wrench)
	{
		pub = nh.advertise<geometry_msgs::Wrench>(topic_name, 100);
	}
	else 
	{
		pub = nh.advertise<geometry_msgs::WrenchStamped>(topic_name, 100);
	}

	ros::Rate pub_rate(pub_rate_hz);
	geometry_msgs::WrenchStamped data;


	unsigned int packetCount = 0; 
	ros::Time startTime(ros::Time::now());	

	ROS_INFO("--------------------------------------------------------------------------------------------------------------");
	ROS_INFO("Optoforce 6 axis Force/Torque sensor");
	ROS_INFO("IP address: %s", ip.c_str());
	ROS_INFO("Publish rate: %f Hz", pub_rate_hz);
	ROS_INFO("Filter setting: %i (0 = No filter; 1 = 500 Hz; 2 = 150 Hz; 3 = 50 Hz; 4 = 15 Hz; 5 = 5 Hz; 6 = 1.5 Hz)", filter);
	ROS_INFO("Robot frame: %s", frame_id.c_str());
	ROS_INFO("Sensor data topic name: %s", topic_name.c_str());
	ROS_INFO("--------------------------------------------------------------------------------------------------------------");

	while (ros::ok())
	{
		if (etherdaq->waitForNewData())
		{
			etherdaq->getData(data);
			packetCount++; 
			if (publish_wrench) 
			{
				data.header.frame_id = frame_id;
				pub.publish(data.wrench);
			}
			else 
			{
				data.header.frame_id = frame_id;
				pub.publish(data);
			}
		}


		ros::spinOnce();
		pub_rate.sleep();
	}

	if (etherdaq != NULL)
	{
		delete etherdaq;
	}

	return 0;
}
