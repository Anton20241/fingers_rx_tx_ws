#include <iostream>
#include <ros/ros.h>
#include <boost/asio.hpp>
#include <std_msgs/ByteMultiArray.h>
#include <boost_rs485_ClientServer.hpp>

int main(int argc, char** argv){
	std::string devPort = "";
	if(argc == 2) {
		devPort = argv[1];
	} else {
		std::cerr << "[program_name][devPort(1, 2...)]" << std::endl;
		return -1;
	}

	try{
		std::cout << "master_topic_receiver is running!" << std::endl;
		ros::init(argc, argv, "master_topic_receiver");
		boost_rs485::Boost_RS485_Master boostRS485_transp("/dev/ttyUSB" + devPort);
		protocol_master::ProtocolMaster boostRS485_prot_master(boostRS485_transp);
		Boost_RS485_Server raspbPi(boostRS485_prot_master);
		ros::spin();
	} catch (std::exception e){
		std::cerr << "Exeption: " << e.what() << std::endl;
	}
  return 0;
};
