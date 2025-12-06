#include <iostream>
#include <thread>

#include <sensor_msgs/msg/NavSatFixPubSubTypes.hpp>

#include "node_executor.h"

int main(int argc, char** argv) {
	std::cout << "Starting GPS publisher\n";
	auto topic_name = std::string("rt/GpsTopic");

	Node_executor<sensor_msgs::msg::NavSatFix, sensor_msgs::msg::NavSatFixPubSubType> node_executor = {};

	auto ret = node_executor.run_node(topic_name);

	return (ret ? 0 : 1);
}