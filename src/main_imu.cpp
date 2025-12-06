#include <iostream>
#include <thread>

#include <sensor_msgs/msg/ImuPubSubTypes.hpp>

#include "node_executor.h"

int main(int argc, char** argv) {
	std::cout << "Starting IMU publisher\n";
	auto topic_name = std::string("rt/ImuTopic");

	Node_executor<sensor_msgs::msg::Imu, sensor_msgs::msg::ImuPubSubType> node_executor = {};

	auto ret = node_executor.run_node(topic_name);

	return (ret ? 0 : 1);
}
