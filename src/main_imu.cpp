#include <iostream>
#include <thread>

#include <sensor_msgs/msg/ImuPubSubTypes.hpp>

#include "node_executor.h"

int main(int argc, char** argv) {
	std::cout << "Starting IMU node\n";
	auto topic_name_publisher = std::string("rt/ImuTopic");
	auto topic_name_subscriber = std::string("rt/fix");

	Node_executor<sensor_msgs::msg::Imu, sensor_msgs::msg::ImuPubSubType> node_executor = {};

	auto ret = node_executor.run_node(topic_name_publisher, topic_name_subscriber);

	return (ret ? 0 : 1);
}
