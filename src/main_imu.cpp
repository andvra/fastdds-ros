#include <conio.h>
#include <csignal>
#include <iostream>
#include <thread>

#include <sensor_msgs/msg/ImuPubSubTypes.hpp>

#include "message_publisher.h"
#include "node_executor.h"

std::atomic<bool> do_shutdown = false;

// NB this will only work for Windows
// TODO Write a Linux equivalent
void key_handler() {
	while (!do_shutdown) {
		if (_kbhit() && (_getch()=='q')) {
			do_shutdown = true;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
	std::cout << "Left key_handler\n";
}

template<typename TMessage, typename TType>
bool run_node(std::string topic_name) {
	auto thread_keyhandler = std::thread(key_handler);

	auto message = TMessage();
	auto publisher = new Message_publisher<TMessage, TType>(message);

	if (!publisher->init(topic_name)) {
		std::cout << "Could not initialize published\n";
		delete publisher;
		return false;
	}

	auto thread_publisher = std::thread([&]() {
		publisher->run();
		});

	auto done = false;

	while (!done) {
		if (do_shutdown) {
			publisher->shutdown();
			done = true;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}

	if (thread_keyhandler.joinable()) {
		thread_keyhandler.join();
	}

	if (thread_publisher.joinable()) {
		thread_publisher.join();
	}

	delete publisher;

	return true;
}

int main(int argc, char** argv) {
	std::cout << "Starting IMU publisher\n";
	auto topic_name = std::string("rt/ImuTopic");

	auto ret = run_node<sensor_msgs::msg::Imu, sensor_msgs::msg::ImuPubSubType>(topic_name);

	return (ret ? 0 : 1);
}
