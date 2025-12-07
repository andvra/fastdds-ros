#pragma once

#include <conio.h>

#include "message_publisher.h"
#include "message_subscriber.h"

template<typename TMessage, typename TType>
class Node_executor {
private:
	std::atomic<bool> do_shutdown = false;

	// NB this will only work for Windows
	// TODO Write a Linux equivalent
	void key_handler() {
		while (!do_shutdown) {
			if (_kbhit() && (_getch() == 'q')) {
				do_shutdown = true;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}

		std::cout << "Shutting down..\n";
	}

public:
	bool run_node(std::string topic_name_publisher, std::string topic_name_subscriber) {
		auto thread_keyhandler = std::thread(&Node_executor::key_handler, this);

		auto message = TMessage();
		auto publisher = new Message_publisher<TMessage, TType>(message);

		if (!publisher->init(topic_name_publisher)) {
			std::cout << "Could not initialize publisher\n";
			delete publisher;
			return false;
		}

		auto thread_publisher = std::thread([&]() {
			publisher->run();
			});

		auto subscriber = new Message_subscriber<TMessage, TType>();

		if (!subscriber->init(topic_name_subscriber)) {
			std::cout << "Could not initialize subscriber\n";
			delete subscriber;
			return false;
		}

		auto thread_subscriber = std::thread([&]() {
			subscriber->run();
			});

		auto done = false;

		while (!done) {
			if (do_shutdown) {
				publisher->shutdown();
				subscriber->shutdown();
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

		if (thread_subscriber.joinable()) {
			thread_subscriber.join();
		}

		delete publisher;

		return true;
	}
};
