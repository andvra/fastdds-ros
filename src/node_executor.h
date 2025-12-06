#pragma once

#include "message_publisher.h"

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
		std::cout << "Left key_handler\n";
	}

public:
	bool run_node(std::string topic_name) {
		auto thread_keyhandler = std::thread(&Node_executor::key_handler, this);

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
};
