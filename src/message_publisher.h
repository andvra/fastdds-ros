#pragma once

#include <chrono>
#include <thread>

#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/publisher/DataWriterListener.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>

using namespace eprosima::fastdds::dds;

// TMessage = Message definition, eg. sensor_msgs::msg::Imu
// TType = Type definition, eg. sensor_msgs::msg::ImuPubSubType
template<typename TMessage, typename TType>
class Message_publisher {
private:
	TMessage msg;
	DomainParticipant* participant;
	Publisher* publisher;
	Topic* topic;
	DataWriter* writer;
	TypeSupport type;

	class PubListener : public DataWriterListener
	{
	public:

		PubListener() : matched(0) {
		}

		~PubListener() override {
		}

		void on_publication_matched(
			DataWriter*,
			const PublicationMatchedStatus& info) override {
			if (info.current_count_change == 1) {
				matched = info.total_count;
				std::cout << "Publisher matched." << std::endl;
			}
			else if (info.current_count_change == -1) {
				matched = info.total_count;
				std::cout << "Publisher unmatched." << std::endl;
			}
			else {
				std::cout << info.current_count_change
					<< " is not a valid value for PublicationMatchedStatus current count change." << std::endl;
			}
		}

		std::atomic_int matched;

	};

	PubListener listener;
	bool done = false;

public:

	Message_publisher(TMessage initial_message = {})
		: participant(nullptr)
		, publisher(nullptr)
		, topic(nullptr)
		, writer(nullptr)
		, msg(initial_message)
		, type(new TType()) {
	}

	virtual ~Message_publisher() {
		if (writer != nullptr) {
			publisher->delete_datawriter(writer);
		}

		if (publisher != nullptr) {
			participant->delete_publisher(publisher);
		}

		if (topic != nullptr) {
			participant->delete_topic(topic);
		}

		DomainParticipantFactory::get_instance()->delete_participant(participant);
	}

	//!Initialize the publisher
	bool init(std::string topic_name) {
		auto participantQos = DomainParticipantQos();
		participantQos.name("participantpublisher");
		participant = DomainParticipantFactory::get_instance()->create_participant(0, participantQos);

		if (participant == nullptr) {
			return false;
		}

		// Register the Type
		type.register_type(participant);

		// Create the publications Topic
		auto qos_topic = TopicQos();
		// To be ROS2 compatible we add "rt/" before the topic name
		topic = participant->create_topic(topic_name, type.get_type_name(), qos_topic);

		if (topic == nullptr) {
			return false;
		}

		// Create the Publisher
		auto qos_publisher = PublisherQos();
		publisher = participant->create_publisher(qos_publisher, nullptr);

		if (publisher == nullptr) {
			return false;
		}

		// Create the DataWriter
		writer = publisher->create_datawriter(topic, DATAWRITER_QOS_DEFAULT, &listener);

		if (writer == nullptr) {
			return false;
		}
		return true;
	}

	//!Send a publication
	bool publish() {
		writer->write(&msg);
		return true;
	}

	void update_message(TMessage new_msg) {
		msg = new_msg;
	}

	void shutdown() {
		done = true;
	}

	//!Run the Publisher
	void run() {
		uint32_t samples_sent = 0;

		while (!done) {
			if (publish()) {
				samples_sent++;
				std::cout << "Posting message of type " << type.get_type_name() << ". It has ID " << samples_sent << std::endl;
			}

			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		}

		std::cout << "Stopped publishing\n";
	}
};