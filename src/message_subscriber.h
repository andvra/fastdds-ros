#pragma once

#include <chrono>
#include <thread>

#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/DataReaderListener.hpp>
#include <fastdds/dds/subscriber/qos/DataReaderQos.hpp>
#include <fastdds/dds/subscriber/SampleInfo.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>

using namespace eprosima::fastdds::dds;

// TMessage = Message definition, eg. sensor_msgs::msg::Imu
// TType = Type definition, eg. sensor_msgs::msg::ImuPubSubType
template<typename TMessage, typename TType>
class Message_subscriber
{
private:
    DomainParticipant* participant;
    Subscriber* subscriber;
    DataReader* reader;
    Topic* topic;
    TypeSupport type;

    class SubListener : public DataReaderListener
    {
    public:
        SubListener() : num_samples(0) {
        }

        ~SubListener() override {
        }

        void on_subscription_matched(DataReader*, const SubscriptionMatchedStatus& info) override {
            if (info.current_count_change == 1) {
                std::cout << "Subscriber matched." << std::endl;
            }
            else if (info.current_count_change == -1) {
                std::cout << "Subscriber unmatched." << std::endl;
            }
            else {
                std::cout << info.current_count_change << " is not a valid value for SubscriptionMatchedStatus current count change" << std::endl;
            }
        }

        void on_data_available(DataReader* reader) override {
            SampleInfo info;

            if (reader->take_next_sample(&msg, &info) == eprosima::fastdds::dds::RETCODE_OK) {
                if (info.valid_data) {
                    num_samples++;
                    std::cout << "Got new message!\n";
                }
            }
        }

        TMessage msg;
        std::atomic_int num_samples;
    };

    SubListener listener;
    bool done = false;
public:
    Message_subscriber()
        : participant(nullptr)
        , subscriber(nullptr)
        , topic(nullptr)
        , reader(nullptr)
        , type(new TType()) {
    }

    virtual ~Message_subscriber(){
        if (reader != nullptr){
            subscriber->delete_datareader(reader);
        }
        if (topic != nullptr){
            participant->delete_topic(topic);
        }
        if (subscriber != nullptr){
            participant->delete_subscriber(subscriber);
        }
        DomainParticipantFactory::get_instance()->delete_participant(participant);
    }

    bool init(std::string topic_name){
        DomainParticipantQos participantQos;
        participantQos.name("participantsubscriber");
        participant = DomainParticipantFactory::get_instance()->create_participant(0, participantQos);

        if (participant == nullptr){
            return false;
        }

        type.register_type(participant);
        auto qos_topic = TopicQos();
        topic = participant->create_topic(topic_name, type.get_type_name(), qos_topic);

        if (topic == nullptr){
            return false;
        }

        // Create the Subscriber
        auto qos_subscriber = SubscriberQos();
        subscriber = participant->create_subscriber(qos_subscriber, nullptr);

        if (subscriber == nullptr){
            return false;
        }

        // Create the DataReader
        auto qos_datareader = DATAREADER_QOS_DEFAULT;
        qos_datareader.reliability().kind = RELIABLE_RELIABILITY_QOS;
        qos_datareader.history().kind = KEEP_LAST_HISTORY_QOS;
        reader = subscriber->create_datareader(topic, qos_datareader, &listener);

        if (reader == nullptr){
            return false;
        }

        return true;
    }

    void shutdown() {
        done = true;
    }

    //!Run the Subscriber
    void run(){
        while (!done){
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        std::cout << "Stopped subscribing\n";
    }

};

//int main(
//    int argc,
//    char** argv)
//{
//    std::cout << "Starting subscriber." << std::endl;
//    uint32_t samples = 10;
//
//    HelloWorldSubscriber* mysub = new HelloWorldSubscriber();
//    if (mysub->init())
//    {
//        mysub->run(samples);
//    }
//
//    delete mysub;
//    return 0;
//}