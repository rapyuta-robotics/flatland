#ifndef FLATLAND_SERVER_MESSAGE_SERVER_H
#define FLATLAND_SERVER_MESSAGE_SERVER_H

#include <unordered_map>
#include <vector>
#include <deque>
#include <ros/ros.h>

namespace flatland_server {

    class MessageServer;
    class MessageTopicBase {};

    template <class T>
    class MessageTopic : public MessageTopicBase {
    public:
        ros::Duration message_life;
        std::deque<std::pair<ros::Time, T> > messages;

        MessageTopic(ros::Duration message_life) : message_life(message_life) {}
    };

    template <class T>
    class Subscriber {
    public:
        MessageTopic<T>* topic;

        T* receive();
    };

    template <class T>
    class Publisher {
    public:
        MessageTopic<T>* topic;

        void publish(T&);
    };

    class MessageServer {
    public:
        std::unordered_map<std::string, MessageTopicBase *> messageTopics;

        template <class T>
        MessageTopic<T>* create_topic(std::string name, ros::Duration message_life = ros::Duration(1));

        template <class T>
        Subscriber<T> subscribe(std::string name);

        template <class T>
        Publisher<T> advertise(std::string name, ros::Duration message_lifetime = ros::Duration(1));
    };



template<class T>
MessageTopic<T>* MessageServer::create_topic(std::string name, ros::Duration message_life) {
    flatland_server::MessageTopic<T> *topic;

    if (messageTopics.find(name) == messageTopics.end()) {
        topic = new flatland_server::MessageTopic<T>(message_life);
        messageTopics.insert({name, topic});
    } else {
        topic = (MessageTopic<T>*) messageTopics[name];
    }
    return topic;
}

template<class T>
Subscriber<T> MessageServer::subscribe(std::string name) {
    flatland_server::MessageTopic<T> *topic = create_topic<T>(name);
    Subscriber<T> subscriber;
    subscriber.topic = topic;
    return subscriber;
}

template<class T>
Publisher<T> MessageServer::advertise(std::string name, ros::Duration message_lifetime) {
    flatland_server::MessageTopic<T> *topic = create_topic<T>(name);
    Publisher<T> publisher;
    publisher.topic = topic;
    return publisher;

}

template <class T>
void Publisher<T>::publish(T& t) {

    topic->messages.push_front({ros::Time::now(), t});

    // Delete old messages
    while(!topic->messages.empty()) {
        std::pair<ros::Time, T> msg = topic->messages.back();
        if (msg.first < ros::Time::now() - topic->message_life) {
            topic->messages.pop_back();
            continue;
        }
        break;
    }
}

template <class T>
T* Subscriber<T>::receive() {

    if (!topic->messages.empty()) {
        std::pair<ros::Time, T>* t = &topic->messages.front();
        return &t->second;
    }
    return nullptr;
}

}

#endif //FLATLAND_SERVER_MESSAGE_H
