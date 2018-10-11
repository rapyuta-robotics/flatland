#ifndef FLATLAND_SERVER_MESSAGE_SERVER_H
#define FLATLAND_SERVER_MESSAGE_SERVER_H

#include <deque>
#include <unordered_map>
#include <vector>

#include <ros/ros.h>

namespace flatland_server {

class MessageServer;
class MessageTopicBase {
public:
    virtual void clean_old_messages() = 0;
};

template <class T>
class MessageTopic : public MessageTopicBase {
 public:
  ros::Duration message_life_;
  std::deque<std::pair<ros::Time, T> > messages_;

  MessageTopic(ros::Duration message_life) : message_life_(message_life) {}

  void clean_old_messages() override;
};

template <class T>
class Subscriber {
 public:
  MessageTopic<T>* topic_;

  T* receive();
};

template <class T>
class Publisher {
 public:
  MessageTopic<T>* topic_;

  void publish(T&);
};

class MessageServer {
 public:
  std::unordered_map<std::string, MessageTopicBase*> messageTopics_;

  template <class T>
  MessageTopic<T>* create_topic(std::string name,
                                ros::Duration message_life = ros::Duration(1));

  template <class T>
  Subscriber<T> subscribe(std::string name);

  template <class T>
  Publisher<T> advertise(std::string name,
                         ros::Duration message_lifetime = ros::Duration(1));

  void clean_old_topics();
};

template <class T>
MessageTopic<T>* MessageServer::create_topic(std::string name,
                                             ros::Duration message_life) {
  flatland_server::MessageTopic<T>* topic;

  if (messageTopics_.find(name) == messageTopics_.end()) {
    topic = new flatland_server::MessageTopic<T>(message_life);
    messageTopics_.insert({name, topic});
  } else {
    topic = (MessageTopic<T>*)messageTopics_[name];
  }
  return topic;
}

template <class T>
Subscriber<T> MessageServer::subscribe(std::string name) {
  flatland_server::MessageTopic<T>* topic = create_topic<T>(name);
  Subscriber<T> subscriber;
  subscriber.topic_ = topic;
  return subscriber;
}

template <class T>
Publisher<T> MessageServer::advertise(std::string name,
                                      ros::Duration message_lifetime) {
  flatland_server::MessageTopic<T>* topic = create_topic<T>(name);
  Publisher<T> publisher;
  publisher.topic_ = topic;
  return publisher;
}

template <class T>
void Publisher<T>::publish(T& t) {
  topic_->messages_.push_front({ros::Time::now(), t});
}

template <class T>
T* Subscriber<T>::receive() {
  if (!topic_->messages_.empty()) {
    std::pair<ros::Time, T>* t = &topic_->messages_.front();
    return &t->second;
  }
  return nullptr;
}

template <class T>
void MessageTopic<T>::clean_old_messages() {
    ROS_INFO_STREAM(messages_.size());

    ros::Time now = ros::Time::now();
    // Delete old messages
    while (!messages_.empty()) {
        std::pair<ros::Time, T> msg = messages_.back();
        if (msg.first < now - message_life_) {
            messages_.pop_back();
            continue;
        }
        break;
    }
}

}

#endif  // FLATLAND_SERVER_MESSAGE_H
