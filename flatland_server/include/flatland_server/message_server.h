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
  ros::Duration message_life_;
  std::deque<std::pair<ros::Time, T>> messages_;

 public:
  MessageTopic(ros::Duration message_life) : message_life_(message_life) {}

  const std::deque<std::pair<ros::Time, T>>& get_messages() const {
    return messages_;
  }

  const T* get_latest_message() const;
  void add_new_message(const T& msg);
  void clean_old_messages() override;
};

template <class T>
class Subscriber {
  MessageTopic<T>* topic_;

    Subscriber(MessageTopic<T>* topic) : topic_(topic) {}

public:
    Subscriber() {}
    ~Subscriber() {}
    const T* receive() const;

    friend class MessageServer;
};

template <class T>
class Publisher {
  MessageTopic<T>* topic_;

    Publisher(MessageTopic<T>* topic) : topic_(topic) {}

public:
    Publisher() {}
    ~Publisher() {}
    void publish(const T&);

    friend class MessageServer;
};

class MessageServer {
 public:
  std::unordered_map<std::string, std::unique_ptr<MessageTopicBase>>
      messageTopics_;

  template <class T>
  MessageTopic<T>* create_topic(
      const std::string& name,
      const ros::Duration& message_life = ros::Duration(1));

  template <class T>
  Subscriber<T> subscribe(const std::string& name);

  template <class T>
  Publisher<T> advertise(
      const std::string& name,
      const ros::Duration& message_lifetime = ros::Duration(1));

  void clean_old_topics();
};

///
/// Start of Implementation section
///

template <class T>
MessageTopic<T>* MessageServer::create_topic(
    const std::string& name, const ros::Duration& message_life) {
  flatland_server::MessageTopic<T>* topic;
  auto msgTopic = messageTopics_.find(name);
  if (msgTopic == messageTopics_.end()) {
    topic = new flatland_server::MessageTopic<T>(message_life);
    messageTopics_.emplace(name, std::unique_ptr<MessageTopic<T>>(topic));
  } else {
    topic = static_cast<MessageTopic<T>*>((msgTopic->second).get());
  }
  return topic;
}

template <class T>
Subscriber<T> MessageServer::subscribe(const std::string& name) {
  flatland_server::MessageTopic<T>* topic = create_topic<T>(name);
  return Subscriber<T>(topic);
}

template <class T>
Publisher<T> MessageServer::advertise(const std::string& name,
                                      const ros::Duration& message_lifetime) {
  flatland_server::MessageTopic<T>* topic =
      create_topic<T>(name, message_lifetime);
  return Publisher<T>(topic);
}

template <class T>
void Publisher<T>::publish(const T& t) {
  topic_->add_new_message(t);
}

template <class T>
const T* Subscriber<T>::receive() const {
  if (!topic_->get_messages().empty()) {
    return topic_->get_latest_message();
  }
  return nullptr;
}

template <class T>
void MessageTopic<T>::clean_old_messages() {
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

template <class T>
void MessageTopic<T>::add_new_message(const T& msg) {
  ros::Time now = ros::Time::now();
  messages_.emplace_back(now, msg);
}

template <class T>
const T* MessageTopic<T>::get_latest_message() const  {
    return &((messages_.front()).second);
}
}

#endif  // FLATLAND_SERVER_MESSAGE_H
