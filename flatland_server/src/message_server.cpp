#include <flatland_server/message_system/message_server.h>

namespace flatland_server {

MessageServer::MessageServer() {}
MessageServer::~MessageServer() {}

template <class T>
Publisher<T>& MessageServer::advertise(MessageType type) {
    // Add new topic if it doesn't currently exist
    if (messageTopics.find(type) == messageTopics.end()) {
        MessageTopic<T> newTopic;
        messageTopics.insert({type, newTopic});
        newTopic.topic_name = type;
    }

    // Find the topic
    MessageTopic<T> topic = messageTopics[type];
    Publisher<T> publisher;
    publisher.topic = topic;

    topic.publishers.insert(publisher);
    return publisher;
}

template <class T>
Subscriber<T>& MessageServer::subscribe(MessageType type) {
    // Add new topic if it doesn't currently exist
    if (messageTopics.find(type) == messageTopics.end()) {
        MessageTopic<T> newTopic;
        messageTopics.insert({type, newTopic});
        newTopic.type = type;
    }

    // Find the topic
    MessageTopic<T> topic = messageTopics[type];
    Subscriber<T> subscriber;
    subscriber.topic = topic;

    topic.subscribers.insert(subscriber);
    return subscriber;
}

template <class T>
void Publisher::publish(T) {
    topic->messages.push(T);
}

template <class T>
void Subscriber::subscribe(T) {

}


}