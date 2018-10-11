#ifndef FLATLAND_SERVER_MESSAGE_SERVER_H
#define FLATLAND_SERVER_MESSAGE_SERVER_H

#include <unordered_map>
#include <vector>
#include <queue>

namespace flatland_server {

    class MessageServer;
    class MessageTopicBase {};

    template <class T>
    class MessageTopic : public MessageTopicBase {
    public:
        std::string type;
        std::string source;
        std::queue<T> *messages;
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
        MessageTopic<T>* create_topic(std::string name);

        template <class T>
        Subscriber<T> subscribe(std::string name);

        template <class T>
        Publisher<T> advertise(std::string name);
    };


template<class T>
MessageTopic<T>* MessageServer::create_topic(std::string name) {
    flatland_server::MessageTopic<T> *topic;

    if (messageTopics.find(name) == messageTopics.end()) {
        topic = new flatland_server::MessageTopic<T>();
        topic->messages = new std::queue<T>();
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
Publisher<T> MessageServer::advertise(std::string name) {
    flatland_server::MessageTopic<T> *topic = create_topic<T>(name);
    Publisher<T> publisher;
    publisher.topic = topic;
    return publisher;

}

template <class T>
void Publisher<T>::publish(T& t) {
    topic->messages->push(t);
}

template <class T>
T* Subscriber<T>::receive() {
    if (!topic->messages->empty()) {
        T* t = &topic->messages->front();
        topic->messages->pop();
        return t;
    }
    return nullptr;
}

}

#endif //FLATLAND_SERVER_MESSAGE_H
