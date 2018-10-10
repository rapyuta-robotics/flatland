#ifndef FLATLAND_SERVER_MESSAGE_SERVER_H
#define FLATLAND_SERVER_MESSAGE_SERVER_H

#include <map>
#include <vector>
#include <queue>

namespace flatland_server {

    class MessageServer;
    template <class T>
    class Subscriber;
    template <class T>
    class Publisher;
    class MessageTopicBase {};

    enum MessageType {
        UIAction,
        HumanAction
    };

    template <class T>
    class MessageTopic : MessageTopicBase {
    public:
        MessageType type;
        std::string source;
        std::vector<Publisher<T>> publishers;
        std::vector<Subscriber<T>> subscribers;
        std::queue<T> messages;
    };

    template <class T>
    class Subscriber {
    public:
        MessageTopic<T>* topic;
        T receive();
    };

    template <class T>
    class Publisher {
    public:
        MessageTopic<T>* topic;
        void publish(T);
    };


    class MessageServer {
    public:
        std::map<MessageType, MessageTopicBase> messageTopics;

        template <class T>
        Publisher<T>& advertise(MessageType type, std::string source);

        template <class T>
        Subscriber<T>& subscribe(MessageType type);

        MessageServer();
        ~MessageServer();
    };

}

#endif //FLATLAND_SERVER_MESSAGE_H
