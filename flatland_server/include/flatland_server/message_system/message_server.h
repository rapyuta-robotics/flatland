#ifndef FLATLAND_SERVER_MESSAGE_SERVER_H
#define FLATLAND_SERVER_MESSAGE_SERVER_H

#include <map>
#include <vector>
#include <queue>

namespace flatland_server {

    class MessageServer;
    class MessageTopic;

    template <class T>
    class Message {
        std::string source;
        T type;
    };

    template <class T>
    class Subscriber {
        MessageTopic* server;
        void publish(T);
    };

    template <class T>
    class Publisher {
        MessageTopic* server;
        void publish(T);
    };

    class MessageTopicBase {};

    template <class T>
    class MessageTopic : MessageTopicBase {
        MessageType type;
        std::vector<Publisher<T>> publishers;
        std::vector<Subscriber<T>> subscribers;
        std::queue<T> messages;
    };

    enum MessageType {
        RobotAction,
        HumanAction
    };

    class MessageServer {
    public:
        std::map<MessageType, MessageTopicBase> messageTopics;

        template <class T>
        Publisher<T>& advertise(MessageType type);

        template <class T>
        Subscriber<T>& subscribe(MessageType type);

        MessageServer();
        ~MessageServer();
    };

}

#endif //FLATLAND_SERVER_MESSAGE_H
