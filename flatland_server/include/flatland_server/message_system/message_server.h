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
    class MessageTopic : public MessageTopicBase {
    public:
        MessageType type;
        std::string source;
        std::queue<T> *messages;
    };

    template <class T>
    class Subscriber {
    public:
        MessageTopic<T>* topic;
    };

    template <class T>
    class Publisher {
    public:
        MessageTopic<T>* topic;
    };


    class MessageServer {
    public:
        std::map<MessageType, MessageTopicBase *> messageTopics;
    };

}

#endif //FLATLAND_SERVER_MESSAGE_H
