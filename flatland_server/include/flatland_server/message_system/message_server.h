#ifndef FLATLAND_SERVER_MESSAGE_SERVER_H
#define FLATLAND_SERVER_MESSAGE_SERVER_H

#include <flatland_server/message_system/publisher.h>
#include <flatland_server/message_system/subscriber.h>

namespace flatland_server {

    class MessageServer {
    public:
        template <class T>
        Publisher<T> advertise(std::string name);

        template <class T>
        Subscriber<T> subscribe(std::string name);
    };

}

#endif //FLATLAND_SERVER_MESSAGE_H
