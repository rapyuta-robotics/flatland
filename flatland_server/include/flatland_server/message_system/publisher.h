#ifndef FLATLAND_SERVER_MESSAGE_SYSTEM_PUBLISHER_H
#define FLATLAND_SERVER_MESSAGE_SYSTEM_PUBLISHER_H

#include <boost/shared_ptr.hpp>
#include <boost/fusion/container/list.hpp>
#include <flatland_server/message_system/message_server.h>

namespace flatland_server {

    template <class T>
    class Publisher {
        MessageServer* server;
        void publish(T);
    };

}

#endif //FLATLAND_SERVER_MESSAGE_H
