#ifndef FLATLAND_SERVER_MESSAGE_H
#define FLATLAND_SERVER_MESSAGE_H

#include <boost/shared_ptr.hpp>
#include <boost/fusion/container/list.hpp>

namespace flatland_server {

    template <class T>
    class Message {
        std::string source;
        T type;
    };

}

#endif //FLATLAND_SERVER_MESSAGE_H
