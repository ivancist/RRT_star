#ifndef RRT_STAR_WEBSOCKETSERVER_H
#define RRT_STAR_WEBSOCKETSERVER_H

#include <set>
#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>
#include "Environment.h"
#include "stoppableThread.h"

typedef websocketpp::server<websocketpp::config::asio> server;

class WebSocketServer {
public:
    ~WebSocketServer();

//    std::map<void*, std::unique_ptr<std::thread>> runningThreads;
    std::map<void*, std::shared_ptr<StoppableThread>> runningThreads;

    void start(int port = 9002, void (*onOpenCallback)(websocketpp::connection_hdl) = nullptr);

    static void join();

    void stop();

    void binaryBroadcast(const std::string &topic, const std::string &message);

    void binarySend(websocketpp::connection_hdl hdl, const std::string &topic, const std::string &message);

    server *getServer();

    std::set<websocketpp::connection_hdl, std::owner_less<websocketpp::connection_hdl>> *getConnections();

    void setOnOpenCallback(std::function<void(websocketpp::connection_hdl)> onOpenCallback);

private:
    server ws_server;
    std::set<websocketpp::connection_hdl, std::owner_less<websocketpp::connection_hdl>> connections;
};

#endif //RRT_STAR_WEBSOCKETSERVER_H
