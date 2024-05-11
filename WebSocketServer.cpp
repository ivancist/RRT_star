#include "WebSocketServer.h"

server ws_server;
std::thread wsThread;
std::set<websocketpp::connection_hdl, std::owner_less<websocketpp::connection_hdl>> connections;
std::map<void *, std::unique_ptr<StoppableThread>> runningThreads;

server *WebSocketServer::getServer() {
    return &ws_server;
}

std::set<websocketpp::connection_hdl, std::owner_less<websocketpp::connection_hdl>> *WebSocketServer::getConnections() {
    return &connections;
}

void
WebSocketServer::binarySend(websocketpp::connection_hdl hdl, const std::string &topic, const std::string &message) {
    uint32_t topicLength = static_cast<uint32_t>(topic.length());
    std::vector<uint8_t> header(4);
    std::memcpy(header.data(), &topicLength, sizeof(topicLength));

    // Encode topic string
    std::vector<uint8_t> encodedTopic(topic.begin(), topic.end());

    // Concatenate header, encoded topic, and binary message
    std::vector<uint8_t> result(header);
    result.insert(result.end(), encodedTopic.begin(), encodedTopic.end());
    result.insert(result.end(), message.begin(), message.end());

    if (connections.find(hdl) == connections.end()) {
        return;
    }
    ws_server.send(hdl, result.data(), result.size(), websocketpp::frame::opcode::binary);
}

void WebSocketServer::binaryBroadcast(const std::string &topic, const std::string &message) {
    std::cout << "Broadcasting to " << connections.size() << " connections" << std::endl;
    for (auto hdl: connections) {
        binarySend(hdl, topic, message);
    }
}

void on_message(websocketpp::connection_hdl hdl, const server::message_ptr msg, WebSocketServer *wsServer) {
    std::cout << "on_message called with hdl: " << hdl.lock().get() << " and message: " << msg->get_payload()
              << std::endl;
    wsServer->getServer()->send(hdl, "Message received", msg->get_opcode());

    if (msg->get_payload() == "stopServer") {
        wsServer->stop();
    }
}

std::function<void(websocketpp::connection_hdl)> on_open_callback_ = nullptr;
void on_open(websocketpp::connection_hdl hdl, WebSocketServer *wsServer) {
    wsServer->getConnections()->insert(hdl);

    if (on_open_callback_ != nullptr) {
        on_open_callback_(hdl);
    }
}

void WebSocketServer::setOnOpenCallback(std::function<void(websocketpp::connection_hdl)> onOpenCallback) {
    on_open_callback_ = std::move(onOpenCallback);
}

void on_close(websocketpp::connection_hdl hdl, WebSocketServer *wsServer) {
    if (wsServer->runningThreads.find(hdl.lock().get()) != wsServer->runningThreads.end()) {
        wsServer->runningThreads[hdl.lock().get()]->stopThread();
        wsServer->runningThreads.erase(hdl.lock().get());
    }
    wsServer->getConnections()->erase(hdl);

}

void WebSocketServer::start(int port, void (*onOpenCallback)(websocketpp::connection_hdl)) {
    // Disable logging for frame headers and payloads
    ws_server.clear_access_channels(
            websocketpp::log::alevel::frame_header | websocketpp::log::alevel::frame_payload);
//    ws_server.clear_access_channels(websocketpp::log::alevel::all);

    ws_server.init_asio();
    ws_server.set_reuse_addr(true);

    ws_server.set_message_handler(std::bind(&on_message,std::placeholders::_1, std::placeholders::_2, this));
    ws_server.set_open_handler(std::bind(&on_open, std::placeholders::_1, this));
    ws_server.set_close_handler(std::bind(&on_close, std::placeholders::_1, this));

    ws_server.listen(port);
    ws_server.start_accept();
    std::thread t([this]() {
        ws_server.run();
    });
    wsThread = std::move(t);
}

WebSocketServer::~WebSocketServer() {
    this->stop();
}

void WebSocketServer::stop() {
    std::cout << "Stopping WebSocket server" << std::endl;
    ws_server.stop_listening();
    ws_server.stop();
}

void WebSocketServer::join() {
    wsThread.join();
}
