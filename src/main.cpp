#include "WebSocketServer.h"
#include "WebSocketClient.h"
#include "RRT_star.h"
#include <nlohmann/json.hpp>


WebSocketServer wsServer;
std::shared_ptr<Environment> env = std::make_shared<Environment>();
octomap::OcTree *tree = new octomap::OcTree("../octomap.bt");
std::map<void*, std::shared_ptr<WebSocketClient>> clients;

void onCloseCallback(websocketpp::connection_hdl hdl) {
    std::cout << "Connection closed" << std::endl;
    if (clients.find(hdl.lock().get()) != clients.end()) {
        std::cout << "Deleting client" << std::endl;
        clients[hdl.lock().get()]->close();
        std::cout << "Client deleted" << std::endl;
//        clients.erase(hdl.lock().get());
//        std::cout << "Client erased" << std::endl;
    }
}

void onMessageCallback(websocketpp::connection_hdl hdl,
                       const websocketpp::server<websocketpp::config::asio>::message_ptr msg) {
    clients[hdl.lock().get()]->handleMessage(msg);
}

void onOpenCallback(websocketpp::connection_hdl hdl) {
    const auto client = std::make_shared<WebSocketClient>(hdl, &wsServer);

//    // TODO CHANGE THIS TO USE THE NEW ENVIRONMENT
//    client->setEnvironment(env);
//    std::stringstream buffer;
//    env->tree->writeBinaryData(buffer);
//    std::string str = buffer.str();
//    wsServer.binarySend(hdl, "octomap", str);

    clients[hdl.lock().get()] = client;
}


int main() {
    auto start_env = std::chrono::high_resolution_clock::now();
    initializeEnvironment(env, tree, 1.0); // 1.0 is the maxDist = stepLength
//    std::cout << "Time for environment initialization: "
//              << std::chrono::duration_cast<std::chrono::milliseconds>(
//                      std::chrono::high_resolution_clock::now() - start_env).count() << "ms" << std::endl;
    wsServer.setOnOpenCallback([](websocketpp::connection_hdl hdl) {
        onOpenCallback(hdl);
    });

    wsServer.setOnMessageCallback([](websocketpp::connection_hdl hdl, const websocketpp::server<websocketpp::config::asio>::message_ptr msg) {
        onMessageCallback(hdl, msg);
    });

    wsServer.setOnCloseCallback([](websocketpp::connection_hdl hdl) {
        onCloseCallback(hdl);
    });

    wsServer.start();
    WebSocketServer::join();
    return 0;
}