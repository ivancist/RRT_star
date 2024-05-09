#include "WebSocketServer.h"
#include "RRT_star.h"
#include <nlohmann/json.hpp>

WebSocketServer wsServer;
octomap::OcTree *tree = new octomap::OcTree("../octomap.bt");

void myCallback(std::vector<Node *> *nodes, websocketpp::connection_hdl hdl) {
    nlohmann::json jsonArray;
    for (const auto &node: *nodes) {
        jsonArray.push_back({{"x", node->x},
                             {"y", node->y},
                             {"z", node->z}});
    }
    std::string jsonString = jsonArray.dump();
    wsServer.binarySend(hdl,"octomap_path", jsonString);
    return;
}

void onOpenCallback(websocketpp::connection_hdl hdl) {
    auto rrtThreadPtr = std::make_shared<StoppableThread>();
    rrtThreadPtr->startThread([hdl, rrtThreadPtr]() {
        std::stringstream buffer;
        tree->writeBinaryData(buffer);
        std::string str = buffer.str();
        wsServer.binarySend(hdl, "octomap", str);
        Node start{4, 1.5, 4, nullptr, 0};
        Node goal{4, 1.5, 2, nullptr, std::numeric_limits<double>::max()};
//        Node start{-3, 1, 4, nullptr, 0};
//        Node goal{15, 2, 2, nullptr, std::numeric_limits<double>::max()};
        nlohmann::json endpointsJson = {
                {"start", {{"x", start.x}, {"y", start.y}, {"z", start.z}, {"color", "red"}}},
                {"goal",  {{"x", goal.x},  {"y", goal.y},  {"z", goal.z},  {"color", "green"}}}
        };
        wsServer.binaryBroadcast("octomap_endpoints", endpointsJson.dump());
        std::vector<Node *> path = rrtStar(&start, &goal, tree, myCallback,hdl, rrtThreadPtr);
        if (!path.empty()) {
            std::cout << "Path found with " << path.size() << " nodes" << std::endl;
        } else {
            std::cout << "Path not found" << std::endl;
        }
    });
    rrtThreadPtr->detach();
    wsServer.runningThreads[hdl.lock().get()] = std::move(rrtThreadPtr);
}


int main() {
    wsServer.setOnOpenCallback([](websocketpp::connection_hdl hdl) {
        onOpenCallback(hdl); // Call your original onOpenCallback function
    });
    wsServer.start();
    WebSocketServer::join();
    return 0;
}