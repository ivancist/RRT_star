//
// Created by Ivan Cisternino on 10/06/24.
//

#ifndef OPPI_WEBSOCKETCLIENT_H
#define OPPI_WEBSOCKETCLIENT_H

#include <websocketpp/client.hpp>
#include "WebSocketServer.h"
#include "Environment.h"
#include "stoppableThread.h"
#include <nlohmann/json.hpp>
#include "RRT_star.h"
#include "functional"
#include <memory>

struct partialPathThread {
    nlohmann::json start;
    nlohmann::json goal;
    std::shared_ptr<StoppableThread> thread;
    std::shared_ptr<ComputedPath> partialPath;
    bool toDelete = false;
};

class WebSocketClient {
public:
    WebSocketClient(websocketpp::connection_hdl hdl, WebSocketServer *wsServer);

    void setEnvironment(std::shared_ptr<Environment> env);

    void setParameters(std::shared_ptr<RRTStarParameters> parameters);

    void close();

    void handleMessage(websocketpp::server<websocketpp::config::asio>::message_ptr msg);

    static std::string computedPathToString(std::shared_ptr<ComputedPath> path, bool time = false, bool calculateCost = false);

private:
    WebSocketServer* wsServer_;
    websocketpp::connection_hdl hdl_;
    std::shared_ptr<Environment> env_;
    std::shared_ptr<RRTStarParameters> parameters_;
    std::shared_ptr<std::vector<partialPathThread>> threads_;
    std::shared_ptr<RRTStar> rrtStar_;

    void onPathFoundCallback(std::shared_ptr<ComputedPath> returnPath);

    void handleMapMessage(const nlohmann::json &msg);

    void handleParametersMessage(const nlohmann::json &msg);

    void handleRunMessage( nlohmann::json &msg);

    void deleteOldWaypoints(std::map<int, octomap::point3d> &waypoints);

    std::shared_ptr<ComputedPath> stopUselessThreads(nlohmann::json &waypoints, int i);

    std::shared_ptr<ComputedPath> pathPlanningThread(std::shared_ptr<StoppableThread> thread, nlohmann::json waypoints);
};


#endif //OPPI_WEBSOCKETCLIENT_H
