#include "WebSocketClient.h"
#include <utility>

void WebSocketClient::onPathFoundCallback(std::shared_ptr<ComputedPath> returnPath) {
    // json with time and path (array of nodes with x, y, z coordinates)
    nlohmann::json pathArray;
    std::cout << "Path foundddddd" << std::endl;
    for (const auto &node: *returnPath->path) {
        pathArray.push_back({{"x", node->x},
                             {"y", node->y},
                             {"z", node->z}});
    }
    nlohmann::json jsonArray = {
            {"time", returnPath->time_in_microseconds},
            {"path", pathArray}
    };
    std::string jsonString = jsonArray.dump();
    std::cout << "Sending pathhhhh" << std::endl;
    wsServer_->binarySend(hdl_, "octomap_path", jsonString);
}

WebSocketClient::WebSocketClient(websocketpp::connection_hdl hdl, WebSocketServer *wsServer) : hdl_(hdl),
                                                                                               wsServer_(wsServer) {
    threads_ = std::make_shared<std::vector<partialPathThread>>();
    parameters_ = std::make_shared<RRTStarParameters>();
    rrtStar_ = std::make_shared<RRTStar>();
    rrtStar_->parameters = parameters_;
}

void WebSocketClient::setEnvironment(std::shared_ptr<Environment> env) {
    env_ = std::move(env);
}

void WebSocketClient::setParameters(std::shared_ptr<RRTStarParameters> parameters) {
    parameters_ = std::move(parameters);
}

void WebSocketClient::close() {
    for (auto &thread: *threads_) {
        thread.thread->stopThread();
    }
}

void WebSocketClient::handleParametersMessage(const nlohmann::json &msg) {
    const RRTStarParameters parameters{};

    if (msg.find("threshold") != msg.end()) {
        parameters_->threshold = msg["threshold"];
    }
    if (msg.find("stepLength") != msg.end()) {
        parameters_->stepLength = msg["stepLength"];
    }
    if (msg.find("stayAway") != msg.end()) {
        parameters_->stayAway = msg["stayAway"];
    }
    if (msg.find("bias") != msg.end()) {
        parameters_->bias = msg["bias"];
    }
    if (msg.find("MAX_OPTIMIZING_ITERATIONS") != msg.end()) {
        parameters_->MAX_OPTIMIZING_ITERATIONS = msg["MAX_OPTIMIZING_ITERATIONS"];
    }

    setParameters(parameters_);
}

void WebSocketClient::handleMapMessage(const nlohmann::json &msg) {
    std::string map = msg["map"];
    auto *selectedOctoMap = new octomap::OcTree("../maps/" + map);
    env_ = std::make_shared<Environment>();
    initializeEnvironment(env_, selectedOctoMap, parameters_->stepLength);
}

void WebSocketClient::deleteOldWaypoints(std::map<int, octomap::point3d> &waypointsMap) {
    int debugIndex = 0;
    for (auto it = threads_->begin(); it != threads_->end();) {
        std::cout << "Checking thread" << debugIndex << std::endl;
        debugIndex++;

        if (waypointsMap.find(it->start["id"]) == waypointsMap.end() ||
            waypointsMap.find(it->goal["id"]) == waypointsMap.end()) {
            std::cout << "Stopping thread" << std::endl;

            it->thread->stopThread();
            it = threads_->erase(it);
        } else {
            ++it;
        }
    }
}

std::shared_ptr<ComputedPath> WebSocketClient::stopUselessThreads(nlohmann::json &waypoints, int i) {
    std::shared_ptr<ComputedPath> cp = nullptr;
    int startId = waypoints[i]["id"];
    int goalId = waypoints[i + 1]["id"];
    for (auto it = threads_->begin(); it != threads_->end();) {
        if (it->start["id"] == startId) { // same startId
            if (it->goal["id"] != goalId || it->start != waypoints[i] || it->goal != waypoints[i + 1]) {
                // different goalId or position changed
                it->thread->stopThread();
                it = threads_->erase(it);
            } else { // same start and goal
                cp = it->partialPath;
                ++it;
            }
        } else {
            ++it;
        }
    }
    return cp;
}

std::string WebSocketClient::computedPathToString(std::shared_ptr<ComputedPath> path, bool time, bool calculateCost) {
    nlohmann::json pathArray;
    for (const auto &node: *path->path) {
        pathArray.push_back({{"x", node->x},
                             {"y", node->y},
                             {"z", node->z}});
    }
    nlohmann::json jsonArray = {
            {"path", pathArray}
    };
    if (time) {
        jsonArray["time"] = path->time_in_microseconds;
    }
    if (calculateCost) {
        double cost = 0;
        for (int j = 0; j < path->path->size() - 1; ++j) {
            cost += *(*path->path)[j] - *((*path->path)[j + 1]);
        }
        jsonArray["cost"] = cost;
    } else {
        jsonArray["cost"] = path->cost;
    }
    std::cout << "Computed path: " << jsonArray.dump() << std::endl;
    return jsonArray.dump();
}

std::shared_ptr<ComputedPath> WebSocketClient::pathPlanningThread(std::shared_ptr<StoppableThread> thread,
                                                                  std::vector<octomap::point3d> waypoints) {
    Node start = {waypoints[0].x(), waypoints[0].y(), waypoints[0].z(), nullptr, 0};
    Node goal = {waypoints[1].x(), waypoints[1].y(), waypoints[1].z(), nullptr,
                 std::numeric_limits<double>::max()};

    auto start_time = std::chrono::high_resolution_clock::now();

    std::cout << "Before RRT*" << std::endl;
    // bind function to call when path is found.
    std::shared_ptr<ComputedPath> fRet = rrtStar_->rrtStar(&waypoints[0], &waypoints[1], env_, std::bind(&WebSocketClient::onPathFoundCallback, this,
                                                                        std::placeholders::_1), thread);
    std::cout << "After RRT*" << std::endl;

    if (!fRet->path->empty()) {
        std::cout << "Path found" << std::endl;
        std::string completePathString = computedPathToString(fRet, true);
        std::cout << "Sending path" << std::endl;
        wsServer_->binarySend(hdl_, "octomap_completed", completePathString);

        rrtStar_->pathPruning(fRet->path);
        std::string prunedPathString = computedPathToString(fRet, false);
        wsServer_->binarySend(hdl_, "octomap_optimized_path", prunedPathString);
    } else {
        std::cout << "Path not found" << std::endl;
    }
    // set stop flag
    thread->stopThread();

    // print number of threads not stopped
    int runningThreadsCount = 0;
    for (const auto &ppThread: *threads_) {
        if (!ppThread.thread->isStopRequested()) {
            runningThreadsCount++;
        }
    }
    std::cout << "Running threads: " << runningThreadsCount << std::endl;

    for (auto node: *fRet->path) {
        std::cout << node->x << " " << node->y << " " << node->z << std::endl;
    }
    std::cout << "***************" << std::endl;

    return fRet;
}

void WebSocketClient::handleRunMessage(nlohmann::json &msg) {
    std::map<int, octomap::point3d> waypointsMap;
    for (const auto &waypoint: msg["waypoints"]) {
        waypointsMap[waypoint["id"]] = octomap::point3d(waypoint["coords"]["x"], waypoint["coords"]["y"],
                                                        waypoint["coords"]["z"]);
    }
    // if there are less than 2 waypoints, return
    if (waypointsMap.size() < 2) {
        std::cout << "Not enough waypoints" << std::endl;
        wsServer_->binarySend(hdl_, "error", "Not enough waypoints");
        return;
    }

    // find for threads containing ids not in the new waypoints, stop them and then remove them
    deleteOldWaypoints(waypointsMap);

    std::vector<std::shared_ptr<ComputedPath>> partialPaths(waypointsMap.size() - 1);

// Per ogni coppia di waypoints, eseguire RRT* in diversi thread
    for (int i = 0; i < waypointsMap.size() - 1; ++i) {

        std::shared_ptr<ComputedPath> foundPath = stopUselessThreads(msg["waypoints"], i);
        if (foundPath != nullptr) {
            partialPaths[i] = foundPath;
            continue;
        }

        partialPaths[i] = std::make_shared<ComputedPath>();

        std::cout << "Before thread" << std::endl;
        std::vector<octomap::point3d> waypoints = {waypointsMap[i], waypointsMap[i + 1]};

        auto partialPathPtr = partialPaths[i];

        auto rrtThreadPtr = std::make_shared<StoppableThread>();
        rrtThreadPtr->startThread([this, rrtThreadPtr, waypoints, partialPathPtr]() {
            std::cout << "Thread started" << std::endl;

            std::shared_ptr<ComputedPath> computedPath = std::move(pathPlanningThread(rrtThreadPtr, waypoints));

            std::cout << "Computed" << std::endl;
            for (const auto &node: *computedPath->path) {
                std::cout << node->x << " " << node->y << " " << node->z << std::endl;
            }
            std::cout << "------------" << std::endl;

            // Salvare il percorso calcolato in partialPathPtr
            partialPathPtr->path = computedPath->path;
            partialPathPtr->cost = computedPath->cost;
            partialPathPtr->time_in_microseconds = computedPath->time_in_microseconds;
            std::cout << "Thread finished" << std::endl;
        });
        std::cout << "After thread" << std::endl;
        threads_->emplace_back(partialPathThread{msg["waypoints"][i], msg["waypoints"][i + 1], rrtThreadPtr});
    }

    // wait for all threads to finish
    for (auto &thread: *threads_) {
        thread.thread->join();
    }

    std::cout << "All threads finished" << std::endl;

    // concatenate all partial paths (create a new ComputedPath object that contains all points always avoid adding last point except for last path)
    std::shared_ptr<ComputedPath> fRet = std::make_shared<ComputedPath>();
    fRet->path = std::make_shared<std::vector<Node *>>();

    std::cout << "Concatenating paths" << std::endl;

    for (int i = 0; i < partialPaths.size(); ++i) {
        for (int j = 0; j < partialPaths[i]->path->size(); ++j) {
            std::cout << "Adding point " << j << " from path " << i << std::endl;
            std::cout << (*partialPaths[i]->path)[j]->x << " " << (*partialPaths[i]->path)[j]->y << " "
                      << (*partialPaths[i]->path)[j]->z << std::endl;
            fRet->path->push_back((*partialPaths[i]->path)[j]);
        }
        if (i != partialPaths.size() - 1) {
            fRet->path->pop_back();
        }
    }

    std::cout << "Path smoothing" << std::endl;
    rrtStar_->pathSmoothing(fRet->path, 0.5, 10);
    std::string jsonString = computedPathToString(fRet, true, true);
    wsServer_->binarySend(hdl_, "octomap_smoothed_path", jsonString);
}

void WebSocketClient::handleMessage(const websocketpp::server<websocketpp::config::asio>::message_ptr msg) {
    nlohmann::json json = nlohmann::json::parse(msg->get_payload());
    std::cout << "Received message: " << json["topic"] << std::endl;
    switch (json["topic"].get<std::string>()[0]) {
        case 'p': {
            // Parameters
            std::cout << "Received setParameters command" << std::endl;
            handleParametersMessage(json);
            break;
        }
        case 'm': {
            // Map
            std::cout << "Received setMap command" << std::endl;
            handleMapMessage(json);
            break;
        }
        case 'r': {
            // Run
            std::cout << "Received run command" << std::endl;
            handleRunMessage(json);
        }
        default:
            break;
    }
}
