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
            {"start", returnPath->start},
            {"goal", returnPath->goal},
            {"cost", returnPath->cost},
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
            std::cout << "Stopping thread, " << threads_->size() << std::endl;

            it->thread->stopThread();
            it->toDelete = true;
            it->thread->join();
            it = threads_->erase(it);
            std::cout << "Thread stopped and erased, " << threads_->size() << " remaining" << std::endl;
        } else {
            ++it;
        }
    }
}

std::shared_ptr<ComputedPath> WebSocketClient::stopUselessThreads(nlohmann::json &waypoints, int i) {
    std::shared_ptr<ComputedPath> cp = nullptr;
    int startId = waypoints[i]["id"];
    int goalId = waypoints[i + 1]["id"];
    bool toDelete = false;

    for (auto it = threads_->begin(); it != threads_->end();) {
        if (it->start["id"] == startId) { // same startId
            std::cout << "Same startId" << std::endl;
            if (it->goal["id"] != goalId || it->start != waypoints[i] || it->goal != waypoints[i + 1]) {
                toDelete = true;
                // different goalId or position changed
                std::cout << "Different goalId or position changed" << std::endl;
                it->thread->stopThread();
                it->toDelete = true;
                std::cout << "Thread stopped" << std::endl;
            } else { // same start and goal
                std::cout << "Same start and goal" << std::endl;
                cp = std::make_shared<ComputedPath>(*it->partialPath);
                std::cout << "Path found in thread with size: " << cp->path->size() << std::endl;
            }
            ++it;
        } else {
            std::cout << "Different startId" << std::endl;
            ++it;
        }
    }
    if (toDelete) {
        std::cout << "Erasing threads" << std::endl;
        try {
            for (auto it = threads_->begin(); it != threads_->end();) {
                if (it->toDelete) {
                    std::cout << "Trying to erase thread, " << threads_->size() << std::endl;
                    while (it->thread->isJoined() != nullptr && *(it->thread->isJoined()) == false) {}
                    std::cout << "Thread is joined: " << it->start << std::endl;
                    it->thread->join();
                    it = threads_->erase(it);
                    std::cout << "Thread erased, " << threads_->size() << " remaining" << std::endl;
                } else {
                    ++it;
                }
            }
            std::cout << "Threads erased" << std::endl;
        } catch (std::exception &e) {
            std::cout << "Exception: " << e.what() << std::endl;
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
            {"start", path->start},
            {"goal", path->goal},
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
    return jsonArray.dump();
}

std::shared_ptr<ComputedPath> WebSocketClient::pathPlanningThread(std::shared_ptr<StoppableThread> thread,
                                                                  nlohmann::json waypoints) {

    auto start_time = std::chrono::high_resolution_clock::now();

    // bind function to call when path is found.
    std::shared_ptr<ComputedPath> fRet = rrtStar_->rrtStar(waypoints, env_,
                                                           std::bind(&WebSocketClient::onPathFoundCallback, this,
                                                                     std::placeholders::_1), thread);

    if (!fRet->path->empty()) {
        std::string completePathString = computedPathToString(fRet, true);
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
    std::cout << "***************" << std::endl;

    return fRet;
}

void WebSocketClient::handleRunMessage(nlohmann::json &msg) {
    std::map<int, octomap::point3d> waypointsMap = std::map<int, octomap::point3d>();
    std::vector<int> waypointsIds;
    for (const auto &waypoint: msg["waypoints"]) {
        waypointsIds.push_back(waypoint["id"]);
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

    for (int i = 0; i < waypointsMap.size() - 1; ++i) {
        std::shared_ptr<ComputedPath> foundPath = stopUselessThreads(msg["waypoints"], i);
        if (foundPath != nullptr) {
            partialPaths[i] = std::move(foundPath);
        } else {
            partialPaths[i] = nullptr;
        }
    }
    std::cout << "n Waypoints: " << waypointsMap.size() << std::endl;
// Per ogni coppia di waypoints, eseguire RRT* in diversi thread
    for (int i = 0; i < waypointsMap.size() - 1; ++i) {
        std::cout << "Checking waypoint " << i << " of" << waypointsMap.size() << std::endl;
        if (partialPaths[i] != nullptr) {
            continue;
        }
        std::cout << "A:" << waypointsMap.size() << std::endl;
        partialPaths[i] = std::make_shared<ComputedPath>();

//        std::vector<octomap::point3d> waypoints = {waypointsMap[waypointsIds[i]], waypointsMap[waypointsIds[i + 1]]};
        nlohmann::json waypoints;
        waypoints.push_back(msg["waypoints"][i]);
        waypoints.push_back(msg["waypoints"][i + 1]);

        auto partialPathPtr = partialPaths[i];
        auto rrtThreadPtr = std::make_shared<StoppableThread>();
        rrtThreadPtr->startThread([this, rrtThreadPtr, waypoints, partialPathPtr, msg, i]() {

            std::shared_ptr<ComputedPath> computedPath = std::move(pathPlanningThread(rrtThreadPtr, waypoints));

            // find thread in threads_ and update the path. Search by start and goal
            for (auto &thread: *threads_) {
                if (thread.start == msg["waypoints"][i] && thread.goal == msg["waypoints"][i + 1]) {
                    thread.partialPath = computedPath;
                    break;
                }
            }

            *partialPathPtr = *computedPath;
        });

        threads_->emplace_back(partialPathThread{msg["waypoints"][i], msg["waypoints"][i + 1], rrtThreadPtr});
    }


    std::thread smoothThread(
            [this, partialPaths]() {
                // create a vector with all current threads
                std::vector<std::shared_ptr<StoppableThread>> threadsPtrs;
                for (auto &thread: *threads_) {
                    if (!thread.partialPath) {
                        threadsPtrs.push_back(thread.thread);
                    }
                }
                std::cout << "Threads pointers copied" << std::endl;
                // wait for all threads to finish
                bool stillAlive = true;
                for (auto &ppt: threadsPtrs) {
                    std::cout << "Joining thread" << std::endl;
                    try {
                        ppt->join();
                        std::cout << "Thread joined" << std::endl;
                        // if threads_ have still contains the thread (thread == ppt), then is still alive
                        if (std::find_if(threads_->begin(), threads_->end(),
                                         [ppt](const partialPathThread &ppt2) { return ppt2.thread == ppt; }) ==
                            threads_->end()) {
                            stillAlive = false;
                        }
                    } catch (std::exception &e) {
                        std::cout << "Exception: " << e.what() << std::endl;
                        stillAlive = false;
                    }

                }
                if (!stillAlive) {
                    std::cout << "Partial paths not ready" << std::endl;
                    return;
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
                std::cout << std::endl << std::endl << "-------------------------------" << std::endl << std::endl;

            }
    );
    std::cout << "Smooth thread started" << std::endl;
    smoothThread.detach();
    std::cout << "Smooth thread detached" << std::endl;
    waypointsMap.clear();
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
