#include "WebSocketServer.h"
#include "RRT_star.h"
#include <nlohmann/json.hpp>
#include "base64.h"

WebSocketServer wsServer;
std::shared_ptr<Environment> env = std::make_shared<Environment>();
octomap::OcTree *tree = new octomap::OcTree("../octomap.bt");
std::map<void *, std::vector<std::shared_ptr<StoppableThread>> *> runningThreads;
std::map<void *, std::map<std::string, double>> clientsPreferences;
std::map<void *, std::shared_ptr<Environment>> clientEnvironments;


void onPathFoundCollback(ReturnPath *returnPath, websocketpp::connection_hdl hdl) {
    // json with time and path (array of nodes with x, y, z coordinates)
    nlohmann::json pathArray;
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
    wsServer.binarySend(hdl, "octomap_path", jsonString);
    return;
}

void onCloseCallback(websocketpp::connection_hdl hdl) {
    std::cout << "Connection closed" << std::endl;
    if (runningThreads.find(hdl.lock().get()) != runningThreads.end()) {
        for (auto &thread: *runningThreads[hdl.lock().get()]) {
            thread->stopThread();
        }
        runningThreads.erase(hdl.lock().get());
    }
    if (clientsPreferences.find(hdl.lock().get()) != clientsPreferences.end()) {
        clientsPreferences.erase(hdl.lock().get());
    }
    if (clientEnvironments.find(hdl.lock().get()) != clientEnvironments.end()) {
        clientEnvironments.erase(hdl.lock().get());
    }
}

void onMessageCallback(websocketpp::connection_hdl hdl,
                       const websocketpp::server<websocketpp::config::asio>::message_ptr msg) {
    const nlohmann::json json = nlohmann::json::parse(msg->get_payload());
    std::cout << "Received message: " << json["topic"] << std::endl;
    switch (json["topic"].get<std::string>()[0]) {
        case 'p': {
            // Preferences
            std::cout << "Received preferences" << std::endl;
            clientsPreferences[hdl.lock().get()] = json["preferences"];
            break;
        }
        case 'm': {
            // Map
            std::string map = json["map"];
            std::cout << "Received map: " << map << std::endl;

            auto *selectedOctoMap = new octomap::OcTree("../maps/" + map);
            clientEnvironments[hdl.lock().get()] = std::make_shared<Environment>();
            double stepLength;
            if (clientsPreferences.find(hdl.lock().get()) != clientsPreferences.end() &&
                clientsPreferences[hdl.lock().get()].find("stepLength") != clientsPreferences[hdl.lock().get()].end()) {
                stepLength = clientsPreferences[hdl.lock().get()]["stepLength"];
            } else {
                stepLength = 1.0;
            }
            initializeEnvironment(clientEnvironments[hdl.lock().get()], selectedOctoMap, stepLength);
            break;
        }
        case 's': {
            // Stop
            if (runningThreads.find(hdl.lock().get()) != runningThreads.end()) {
                for (auto &thread: *runningThreads[hdl.lock().get()]) {
                    thread->stopThread();
                }
                runningThreads.erase(hdl.lock().get());
            }
            break;
        }
        case 'r': {
            // Run
            std::cout << "Received run command" << std::endl;
            std::vector<octomap::point3d> waypoints;
            for (const auto &waypoint: json["waypoints"]) {
                // waypoints have this structure [{id:0,coords:{x:"",y:"",z:""},color:"#ff0000"}, {id:1,coords:{x:"",y:"",z:""},color:"#0000ff"}]
                waypoints.emplace_back(octomap::point3d(waypoint["coords"]["x"], waypoint["coords"]["y"],
                                                     waypoint["coords"]["z"]));
            }
            if (waypoints.size() < 2) {
                std::cout << "Not enough waypoints" << std::endl;
                break;
            }
            // for each pair of waypoints, run RRT* in different threads
            for (int i = 0; i < waypoints.size() - 1; ++i) {
                auto rrtThreadPtr = std::make_shared<StoppableThread>();
                rrtThreadPtr->startThread([hdl, rrtThreadPtr, i, waypoints]() {
                    Node start = {waypoints[i].x(), waypoints[i].y(), waypoints[i].z(), nullptr, 0};
                    Node goal = {waypoints[i + 1].x(), waypoints[i + 1].y(), waypoints[i + 1].z(), nullptr,
                                 std::numeric_limits<double>::max()};
                    RRTStar rrt_star;
                    // set preferences
                    if (clientsPreferences.find(hdl.lock().get()) != clientsPreferences.end()) {
                        if (clientsPreferences[hdl.lock().get()].find("MAX_OPTIMIZING_ITERATIONS") !=
                            clientsPreferences[hdl.lock().get()].end()) {
                            rrt_star.MAX_OPTIMIZING_ITERATIONS = static_cast<int>(
                                    clientsPreferences[hdl.lock().get()]["MAX_OPTIMIZING_ITERATIONS"]);
                        }
                        if (clientsPreferences[hdl.lock().get()].find("threshold") !=
                            clientsPreferences[hdl.lock().get()].end()) {
                            rrt_star.threshold = clientsPreferences[hdl.lock().get()]["threshold"];
                        }
                        if (clientsPreferences[hdl.lock().get()].find("stepLength") !=
                            clientsPreferences[hdl.lock().get()].end()) {
                            rrt_star.stepLength = clientsPreferences[hdl.lock().get()]["stepLength"];
                        }
                        if (clientsPreferences[hdl.lock().get()].find("stayAway") !=
                            clientsPreferences[hdl.lock().get()].end()) {
                            rrt_star.stayAway = clientsPreferences[hdl.lock().get()]["stayAway"];
                        }
                        if (clientsPreferences[hdl.lock().get()].find("bias") !=
                            clientsPreferences[hdl.lock().get()].end()) {
                            rrt_star.bias = clientsPreferences[hdl.lock().get()]["bias"];
                        }
                    }
                    auto start_time = std::chrono::high_resolution_clock::now();
                    FinalReturn fRet = rrt_star.rrtStar(&start, &goal, *clientEnvironments[hdl.lock().get()],
                                                        onPathFoundCollback, hdl, rrtThreadPtr);
                    if (!fRet.path->empty()) {
                        std::cout << "Final path found in " << fRet.time_in_microseconds << " microseconds"
                                  << std::endl;
                        std::cout << "Completed with " << fRet.num_nodes << " nodes" << std::endl;
                        std::cout << "Cost = " << fRet.cost << std::endl;
                        nlohmann::json completePath;
                        for (const auto &node: *fRet.path) {
                            completePath.push_back({{"x", node->x},
                                                    {"y", node->y},
                                                    {"z", node->z}});
                        }
                        nlohmann::json completeJson = {
                                {"path",      completePath},
                                {"time",      fRet.time_in_microseconds},
                                {"num_nodes", fRet.num_nodes},
                                {"cost",      fRet.cost}
                        };
                        wsServer.binarySend(hdl, "octomap_completed", completeJson.dump());

                        rrt_star.pathPruning(fRet.path);
                        std::cout << "Optimizing path with " << fRet.path->size() << " nodes" << std::endl;
                        double newCost = 0;
                        for (int i = 0; i < fRet.path->size() - 1; ++i) {
                            newCost += *(*fRet.path)[i] - *((*fRet.path)[i + 1]);
                        }
                        std::cout << "Optimizing path with cost:" << newCost << std::endl;
                        nlohmann::json optimizedPath;
                        for (const auto &node: *fRet.path) {
                            optimizedPath.push_back({{"x", node->x},
                                                     {"y", node->y},
                                                     {"z", node->z}});
                        }
                        nlohmann::json optArray = {{"cost", newCost},
                                                   {"path", optimizedPath}};
                        std::string jsonString = optArray.dump();
                        wsServer.binarySend(hdl, "octomap_optimized_path", jsonString);

                        std::cout << "Path smoothing" << std::endl;
                        optimizedPath.clear();
                        rrt_star.pathSmoothing(fRet.path, 0.5, 10);
                        std::cout << "Smoothed path with " << fRet.path->size() << " nodes" << std::endl;
                        for (const auto &node: *fRet.path) {
                            optimizedPath.push_back({{"x", node->x},
                                                     {"y", node->y},
                                                     {"z", node->z}});
                        }
                        newCost = 0;
                        for (int i = 0; i < fRet.path->size() - 1; ++i) {
                            newCost += *(*fRet.path)[i] - *((*fRet.path)[i + 1]);
                        }
                        optArray = {{"cost", newCost},
                                    {"path", optimizedPath}};
                        jsonString = optArray.dump();
                        wsServer.binarySend(hdl, "octomap_smoothed_path", jsonString);
                        std::cout << "Finish time " << std::chrono::duration_cast<std::chrono::microseconds>(
                                std::chrono::high_resolution_clock::now() - start_time).count() << "microseconds"
                                  << std::endl;
                    } else {
                        std::cout << "Path not found" << std::endl;
                    }
                });
                rrtThreadPtr->detach();
                runningThreads[hdl.lock().get()]->push_back(rrtThreadPtr);
            }
        }
        default:
            break;
    }

}

void onOpenCallback(websocketpp::connection_hdl hdl) {
    runningThreads[hdl.lock().get()] = new std::vector<std::shared_ptr<StoppableThread>>();

    // TODO CHANGE THIS TO USE THE NEW ENVIRONMENT
    clientEnvironments[hdl.lock().get()] = env;
    std::stringstream buffer;
    env->tree->writeBinaryData(buffer);
    std::string str = buffer.str();
    wsServer.binarySend(hdl, "octomap", str);
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
    wsServer.setOnMessageCallback(
            [](websocketpp::connection_hdl hdl,
               const websocketpp::server<websocketpp::config::asio>::message_ptr msg) {
                onMessageCallback(hdl, msg);
            });
    wsServer.setOnCloseCallback([](websocketpp::connection_hdl hdl) {
        onCloseCallback(hdl);
    });
    wsServer.start();
    WebSocketServer::join();
    return 0;
}