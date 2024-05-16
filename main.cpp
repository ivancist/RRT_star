#include "WebSocketServer.h"
#include "RRT_star.h"
#include <nlohmann/json.hpp>

WebSocketServer wsServer;
std::shared_ptr<Environment> env = std::make_shared<Environment>();
octomap::OcTree* tree = new octomap::OcTree("../octomap.bt");


void myCallback(ReturnPath *returnPath, websocketpp::connection_hdl hdl) {
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

void onOpenCallback(websocketpp::connection_hdl hdl) {
    auto rrtThreadPtr = std::make_shared<StoppableThread>();
    rrtThreadPtr->startThread([hdl, rrtThreadPtr]() {
//        std::shared_ptr tree = std::make_shared<octomap::OcTree>("../octomap.bt");
        std::stringstream buffer;
        env->tree->writeBinaryData(buffer);
        std::string str = buffer.str();
        wsServer.binarySend(hdl, "octomap", str);
//        Node start{4, -5, 1.2, nullptr, 0};
//        Node goal{4, 4, 1.2, nullptr, std::numeric_limits<double>::max()};
        Node start{-3, 3, 1, nullptr, 0};
        Node goal{15, 1, 2, nullptr, std::numeric_limits<double>::max()};
        nlohmann::json endpointsJson = {
                {"start", {{"x", start.x}, {"y", start.y}, {"z", start.z}, {"color", "red"}}},
                {"goal",  {{"x", goal.x},  {"y", goal.y},  {"z", goal.z},  {"color", "green"}}}
        };
        wsServer.binarySend(hdl, "octomap_endpoints", endpointsJson.dump());

        RRTStar rrt_star;

        FinalReturn fRet = rrt_star.rrtStar(&start, &goal, *env, .6, myCallback, hdl, rrtThreadPtr);
//        FinalReturn fRet = rrtStar(&start, &goal, "../octomap.bt", myCallback, hdl, rrtThreadPtr);
        if (!fRet.path->empty()) {
            std::cout << "Final path found in " << fRet.time_in_microseconds << " microseconds" << std::endl;
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

            std::size_t numNodes = fRet.path->size();
            for (int i = 0; i < numNodes - 2; ++i) {
                octomap::point3d startNode((*fRet.path)[i]->x, (*fRet.path)[i]->y, (*fRet.path)[i]->z);
//                for (int j = i+2; j < numNodes; ++j) {
                for (int j = numNodes - 1; j > i + 1; --j) {
                    octomap::point3d goalNode((*fRet.path)[j]->x, (*fRet.path)[j]->y, (*fRet.path)[j]->z);
                    octomap::point3d direction = goalNode - startNode;
                    octomap::point3d temp;
                    double distance = startNode.distance(goalNode);
//                    if (!tree->castRay(startNode, direction, temp, true, distance)) {
                    if (!rrt_star.checkLinkCollisionWithDistMap((*fRet.path)[i], (*fRet.path)[j])) {
                        for (int k = i + 1; k < j; ++k) {
                            delete (*fRet.path)[k];
                        }
                        fRet.path->erase(fRet.path->begin() + i + 1, fRet.path->begin() + j);
                        numNodes = fRet.path->size();
                        break;
                    }
                }
            }
            std::cout << "Optimizing path with " << numNodes << " nodes" << std::endl;
            double newCost = 0;
            for (int i = 0; i < numNodes - 1; ++i) {
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
        } else {
            std::cout << "Path not found" << std::endl;
        }
    });
    rrtThreadPtr->detach();
    wsServer.runningThreads[hdl.lock().get()] = std::move(rrtThreadPtr);
}


int main() {
    auto start_env = std::chrono::high_resolution_clock::now();
    initializeEnvironment(env, tree, 1.0); // 1.0 is the maxDist = stepLength
    std::cout << "Time for environment initialization: "
              << std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::high_resolution_clock::now() - start_env).count() << "ms" << std::endl;
    wsServer.setOnOpenCallback([](websocketpp::connection_hdl hdl) {
        onOpenCallback(hdl); // Call your original onOpenCallback function
    });
    wsServer.start();
    WebSocketServer::join();
    return 0;
}