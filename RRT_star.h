#ifndef RRT_STAR_RRT_STAR_H
#define RRT_STAR_RRT_STAR_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>
#include <random>
#include <chrono>

#include "Environment.h"
#include "WebSocketServer.h"

struct Node {
    double x{}, y{}, z{};
    Node *parent{};
    double cost{};
    std::vector<Node *> children;

    double operator-(const Node &node) const {
        double dx = x - node.x;
        double dy = y - node.y;
        double dz = z - node.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }
};

struct ReturnPath{
    std::vector<Node *>* path;
    long time_in_microseconds;
};

struct FinalReturn{
    std::shared_ptr<std::vector<Node *>> path;
    long time_in_microseconds;
    size_t num_nodes;
    double cost;
};
std::vector<Node *> rrtStar(Node *start, Node *goal, int width, int height);
FinalReturn rrtStar(Node *start, Node *goal, std::shared_ptr<octomap::OcTree> &tree,double stayAwayDesired,void (*pathFoundCallback)(ReturnPath*,websocketpp::connection_hdl) = nullptr,websocketpp::connection_hdl hdl = {}, const std::shared_ptr<StoppableThread>& stopReq = nullptr);

bool checkMultipleRayCollision(Node *node1, Node *node2, std::shared_ptr<octomap::OcTree> &octree);
bool checkLinkCollisionWithDistmap(Node *node1, Node *node2, std::shared_ptr<DynamicEDTOctomap> &distmap);

#endif //RRT_STAR_RRT_STAR_H
