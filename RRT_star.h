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
std::vector<Node *> rrtStar(Node *start, Node *goal, int width, int height);
std::vector<Node *> rrtStar(Node *start, Node *goal, octomap::OcTree *tree,void (*pathFoundCallback)(std::vector<Node*>*,websocketpp::connection_hdl) = nullptr,websocketpp::connection_hdl hdl = {}, const std::shared_ptr<StoppableThread>& stopReq = nullptr);

#endif //RRT_STAR_RRT_STAR_H
