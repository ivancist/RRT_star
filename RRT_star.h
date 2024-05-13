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

struct ReturnPath {
    std::vector<Node *> *path;
    long time_in_microseconds;
};

struct FinalReturn {
    std::shared_ptr<std::vector<Node *>> path;
    long time_in_microseconds;
    size_t num_nodes;
    double cost;
};

class RRTStar {
public:
    int MAX_OPTIMIZING_ITERATIONS = 1 * 1000;
    const int waitBeforeClosing = 3 * 1000;
    double threshold = 2; // with distmap: threshold <= stayAway
    double stepLength = 1; // stepLength <= threshold
    double stayAway = .6;
    double bias = 0.2;
    const int refreshView = 1000;
    int searchAtDepth = 0;
    std::shared_ptr<Environment> env;

    void getDirection(Node *node1, Node *node2, Node *direction);

    void getVersor(Node *node1, Node *node2, Node *versor);

    bool checkMultipleRayCollision(Node *node1, Node *node2);

    bool checkLinkCollisionWithDistMap(Node *node1, Node *node2);

    std::vector<Node *> rrtStar(Node *start, Node *goal, std::shared_ptr<Environment> &env);

    FinalReturn rrtStar(Node *start, Node *goal, std::shared_ptr<Environment> &environment, double stayAwayDesired,
                        void (*pathFoundCallback)(ReturnPath *, websocketpp::connection_hdl),
                        websocketpp::connection_hdl hdl,
                        const std::shared_ptr<StoppableThread> &stoppableThreadPtr);

private:
    double distance(Node *node1, Node *node2);

    double distance(Node *node, double x, double y);

    double randomDouble(double min, double max);

    Node *sampleRandomNode();

    Node *sampleRandomNode(Node *goal);

    Node *nearestNodeInTree(std::vector<Node *> &tree, Node *randomNode);

    std::vector<Node *> nearestNodesInTree(std::vector<Node *> &tree, Node *newNode);

    bool checkCollision(Node *node);

    bool checkCollision(Node *node1, Node *node2);

    bool checkRayCollision(Node *node1, Node *node2);

    void calcolaVerticiQuadrato(octomap::point3d *centro, octomap::point3d *versore, octomap::point3d vertici[],
                                double lato);

    void recalculateCostOfChildren(Node *node, double delta);

    Node *extendTree(Node *nearestNode, Node *randomNode, std::vector<Node *> &tree);

    Node *extend3DTree(Node *nearestNode, Node *randomNode, std::vector<Node *> &tree);

    void connectToGoal(Node *lastNode, Node *goal);

    std::vector<Node *> getPath(Node *goal);

    void visualize(const std::vector<Node *> &tree, Node *goal, bool finished = false);

};

std::vector<Node *> rrtStar(Node *start, Node *goal, int width, int height);

FinalReturn rrtStar(Node *start, Node *goal, std::shared_ptr<Environment> &env, double stayAwayDesired,
                    void (*pathFoundCallback)(ReturnPath *, websocketpp::connection_hdl) = nullptr,
                    websocketpp::connection_hdl hdl = {}, const std::shared_ptr<StoppableThread> &stopReq = nullptr);

bool checkMultipleRayCollision(Node *node1, Node *node2, std::shared_ptr<octomap::OcTree> &octree);

bool checkLinkCollisionWithDistmap(Node *node1, Node *node2, std::shared_ptr<DynamicEDTOctomap> &distmap);

#endif //RRT_STAR_RRT_STAR_H
