#include "RRT_star.h"

const int MAX_OPTIMIZING_ITERATIONS = 20 * 1000;
const int waitBeforeClosing = 3 * 1000;
//const int threshold = 30;
//double stepLength = 20;
//const double stayAway = 10;
const double threshold = .5;
double stepLength = .3;
const double stayAway = 1;
const double bias = 0.1;
const int refreshView = 5000;
Environment env;

double distance(Node *node1, Node *node2) {
    double dx = node1->x - node2->x;
    double dy = node1->y - node2->y;
    double dz = node1->z - node2->z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double distance(Node *node, double x, double y) {
    double dx = node->x - x;
    double dy = node->y - y;
    return std::sqrt(dx * dx + dy * dy);
}

void getDirection(Node *node1, Node *node2, Node *direction) {
    // Calculate the direction from the nearest node to the random node
    direction->x = node1->x - node2->x;
    direction->y = node1->y - node2->y;
    direction->z = node1->z - node2->z;
}

void getVersor(Node *node1, Node *node2, Node *versor) {
    double dx = node2->x - node1->x;
    double dy = node2->y - node1->y;
    double dz = node2->z - node1->z;
    double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

    // Normalize the direction vector
    versor->x = dx / dist;
    versor->y = dy / dist;
    versor->z = dz / dist;
}

// Function to generate a random double within a given range
double randomDouble(double min, double max) {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(min, max);
    return dis(gen);
}

// TODO Other sampling methods
Node *sampleRandomNode() {
    // Generate random x and y within the environment bounds
    double x = randomDouble(-5, env.width);
    double y = randomDouble(0, env.height);
    double z = randomDouble(-5, env.depth);

    // Create a node with the random coordinates
    return new Node{x, y, z};
}

Node *sampleRandomNode(Node *goal) {
    if (randomDouble(0, 1) < bias) {
        return new Node{goal->x, goal->y, goal->z};
    } else {
        return sampleRandomNode();
    }
}

Node *nearestNodeInTree(std::vector<Node *> &tree, Node *randomNode) {
    Node *nearestNode = nullptr;
    double minDistance = std::numeric_limits<double>::max();

    for (Node *node: tree) {
        double dist = distance(node, randomNode);
        if (dist < minDistance) {
            minDistance = dist;
            nearestNode = node;
        }
    }
    return nearestNode;
}

std::vector<Node *> nearestNodesInTree(std::vector<Node *> &tree, Node *newNode) {
    std::vector<Node *> nearestNodes;
    size_t nNodeIndex;
    double minCost = newNode->cost;

    for (Node *node: tree) {
        double dist = distance(node, newNode);
        if (dist <= threshold) {
            double totalCost = node->cost + dist;
            nearestNodes.push_back(node);
            if (totalCost < minCost) {
                minCost = totalCost;
                nNodeIndex = nearestNodes.size() - 1;
            }
        }
    }

    if (nearestNodes.empty()) {
        return nearestNodes;
    }
    std::rotate(nearestNodes.begin(), nearestNodes.begin() + static_cast<int>(nNodeIndex), nearestNodes.end());
    return nearestNodes;
}

bool checkCollision(Node *node) {
    return std::any_of(env.obstacles.begin(), env.obstacles.end(), [&](const Obstacle &obstacle) {
        if (obstacle.type == Obstacle::CIRCLE) {
            double dist = distance(node, obstacle.circle.x, obstacle.circle.y);
            return dist <= obstacle.circle.radius + stayAway;
        } else {
            return node->x >= obstacle.rectangle.x - stayAway &&
                   node->x <= obstacle.rectangle.x + obstacle.rectangle.width + stayAway &&
                   node->y >= obstacle.rectangle.y - stayAway &&
                   node->y <= obstacle.rectangle.y + obstacle.rectangle.height + stayAway;
        }
    });
}

bool checkCollision(Node *node1, Node *node2) {
    double dx = node2->x - node1->x;
    double dy = node2->y - node1->y;
    double dist = std::sqrt(dx * dx + dy * dy);

    // Normalize the direction vector
    double dirX = dx / dist;
    double dirY = dy / dist;

    // Check for collisions along the line between the two nodes
    for (int i = 0; i < dist; i++) {
        Node node{node1->x + dirX * i, node1->y + dirY * i};
        Node *nodePtr = &node;
        if (checkCollision(nodePtr)) {
            return true;
        }
    }

    return false;
}

// TODO: Optimize
void recalculateCostOfChildren(Node *node, double delta) {
    for (Node *child: node->children) {
        child->cost += delta;
        recalculateCostOfChildren(child, delta);
    }
}

Node *extendTree(Node *nearestNode, Node *randomNode, std::vector<Node *> &tree) {
    // Calculate the direction from the nearest node to the random node
    double dx = randomNode->x - nearestNode->x;
    double dy = randomNode->y - nearestNode->y;
    double dz = randomNode->z - nearestNode->z;
    double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

    // Normalize the direction vector
    double dirX = dx / dist;
    double dirY = dy / dist;
    double dirZ = dz / dist;

    // Calculate the new node's position by moving along the direction vector by the step length
    double newX = nearestNode->x + dirX * stepLength;
    double newY = nearestNode->y + dirY * stepLength;
    double newZ = nearestNode->z + dirZ * stepLength;

    // Calculate the cost of the new node
//    double newCost = nearestNode->cost + distance(nearestNode, new Node{newX, newY, nullptr});

    Node tempNode{newX, newY, newZ, nullptr, std::numeric_limits<double>::max()};
    if (checkCollision(&tempNode)) {
        return nullptr; // Collision detected, do not extend the tree
    }

    std::vector<Node *> nearestNodes = nearestNodesInTree(tree, &tempNode);
    while (checkCollision(nearestNodes[0], &tempNode)) {
        nearestNodes.erase(nearestNodes.begin());
        if (nearestNodes.empty()) {
            return nullptr;
        }
    }
    Node *parentNode = nearestNodes[0];

    double newNodeCost = parentNode->cost + distance(parentNode, &tempNode);
    Node *newNode = new Node{newX, newY, newZ, parentNode, newNodeCost};
    parentNode->children.push_back(newNode);

    for (size_t i = 1; i < nearestNodes.size(); i++) {
        Node *node = nearestNodes[i];
        double distanceToNewNode = distance(node, &tempNode);
        if (newNodeCost + distanceToNewNode < node->cost && !checkCollision(node, &tempNode)) {
            Node *parent = node->parent;
            parent->children.erase(std::remove(parent->children.begin(), parent->children.end(), node),
                                   parent->children.end());
            node->parent = newNode;
            double oldCost = node->cost;
            node->cost = newNodeCost + distanceToNewNode;
            newNode->children.push_back(node);
            recalculateCostOfChildren(node, node->cost - oldCost);
        }
    }

    return newNode;
}

Node *extendTree(Node *nearestNode, Node *randomNode, std::vector<Node *> &tree, octomap::OcTree *octree) {
    Node versor{};
    getVersor(nearestNode, randomNode, &versor);

    // Calculate the new node's position by moving along the direction vector by the step length
    double newX = nearestNode->x + versor.x * stepLength;
    double newY = nearestNode->y + versor.y * stepLength;
    double newZ = nearestNode->z + versor.z * stepLength;

    // Calculate the cost of the new node
//    double newCost = nearestNode->cost + distance(nearestNode, new Node{newX, newY, nullptr});

    Node tempNode{newX, newY, newZ, nullptr, std::numeric_limits<double>::max()};
    if (octree->search(tempNode.x, tempNode.y, tempNode.z, 15) != nullptr) {
        return nullptr; // Collision detected, do not extend the tree
    }

    std::vector<Node *> nearestNodes = nearestNodesInTree(tree, &tempNode);
    bool collision = true;
    while (!nearestNodes.empty() && collision) {
        octomap::point3d tempPoint = octomap::point3d(tempNode.x, tempNode.y, tempNode.z);
        getDirection(nearestNodes[0], &tempNode, &versor);
        if (versor.x == 0 && versor.y == 0 && versor.z == 0) {
            return nullptr;
        }
        collision = octree->castRay(octomap::point3d(nearestNodes[0]->x, nearestNodes[0]->y, nearestNodes[0]->z),
                                    octomap::point3d(versor.x, versor.y, versor.z),
                                    tempPoint, true, threshold);
        if (collision) {
            nearestNodes.erase(nearestNodes.begin());
        }
    }
    if (collision) {
        return nullptr;
    }

    Node *parentNode = nearestNodes[0];

    double newNodeCost = parentNode->cost + distance(parentNode, &tempNode);
    Node *newNode = new Node{newX, newY, newZ, parentNode, newNodeCost};
    parentNode->children.push_back(newNode);

    for (size_t i = 1; i < nearestNodes.size(); i++) {
        Node *node = nearestNodes[i];
        double distanceToNewNode = distance(node, &tempNode);
        getDirection(node, &tempNode, &versor);
        octomap::point3d tempPoint = octomap::point3d(tempNode.x, tempNode.y, tempNode.z);
        if (newNodeCost + distanceToNewNode < node->cost &&
            !octree->castRay(octomap::point3d(node->x, node->y, node->z),
                             octomap::point3d(versor.x, versor.y, versor.z),
                             tempPoint, true, threshold)) {
            Node *parent = node->parent;
            parent->children.erase(std::remove(parent->children.begin(), parent->children.end(), node),
                                   parent->children.end());
            node->parent = newNode;
            double oldCost = node->cost;
            node->cost = newNodeCost + distanceToNewNode;
            newNode->children.push_back(node);
            recalculateCostOfChildren(node, node->cost - oldCost);
        }
    }
    return newNode;
}

void connectToGoal(Node *lastNode, Node *goal) {
    // Calculate the cost of the goal node
    double goalCost = lastNode->cost + distance(lastNode, goal);

    // Update the goal node to point to the new goal node
    goal->parent = lastNode;
    goal->cost = goalCost;
}

std::vector<Node *> getPath(Node *goal) {
    std::vector<Node *> path;
    Node *node = goal;
    while (node != nullptr) {
        path.push_back(node);
        node = node->parent;
    }
    std::reverse(path.begin(), path.end());
    return path;
}


//TODO Read and optimize
void visualize(const std::vector<Node *> &tree, Node *goal, bool finished = false) {
    cv::Mat image(env.height, env.width, CV_8UC3, cv::Scalar(255, 255, 255));
    cv::namedWindow("RRT* Visualization", cv::WINDOW_AUTOSIZE);

    // Draw the obstacles
    for (const Obstacle &obstacle: env.obstacles) {
        if (obstacle.type == Obstacle::CIRCLE) {
            cv::circle(image, cv::Point(static_cast<int>(obstacle.circle.x), static_cast<int>(obstacle.circle.y)),
                       static_cast<int>(obstacle.circle.radius),
                       cv::Scalar(0, 0, 0), -1);
        } else {
            cv::rectangle(image,
                          cv::Point(static_cast<int>(obstacle.rectangle.x), static_cast<int>(obstacle.rectangle.y)),
                          cv::Point(static_cast<int>(obstacle.rectangle.x + obstacle.rectangle.width),
                                    static_cast<int>(obstacle.rectangle.y + obstacle.rectangle.height)),
                          cv::Scalar(0, 0, 0), -1);
        }
    }

    // Draw the start
    cv::circle(image, cv::Point(static_cast<int>(tree[0]->x), static_cast<int>(tree[0]->y)), 5, cv::Scalar(255, 0, 0),
               -1);

    // Draw the tree
    for (Node *node: tree) {
        if (node->parent != nullptr) {
            cv::Point pt1(static_cast<int>(node->x), static_cast<int>(node->y));
            cv::Point pt2(static_cast<int>(node->parent->x), static_cast<int>(node->parent->y));
            cv::line(image, pt1, pt2, cv::Scalar(0, 0, 255), 1);
            cv::circle(image, pt1, 1, cv::Scalar(0, 0, 0), 1);
        }
    }
    if (finished) {
        // Draw the path
        Node *node = goal;
        int numNodes = 1;
        while (node->parent != nullptr) {
            cv::Point pt1(static_cast<int>(node->x), static_cast<int>(node->y));
            cv::Point pt2(static_cast<int>(node->parent->x), static_cast<int>(node->parent->y));
            cv::line(image, pt1, pt2, cv::Scalar(0, 255, 0), 2);
            node = node->parent;
            numNodes++;
        }
        std::string numNodesStr = "Number of Nodes: " + std::to_string(numNodes);
        std::string pathCostStr = "Path Cost: " + std::to_string(goal->cost);
        cv::putText(image, numNodesStr, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 0), 2);
        cv::putText(image, pathCostStr, cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 0), 2);
    }

    // Draw the goal
    cv::circle(image, cv::Point(static_cast<int>(goal->x), static_cast<int>(goal->y)), 5, cv::Scalar(0, 255, 0), -1);

    cv::imshow("RRT* Visualization", image);
    cv::waitKey(1); // Wait for 1 millisecond to allow the window to update
}

std::vector<Node *> rrtStar(Node *start, Node *goal, int width, int height) {
    initializeEnvironment(&env, width, height);

    auto start_ts = std::chrono::high_resolution_clock::now();

    // Initialize the tree with the start node
    std::vector<Node *> tree;
    tree.reserve(MAX_OPTIMIZING_ITERATIONS);
    tree.push_back(start);
    bool finish = false;
    int iteration_after_finish = 0;
    // Main loop of the RRT* algorithm

    int iter = 0;
    while (!finish || iteration_after_finish < MAX_OPTIMIZING_ITERATIONS) {
        // Sample a random point in the environment
//        Node *randomNode = sampleRandomNode(goal);
        Node *randomNode = sampleRandomNode();
        // Find the nearest node in the tree to the random point
        Node *nearestNode = nearestNodeInTree(tree, randomNode);

        // Extend the tree towards the random point
        Node *newNode = extendTree(nearestNode, randomNode, tree);
        delete randomNode;
        if (newNode == nullptr) {
            continue; // Collision detected, skip to the next iteration
        }

        tree.push_back(newNode);

        // If the new node is close enough to the goal, connect it to the goal
        double distanceToGoal = distance(newNode, goal);
        if (distanceToGoal < threshold && (goal->cost > newNode->cost + distanceToGoal)) {
            connectToGoal(newNode, goal);
//            if (!finish){
//                finish = true;
//                auto stop_ts = std::chrono::high_resolution_clock::now();
//                auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop_ts - start_ts);
//                std::cout << "First contact iteration: " << iter << " Time: " << duration.count() << " microseconds" << std::endl;
//            }
            finish = true;
        }
        if (finish) {
            iteration_after_finish++;
        }
        if (iter % refreshView == 0) {
            auto stop_ts = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop_ts - start_ts);
            std::cout << "Iteration: " << iter << " Time: " << duration.count() << " microseconds" << std::endl;
            visualize(tree, goal, finish);
        }
        iter++;
    }
    cv::waitKey(waitBeforeClosing);
    return tree;
}

std::vector<Node *>
rrtStar(Node *start, Node *goal, octomap::OcTree *octree, void (*pathFoundCallback)(std::vector<Node *> *,websocketpp::connection_hdl), websocketpp::connection_hdl hdl,
        const std::shared_ptr<StoppableThread>& stoppableThreadPtr) {
    initializeEnvironment(&env, octree);

    auto start_ts = std::chrono::high_resolution_clock::now();

    // Initialize the tree with the start node
    std::vector<Node *> tree;
    tree.reserve(MAX_OPTIMIZING_ITERATIONS);
    tree.push_back(start);
    bool finish = false;
    int iteration_after_finish = 0;
    // Main loop of the RRT* algorithm

    int iter = 0;
    while ((!finish || iteration_after_finish < MAX_OPTIMIZING_ITERATIONS) && !stoppableThreadPtr->isStopRequested()) {
        // Sample a random point in the environment
        Node *randomNode = sampleRandomNode(goal);
//        Node *randomNode = sampleRandomNode();
        // Find the nearest node in the tree to the random point
        Node *nearestNode = nearestNodeInTree(tree, randomNode);

        // Extend the tree towards the random point
        Node *newNode = extendTree(nearestNode, randomNode, tree, octree);
        delete randomNode;
        if (newNode == nullptr) {
            continue; // Collision detected, skip to the next iteration
        }

        tree.push_back(newNode);

        // If the new node is close enough to the goal, connect it to the goal
        double distanceToGoal = distance(newNode, goal);
        if (distanceToGoal < threshold && (goal->cost > newNode->cost + distanceToGoal)) {
            connectToGoal(newNode, goal);
            std::vector<Node *> path = getPath(goal);
            if (pathFoundCallback != nullptr) {
                pathFoundCallback(&path, hdl);
            }
            std::cout << "Path found with " << path.size() << " nodes" << std::endl;

//            if (!finish){
//                finish = true;
//                auto stop_ts = std::chrono::high_resolution_clock::now();
//                auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop_ts - start_ts);
//                std::cout << "First contact iteration: " << iter << " Time: " << duration.count() << " microseconds" << std::endl;
//            }
            finish = true;
        }
        if (finish) {
            iteration_after_finish++;
        }
        if (iter % refreshView == 0) {
            auto stop_ts = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop_ts - start_ts);
            std::cout << "Iteration: " << iter << " Time: " << duration.count() << " microseconds | " << finish
                      << " -> after finish " << iteration_after_finish << std::endl;
        }
        iter++;
    }
    if (stoppableThreadPtr->isStopRequested()) {
        std::cout << "Thread stopped" << std::endl;
        return finish ? getPath(goal) : std::vector<Node *>();
    }
    return getPath(goal);
}


