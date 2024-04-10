#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>
#include <random>
#include <chrono>

const int MAX_ITERATIONS = 10000;
const int width = 600;
const int height = 600;
const int threshold = 30;
double stepLength = 20;
const double stayAway = 10;
const int refreshView = 1000;

struct Node {
    double x, y;
    Node *parent;
    double cost;
    std::vector<Node *> children;
};

struct Circle {
    double x, y; // Center of the circle
    double radius; // Radius of the circle
};

struct Rectangle {
    double x, y; // Top-left corner of the rectangle
    double width, height; // Width and height of the rectangle
};

struct Obstacle {
    enum Type {
        CIRCLE, RECTANGLE
    } type;
    union {
        Circle circle;
        Rectangle rectangle;
    };

    Obstacle(Type type, const Circle &circle) : type(type), circle(circle) {}

    Obstacle(Type type, const Rectangle &rectangle) : type(type), rectangle(rectangle) {}
};

struct Environment {
    double width, height;
    std::vector<Obstacle> obstacles;
};

Environment env{width, height};

double distance(Node *node1, Node *node2) {
    double dx = node1->x - node2->x;
    double dy = node1->y - node2->y;
    return std::sqrt(dx * dx + dy * dy);
}

double distance(Node *node, double x, double y) {
    double dx = node->x - x;
    double dy = node->y - y;
    return std::sqrt(dx * dx + dy * dy);
}

// Function to generate a random double within a given range
double randomDouble(double min, double max) {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(min, max);
    return dis(gen);
}

Node *sampleRandomNode() {
    // Generate random x and y within the environment bounds
    double x = randomDouble(0, env.width);
    double y = randomDouble(0, env.height);

    // Create a new node with the random coordinates
    return new Node{x, y, nullptr};
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
        double totalCost = node->cost + dist;
        if (dist <= threshold) {
            nearestNodes.push_back(node);
            if (totalCost < minCost) {
                minCost = totalCost;
                nNodeIndex = nearestNodes.size() - 1;
            }
        }
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
void recalculateCostOfChildren(Node * node, double delta) {
    for (Node *child: node->children) {
        child->cost += delta;
        recalculateCostOfChildren(child, delta);
    }
}

Node *extendTree(Node *nearestNode, Node *randomNode, std::vector<Node *> &tree) {
    // Calculate the direction from the nearest node to the random node
    double dx = randomNode->x - nearestNode->x;
    double dy = randomNode->y - nearestNode->y;
    double dist = std::sqrt(dx * dx + dy * dy);

    // Normalize the direction vector
    double dirX = dx / dist;
    double dirY = dy / dist;

    // Calculate the new node's position by moving along the direction vector by the step length
    double newX = nearestNode->x + dirX * stepLength;
    double newY = nearestNode->y + dirY * stepLength;

    // Calculate the cost of the new node
//    double newCost = nearestNode->cost + distance(nearestNode, new Node{newX, newY, nullptr});

    Node tempNode{newX, newY, nullptr, std::numeric_limits<double>::max()};
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
    Node *newNode = new Node{newX, newY, parentNode, newNodeCost};
    parentNode->children.push_back(newNode);

    for (size_t i = 1; i < nearestNodes.size(); i++) {
        Node *node = nearestNodes[i];
        double distanceToNewNode = distance(node, &tempNode);
        if (newNodeCost + distanceToNewNode < node->cost) {
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


//TODO Rivedere e ottimizzare
void visualize(const std::vector<Node *> &tree, Node *goal, bool finished = false) {
    cv::Mat image(height, width, CV_8UC3, cv::Scalar(255, 255, 255));
    cv::namedWindow("RRT* Visualization", cv::WINDOW_AUTOSIZE);

    // Draw the obstacles
    for (const Obstacle &obstacle: env.obstacles) {
        if (obstacle.type == Obstacle::CIRCLE) {
            cv::circle(image, cv::Point(obstacle.circle.x, obstacle.circle.y), obstacle.circle.radius,
                       cv::Scalar(0, 0, 0), -1);
        } else {
            cv::rectangle(image, cv::Point(obstacle.rectangle.x, obstacle.rectangle.y),
                          cv::Point(obstacle.rectangle.x + obstacle.rectangle.width,
                                    obstacle.rectangle.y + obstacle.rectangle.height),
                          cv::Scalar(0, 0, 0), -1);
        }
    }

    // Draw the start
    cv::circle(image, cv::Point(tree[0]->x, tree[0]->y), 5, cv::Scalar(255, 0, 0), -1);

    // Draw the tree
    for (Node *node: tree) {
        if (node->parent != nullptr) {
            cv::Point pt1(node->x, node->y);
            cv::Point pt2(node->parent->x, node->parent->y);
            cv::line(image, pt1, pt2, cv::Scalar(0, 0, 255), 1);
            cv::circle(image, pt1, 1, cv::Scalar(0, 0, 0), 1);
        }
    }
    if (finished) {
        // Draw the path
        Node *node = goal;
        int numNodes = 1;
        while (node->parent != nullptr) {
            cv::Point pt1(node->x, node->y);
            cv::Point pt2(node->parent->x, node->parent->y);
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
    cv::circle(image, cv::Point(goal->x, goal->y), 5, cv::Scalar(0, 255, 0), -1);

    cv::imshow("RRT* Visualization", image);
    cv::waitKey(1); // Wait for 1 millisecond to allow the window to update
}

std::vector<Node *> rrtStar(Node *start, Node *goal) {
    auto start_ts = std::chrono::high_resolution_clock::now();

    // Initialize the tree with the start node
    std::vector<Node *> tree;
    tree.reserve(MAX_ITERATIONS);
    tree.push_back(new Node{start->x, start->y, nullptr, 0});
    bool finish = false;
    int iteration_after_finish = 0;
    // Main loop of the RRT* algorithm

    int iter = 0;
    while (!finish || iteration_after_finish < MAX_ITERATIONS) {


        // Sample a random point in the environment
        Node *randomNode = sampleRandomNode();
        // Find the nearest node in the tree to the random point
        Node *nearestNode = nearestNodeInTree(tree, randomNode);

        // Extend the tree towards the random point
        Node *newNode = extendTree(nearestNode, randomNode, tree);
        if (newNode == nullptr) {
            continue; // Collision detected, skip to the next iteration
        }

        tree.push_back(newNode);

        // If the new node is close enough to the goal, connect it to the goal
        double distanceToGoal = distance(newNode, goal);
        if (distanceToGoal < threshold && (goal->cost > newNode->cost + distanceToGoal)) {
            connectToGoal(newNode, goal);
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
    return tree;
}

int main() {
    env.obstacles.emplace_back(Obstacle::CIRCLE, Circle{250, 180, 30});
    env.obstacles.emplace_back(Obstacle::RECTANGLE, Rectangle{30, 400, 50, 50});
    env.obstacles.emplace_back(Obstacle::RECTANGLE, Rectangle{110, 400, 370, 50});
    env.obstacles.emplace_back(Obstacle::RECTANGLE, Rectangle{0, 100, 30, 500});
    env.obstacles.emplace_back(Obstacle::RECTANGLE, Rectangle{300, 100, 200, 50});
    env.obstacles.emplace_back(Obstacle::RECTANGLE, Rectangle{520, 80, 20, 470});

    Node start{450, 50, nullptr, 0};
    Node goal{440, 550, nullptr, std::numeric_limits<double>::max()};

    std::vector<Node *> tree = rrtStar(&start, &goal);
    cv::waitKey(5000);
    return 0;
}
