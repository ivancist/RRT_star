#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>
#include <random>
#include <unordered_set>
#include <chrono>

const int MAX_ITERATIONS = 15000;
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

Node *nearestNodeInTree(std::vector<Node *> &tree, Node *randomNode, bool considerCost = false,
                        const std::vector<Node *> excludeNodes = {}) {
    Node *nearestNode = nullptr;
    double minDistance = std::numeric_limits<double>::max();
    double minCost = randomNode->cost;

    std::unordered_set<Node *> excludeSet(excludeNodes.begin(), excludeNodes.end());

    for (Node *node: tree) {
        // Skip excluded nodes
        if (excludeSet.count(node) > 0) {
            continue;
        }

        double dist = distance(node, randomNode);
        if (!considerCost) {
            if (dist < minDistance) {
                minDistance = dist;
                nearestNode = node;
            }
        } else {
            // Calculate the total cost considering the distance to the random node
            double totalCost = node->cost + dist;
            if (dist <= threshold && totalCost < minCost) {
                minCost = totalCost;
                nearestNode = node;
            }
        }
    }

    return nearestNode;
}

bool checkCollision(Node *node) {
    for (const Obstacle &obstacle: env.obstacles) {
        if (obstacle.type == Obstacle::CIRCLE) {
            double dist = distance(node, new Node{obstacle.circle.x, obstacle.circle.y, nullptr});
            if (dist <= obstacle.circle.radius + stayAway) {
                return true;
            }
        } else {
            if (node->x >= obstacle.rectangle.x - stayAway &&
                node->x <= obstacle.rectangle.x + obstacle.rectangle.width + stayAway
                && node->y >= obstacle.rectangle.y - stayAway &&
                node->y <= obstacle.rectangle.y + obstacle.rectangle.height + stayAway) {
                return true;
            }
        }
    }
    return false;
}

bool checkCollision(Node *node1, Node *node2) {
    double dx = node2->x - node1->x;
    double dy = node2->y - node1->y;
    double dist = std::sqrt(dx * dx + dy * dy);

    // Normalize the direction vector
    double dirX = dx / dist;
    double dirY = dy / dist;

    // Check for collisions along the line between the two nodes
    for (double i = 0; i < dist; i += 1) {
        Node *node = new Node{node1->x + dirX * i, node1->y + dirY * i, nullptr};
        if (checkCollision(node)) {
            return true;
        }
    }

    return false;
}

Node *extendTree(Node *nearestNode, Node *randomNode, double stepLength, std::vector<Node *> &tree) {
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

    std::vector<Node *> excludeNodes;
    Node *parentNode = nullptr;
    do {
        parentNode = nearestNodeInTree(tree, &tempNode, true, excludeNodes);
        if (parentNode == nullptr) {
            return nullptr;
        }
        excludeNodes.push_back(parentNode);
    } while (checkCollision(parentNode, &tempNode) && excludeNodes.size() < tree.size());


    double newCost = parentNode->cost + distance(parentNode, &tempNode);

    // Create a new node at the new position with the calculated cost
    Node *newNode = new Node{newX, newY, parentNode, newCost};
    return newNode;
}

void connectToGoal(Node *lastNode, Node *goal) {
    // Calculate the cost of the goal node
    double goalCost = lastNode->cost + distance(lastNode, goal);

    // Update the goal node to point to the new goal node
    goal->parent = lastNode;
    goal->cost = goalCost;
}

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


// IN QUESTO MODO UN NODO GENITORE PUO' ESSERE RICALCOLATO PIU' VOLTE
// TODO: Optimize
// SI PUO' LAVORARE CON UN ARRAY RIORDINATO PRIMA
// SI PUO' CREARE NODE CON FIGLI, PER EVITARE DI RICALCOLARE I COSTI
void recalculateCosts(Node *node) {
    if (node->parent != nullptr) {
        recalculateCosts(node->parent);
        node->cost = node->parent->cost + distance(node, node->parent);
    }
}

std::vector<Node *> rrtStar(Node *start, Node *goal) {

    auto start_ts = std::chrono::high_resolution_clock::now();

    // Initialize the tree with the start node
    std::vector<Node *> tree;
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
        Node *newNode = extendTree(nearestNode, randomNode, stepLength, tree);
        if (newNode == nullptr) {
            continue; // Collision detected, skip to the next iteration
        }

        // TODO: Optimize
        double dist;
        for (Node *node: tree) {
            dist = distance(node, newNode);
            if (dist < threshold && node->cost > newNode->cost + dist) {
                if (checkCollision(node, newNode)) {
                    continue;
                }
                node->parent = newNode;
                node->cost = newNode->cost + dist;
            }
            recalculateCosts(node);
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
            std::cout << "Iteration: " << iter << " Time: "<< duration.count() << " microseconds" << std::endl;
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
