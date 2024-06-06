#include "RRT_star.h"

double RRTStar::distance(Node *node1, Node *node2) {
    double dx = node1->x - node2->x;
    double dy = node1->y - node2->y;
    double dz = node1->z - node2->z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double RRTStar::distance(Node *node, double x, double y) {
    double dx = node->x - x;
    double dy = node->y - y;
    return std::sqrt(dx * dx + dy * dy);
}

void RRTStar::getDirection(Node *node1, Node *node2, Node *direction) {
    // Calculate the direction from the nearest node to the random node
    direction->x = node2->x - node1->x;
    direction->y = node2->y - node1->y;
    direction->z = node2->z - node1->z;
}

void RRTStar::getVersor(Node *node1, Node *node2, Node *versor) {
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
double RRTStar::randomDouble(double min, double max) {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dis(min, max);
    return dis(gen);
}

// TODO Other sampling methods
Node *RRTStar::sampleRandomNode() {
    // Generate random x and y within the environment bounds
    double x = randomDouble(env.offset_x, env.x + env.offset_x);
    double y = randomDouble(env.offset_y, env.y + env.offset_y);
    double z = randomDouble(env.offset_z, env.z + env.offset_z);

    // Create a node with the random coordinates
    return new Node{x, y, z};
}

Node *RRTStar::sampleRandomNode(Node *goal) {
    if (randomDouble(0, 1) < bias) {
        return new Node{goal->x, goal->y, goal->z};
    } else {
        return sampleRandomNode();
    }
}

Node *RRTStar::nearestNodeInTree(std::vector<Node *> &tree, Node *randomNode) {
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

std::vector<Node *> RRTStar::nearestNodesInTree(std::vector<Node *> &tree, Node *newNode) {
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

bool RRTStar::checkCollision(Node *node) {
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

bool RRTStar::checkCollision(Node *node1, Node *node2) {
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

void RRTStar::calcolaVerticiQuadrato(octomap::point3d *centro, octomap::point3d *versore, octomap::point3d vertici[],
                                     double lato) {
    // Calcola il vettore ortogonale al versore
    octomap::point3d vettoreOrtogonale1, vettoreOrtogonale2;

    if (versore->x() == 0 && versore->y() == 0) {
        vettoreOrtogonale1 = {1, 0, 0};
        vettoreOrtogonale2 = {0, 1, 0};
    } else {
        vettoreOrtogonale1 = {versore->y(), -versore->x(), 0};
        vettoreOrtogonale2 = {-versore->z() * versore->x(), -versore->z() * versore->y(),
                              versore->x() * versore->x() + versore->y() * versore->y()};
    }

    // Normalizza i vettori ortogonali
    double lunghezza1 = sqrt(vettoreOrtogonale1.x() * vettoreOrtogonale1.x() +
                             vettoreOrtogonale1.y() * vettoreOrtogonale1.y() +
                             vettoreOrtogonale1.z() * vettoreOrtogonale1.z());
    double lunghezza2 = sqrt(vettoreOrtogonale2.x() * vettoreOrtogonale2.x() +
                             vettoreOrtogonale2.y() * vettoreOrtogonale2.y() +
                             vettoreOrtogonale2.z() * vettoreOrtogonale2.z());
    vettoreOrtogonale1.x() /= lunghezza1;
    vettoreOrtogonale1.y() /= lunghezza1;
    vettoreOrtogonale1.z() /= lunghezza1;
    vettoreOrtogonale2.x() /= lunghezza2;
    vettoreOrtogonale2.y() /= lunghezza2;
    vettoreOrtogonale2.z() /= lunghezza2;

    // Calcola i quattro vertici

    vertici[0].x() = centro->x() - lato / 2 * vettoreOrtogonale1.x() - lato / 2 * vettoreOrtogonale2.x();
    vertici[0].y() = centro->y() - lato / 2 * vettoreOrtogonale1.y() - lato / 2 * vettoreOrtogonale2.y();
    vertici[0].z() = centro->z() - lato / 2 * vettoreOrtogonale1.z() - lato / 2 * vettoreOrtogonale2.z();
    vertici[1].x() = centro->x() + lato / 2 * vettoreOrtogonale1.x() - lato / 2 * vettoreOrtogonale2.x();
    vertici[1].y() = centro->y() + lato / 2 * vettoreOrtogonale1.y() - lato / 2 * vettoreOrtogonale2.y();
    vertici[1].z() = centro->z() + lato / 2 * vettoreOrtogonale1.z() - lato / 2 * vettoreOrtogonale2.z();
    vertici[2].x() = centro->x() + lato / 2 * vettoreOrtogonale1.x() + lato / 2 * vettoreOrtogonale2.x();
    vertici[2].y() = centro->y() + lato / 2 * vettoreOrtogonale1.y() + lato / 2 * vettoreOrtogonale2.y();
    vertici[2].z() = centro->z() + lato / 2 * vettoreOrtogonale1.z() + lato / 2 * vettoreOrtogonale2.z();
    vertici[3].x() = centro->x() - lato / 2 * vettoreOrtogonale1.x() + lato / 2 * vettoreOrtogonale2.x();
    vertici[3].y() = centro->y() - lato / 2 * vettoreOrtogonale1.y() + lato / 2 * vettoreOrtogonale2.y();
    vertici[3].z() = centro->z() - lato / 2 * vettoreOrtogonale1.z() + lato / 2 * vettoreOrtogonale2.z();
}

bool RRTStar::checkRayCollision(Node *node1, Node *node2) {
    Node versor{};
    getDirection(node1, node2, &versor);
    if (versor.x == 0 && versor.y == 0 && versor.z == 0) {
        return true;
    }
    octomap::point3d collisionPoint{};
    if (env.tree->castRay(octomap::point3d(node1->x, node1->y, node1->z),
                          octomap::point3d(versor.x, versor.y, versor.z),
                          collisionPoint, true, *node1 - *node2 + stayAway)) {
        return true;
    }
    return false;
}

bool RRTStar::checkMultipleRayCollision(Node *node1, Node *node2) {
    Node versor{};
    getVersor(node1, node2, &versor);
    if (versor.x == 0 && versor.y == 0 && versor.z == 0) {
        return true;
    }
    octomap::point3d collisionPoint{};
    if (env.tree->castRay(octomap::point3d(node1->x, node1->y, node1->z),
                          octomap::point3d(versor.x, versor.y, versor.z),
                          collisionPoint, true, *node1 - *node2 + stayAway)) {
        return true;
    }
    octomap::point3d origin(node1->x, node1->y, node1->z);
    octomap::point3d versorPoint(versor.x, versor.y, versor.z);
    octomap::point3d vertexes[4];
    calcolaVerticiQuadrato(&origin, &versorPoint, vertexes, stayAway);
    for (int i = 0; i < 4; i++) {
        if (env.tree->castRay(vertexes[i], versorPoint, collisionPoint, true, *node1 - *node2 + stayAway)) {
            return true;
        }
    }
    return false;
}

bool RRTStar::checkLinkCollisionWithDistMap(Node *node1, Node *node2) {
    Node versor{};
    getVersor(node1, node2, &versor);
    if (versor.x == 0 && versor.y == 0 && versor.z == 0) {
        return true;
    }
    double distToTarget = *node1 - *node2;
    double dist = safeStayAway;
    double oldDist;
    octomap::point3d stepNode = octomap::point3d(node1->x, node1->y, node1->z);
    while (distToTarget > dist) {
        stepNode = octomap::point3d(versor.x * dist + stepNode.x(), versor.y * dist + stepNode.y(),
                                    versor.z * dist + stepNode.z());
        oldDist = dist;
        dist = env.distmap->getDistance(stepNode);
        if (dist < safeStayAway) {
            return true;
        }
        distToTarget -= oldDist;
    }
    return false;
}

void RRTStar::recalculateCostOfChildren(Node *node, double delta) {
    for (Node *child: node->children) {
        child->cost += delta;
        recalculateCostOfChildren(child, delta);
    }
}

Node *RRTStar::extendTree(Node *nearestNode, Node *randomNode, std::vector<Node *> &tree) {
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

Node *RRTStar::extend3DTree(Node *nearestNode, Node *randomNode, std::vector<Node *> &tree) {
    Node versor{};
    getVersor(nearestNode, randomNode, &versor);

    // Calculate the new node's position by moving along the direction vector by the step length
    double newX = nearestNode->x + versor.x * stepLength;
    double newY = nearestNode->y + versor.y * stepLength;
    double newZ = nearestNode->z + versor.z * stepLength;

    // Calculate the cost of the new node
//    double newCost = nearestNode->cost + distance(nearestNode, new Node{newX, newY, nullptr});

    Node tempNode{newX, newY, newZ, nullptr, std::numeric_limits<double>::max()};
//    if (env.tree->search(tempNode.x, tempNode.y, tempNode.z, searchAtDepth) != nullptr) {
    if (env.distmap->getDistance(octomap::point3d(tempNode.x, tempNode.y, tempNode.z)) < stayAway) {
        return nullptr; // Collision detected, do not extend the tree
    }


    std::vector<Node *> nearestNodes = nearestNodesInTree(tree, &tempNode);
//    bool collision = true;
//    while (!nearestNodes.empty() && collision) {
//        octomap::point3d tempPoint{};
//        getDirection(nearestNodes[0], &tempNode, &versor);
//        if (versor.x == 0 && versor.y == 0 && versor.z == 0) {
//            return nullptr;
//        }
//        collision = checkRayCollision(nearestNodes[0], &tempNode);
//        if (collision) {
//            nearestNodes.erase(nearestNodes.begin());
//        }
//    }
//    if (collision) {
//        return nullptr;
//    }
    if (threshold > stayAway) {
        bool collision = true;
        while (!nearestNodes.empty() && collision) {
            collision = checkLinkCollisionWithDistMap(nearestNodes[0], &tempNode);
            if (collision) {
                nearestNodes.erase(nearestNodes.begin());
            }
        }
        if (collision) {
            return nullptr;
        }
    } else if (nearestNodes.empty()) {
        return nullptr;
    }

    Node *parentNode = nearestNodes[0];

    double newNodeCost = parentNode->cost + distance(parentNode, &tempNode);
    Node *newNode = new Node{newX, newY, newZ, parentNode, newNodeCost};
    parentNode->children.push_back(newNode);

    for (size_t i = 1; i < nearestNodes.size(); i++) {
        Node *node = nearestNodes[i];
        double distanceToNewNode = *node - tempNode;
        if (newNodeCost + distanceToNewNode < node->cost
            // && !checkRayCollision(node, &tempNode)
            && !checkLinkCollisionWithDistMap(node, &tempNode)
                ) {
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

void RRTStar::connectToGoal(Node *lastNode, Node *goal) {
    // Calculate the cost of the goal node
    double goalCost = lastNode->cost + distance(lastNode, goal);

    // Update the goal node to point to the new goal node
    goal->parent = lastNode;
    goal->cost = goalCost;
}

std::vector<Node *> RRTStar::getPath(Node *goal) {
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
void RRTStar::visualize(const std::vector<Node *> &tree, Node *goal, bool finished) {
    cv::Mat image(env.y, env.x, CV_8UC3, cv::Scalar(255, 255, 255));
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
    cv::circle(image, cv::Point(static_cast<int>(tree[0]->x), static_cast<int>(tree[0]->y)), 5,
               cv::Scalar(255, 0, 0),
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
    cv::circle(image, cv::Point(static_cast<int>(goal->x), static_cast<int>(goal->y)), 5, cv::Scalar(0, 255, 0),
               -1);

    cv::imshow("RRT* Visualization", image);
    cv::waitKey(1); // Wait for 1 millisecond to allow the window to update
}

std::vector<Node *> RRTStar::rrtStar(Node *start, Node *goal, Environment &env) {
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

FinalReturn
RRTStar::rrtStar(Node *start, Node *goal, Environment &environment, double stayAwayDesired,
                 void (*pathFoundCallback)(ReturnPath *, websocketpp::connection_hdl), websocketpp::connection_hdl hdl,
                 const std::shared_ptr<StoppableThread> &stoppableThreadPtr) {
    env = environment;
    stayAway = stayAwayDesired;
    safeStayAway = stayAwayDesired / cos(M_PI / 6);
//        int depth = env.tree->getTreeDepth();
//        double resolution = env.tree->getResolution();
//        if (stayAway > resolution) {
//            searchAtDepth = depth - ceil(log2(stayAway / resolution));
//        }

    // Initialize the tree with the start node
    std::vector<Node *> tree;
    tree.reserve(MAX_OPTIMIZING_ITERATIONS);
    tree.push_back(start);
    bool finish = false;
    int iteration_after_finish = 0;
    int iter = 0;
    auto start_ts = std::chrono::high_resolution_clock::now();

    while ((!finish || iteration_after_finish < MAX_OPTIMIZING_ITERATIONS) &&
           !stoppableThreadPtr->isStopRequested()) {
        // Sample a random point in the environment
        Node *randomNode = sampleRandomNode(goal);
//        Node *randomNode = sampleRandomNode();
        // Find the nearest node in the tree to the random point
        Node *nearestNode = nearestNodeInTree(tree, randomNode);
        // Extend the tree towards the random point
        Node *newNode = extend3DTree(nearestNode, randomNode, tree);
        delete randomNode;
        if (newNode == nullptr) {
            continue; // Collision detected, skip to the next iteration
        }

        tree.push_back(newNode);

        // If the new node is close enough to the goal, connect it to the goal
        double distanceToGoal = distance(newNode, goal);
        if (distanceToGoal < threshold && (goal->cost > newNode->cost + distanceToGoal) &&
            !checkLinkCollisionWithDistMap(newNode, goal)) {
            connectToGoal(newNode, goal);
            std::vector<Node *> path = getPath(goal);
            if (pathFoundCallback != nullptr) {
                ReturnPath returnPath{&path, std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::high_resolution_clock::now() - start_ts).count()};
                pathFoundCallback(&returnPath, hdl);
            }
//            std::cout << "Path found with " << path.size() << " nodes" << std::endl;
//
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
        if (refreshView != -1 && iter % refreshView == 0) {
            auto stop_ts = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop_ts - start_ts);
            std::cout << "Iteration: " << iter << " Time: " << duration.count() << " microseconds | " << finish
                      << " -> after finish " << iteration_after_finish << std::endl;
        }
        iter++;
    }
    std::shared_ptr<std::vector<Node *>> retPath = std::make_shared<std::vector<Node *>>(getPath(goal));
    if (stoppableThreadPtr->isStopRequested()) {
        std::cout << "Thread stopped" << std::endl;
        if (!finish) {
            //clear the path
            retPath->clear();
            return FinalReturn{std::move(retPath), std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::high_resolution_clock::now() - start_ts).count(), tree.size()};
        }
    }
    return FinalReturn{std::move(retPath), std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::high_resolution_clock::now() - start_ts).count(), tree.size(), goal->cost};
}

void RRTStar::pathPruning(std::shared_ptr<std::vector<Node *>> &path) {
    std::size_t numNodes = path->size();
    for (int i = 0; i < numNodes - 2; ++i) {
//                octomap::point3d startNode((*path.path)[i]->x, (*path.path)[i]->y, (*path.path)[i]->z);
//                for (int j = i+2; j < numNodes; ++j) {
        for (int j = numNodes - 1; j > i + 1; --j) {
//                    octomap::point3d goalNode((*path.path)[j]->x, (*path.path)[j]->y, (*path.path)[j]->z);
//                    octomap::point3d direction = goalNode - startNode;
//                    octomap::point3d temp;
//                    double distance = startNode.distance(goalNode);
//                    if (!tree->castRay(startNode, direction, temp, true, distance)) {
            if (!checkLinkCollisionWithDistMap((*path)[i], (*path)[j])) {
                for (int k = i + 1; k < j; ++k) {
                    delete (*path)[k];
                }
                path->erase(path->begin() + i + 1, path->begin() + j);
                numNodes = path->size();
                break;
            }
        }
    }
}

// 3D Quadradic Bezier curve
void RRTStar::pathSmoothing(std::shared_ptr<std::vector<Node *>> &path, float percent, int density) {
    int count;
    for (int i = 0; i < path->size() - 2; i += count) {
        octomap::point3d p0 = octomap::point3d(path->at(i)->x, path->at(i)->y, path->at(i)->z);
        octomap::point3d p1 = octomap::point3d(path->at(i + 1)->x, path->at(i + 1)->y, path->at(i + 1)->z);
        octomap::point3d p2 = octomap::point3d(path->at(i + 2)->x, path->at(i + 2)->y, path->at(i + 2)->z);
        octomap::point3d vers1, vers2;
        double dist1 = p1.distance(p0);
        double dist2 = p2.distance(p1);
        vers1 = (p1 - p0) * (1.0 / dist1);
        vers2 = (p2 - p1) * (1.0 / dist2);
        bool collision = true;
        while (collision) {
            p0 = p1 - vers1 * dist1 * percent;
            p2 = p1 + vers2 * dist2 * percent;
            count = 0;
            std::vector<Node *> bezier;
            for (float j = 0; j < 1; j += 1.0 / density) {
                count++;
                Node *newNode = new Node();
                newNode->x = (1 - j) * (1 - j) * p0.x() + 2 * (1 - j) * j * p1.x() + j * j * p2.x();
                newNode->y = (1 - j) * (1 - j) * p0.y() + 2 * (1 - j) * j * p1.y() + j * j * p2.y();
                newNode->z = (1 - j) * (1 - j) * p0.z() + 2 * (1 - j) * j * p1.z() + j * j * p2.z();
                bezier.push_back(newNode);

                if (env.distmap->getDistance(octomap::point3d(newNode->x, newNode->y, newNode->z)) < stayAway) {
                    std::cerr << "Collision detected in path smoothing" << std::endl;
                    break;
                }
            }
            if (count == density) {
                path->erase(path->begin() + i + 1);
                path->insert(path->begin() + i + 1, bezier.begin(), bezier.end());
                collision = false;
            } else {
                percent /= 2;
                density /= sqrt(2);
                if (percent < 0.05 || density < 2) {
                    break;
                }
            }
        }
    }
}