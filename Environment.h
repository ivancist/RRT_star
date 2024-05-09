#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

#include <vector>
#include <iostream>
#include <octomap/OcTree.h>


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

    Obstacle(Type type, const Circle &circle);
    Obstacle(Type type, const Rectangle &rectangle);
};

struct Environment {
    double width, height, depth;
    octomap::OcTree* tree = nullptr;
    std::vector<Obstacle> obstacles;
};

void initializeEnvironment(Environment* env, double width, double height);
void initializeEnvironment(Environment* env, octomap::OcTree* tree);
void initializeEnvironment(Environment* env, octomap::OcTree* tree, double width, double height, double depth);

#endif // ENVIRONMENT_H
