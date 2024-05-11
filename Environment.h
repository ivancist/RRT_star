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
    double width, height; // Width and y of the rectangle
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
    double x, y, z;
    double offset_x, offset_y, offset_z;
    octomap::OcTree* tree = nullptr;
    std::vector<Obstacle> obstacles;
};

void initializeEnvironment(Environment* env, double width, double height);
void initializeEnvironment(Environment* env, octomap::OcTree* tree, bool autoSize = true);
void initializeEnvironment(Environment* env, octomap::OcTree* tree, double x, double y, double z, double offset_x = 0, double offset_y = 0, double offset_z = 0);

#endif // ENVIRONMENT_H
