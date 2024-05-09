#include "Environment.h"

Obstacle::Obstacle(Type type, const Circle &circle) : type(type), circle(circle) {}
Obstacle::Obstacle(Type type, const Rectangle &rectangle) : type(type), rectangle(rectangle) {}

void initializeEnvironment(Environment* env, double width, double height) {
    env->width = width;
    env->height = height;
    env->depth = 0;

    env->obstacles.emplace_back(Obstacle::CIRCLE, Circle{250, 180, 30});
    env->obstacles.emplace_back(Obstacle::CIRCLE, Circle{400, 270, 45});
    env->obstacles.emplace_back(Obstacle::CIRCLE, Circle{150, 270, 55});
    env->obstacles.emplace_back(Obstacle::RECTANGLE, Rectangle{30, 400, 50, 50});
    env->obstacles.emplace_back(Obstacle::RECTANGLE, Rectangle{110, 400, 200, 50});
    env->obstacles.emplace_back(Obstacle::RECTANGLE, Rectangle{350, 400, 130, 50});
    env->obstacles.emplace_back(Obstacle::RECTANGLE, Rectangle{0, 100, 30, 500});
    env->obstacles.emplace_back(Obstacle::RECTANGLE, Rectangle{280, 100, 250, 20});//w200
//    env->obstacles.emplace_back(Obstacle::RECTANGLE, Rectangle{300, 300, 150, 20});
//    env->obstacles.emplace_back(Obstacle::RECTANGLE, Rectangle{60, 350, 90, 20});
    env->obstacles.emplace_back(Obstacle::RECTANGLE, Rectangle{520, 40, 20, 510});
    env->obstacles.emplace_back(Obstacle::RECTANGLE, Rectangle{200, 440, 10, 60});
    env->obstacles.emplace_back(Obstacle::RECTANGLE, Rectangle{200, 540, 10, 60});
    env->obstacles.emplace_back(Obstacle::RECTANGLE, Rectangle{350, 450, 10, 120});
}

// 3D Depth is the height of the environment
void initializeEnvironment(Environment* env, octomap::OcTree* tree, double width, double height, double depth) {
    env->width = width;
    env->height = height;
    env->depth = depth;
    env->tree = tree;
}

void initializeEnvironment(Environment* env, octomap::OcTree* tree) {
    initializeEnvironment(env, tree, 10, 4, 20);
}