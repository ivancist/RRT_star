#include "Environment.h"

Obstacle::Obstacle(Type type, const Circle &circle) : type(type), circle(circle) {}

Obstacle::Obstacle(Type type, const Rectangle &rectangle) : type(type), rectangle(rectangle) {}

void initializeEnvironment(std::shared_ptr<Environment> &env, double width, double height) {
    env->x = width;
    env->y = height;
    env->z = 0;

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

// 3D Depth is the y of the environment
void initializeEnvironment(std::shared_ptr<Environment> &env, octomap::OcTree *&tree, double x, double y, double z,
                           double offset_x,
                           double offset_y, double offset_z) {
    env->x = x;
    env->y = y;
    env->z = z;
    env->offset_x = offset_x;
    env->offset_y = offset_y;
    env->offset_z = offset_z;
    env->tree = tree;
}

void initializeEnvironment(std::shared_ptr<Environment> &env, octomap::OcTree *&tree, bool autoConfig) {
    if (autoConfig) {
        tree->getMetricSize(env->x, env->y, env->z);
        tree->getMetricMin(env->offset_x, env->offset_y, env->offset_z);
        env->tree = tree;
    } else {
        initializeEnvironment(env, tree, 20, 10, 4);
    }
}

void initializeEnvironment(std::shared_ptr<Environment> &env, octomap::OcTree *&tree, double maxDist) {
    initializeEnvironment(env, tree, true);
//    DynamicEDTOctomap distmap(maxDist, tree, octomap::point3d(env->offset_x, env->offset_y, env->offset_z), octomap::point3d(env->x, env->y, env->z), false);

    env->distmap = new DynamicEDTOctomap(maxDist, &*tree, octomap::point3d(env->offset_x, env->offset_y, env->offset_z),
                                         octomap::point3d(env->x + env->offset_x, env->y + env->offset_y, env->z + env->offset_z), false);
    env->distmap->update();
    std::cout << env->offset_x << " " << env->offset_y << " " << env->offset_z << std::endl;
    std::cout << env->x << " " << env->y << " " << env->z << std::endl;
}