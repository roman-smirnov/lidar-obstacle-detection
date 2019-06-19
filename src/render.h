#ifndef RENDER_H
#define RENDER_H

#include "box.h"
#include "common.h"

void renderGround(Viewer& viewer, const PointCloud& cloud);

void renderObstacle(Viewer& viewer, const PointCloud& cloud, int id);

void renderBox(Viewer& viewer, Box box, int id);

#endif