#pragma once

#include "mission.h"

extern const double VERTICAL_THRESHOLD;
extern const double HORIZONTAL_THRESHOLD;

double getSystemTime();

int getCoordsDouble(double& latitude, double& longitude, double& altitude);

double getDistanceBetween(
    double latitude1, double longitude1, double latitude2, double longitude2
);

int getDistanceTo(double latitude, double longitude, double& distance);

double getHorizontalSpeed();

double getVerticalSpeed();

void getHomeCoord(double& latitude, double& longitude, double& altitude);

uint32_t getNextCommandIndex();

MissionCommand* getNextCommand();
