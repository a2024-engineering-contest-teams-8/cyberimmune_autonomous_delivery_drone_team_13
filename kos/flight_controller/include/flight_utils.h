#pragma once

#include "mission.h"

#define RETRY_DELAY_SEC 1

extern const double VERTICAL_THRESHOLD;
extern const double HORIZONTAL_THRESHOLD;

int sendSignedMessage(char* method, char* response, char* errorMessage, uint8_t delay);

bool isFlyAccepted();

void sendLogs(const char* message);

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

bool hasWaypointChanged();

CommandWaypoint getOldWaypoint();
