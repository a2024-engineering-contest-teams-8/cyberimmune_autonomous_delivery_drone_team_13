#include <stdio.h>
#include "math.h"
#include "time.h"
#include "unistd.h"
#include <string.h>

#include "../../shared/include/ipc_messages_navigation_system.h"
#include "../../shared/include/ipc_messages_credential_manager.h"
#include "../../shared/include/ipc_messages_server_connector.h"

#include "../include/flight_utils.h"

const double DEG_TO_RAD = M_PI / 180.0;
const double EARTH_RADIUS = 6371000.0;

const double SPEED_UPDATE_DURATION = 0.2;
const double COMMAND_UPDATE_DURATION = 0.1;

const double VERTICAL_THRESHOLD = 0.5;
const double HORIZONTAL_THRESHOLD = 3.0;

double homeLatitude = 0.0;
double homeLongitude = 0.0;
double homeAltitude = 0.0;
CommandWaypoint oldWaypoint = { 0, 0, 0 };

int sendSignedMessage(char* method, char* response, char* errorMessage, uint8_t delay) {
    char message[512] = {0};
    char signature[257] = {0};
    char request[1024] = {0};
    snprintf(message, 512, "%s?%s", method, BOARD_ID);

    while (!signMessage(message, signature)) {
        fprintf(stderr, "[%s] Warning: Failed to sign %s message at Credential Manager. Trying again in %ds\n", ENTITY_NAME, errorMessage, delay);
        sleep(delay);
    }
    snprintf(request, 1024, "%s&sig=0x%s", message, signature);

    while (!sendRequest(request, response)) {
        fprintf(stderr, "[%s] Warning: Failed to send %s request through Server Connector. Trying again in %ds\n", ENTITY_NAME, errorMessage, delay);
        sleep(delay);
    }

    uint8_t authenticity = 0;
    while (!checkSignature(response, authenticity) || !authenticity) {
        fprintf(stderr, "[%s] Warning: Failed to check signature of %s response received through Server Connector. Trying again in %ds\n", ENTITY_NAME, errorMessage, delay);
        sleep(delay);
    }

    return 1;
}

bool isFlyAccepted() {
    char response[1024] = {0};
    sendSignedMessage("/api/fly_accept", response, "fly_accept", RETRY_DELAY_SEC);

    return strstr(response, "$Arm: 0#") != NULL;
}

// void writeLogs(const char* message) {
//     char response[1024] = {0};
//     char request[4096] = {0};

//     snprintf("/api/logs?%s&log=%s", 4096, BOARD_ID, message);

//     fprintf(stderr, message);

//     if (!sendRequest(request, response)) {
//         fprintf(stderr, "[Error] writeLogs: unable to send request to ORVD");
//     }
// }

double getSystemTime() {
    auto t = clock();
    return (double)t / CLOCKS_PER_SEC;
}

int getCoordsDouble(double& latitude, double& longitude, double& altitude) {
    int32_t i_latitude, i_longitude, i_altitude;
    int result = getCoords(i_latitude, i_longitude, i_altitude);
    if (!result) {
        return result;
    }

    latitude = (double)i_latitude / 1e7;
    longitude = (double)i_longitude / 1e7;
    altitude = (double)i_altitude / 100.0;

    return 0;
}

double getDistanceBetween(
    double latitude1, double longitude1, double latitude2, double longitude2
) {
    latitude1 *= DEG_TO_RAD;
    longitude1 *= DEG_TO_RAD;
    latitude2 *= DEG_TO_RAD;
    longitude2 *= DEG_TO_RAD;

    double d = cos(latitude1) * cos(latitude2) * (1.0 - cos(longitude2 - longitude1));
    d += 1.0;
    d -= cos(latitude2 - latitude1);
    d = 2.0 * EARTH_RADIUS * asin(sqrt(d / 2.0));

    return d;
}

int getDistanceTo(double latitude, double longitude, double& distance) {
    const double EARTH_RADIUS = 6371000.0;

    double curr_lat, curr_lon, curr_alt;
    int result = getCoordsDouble(curr_lat, curr_lon, curr_alt);
    if (result) {
        return result;
    }

    distance = getDistanceBetween(curr_lat, curr_lon, latitude, longitude);

    return 0;
}

double getHorizontalSpeed() {
    static bool hasValue = false;
    static double lat, lon, lastLat, lastLon;
    static double speed = 0.0;
    static double lastUpdateTime = 0.0;

    double time = getSystemTime();
    if (time - lastUpdateTime > SPEED_UPDATE_DURATION) {
        lastLat = lat;
        lastLon = lon;

        double alt;
        if (getCoordsDouble(lat, lon, alt)) {
            fprintf(stderr, "[Error] getHorizontalSpeed: Unable to read coords\n");
        } else if (hasValue) {
            double d = getDistanceBetween(lat, lon, lastLat, lastLon);
            speed = d / (time - lastUpdateTime);
        }

        hasValue = true;
        lastUpdateTime = time;
    }

    return speed;
}

double getVerticalSpeed() {
    static bool hasValue = false;
    static double alt, lastAlt;
    static double speed = 0.0;
    static double lastUpdateTime = 0.0;

    double time = getSystemTime();
    if (time - lastUpdateTime > SPEED_UPDATE_DURATION) {
        lastAlt = alt;

        double lat, lon;
        if (getCoordsDouble(lat, lon, alt)) {
            fprintf(stderr, "[Error] getVerticalSpeed: Unable to read coords\n");
        } else if (hasValue) {
            speed = (alt - lastAlt) / (time - lastUpdateTime);
        }

        hasValue = true;
        lastUpdateTime = time;
    }

    return speed;
}

void getHomeCoord(double& latitude, double& longitude, double& altitude) {
    latitude = homeLatitude;
    longitude = homeLongitude;
    altitude = homeAltitude;
}

uint32_t getNextCommandIndex() {
    static uint32_t nextIndex = 0;
    static double lastUpdateTime = 0.0;

    double time = getSystemTime();
    if (time - lastUpdateTime > COMMAND_UPDATE_DURATION) {
        double currLat, currLon, currAlt;
        if (getCoordsDouble(currLat, currLon, currAlt)) {
            fprintf(stderr, "[Error] getNextCommandIndex: Unable to read coords\n");
            return nextIndex;
        }

        auto command = commands[nextIndex];
        switch (command.type) {
            case CommandType::HOME: {
                homeLatitude = currLat;
                homeLongitude = currLon;
                homeAltitude = currAlt;
                ++nextIndex;
                break;
            }
            case CommandType::TAKEOFF: {
                double targetAlt = (double)command.content.takeoff.altitude / 100.0 + homeAltitude;
                if (abs(currAlt - targetAlt) < VERTICAL_THRESHOLD) {
                    ++nextIndex;
                }
                break;
            }
            case CommandType::WAYPOINT: {
                auto waypoint = command.content.waypoint;
                double d = getDistanceBetween(
                    currLat, currLon,
                    (double)waypoint.latitude / 1e7,
                    (double)waypoint.longitude / 1e7
                );
                double h = abs(currAlt - homeAltitude - (double)waypoint.altitude / 100.0);
                if (d < HORIZONTAL_THRESHOLD && h < VERTICAL_THRESHOLD) {
                    oldWaypoint = waypoint;
                    ++nextIndex;
                }
                break;
            }
            default:
                ++nextIndex;
                break;
        }

        lastUpdateTime = time;
    }

    if (nextIndex >= commandNum) {
        nextIndex = commandNum - 1;
    }

    return nextIndex;
}

MissionCommand* getNextCommand() {
    return commands + getNextCommandIndex();
}

CommandWaypoint getOldWaypoint() {
    return oldWaypoint;
}
