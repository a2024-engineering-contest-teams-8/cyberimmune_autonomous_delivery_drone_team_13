#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>

#include "../../shared/include/ipc_messages_autopilot_connector.h"
#include "../../shared/include/ipc_messages_periphery_controller.h"
#include "../include/flight_utils.h"

#include "../include/controller.h"

const double STATS_PRINT_DELAY = 1.0;
const double APM_UPDATE_DELAY = 1.0;
const double KILL_SWITCH_UPDATE_DURATION = 1.0;
const double SPEED_UPDATE_DURATION = 0.1;
const double MOVEMENT_UPDATE_DURATION = 0.1;
const double CARGO_UPDATE_DURATION = 0.2;

const double COMING_SPEED_THRESHOLD = 1.5;
const double MAX_COMING_SPEED_THRESHOLD = 0.5;
const double MAX_HORIZONTAL_SPEED = 3.0;
const double MAX_VERTICAL_SPEED = 2.0;
const double MAX_VERTICAL_THRESHOLD = 2.0;

void printRoutine() {
    static double lastUpdateTime = 0.0;

    double time = getSystemTime();
    if (time - lastUpdateTime <= STATS_PRINT_DELAY) {
        return;
    }
    lastUpdateTime = time;

    double lat, lon, alt, homeLat, homeLon, homeAlt, horizSpeed, vertSpeed;
    if (getCoordsDouble(lat, lon, alt)) {
        fprintf(stderr, "[Error] printStats: Unable to read coords\n");
        return;
    }
    horizSpeed = getHorizontalSpeed();
    vertSpeed = getVerticalSpeed();
    getHomeCoord(homeLat, homeLon, homeAlt);

    int nextCommandIndex = getNextCommandIndex();
    MissionCommand* nextCommand = getNextCommand();

    char log[1024];

    fprintf(stderr, "\n");
    fprintf(stderr, "[Info] latitude: %f deg.\n", lat);
    fprintf(stderr, "[Info] longitude: %f deg.\n", lon);
    fprintf(stderr, "[Info] altitude: %f m.\n", alt);
    fprintf(stderr, "[Info] home latitude: %f deg.\n", homeLat);
    fprintf(stderr, "[Info] home longitude: %f deg.\n", homeLon);
    fprintf(stderr, "[Info] home altitude: %f m.\n", homeAlt);
    fprintf(stderr, "[Info] horizontal speed: %f m/s\n", horizSpeed);
    fprintf(stderr, "[Info] vertical speed: %f m/s\n", vertSpeed);
    fprintf(stderr, "[Info] next command index: %i\n", nextCommandIndex);
    fprintf(stderr, "[Info] next command: ");

    switch (nextCommand->type) {
        case CommandType::HOME:
            fprintf(stderr, "HOME");
            break;
        case CommandType::TAKEOFF:
            fprintf(stderr, "TAKEOFF %f m.", (double)nextCommand->content.takeoff.altitude / 100.0);
            break;
        case CommandType::WAYPOINT: {
            auto waypoint = nextCommand->content.waypoint;
            double targetLat = (double)waypoint.latitude / 1e7;
            double targetLon = (double)waypoint.longitude / 1e7;
            fprintf(
                stderr,
                "WAYPOINT lat %f deg, long %f deg, alt %f m., dist: %f m.",
                targetLat,
                targetLon,
                (double)waypoint.altitude / 100.0,
                getDistanceBetween(lat, lon, targetLat, targetLon)
            );
            break;
        }
        case CommandType::LAND:
            fprintf(stderr, "LAND");
            break;
        case CommandType::SET_SERVO: {
            auto servo = nextCommand->content.servo;
            fprintf(stderr, "SERVO number %i, pwm %i", servo.number, servo.pwm);
            break;
        }
    }

    fprintf(stderr, "\n");
}

int32_t getTargetSpeed() {
    switch (getNextCommandIndex()) {
        default:
            return 2;
    }
}

void updateSpeed() {
    changeSpeed(getTargetSpeed());
}

void updateWaypoint() {
    auto command = getNextCommand();
    auto waypoint = command->content.waypoint;
    if (command->type == CommandType::WAYPOINT) {
        changeWaypoint(
            waypoint.latitude,
            waypoint.longitude,
            waypoint.altitude
        );
    }
}

void armRoutine() {
    static double lastUpdateTime = 0.0;
    static bool isArmForbidden = false;

    double time = getSystemTime();
    if (time - lastUpdateTime <= APM_UPDATE_DELAY) {
        return;
    }

    if (!isArmForbidden && !isFlyAccepted()) {
        isArmForbidden = true;
        fprintf(stderr, "[Info] armRoutine: Flight paused\n");
        if (!forbidArm()) {
            fprintf(stderr, "[Error] armRoutine: Failed to forbid arm through Autopilot Connector\n");
        }
        if (!pauseFlight()) {
            fprintf(stderr, "[Error] armRoutine: Failed to pause flight\n");
        }
    } else if (isArmForbidden && isFlyAccepted()) {
        isArmForbidden = false;
        fprintf(stderr, "[Info] armRoutine: Flight resumed\n");
        if (!permitArm())
            fprintf(stderr, "[Error] armRoutine: Failed to permit arm\n");
        if (!resumeFlight()) {
            fprintf(stderr, "[Error] armRoutine: Failed to resume flight\n");
        }
    }
}

void speedRoutine() {
    static double lastUpdateTime = 0.0;

    double horizSpeed = getHorizontalSpeed();
    double vertSpeed = abs(getVerticalSpeed());

    double time = getSystemTime();
    if (time - lastUpdateTime <= SPEED_UPDATE_DURATION) {
        return;
    }
    lastUpdateTime = time;

    if (horizSpeed > MAX_HORIZONTAL_SPEED) {
        updateSpeed();
        fprintf(stderr, "[Info] speedRoutine: horizontal speed limit reached. speed=%f\n", horizSpeed);
    }

    if (vertSpeed > MAX_VERTICAL_SPEED) {
        updateSpeed();
        fprintf(stderr, "[Info] speedRoutine: vertical speed limit reached. speed=%f\n", vertSpeed);
    }
}

void movementRoutine() {
    static double lastUpdateTime = 0.0;
    static double lastPointDistance = 1e10;
    static double takeoffTime = 0.0;
    static int oldWaypointIndex = -1;

    double time = getSystemTime();
    if (time - lastUpdateTime <= MOVEMENT_UPDATE_DURATION) {
        return;
    }
    double deltaTime = time - lastUpdateTime;
    lastUpdateTime = time;

    auto command = getNextCommand();
    bool hasWaypointChanged = false;
    if (getNextCommandIndex() != oldWaypointIndex) {
        oldWaypointIndex = getNextCommandIndex();
        hasWaypointChanged = true;
    }
    
    if (command->type == CommandType::LAND || getNextCommandIndex() == commandNum - 1) {
        pauseFlight();
        return;
    }

    if (command->type != CommandType::WAYPOINT) {
        return;
    }

    auto waypoint = command->content.waypoint;
    
    double currLat, currLon, currAlt;
    int result = getCoordsDouble(currLat, currLon, currAlt);
    if (result) {
        fprintf(stderr, "[Error] movementRoutine: Unable to read coords\n");
        return;
    }

    double homeLat, homeLon, homeAlt;
    getHomeCoord(homeLat, homeLon, homeAlt);

    double pointDistance = getDistanceBetween(
        currLat,
        currLon,
        (double)waypoint.latitude / 1e7,
        (double)waypoint.longitude / 1e7
    );
    double comingSpeed = (pointDistance - lastPointDistance) / deltaTime;
    if (!hasWaypointChanged && pointDistance > HORIZONTAL_THRESHOLD && comingSpeed > MAX_COMING_SPEED_THRESHOLD && getNextCommandIndex() > 8) {
        setKillSwitch(false);
        fprintf(stderr, "[Info] movementRoutine: mission was changed, kill switch was enabled. comingSpeed=%f\n", comingSpeed);
    }
    
    lastPointDistance = pointDistance;

    auto oldWaypoint = getOldWaypoint();
    double d = getDistanceBetween(
        (double)waypoint.latitude / 1e7,
        (double)waypoint.longitude / 1e7,
        (double)oldWaypoint.latitude / 1e7,
        (double)oldWaypoint.longitude / 1e7
    );
    double h = (pointDistance / d) * (
        (double)oldWaypoint.altitude - (double)waypoint.altitude
    ) + (double)waypoint.altitude;
    h /= 100.0;

    if (isinf(h) || isnan(h)) {
        h = (double)waypoint.altitude / 100.0;
    }

    double vertSpeed = getVerticalSpeed();
    double vertDistance = abs(currAlt - homeAlt - h);
    bool isVertOk = (
        (double)waypoint.altitude / 100.0 > (currAlt - homeAlt) ?
        vertSpeed >= 0.0 :
        vertSpeed <= 0.0
    );
    isVertOk = vertDistance < VERTICAL_THRESHOLD;
    if (!hasWaypointChanged && !isVertOk) {
        changeAltitude(waypoint.altitude);
        fprintf(stderr, "[Info] movementRoutine: moving in wrong vertical direction. vertSpeed=%f\n", vertSpeed);
        return;
    }

    if (!hasWaypointChanged && vertDistance > MAX_VERTICAL_THRESHOLD && getNextCommandIndex() > 2) {
        setKillSwitch(false);
        fprintf(stderr, "[Info] movementRoutine: mission was changed, kill switch was enabled. vertDistance=%f\n", vertDistance);
    }
}

void cargoRoutine() {
    static double lastUpdateTime = 0.0;

    double time = getSystemTime();
    if (time - lastUpdateTime <= CARGO_UPDATE_DURATION) {
        return;
    }
    lastUpdateTime = time;

    auto command = getNextCommand();
    auto servo = command->content.servo;

    if (getNextCommandIndex() >= 8) {
        fprintf(stderr, "[Info] cargoRoutine: received command for cargo drop\n");
        setCargoLock(true);
    } else {
        setCargoLock(false);
    }
}

void updateController() {
    getNextCommandIndex();
    getHorizontalSpeed();
    getVerticalSpeed();

    armRoutine();
    printRoutine();
    speedRoutine();
    movementRoutine();
    cargoRoutine();
}
