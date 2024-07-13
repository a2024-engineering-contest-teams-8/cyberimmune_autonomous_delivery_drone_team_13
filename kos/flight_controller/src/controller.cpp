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

const double MAX_COMING_SPEED_THRESHOLD = 0.4;
const double MAX_HORIZONTAL_SPEED = 2.5;
const double MAX_VERTICAL_SPEED = 2.0;
const double MAX_VERTICAL_THRESHOLD = 2.0;

int32_t getTargetSpeed() {
    switch (getNextCommandIndex()) {
        default:
            return 2;
    }
}

void updateSpeed() {
    changeSpeed(getTargetSpeed());
}

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

    char log[4096];
    switch (nextCommand->type) {
        case CommandType::HOME:
            fprintf(stderr, "HOME");
            snprintf(
                log,
                4096,
                "time%%3A+%.2f,wp%%3A+%02d,+HOME,+h%%3A+%.2f,+alt%%3A+%.2f,+v_speed%%3A+%.2f,+h_speed%%3A+%.2f",
                time,
                nextCommandIndex,
                alt - homeAlt,
                vertSpeed,
                horizSpeed
            );
            break;
        case CommandType::TAKEOFF:
            fprintf(stderr, "TAKEOFF %f m.", (double)nextCommand->content.takeoff.altitude / 100.0);
            snprintf(
                log,
                4096,
                "time%%3A+%.2f,wp%%3A+%02d,+TAKEOFF,+h%%3A+%.2f,+alt%%3A+%.2f,+v_speed%%3A+%.2f,+h_speed%%3A+%.2f",
                time,
                nextCommandIndex,
                (double)nextCommand->content.takeoff.altitude / 100.0,
                alt - homeAlt,
                vertSpeed,
                horizSpeed
            );
            break;
        case CommandType::WAYPOINT: {
            auto waypoint = nextCommand->content.waypoint;
            double targetLat = (double)waypoint.latitude / 1e7;
            double targetLon = (double)waypoint.longitude / 1e7;
            double d = getDistanceBetween(lat, lon, targetLat, targetLon);
            fprintf(
                stderr,
                "WAYPOINT lat %f deg, long %f deg, alt %f m., dist: %f m.\n",
                targetLat,
                targetLon,
                (double)waypoint.altitude / 100.0,
                d
            );
            snprintf(
                log,
                4096,
                "time%%3A+%.2f,wp%%3A+%02d,+WAYPOINT,+d%%3A+%.3f,+alt%%3A+%.2f,+v_speed%%3A+%.2f,+h_speed%%3A+%.2f",
                time,
                nextCommandIndex,
                d,
                alt - homeAlt,
                vertSpeed,
                horizSpeed
            );
            break;
        }
        case CommandType::LAND:
            fprintf(stderr, "LAND");
            snprintf(
                log,
                4096,
                "time%%3A+%.2f,wp%%3A+%02d,+LAND,+alt%%3A+%.2f,+v_speed%%3A+%.2f,+h_speed%%3A+%0.2f",
                time,
                nextCommandIndex,
                alt - homeAlt,
                vertSpeed,
                horizSpeed
            );
            break;
        case CommandType::SET_SERVO: {
            auto servo = nextCommand->content.servo;
            fprintf(stderr, "SERVO number %i, pwm %i", servo.number, servo.pwm);
            snprintf(
                log,
                4096,
                "time%%3A+%.2f,wp%%3A+%02d,+SET_SERVO,+alt%%3A+%.2f,+v_speed%%3A+%.2f,+h_speed%%3A+%.2f",
                time,
                nextCommandIndex,
                alt - homeAlt,
                vertSpeed,
                horizSpeed
            );
            break;
        }
    }
    fprintf(stderr, "\n");

    sendLogs(log);
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
        sendLogs("Flight+paused");
        fprintf(stderr, "[Info] armRoutine: Flight paused\n");
        if (!forbidArm()) {
            fprintf(stderr, "[Error] armRoutine: Failed to forbid arm through Autopilot Connector\n");
        }
        if (!pauseFlight()) {
            fprintf(stderr, "[Error] armRoutine: Failed to pause flight\n");
        }
    } else if (isArmForbidden && isFlyAccepted()) {
        isArmForbidden = false;
        sendLogs("Flight+resumed");
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
        sendLogs("Horiz.+speed+limit+reached");
        fprintf(stderr, "[Info] speedRoutine: horizontal speed limit reached. speed=%f\n", horizSpeed);
    }
}

void movementRoutine() {
    static double lastUpdateTime = 0.0;
    static double lastPointDistance = 1e10;
    static double takeoffTime = 0.0;

    double time = getSystemTime();
    if (time - lastUpdateTime <= MOVEMENT_UPDATE_DURATION) {
        return;
    }
    double deltaTime = time - lastUpdateTime;
    lastUpdateTime = time;

    auto command = getNextCommand();
    
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
    if (pointDistance > HORIZONTAL_THRESHOLD && comingSpeed > MAX_COMING_SPEED_THRESHOLD && getNextCommandIndex() > 9) {
        setKillSwitch(false);
        sendLogs("Kill+switch+activated");
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
    if (vertDistance > 0.5) {
        changeAltitude(waypoint.altitude);
        sendLogs("Moving+in+invalid+vertical+direction");
        fprintf(stderr, "[Info] movementRoutine: moving in wrong vertical direction. vertSpeed=%f\n", vertSpeed);
    }
}

void cargoRoutine() {
    static double lastUpdateTime = 0.0;
    static bool cargoLock = false;

    double time = getSystemTime();
    if (time - lastUpdateTime <= CARGO_UPDATE_DURATION) {
        return;
    }
    lastUpdateTime = time;

    auto command = getNextCommand();
    auto servo = command->content.servo;

    if (!cargoLock && getNextCommandIndex() >= 9) {
        cargoLock = true;
        sendLogs("Cargo+activated");
        fprintf(stderr, "[Info] cargoRoutine: received command for cargo drop\n");
        setCargoLock(true);
    } else if (cargoLock && getNextCommandIndex() < 8) {
        cargoLock = false;
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
