#include "../include/mission.h"
#include "../../shared/include/initialization_interface.h"
#include "../../shared/include/ipc_messages_initialization.h"
#include "../../shared/include/ipc_messages_autopilot_connector.h"
#include "../../shared/include/ipc_messages_credential_manager.h"
#include "../../shared/include/ipc_messages_navigation_system.h"
#include "../../shared/include/ipc_messages_periphery_controller.h"
#include "../../shared/include/ipc_messages_server_connector.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

#define RETRY_DELAY_SEC 1
#define RETRY_REQUEST_DELAY_SEC 5
#define FLY_ACCEPT_PERIOD_US 500000

extern MissionCommand *commands;
extern uint32_t commandNum;

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

double degreeToRadian(double degree){
    return degree * M_PI / 180;
}

int main(void) {
    //Before do anything, we need to ensure, that other modules are ready to work
    while (!waitForInit("periphery_controller_connection", "PeripheryController")) {
        fprintf(stderr, "[%s] Warning: Failed to receive initialization notification from Periphery Controller. Trying again in %ds\n", ENTITY_NAME, RETRY_DELAY_SEC);
        sleep(RETRY_DELAY_SEC);
    }
    while (!waitForInit("autopilot_connector_connection", "AutopilotConnector")) {
        fprintf(stderr, "[%s] Warning: Failed to receive initialization notification from Autopilot Connector. Trying again in %ds\n", ENTITY_NAME, RETRY_DELAY_SEC);
        sleep(RETRY_DELAY_SEC);
    }
    while (!waitForInit("navigation_system_connection", "NavigationSystem")) {
        fprintf(stderr, "[%s] Warning: Failed to receive initialization notification from Navigation System. Trying again in %ds\n", ENTITY_NAME, RETRY_DELAY_SEC);
        sleep(RETRY_DELAY_SEC);
    }
    while (!waitForInit("server_connector_connection", "ServerConnector")) {
        fprintf(stderr, "[%s] Warning: Failed to receive initialization notification from Server Connector. Trying again in %ds\n", ENTITY_NAME, RETRY_DELAY_SEC);
        sleep(RETRY_DELAY_SEC);
    }
    while (!waitForInit("credential_manager_connection", "CredentialManager")) {
        fprintf(stderr, "[%s] Warning: Failed to receive initialization notification from Credential Manager. Trying again in %ds\n", ENTITY_NAME, RETRY_DELAY_SEC);
        sleep(RETRY_DELAY_SEC);
    }
    fprintf(stderr, "[%s] Info: Initialization is finished\n", ENTITY_NAME);

    //Enable buzzer to indicate, that all modules has been initialized
    if (!enableBuzzer())
        fprintf(stderr, "[%s] Warning: Failed to enable buzzer at Periphery Controller\n", ENTITY_NAME);

    //Copter need to be registered at ORVD
    char authResponse[1024] = {0};
    sendSignedMessage("/api/auth", authResponse, "authentication", RETRY_DELAY_SEC);
    fprintf(stderr, "[%s] Info: Successfully authenticated on the server\n", ENTITY_NAME);

    //Constantly ask server, if mission for the drone is available. Parse it and ensure, that mission is correct
    while (true) {
        char missionResponse[1024] = {0};
        if (sendSignedMessage("/api/fmission_kos", missionResponse, "mission", RETRY_DELAY_SEC) && parseMission(missionResponse)) {
            fprintf(stderr, "[%s] Info: Successfully received mission from the server\n", ENTITY_NAME);
            printMission();
            break;
        }
        sleep(RETRY_REQUEST_DELAY_SEC);
    }

    //The drone is ready to arm
    fprintf(stderr, "[%s] Info: Ready to arm\n", ENTITY_NAME);
    while (true) {
        //Wait, until autopilot wants to arm (and fails so, as motors are disabled by default)
        while (!waitForArmRequest()) {
            fprintf(stderr, "[%s] Warning: Failed to receive an arm request from Autopilot Connector. Trying again in %ds\n", ENTITY_NAME, RETRY_DELAY_SEC);
            sleep(RETRY_DELAY_SEC);
        }
        fprintf(stderr, "[%s] Info: Received arm request. Notifying the server\n", ENTITY_NAME);

        //When autopilot asked for arm, we need to receive permission from ORVD
        char armRespone[1024] = {0};
        sendSignedMessage("/api/arm", armRespone, "arm", RETRY_DELAY_SEC);

        if (strstr(armRespone, "$Arm: 0#") != NULL) {
            //If arm was permitted, we enable motors
            fprintf(stderr, "[%s] Info: Arm is permitted\n", ENTITY_NAME);
            while (!setKillSwitch(true)) {
                fprintf(stderr, "[%s] Warning: Failed to permit motor usage at Periphery Controller. Trying again in %ds\n", ENTITY_NAME, RETRY_DELAY_SEC);
                sleep(RETRY_DELAY_SEC);
            }
            if (!permitArm())
                fprintf(stderr, "[%s] Warning: Failed to permit arm through Autopilot Connector\n", ENTITY_NAME);
            break;
        }
        else if (strstr(armRespone, "$Arm: 1#") != NULL) {
            fprintf(stderr, "[%s] Info: Arm is forbidden\n", ENTITY_NAME);
            if (!forbidArm())
                fprintf(stderr, "[%s] Warning: Failed to forbid arm through Autopilot Connector\n", ENTITY_NAME);
        }
        else
            fprintf(stderr, "[%s] Warning: Failed to parse server response\n", ENTITY_NAME);
        fprintf(stderr, "[%s] Warning: Arm was not allowed. Waiting for another arm request from autopilot\n", ENTITY_NAME);
    };

    //If we get here, the drone is able to arm and start the mission
    //The flight is need to be controlled from now on
    //Also we need to check on ORVD, whether the flight is still allowed or it is need to be paused

   while (true){

        //  Speed control   
         
        const double radius_of_earth = 6372795;    

        int32_t lat1,lat2,lon1,lon2,alt1,alt2;

        double cl1,cl2,sl1,sl2,delta,cdelta,sdelta,current_speed,lat1_double,lon1_double,lat2_double,lon2_double,y,x,ad;
        
        getCoords(lat1,lon1,alt1);
        
        lat1_double = (double)lat1 / 10000000;
        lon1_double = (double)lon1 / 10000000;

        fprintf(stderr,">>>>>>>>>>>>>>>>\n");
        fprintf(stderr,"Lat1 %f \n",lat1_double);
        fprintf(stderr,"Lon1 %f \n",lon1_double);

        sleep(3);
        
        getCoords(lat2,lon2,alt2);
        
        lat2_double = (double)lat2 / 10000000;
        lon2_double = (double)lon2 / 10000000;

        fprintf(stderr,">>>>>>>>>>>>>>>>\n");
        fprintf(stderr,"Lat2 %f \n",lat2_double);
        fprintf(stderr,"Lon2 %f \n",lon2_double);

        lat1_double = degreeToRadian(lat1_double);
        lat2_double = degreeToRadian(lat2_double);
        lon1_double = degreeToRadian(lon1_double);
        lon2_double = degreeToRadian(lon2_double);

        cl1 = cos(lat1_double);
        cl2 = cos(lat2_double);
        sl1 = sin(lon1_double);
        sl2 = sin(lon2_double);

        delta = abs(lat2_double - lat1_double);
        cdelta = cos(delta);
        sdelta = sin(delta);

        y = sqrt(pow(cl2*sdelta,2)+pow(cl1*sl2-sl1*cl2*cdelta,2));
        x = sl1*sl2+cl1*cl2*cdelta;

        ad = atan2(y,x);

        current_speed = radius_of_earth * ad;

        fprintf(stderr,">>>>>>>>>>>>>>>>\n");
        fprintf(stderr,"current_speed %f\n",current_speed);

        if(current_speed > 5){
            changeSpeed(3);
        }

        // Latitide control
        
        int32_t alt_assigned,alt_abs;
        bool requestMissionHasAlready = false;

        if (requestMissionHasAlready == false){
            for (int i = 0; i < commandNum; i++) {
                switch (commands[i].type) {
                    case CommandType::TAKEOFF:
                        alt_assigned = commands[i].content.takeoff.altitude;
                        requestMissionHasAlready = true;
                        break;
                    case CommandType::HOME:
                        alt_abs = commands[i].content.waypoint.altitude;
                        break;
                }
            }
        }
        
        alt1 = alt1 - alt_abs - 10;

        fprintf(stderr,"alt_assigned %d\n",alt_assigned);
        fprintf(stderr,"Alt_abs %d\n",alt_abs);
        fprintf(stderr,"Altitude %d\n",alt1);

        if(alt1 > alt_assigned ){
            changeAltitude(alt_assigned);
        }

        // Cargo control

        setCargoLock(0);
    
    }

    return EXIT_SUCCESS;
}
