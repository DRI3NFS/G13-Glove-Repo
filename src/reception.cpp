//  "reception.cpp" by Ralph Ralion Lejano
//  The purpose of this code is to decode the data being received from the glove.
//  The first addressable issue is to optimally decode what gesture the glove is sending using the values received from the gloves
//  Then the code will convert this gesture into a command that will be sent to the drones.
//  This code will be combined with Edcel's send code to send commands to the drone.

//The initial gesture-to-drone-command mapping is as follows:
// 1. Index finger pointing up : ascend
// 2. Index finger pointing down : descend
// 3. Index finger pointing forward : forward
// 4. Thumb pointing backward : backward
// 5. Thumb pointing left : left
// 6. Thumb pointing right : right
// 7. Thumb's down : regular shutoff
// 8. Middle finger: emergency shutoff

#include <Arduino.h>
#include "Glove.h"
#include <iostream>
#include <string>
#include <vector>

uint8_t gloveData;                                //This is the data from the glove received from by the computer
uint8_t phase = PHASE_CONTROL;                      //Current phase of the glove
uint8_t droneID;                                    //Drone IDs to use for selection

//the gesture set up can be moved to Glove.cpp on the communicating computer
//Set up all gestures, gestures declared here are all standardized and are not assigned to specific commands
gesture pointUp;
gesture pointDown;
gesture pointForward;
gesture thumbsBack;
gesture thumbsLeft;
gesture thumbsRight;
gesture thumbsDown;
gesture bird;

void setup(void){
    pointUp.setOrientation(FINGER_UP);
    pointUp.setFingerStates(FLEX,EXTD,FLEX,FLEX,FLEX);

    pointDown.setOrientation(FINGER_DOWN);
    pointDown.setFingerStates(FLEX,EXTD,FLEX,FLEX,FLEX);

    pointForward.setOrientation(THUMB_UP);
    pointForward.setFingerStates(FLEX,EXTD,FLEX,FLEX,FLEX);

    thumbsBack.setOrientation(FINGER_UP);
    thumbsBack.setFingerStates(EXTD,FLEX,FLEX,FLEX,FLEX);

    thumbsLeft.setOrientation(PALM_DOWN);
    thumbsLeft.setFingerStates(EXTD,FLEX,FLEX,FLEX,FLEX);

    thumbsRight.setOrientation(PALM_UP);
    thumbsRight.setFingerStates(EXTD,FLEX,FLEX,FLEX,FLEX);

    thumbsDown.setOrientation(THUMB_DOWN);
    thumbsDown.setFingerStates(EXTD,FLEX,FLEX,FLEX,FLEX);

    bird.setOrientation(FINGER_UP);
    bird.setFingerStates(FLEX,FLEX,EXTD,FLEX,FLEX);
}

void loop() {
    switch(phase){
        case PHASE_CALIBRATION:
            //Code for calibrating the glove this will most likely get moved to the glove main.cpp
            break;
        case PHASE_SELECTION:
            //Code for selecting which drone to use
            break;
        case PHASE_CONTROL:
            //Control code here
            DroneCommand command;

            if(pointUp.checkGesture(gloveData)){
                command = CMD_ASCEND;
            }
            else if(pointDown.checkGesture(gloveData)){
                command = CMD_DESCEND;
            }
            else if(pointForward.checkGesture(gloveData)){
                command = CMD_FORWARD;
        
            }
            else if(thumbsLeft.checkGesture(gloveData)){
                command = CMD_LEFT;
            }
            else if(thumbsRight.checkGesture(gloveData)){
                command = CMD_RIGHT;
            }
            else if(thumbsDown.checkGesture(gloveData)){
                command = CMD_REGULAR_SHUTOFF;
            }
            else if(bird.checkGesture(gloveData)){
                command = CMD_EMERGENCY_SHUTOFF;
            }
            else{
            //return drone float state;
            }
            //Send command to drone

    }
}