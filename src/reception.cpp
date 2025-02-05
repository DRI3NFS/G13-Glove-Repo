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
#include <stdint.h>
#include <algorithm>

uint8_t gloveData;                                  //This is the data from the glove received from by the computer
int controlledDrone;                                //This is the drone ID being controlled during the control phase

enum phase{
    PHASE_SELECTION,
    PHASE_CONTROL
};

int currentPhase = PHASE_SELECTION;

//the gesture set up can be moved to Glove.cpp on the communicating computer
//Set up all gestures, gestures declared here are all standardized and are not assigned to specific commands
gesture pointUp;
gesture pointDown;
gesture pointForward;
gesture thumbsBack;
gesture thumbsLeft;
gesture thumbsRight;
gesture thumbsDown;
gesture thumbsUp;
gesture okay;

std::vector<gesture> gestureBank;                          //This is the gesture bank that will be used to check the gestures
gesture currentGesture;                                    //This is the gesture that the glove is currently doing
gesture foundGesture;                                      //This is the gesture that the program found
int selectedDrone;                                     //This is the drone currently selected during select phase

void setup(void){
    //When it comes to declaring gestures, you have the choice of declaring only orientation,
    //only finger states, or both. If you don't declare one or the other, the gesture will
    //automatically set them as 0. For example, if you declare finger flexion for a gesture
    //you cannot use the gesture's ID for reference, but you can use the gesture's finger states.

    pointUp.setOrientation(FINGER_UP);
    pointUp.setFingerStates(FLEX,EXTD,FLEX,FLEX,FLEX);
    pointDown.addFingerStates(EXTD,EXTD,FLEX,FLEX,FLEX);    
    pointUp.assignDroneCommand(CMD_ASCEND);
    gestureBank.push_back(pointUp);

    pointDown.setOrientation(FINGER_DOWN);
    pointDown.setFingerStates(FLEX,EXTD,FLEX,FLEX,FLEX);
    pointDown.addFingerStates(EXTD,EXTD,FLEX,FLEX,FLEX);
    pointDown.assignDroneCommand(CMD_DESCEND);
    gestureBank.push_back(pointDown);

    pointForward.setOrientation(THUMB_UP);
    pointForward.addOrientation(PALM_UP);
    pointForward.addOrientation(PALM_DOWN);
    pointForward.setFingerStates(FLEX,EXTD,FLEX,FLEX,FLEX);
    pointForward.assignDroneCommand(CMD_FORWARD);
    gestureBank.push_back(pointForward);

    thumbsBack.setOrientation(FINGER_UP);
    thumbsBack.setFingerStates(EXTD,FLEX,FLEX,FLEX,FLEX);
    thumbsBack.assignDroneCommand(CMD_BACK);
    gestureBank.push_back(thumbsBack);

    thumbsLeft.setOrientation(PALM_DOWN);
    thumbsLeft.setFingerStates(EXTD,FLEX,FLEX,FLEX,FLEX);
    thumbsLeft.assignDroneCommand(CMD_LEFT);
    gestureBank.push_back(thumbsLeft);

    thumbsRight.setOrientation(PALM_UP);
    thumbsRight.setFingerStates(EXTD,FLEX,FLEX,FLEX,FLEX);
    thumbsRight.assignDroneCommand(CMD_RIGHT);
    gestureBank.push_back(thumbsRight);

    thumbsDown.setOrientation(THUMB_DOWN);
    thumbsDown.setFingerStates(EXTD,FLEX,FLEX,FLEX,FLEX);
    thumbsDown.assignDroneCommand(CMD_EMERGENCY_SHUTOFF);
    gestureBank.push_back(thumbsDown);

    okay.setFingerStates(FLEX,FLEX,EXTD,EXTD,EXTD);
    okay.assignDroneCommand(CMD_REGULAR_SHUTOFF);           //assigned as regular shutoff as this command will return the program to "Select Phase"
    gestureBank.push_back(okay);

    //sort the gesture bank, make sure this is at the end of the setup
    std::sort(gestureBank.begin(), gestureBank.end());
}

void loop() {
    currentGesture.updateID(gloveData);
    //Steps will be as follows:
    // 1. Check the gestureID that is received from the glove and identify what gesture is being done.
    //This returns a pointer to the gesture, not the object itself.
    std::vector<gesture>::iterator it = std::find_if(gestureBank.begin(), gestureBank.end(), [](const gesture& g){ //this lambda function is used to check the gesture
        return g.checkGesture(gloveData);
    });

    if(it != gestureBank.end()){
        foundGesture = *it;           //could return an error
    }

    else if(it == gestureBank.end()){
        foundGesture.clearCommand();  //defaults to invalid if no gesture is matched
    }

    // 2. Check the current phase of the program and execute the corresponding command.
    switch(currentPhase){
        case PHASE_SELECTION:
            uint8_t checkFlex = currentGesture.getFingerStates();

            if(checkFlex = 0x80){       //select drone 1
                selectedDrone = 1;
                //message glove to light up indicator for drone 1
                break;
            }

            else if(checkFlex = 0xC0){  //select drone 2
                selectedDrone = 2;
                //message glove to light up indicator for drone 2
                break;
            }

            else if(checkFlex = 0x10){  //raise pinky to select all drones
                selectedDrone = 0;
                //message glove to light up all drone indicators
                break;
            }

            if (currentGesture == thumbsUp){
                controlledDrone = selectedDrone;
                currentPhase = PHASE_CONTROL;
                //message glove to light up "In Control" indicator
                //message glove to turn off "Select Drone" indicator
                break;
            }

        case PHASE_CONTROL:
            DroneCommand droneCommand = currentGesture.getDroneCommand();

            //we want to send droneCommand to the correct drone
            //Control code here

            //Send command to drone

    }
}