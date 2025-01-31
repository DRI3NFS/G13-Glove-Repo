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

uint32_t dataPackage;                       //This is the data from the glove received from by the computer
uint8_t thumb, index, middle, ring, pinky;  //Flexion data from the glove
uint8_t orientation;                        //Orientation data from the glove
uint8_t phase;                              //Current phase of the glove
uint8_t droneID;                            //Drone IDs to use for selection

void setup(void){
    //the gesture set up can be moved to Glove.cpp on the communicating computer
    //Set up all gestures, gestures declared here are all standardized and are not assigned to specific commands
    gesture pointUp; 
    pointUp.addAllowedOrientation("FINGER_UP");
    pointUp.setFingerStates(FLEX,EXTD,FLEX,FLEX,FLEX);

    gesture pointDown;
    pointDown.addAllowedOrientation("FINGER_DOWN");
    pointDown.setFingerStates(FLEX,EXTD,FLEX,FLEX,FLEX);

    gesture pointForward;
    pointForward.addAllowedOrientation("PALM_DOWN");
    //the rest of these assignments are optional and can be removed if it makes gesture sorting easier
    pointForward.addAllowedOrientation("THUMB_UP");
    pointForward.addAllowedOrientation("THUMB_DOWN");
    pointForward.addAllowedOrientation("PALM_UP");
    pointForward.setFingerStates(FLEX,EXTD,FLEX,FLEX,FLEX);

    gesture thumbsLeft;
    thumbsLeft.addAllowedOrientation("PALM_DOWN");
    thumbsLeft.setFingerStates(EXTD,FLEX,FLEX,FLEX,FLEX);

    gesture thumbsRight;
    thumbsRight.addAllowedOrientation("PALM_UP");
    thumbsRight.setFingerStates(EXTD,FLEX,FLEX,FLEX,FLEX);

    gesture thumbsDown;
    thumbsDown.addAllowedOrientation("THUMB_DOWN");
    thumbsDown.setFingerStates(EXTD,FLEX,FLEX,FLEX,FLEX);

    gesture bird;
    bird.addAllowedOrientation("FINGER_UP");
    bird.setFingerStates(FLEX,FLEX,EXTD,FLEX,FLEX);
}

void loop() {
    //unpack the data package
    //extract flexion data
    thumb   = extractBits(dataPackage, 15, 2);
    index   = extractBits(dataPackage, 23, 2);
    middle  = extractBits(dataPackage, 21, 2);
    ring    = extractBits(dataPackage, 19, 2);
    pinky   = extractBits(dataPackage, 17, 2);

    //unpack orientation data
    orientation = extractBits(dataPackage, 9, 6);

    //unpack current phase
    phase = extractBits(dataPackage, 7, 2);
    droneID = extractBits(dataPackage, 1, 4);
}