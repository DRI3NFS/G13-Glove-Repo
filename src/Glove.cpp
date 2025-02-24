//Base Docker Version of Glove Header File
//Only includes Gesture components of the Header File

#include "Glove.h"
#include <algorithm>

//Gesture class
//This class is now repurposed to be a means of categorizing gestures and comparing them to the current gesture
//This class also includes the command equivalent of the assigned gesture
gesture::gesture() 
        :   orientation(0), 
            flexion(0), 
            gestureID(0), 
            multipleOrientations(false),
            multipleFlexionStates(false), 
            assignedDroneCommand(CMD_INVALID),
            droneID(0)
            {}


gesture::~gesture() {}       //deconstructor

void gesture::updateID(uint8_t ID){
    gestureID = ID;
    flexion = ID && 0xF8;
    orientation = ID && 0x07;
}

//Get the sensor data of the gesture
uint8_t gesture::getGestureID() const {
    return gestureID;
}

//Set the BASE orientation to be used for IDing a gesture.
void gesture::setOrientation(uint8_t inputOrientation){
    gestureID = (gestureID & 0xF8) | inputOrientation;           //clear last 3 bits
    orientation = inputOrientation;
}

//If a gesture is valid for multiple SPECIFIC BUT NOT ALL orientations, add them using this function.
void gesture::addOrientation(uint8_t inputOrientation){
    if(!multipleOrientations){
        multipleOrientations = true;
        allowedOrientations = {orientation, inputOrientation};
    }
    else{
        allowedOrientations.push_back(inputOrientation);
    }
}

//Returns the orientation of the gesture. If multiple orientations are allowed, only the first orientation is returned.
uint8_t gesture::getOrientation() const {
    return orientation;
}

//Set the flexion states of the fingers for this gesture. The order is thumb, index, middle, ring, pinky.
void gesture::setFingerStates(uint8_t thumb, uint8_t index, uint8_t middle, uint8_t ring, uint8_t pinky) {
    uint8_t fingerStateBits = 0;
    fingerStateBits |= (index << 7);
    fingerStateBits |= (middle << 6);
    fingerStateBits |= (ring << 5);
    fingerStateBits |= (pinky << 4);
    fingerStateBits |= (thumb << 3);

    gestureID = (gestureID & 0x07) | fingerStateBits;  // Replace the first 5 bits with the new finger states
    flexion = fingerStateBits;
}

//If a gesture requires multiple possible finger states, add them using this function.
void gesture::addFingerStates(uint8_t thumb, uint8_t index, uint8_t middle, uint8_t ring, uint8_t pinky){
    uint8_t fingerStateBits = 0;
    fingerStateBits |= (index << 7);
    fingerStateBits |= (middle << 6);
    fingerStateBits |= (ring << 5);
    fingerStateBits |= (pinky << 4);
    fingerStateBits |= (thumb << 3);

    if (!multipleFlexionStates) {
        multipleFlexionStates = true;
        allowedFlexionStates = {flexion, fingerStateBits};
    } else {
        allowedFlexionStates.push_back(fingerStateBits);
    }
}

//Returns flexion states. As in the gestureID of this gesture with orientation omitted.
uint8_t gesture::getFingerStates() const {
    return flexion;
}

void gesture::assignDroneCommand(DroneCommand command){
    assignedDroneCommand = command;
}

DroneCommand gesture::getDroneCommand() const {
    return assignedDroneCommand;
}

void gesture::setDroneID(uint8_t droneNum){
    droneID = droneNum;
}

uint8_t gesture::getDroneID() const {
    return droneID;
}

//Check if the gesture matches the current gesture
bool gesture::checkGesture(uint8_t currentGesture) const {
    //It is implemented that certain gestures can have specific allowed states.
    //So the cases where multiple states are allowed are handled differently than if
    //a gesture only has one allowed state.

    //Case 1: Any orientation is valid, only finger states are checked
    if (orientation == 0) {
        return !multipleFlexionStates
            ? (currentGesture & 0xF8) == flexion
            : std::find(allowedFlexionStates.begin(), allowedFlexionStates.end(), (currentGesture & 0xF8)) != allowedFlexionStates.end();
    }

    //Case 2: Only one finger state is valid for this gesture
    if (!multipleFlexionStates) {
        if (!multipleOrientations) {
            return currentGesture == gestureID;
        } else {
            return std::any_of(allowedOrientations.begin(), allowedOrientations.end(), 
                [currentGesture](uint8_t orientation, uint8_t flexion) { return currentGesture == (flexion + orientation); });
        }
    }

    //Case 3: Multiple finger states is valid for this gesture but only one orientation
    if (!multipleOrientations) {
        return std::any_of(allowedFlexionStates.begin(), allowedFlexionStates.end(),
            [currentGesture](uint8_t flexion, uint8_t orientation) { return currentGesture == (flexion + orientation); });
    }
    
    //Case 4: Multiple finger states and multiple orientations are valid for this gesture
    return std::any_of(allowedFlexionStates.begin(), allowedFlexionStates.end(),
        [this, currentGesture](uint8_t flexion) {
            return std::any_of(allowedOrientations.begin(), allowedOrientations.end(),
                [currentGesture, flexion](uint8_t orientation) { return currentGesture == (flexion + orientation); });
        });
}

//this is used for sorting gestures in a set
bool gesture::operator < (const gesture& otherGesture) const {
    return gestureID < otherGesture.getGestureID();
}

bool gesture::operator > (const gesture& otherGesture) const {
    return gestureID > otherGesture.getGestureID();
}

bool gesture::operator == (const gesture& otherGesture) const {
    return checkGesture(otherGesture.getGestureID());
}

void gesture::operator = (gesture& replacementGesture){
    gestureID = replacementGesture.getGestureID();
    flexion = replacementGesture.getFingerStates();
    orientation = replacementGesture.getOrientation();
}

//removes all gesture information
void gesture::clearGesture(){
    gestureID = 0;
    flexion = 0;
    orientation = 0;

    assignedDroneCommand = CMD_INVALID;
    multipleFlexionStates = false;
    multipleOrientations = false;

    allowedFlexionStates.clear();
    allowedOrientations.clear();

}

//Standardizes gesture command to Neutral
void gesture::clearCommand(){
    assignedDroneCommand = CMD_INVALID;
}