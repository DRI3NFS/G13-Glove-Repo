#ifndef GLOVE_H
#define GLOVE_H

#include <vector>
#include <string>
#include <iostream>
#include <Arduino.h>
#include <stdint.h>

//FOR UNIVERSAL IDS OF GESTURE MESSAGE PACKET
// Defines for the different hand orientations
#define FINGER_UP   0b00000001
#define FINGER_DOWN 0b00000110
#define THUMB_UP    0b00000010
#define THUMB_DOWN  0b00000101
#define PALM_UP     0b00000100
#define PALM_DOWN   0b00000011

// Defines for the different finger flexion states
#define FLEX 0x00
#define EXTD 0x01

// Defines for drone identifiers
#define DRN_1 0b00000001
#define DRN_2 0b00000010
#define DRN_3 0b00000100
#define DRN_4 0b00001000
#define DRN_5 0b00010000

//Commands for controlling the drone
typedef enum {
    CMD_LEFT                = 1,
    CMD_RIGHT               = 2,
    CMD_FORWARD             = 3,
    CMD_BACK                = 4,
    CMD_ASCEND              = 5,
    CMD_DESCEND             = 6,
    CMD_REGULAR_SHUTOFF     = 7,
    CMD_EMERGENCY_SHUTOFF   = 8,
    CMD_INVALID             = 0   
} DroneCommand;

//Omitted idle state since hardly realizable on glove
class FlexSensor {
public:
    //constructor
    FlexSensor(int pin);

    //update raw value
    void updateRaw(){   //must be updated to automatically read from pin within header rather from main.cpp
        rawValue = analogRead(pin);
        this->autoCalibrate();
    }

    //scales sensor range to whatever user specifies it to be
    void adjustScale(int newMin, int newMax){
        scaledMax = newMax;
        scaledMin = newMin;

        scale(&scaledVal, rawValue);
    }

    int getRaw(){
        return rawValue;
    }

    int getScaled(){
        return scaledVal;
    }

    //this requires actual calibration depending on user hand flex ranges
    uint8_t flexCheck(){
        if(scaledVal > 60){
            return EXTD;  //this is extended
        }
        else if(scaledVal <= 60){  
            return FLEX;  //this is flexed state
        }
        else{
            return 0;     //this is an error state
        }
    }

    //add a print function
    void printRaw(){
        Serial.printf("Flex at Pin %d has the value %d.", pin, rawValue);
    }

    void printScaled(){
        Serial.printf("Flex at Pin %d has the scaled value from %d to %d of %d.",
        pin,
        scaledMin,
        scaledMax,
        scaledVal);
    }

private:
    void autoCalibrate() {
        if (rawValue > max) max = rawValue;
        if (rawValue < min) min = rawValue;
    }

    void scale(int* valueToScale, int input){
        int range = max - min;
        if (range == 0) range = 1;  //to eliminate divide by 0 error
        int scaledRange = scaledMax - scaledMin;

        *valueToScale = (((input - min) * (scaledRange)) / (range)) + scaledMin;
    }

    int rawValue;
    int pin;

    int max = 0;
    int min = 10000;

    int scaledVal;
    int scaledMax;
    int scaledMin;
};

FlexSensor::FlexSensor(int pin){ //declare what adc pin it is connected on esp32
    this->pin = pin;
};

class tiltSensor {
//the following are the key motions of the wrist: flexion, extension, radial deviation, ulnar deviation, pronation and supination
//radial and ulnar deviation would require either a magnetometer to detect so will be dropped for now
public:
    void setAccelValues(int16_t MPUax, int16_t MPUay, int16_t MPUaz){
        ax = MPUax;
        ay = MPUay;
        az = MPUaz;

        //automated the conversion to pitch and roll which i assume are more useful values
        pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180 / PI;
        roll = atan2(ay, az) * 180 / PI;
    }

    float getPitch(){
        return pitch;
    }

    float getRoll(){
        return roll;
    }

    //these values are assigned for right hand orientations with your arm pointing forward
    //all of these values purely depend on my own arm's range of motion and are arbitrary
    //thumb up, thumb down, palm down, palm up, point down, point up
    uint8_t getOrientation(){
        if(roll <= -45 && roll >= -135 && pitch >= -45 && pitch <= 45){
            orientation = THUMB_UP;
            return orientation;
        }

        else if(roll <= 45 && roll >= -45 && pitch >= -45 && pitch <= 45){
            orientation = PALM_DOWN;
            return orientation;
        }

        else if(roll <= 135 && roll >= 45 && pitch >= -45 && pitch <= 45){
            orientation = THUMB_UP;
            return orientation;
        }

        else if( ((roll <= -135 && roll >=-180)||(roll <= 180 && roll >= 135)) && pitch >= -45 && pitch <= 45){
            orientation = PALM_UP;
            return orientation;
        }

        if(pitch <= -45){
            orientation = FINGER_DOWN;
            return orientation;
        }

        else if(pitch >= 45){
            orientation = FINGER_UP;
            return orientation;
        }

        else{
            orientation = NULL;
            return orientation;
        };
    }

private:    
    uint8_t orientation;
    int16_t ax, ay, az;

    float pitch;
    float roll;
};

//Gesture class
//This class is now repurposed to be a means of categorizing gestures and comparing them to the current gesture
//This class also includes the command equivalent of the assigned gesture
class gesture {
public:
    //Create a gesture object with a name and a set of finger states.
    gesture() : orientation(0), 
                flexion(0), 
                gestureID(0), 
                multipleOrientations(false),
                multipleFlexionStates(false), 
                assignedDroneCommand(CMD_INVALID){}
    
    ~gesture() {}       //deconstructor

    void updateID(uint8_t ID){
        gestureID = ID;
        flexion = ID && 0xF8;
        orientation = ID && 0x07;
    }

    //Set the BASE orientation to be used for IDing a gesture.
    void setOrientation(uint8_t inputOrientation){
        gestureID = gestureID & 0xF8;           //clear last 3 bits

        gestureID = gestureID + inputOrientation;    //replaces the last three bits with the new orientation
        orientation = inputOrientation;
    }

    //If a gesture is valid for multiple SPECIFIC BUT NOT ALL orientations, add them using this function.
    void addOrientation(uint8_t inputOrientation){
        if(!multipleOrientations){
            multipleOrientations = true;

            allowedOrientations.push_back(this->orientation);
            allowedOrientations.push_back(inputOrientation);
        }
        else{
            allowedOrientations.push_back(inputOrientation);
        }
    }

    //Returns the orientation of the gesture. If multiple orientations are allowed, only the first orientation is returned.
    uint8_t getOrientation(){
        return orientation;
    }

    //Set the flexion states of the fingers for this gesture. The order is thumb, index, middle, ring, pinky.
    void setFingerStates(uint8_t thumb, uint8_t index, uint8_t middle, uint8_t ring, uint8_t pinky) {
        uint8_t fingerHandle = 0xF8;    //0b11111000    
        gestureID = gestureID & 0x07;   //clear first 5 bits

        fingerHandle = fingerHandle & (thumb << 3);
        fingerHandle = fingerHandle & (index << 7);
        fingerHandle = fingerHandle & (middle << 6);
        fingerHandle = fingerHandle & (ring << 5);
        fingerHandle = fingerHandle & (pinky << 4);

        gestureID = gestureID + fingerHandle;
        flexion = fingerHandle;
    }

    //If a gesture requires multiple possible finger states, add them using this function.
    void addFingerStates(uint8_t thumb, uint8_t index, uint8_t middle, uint8_t ring, uint8_t pinky){
        uint8_t fingerHandle = 0xF8;    //0b11111000

        fingerHandle = fingerHandle & (thumb << 3);
        fingerHandle = fingerHandle & (index << 7);
        fingerHandle = fingerHandle & (middle << 6);
        fingerHandle = fingerHandle & (ring << 5);
        fingerHandle = fingerHandle & (pinky << 4);

        if(!multipleFlexionStates){
            multipleFlexionStates = true;

            allowedFlexionStates.push_back(this->flexion);
        }
        else{
            allowedFlexionStates.push_back(fingerHandle);
        }
    }

    //Returns flexion states. As in the gestureID of this gesture with orientation omitted.
    uint8_t getFingerStates() {
        return flexion;
    }

    //Get the sensor data of the gesture
    uint8_t getGestureID() const {
        return gestureID;
    }

    void assignDroneCommand(DroneCommand command){
        assignedDroneCommand = command;
    }

    DroneCommand getDroneCommand(){
        return assignedDroneCommand;
    }

    //Check if the gesture matches the current gesture
    bool checkGesture(uint8_t currentGesture) const{
        //It is implemented that certain gestures can have specific allowed states.
        //So the cases where multiple states are allowed are handled differently than if
        //a gesture only has one allowed state.

        //If the gesture does not have an assigned orientation, only the finger states are checked.
        if(orientation == 0){
            if((currentGesture & 0xF8) == flexion){
                return true;
            }
            else{
                return false;
            }
        }

        //If the gesture has assigned orientation, the function will check depending the following
        //four cases:   1. If the gesture has only one orientation and only one flexion state
        //              2. If the gesture has multiple orientations and only one flexion state
        //              3. If the gesture has only one orientation and multiple flexion states
        //              4. If the gesture has multiple orientations and multiple flexion states
        if(!multipleFlexionStates){
            if(!multipleOrientations){
                if (currentGesture == gestureID) {
                    return true;
                }
                else {
                    return false;
                }
            }
            else if(multipleOrientations){
                for (int i = 0; i < allowedOrientations.size(); i++) {
                    if (currentGesture == (flexion + allowedOrientations[i])) {
                        return true;
                    }
                }
                return false;
            }
        }

        else if(multipleFlexionStates){
            if(!multipleOrientations){
                for (int i = 0; i < allowedFlexionStates.size(); i++) {
                    if (currentGesture == (allowedFlexionStates[i] + orientation)) {
                        return true;
                    }
                }
                return false;
            }
            else if(multipleOrientations){
                for (int i = 0; i < allowedFlexionStates.size(); i++) {
                    for (int j = 0; j < allowedOrientations.size(); j++) {
                        if (currentGesture == (allowedFlexionStates[i] + allowedOrientations[j])) {
                            return true;
                        }
                    }
                }
                return false;
            }
        }
    }

    //this is used for sorting gestures in a set
    bool operator < (const gesture& otherGesture) const {
        return gestureID < otherGesture.getGestureID();
    }

    bool operator > (const gesture& otherGesture) const {
        return gestureID > otherGesture.getGestureID();
    }

    bool operator == (const gesture& otherGesture) const {
        return this->checkGesture(otherGesture.getGestureID());
    }

    //removes all gesture information
    void clearGesture(){
        gestureID = 0;
        flexion = 0;
        orientation = 0;
    }

    void clearCommand(){
        assignedDroneCommand = CMD_INVALID;
    }

private:
    uint8_t gestureID;                  //Sensor data formatted such that the first 5 bits are the finger states and the last 3 bits are the hand orientation
    DroneCommand assignedDroneCommand;  //The DRONE command that is assigned to the gesture

    uint8_t flexion;
    bool multipleFlexionStates;
    std::vector<uint8_t> allowedFlexionStates;
    
    uint8_t orientation;        //This is the base orientation of the gesture (the first orientation added)
    bool multipleOrientations;
    std::vector<uint8_t> allowedOrientations;
};

#endif // GLOVE_H