#ifndef GLOVE_H
#define GLOVE_H

#include <vector>
#include <string>
#include <iostream>
#include <Arduino.h>

//FOR UNIVERSAL IDS OF GESTURE MESSAGE PACKET
//ALL OF THESE VALUES ARE COMPLETELY ARBITRARY, THESE ARE MADE TO MAKE CODING EASIER
//notes: to call from an enum class, you type "name of class"::"entry in class"

// Defines for the different hand orientations
#define FINGER_UP   0b00000001
#define FINGER_DOWN 0b00000010
#define THUMB_UP    0b00000100
#define THUMB_DOWN  0b00001000
#define PALM_UP     0b00010000
#define PALM_DOWN   0b00100000

// Defines for the different finger flexion states
#define FLEX 0x00
#define EXTD 0x01

// Defines for drone identifiers
#define DRN_1 0b00000001
#define DRN_2 0b00000010
#define DRN_3 0b00000100
#define DRN_4 0b00001000

// Defines for controller phase
#define PHASE_CALIBRATION 0x01
#define PHASE_SELECTION   0x02
#define PHASE_CONTROL     0x03

//Commands for controlling the drone
typedef enum {
    CMD_LEFT,
    CMD_RIGHT,
    CMD_FORWARD,
    CMD_BACK,
    CMD_ASCEND,
    CMD_DESCEND,
    CMD_REGULAR_SHUTOFF,
    CMD_EMERGENCY_SHUTOFF,
    CMD_INVALID
} DroneCommand;

//Extract certain bits from data package, args are dataPackage, position of first bit, and length of data to extract
uint8_t extractBits(uint32_t dataPackage, uint8_t position, uint8_t length){
    return (dataPackage >> (position - 1)) & (0xFF << (32 - length));
}

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

    void printPitch(){
        Serial.printf("Pitch = %d\r\n degrees", pitch);
    }

    void printRoll(){
        Serial.printf("Roll = %d\r\n degrees", roll);
    }

private:    
    uint8_t orientation;

    int16_t ax, ay, az;
    int16_t gx, gy, gz;

    float pitch;
    float roll;
};

//Gesture class
//This class is now repurposed to be a means of categorizing gestures and comparing them to the current gesture
//This class also includes the command equivalent of the assigned gesture
class gesture {
public:
    //Create a gesture object with a name and a set of finger states.
    gesture() {}        //constructor
    ~gesture() {}       //deconstructor

    //Add an allowed hand orientation for this gesture. This is for gestures that can have multiple hand orientations.
    void addAllowedOrientation(std::string orientation){
        allowedOrientations.push_back(orientation);
    }

    //Set the flexion states of the fingers for this gesture. The order is thumb, index, middle, ring, pinky.
    void setFingerStates(uint8_t thumb, uint8_t index, uint8_t middle, uint8_t ring, uint8_t pinky) {
        fingerStates[0] = thumb;
        fingerStates[1] = index;
        fingerStates[2] = middle;
        fingerStates[3] = ring;
        fingerStates[4] = pinky;
    }

    //Get the flexion states of the fingers for this gesture.
    void getFingerStates(uint8_t array[5]) {
        array = fingerStates;
    }

    //Get the flexion state of the thumb.
    uint8_t getThumbState() {
        return fingerStates[0];
    }

    //Get the flexion state of the index finger.
    uint8_t getIndexFingerState() {
        return fingerStates[1];
    }

    //Get the flexion state of the middle finger.
    uint8_t getMiddleFingerState() {
        return fingerStates[2];
    }

    //Get the flexion state of the ring finger.
    uint8_t getRingFingerState() {
        return fingerStates[3];
    }

    //Get the flexion state of the pinky finger.
    uint8_t getPinkyFingerState() {
        return fingerStates[4];
    }

private:
    std::string name = "Null Gesture";
    uint8_t fingerStates[5] = {0,0,0,0,0};
    std::vector<std::string> allowedOrientations;
};

#endif