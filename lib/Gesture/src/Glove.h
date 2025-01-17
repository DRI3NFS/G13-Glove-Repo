#ifndef Glove.h
#define Glove.h

#include <vector>
#include <string>
#include <iostream>
#include <Arduino.h>

//i want to eventually implement these to standardize of the orientation values
enum class orient{
    UP,
    FWD,
    DWN
};

enum class fingerState{
    FLEX,
    IDLE,
    EXTD
};

class FlexSensor {
public:
    //constructor
    FlexSensor(int pin);

    //update raw value
    void updateRaw(){   //must be updated to automatically read from pin within header rather from main.cpp
        rawValue = analogRead(pin);
    }

    //manual calibration must also be implemented for when a user has to manually calibrate sensors
    void calibrateIdleMin(int idledNum){
        scale(&scaledIdleMin, idledNum);
    }

    void calibrateIdleMax(int idledNum){
        scale(&scaledIdleMax, idledNum);
    }

    //ideally call this value throughout process to automatically update flex sensor range
    void autoCalibrate() {
        if (rawValue > max) max = rawValue;
        if (rawValue < min) min = rawValue;
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
    int flexCheck(){
        if(scaledVal > 30 && scaledVal <60){
            return 0;                           //this is idle state
        }
        else if(scaledVal < 30){  
            return -1;                          //this is flexed state
        }
        else{
            return 1;                           //this is extended state
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
    int rawValue;
    int pin;

    int max = 0;
    int min = 10000;

    int scaledVal;
    int scaledMax;
    int scaledMin;

    int scaledIdleMin;    //raw sensor value indicating idle hand state post full flexion
    int scaledIdleMax;    //raw sensor value indicating idle hand state post full extension

    void scale(int* valueToScale, int input){
        int range = max - min;
        if (range == 0) range = 1;  //to eliminate divide by 0 error
        int scaledRange = scaledMax - scaledMin;

        *valueToScale = (((input - min) * (scaledRange)) / (range)) + scaledMin;
    }
};

FlexSensor::FlexSensor(int pin){ //declare what adc pin it is connected on esp32
    this->pin = pin;
};

class tiltSensor {
//the following are the key motions of the wrist: flexion, extension, radial deviation, ulnar deviation, pronation and supination
//the goal of this class is to classify what type of movements occured on the wrist using roll and pitch values from two MPU6050s
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
    std::string getOrientation(){
        if(roll <= -45 && roll >= -135 && pitch >= -45 && pitch <= 45){
            orientation = "thumb up";
            return orientation;
        }

        else if(roll <= 45 && roll >= -45 && pitch >= -45 && pitch <= 45){
            orientation = "palm down";
            return orientation;
        }

        else if(roll <= 135 && roll >= 45 && pitch >= -45 && pitch <= 45){
            orientation = "thumb down";
            return orientation;
        }

        else if( ((roll <= -135 && roll >=-180)||(roll <= 180 && roll >= 135)) && pitch >= -45 && pitch <= 45){
            orientation = "palm up";
            return orientation;
        }

        if(pitch <= -45){
            orientation = "pointed down";
            return orientation;
        }

        else if(pitch >= 45){
            orientation = "pointed up";
            return orientation;
        }

        else{
            orientation = "no assigned orientation";
            return orientation;
        };
    }

    //add a print function
    void printOrientation(){
        Serial.print("Orientation is ");
        Serial.println(orientation.c_str());
    }

    void printPitch(){
        Serial.printf("Pitch = %d\r\n", pitch);
    }

    void printRoll(){
        Serial.printf("Roll = %d\r\n", roll);
    }

private:    
    std::string orientation;

    int16_t ax, ay, az;
    int16_t gx, gy, gz;

    float pitch;
    float roll;
};

//Gesture class, the idea is that this code will collect all translated values from the sensor to a smaller data type ie an array of integers
class gesture {
public:
    gesture() {
        name = "";
        fingerStates = {0, 0, 0, 0, 0};
        orientations = {"", ""};
    }

    void setGesture(std::string name, std::vector<int> fingerNum) {
        this->name = name;                      //this is the data that will be sent to the computer
        this->fingerStates = fingerNum;      //this is most likely the value being sent to base computer
    }

    void setHandOrientation(std::string orient){
        this->orientations[0] = orient;
    }

    std::vector<int> getFingerStates() {
        return this->fingerStates;
    }

    std::string getHandOrientation(){
        return orientations[0];
    }

    std::string getForearmOrientation(){
        return orientations[1];
    }

    std::string getName(){
        return name;
    }

    void printFingerStates(){
        Serial.print("Finger states for ");
        Serial.print(this->name.c_str());
        Serial.print(" is: ");
        for (int i = 0; i < fingerStates.size(); i++){
            Serial.printf("%d ",fingerStates[i]);
        }
        Serial.println("");
    }
    
    void printOrientations(){
        Serial.print("Hand and Forearm Orientation for ");
        Serial.print(this->name.c_str());
        Serial.print(" are: ");
        Serial.print(orientations[0].c_str());
        Serial.print(" and ");
        Serial.print(orientations[1].c_str());
        Serial.println("");
    }

    //this is currently designed to only account for hand orientation, not forearm (assume that for now gestures are independent from the forearm motion)
    bool isThis(gesture Gesture4Comp) {
        if(this->checkFlex(Gesture4Comp.getFingerStates())
        && this->checkOrientHand(Gesture4Comp.getHandOrientation())){
            return true;
        }
        else{
            return false;
        }
    }

    //thumb up = 0, palm down = 1, thumb down = 2, palm up = 3, point down = 4, point up = 5
    std::vector<uint8_t> dataToSend(){
        for (int i = 0; i < 5; i++){
            data[i] = fingerStates[i];
        }
        bool j;
        while(j){
            if(this->orientations[1] == "thumb up"){
                data[5] = 0;
                j = false;
            }
            else if(this->orientations[1] == "palm down"){
                data[5] = 1;
                j = false;
            }
            else if(this->orientations[1] == "thumb down"){
                data[5] = 2;
                j = false;
            }
            else if(this->orientations[1] == "palm up"){
                data[5] = 3;
                j = false;
            }
            else if(this->orientations[1] == "pointed down"){
                data[5] = 4;
                j = false;
            }
            else if(this->orientations[1] == "pointed up"){
                data[5] = 5;
                j = false;
            }
            else {
                data[5] = 6; // no assigned orientation
                j = false;
            }
        }
        return data;
    }

private:
    bool checkFlex(std::vector<int> fingerStatesRef){
        if (fingerStates == fingerStatesRef){
            return true;
        }
        else{
            return false;
        }
    }

    bool checkOrientHand(std::string handOrient){
        if (orientations[0] == handOrient){
            return true;
        }
        else{
            false;
        }
    }

    std::string name; //IF YOU WANT TO CHANGE THE DATA TYPE OF THE GESTURE, CHANGE IT HERE
    std::vector<int> fingerStates = {0,0,0,0,0};
    
    tiltSensor handTilt;
    tiltSensor wristTilt;

    std::vector<std::string> orientations = {"",""}; // first element is hand, second element is forearm
    std::vector<uint8_t> data = {0,0,0,0,0,0};
};

#endif