#ifndef GLOVE_H
#define GLOVE_H

#include <vector>
#include <string>
#include <iostream>
#include <stdint.h>
#include <math.h>

//FOR UNIVERSAL IDS OF GESTURE MESSAGE PACKET
// Defines for the different hand orientations
#define FINGER_UP   0b00000001
#define FINGER_DOWN 0b00000110
#define THUMB_UP    0b00000010
#define THUMB_DOWN  0b00000101
#define PALM_UP     0b00000100
#define PALM_DOWN   0b00000011
#define NULL        0b00000000

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
typedef enum : uint8_t {
    CMD_LEFT                = 3,
    CMD_RIGHT               = 4,
    CMD_FORWARD             = 5,
    CMD_BACK                = 6,
    CMD_ASCEND              = 1,
    CMD_DESCEND             = 2,
    CMD_REGULAR_SHUTOFF     = 7,
    CMD_EMERGENCY_SHUTOFF   = 7,
    CMD_INVALID             = 0   
} DroneCommand;



enum phase{
    PHASE_SELECTION,
    PHASE_CONTROL
};

//Omitted idle state since hardly realizable on glove
class FlexSensor {
public:
    //constructor
    FlexSensor(int pin);

    //update raw value
    void updateRaw();

    //scales sensor range to whatever user specifies it to be
    void adjustScale(int newMin, int newMax);

    int getRaw();

    int getScaled();

    //this requires actual calibration depending on user hand flex ranges
    uint8_t flexCheck();
    
private:
    void autoCalibrate();

    void scale(int* valueToScale, int input);

    int rawValue;
    int pin;

    int max = 0;
    int min = 10000;

    int scaledVal;
    int scaledMax;
    int scaledMin;

    int flexState;
};

class tiltSensor {
//the following are the key motions of the wrist: flexion, extension, radial deviation, ulnar deviation, pronation and supination
//radial and ulnar deviation would require either a magnetometer to detect so will be dropped for now
public:
    tiltSensor(MPU6050* mpu);
    ~tiltSensor();

    void setAccelValues();                      //Gets acceleration values from MPU6050
    void setFingerUpRange(int pitchFromUp);
    void setFingerDownRange(int pitchFromDown);
    void setThumbUpRange(int input);
    void setThumbDownRange(int input);
    void setPalmUpRange(int input);
    void setPalmDownRange(int input);

    uint8_t getOrientation();

private:
    MPU6050* mpu;    
    int16_t ax, ay, az;

    float pitch;
    float roll;

    int fingerUpRange;
    int fingerDownRange;
    int thumbUpRange;
    int thumbDownRange;
    int palmUpRange;
    int palmDownRange;
};

//Gesture class
//This class is now repurposed to be a means of categorizing gestures and comparing them to the current gesture
//This class also includes the command equivalent of the assigned gesture
class gesture {
public:
    //Create a gesture object with a name and a set of finger states.
    gesture();
    ~gesture();       //deconstructor

    void updateID(uint8_t ID);
    uint8_t getGestureID() const;    //Get the sensor data of the gesture

    int getGesturePriority() const;

    void setOrientation(uint8_t inputOrientation);    //Set the BASE orientation to be used for IDing a gesture.
    void addOrientation(uint8_t inputOrientation);    //If a gesture is valid for multiple SPECIFIC BUT NOT ALL orientations, add them using this function.
    uint8_t getOrientation() const;    //Returns the orientation of the gesture. If multiple orientations are allowed, only the first orientation is returned.

    void setFingerStates(uint8_t thumb, uint8_t index, uint8_t middle, uint8_t ring, uint8_t pinky) {}    //Set the flexion states of the fingers for this gesture. The order is thumb, index, middle, ring, pinky.
    void addFingerStates(uint8_t thumb, uint8_t index, uint8_t middle, uint8_t ring, uint8_t pinky){}    //If a gesture requires multiple possible finger states, add them using this function.
    uint8_t getFingerStates() const;   //Returns flexion states. As in the gestureID of this gesture with orientation omitted.

    void assignDroneCommand(DroneCommand command); //Assigns a specific command to the gesture. Only one command is allowed.
    DroneCommand getDroneCommand () const;

    void setDroneID(uint8_t droneNum); //Assigns a DroneID for this gesture. Only one ID is allowed.
    uint8_t getDroneID() const;

    bool checkGesture(uint8_t currentGesture) const;    //Check if the gesture matches the current gesture

    //this is used for sorting gestures in a set
    bool operator < (const gesture& otherGesture) const;
    bool operator > (const gesture& otherGesture) const;
    bool operator == (const gesture& otherGesture) const;
    void operator = (gesture& replacementGesture);

    //removes all gesture information
    void clearGesture();
    void clearCommand();

private:
    uint8_t gestureID;                  //Sensor data formatted such that the first 5 bits are the finger states and the last 3 bits are the hand orientation
    DroneCommand assignedDroneCommand;  //The DRONE command that is assigned to the gesture
    bool assignedDroneSelect;
    uint8_t droneID;                        //Corresponding drone ID for this gesture

    uint8_t flexion;
    bool multipleFlexionStates;
    std::vector<uint8_t> allowedFlexionStates;
    
    uint8_t orientation;        //This is the base orientation of the gesture (the first orientation added)
    bool multipleOrientations;
    std::vector<uint8_t> allowedOrientations;

    int gesturePriority;
};

#endif // GLOVE_H