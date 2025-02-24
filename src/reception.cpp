//  "reception.cpp" by Ralph Ralion Lejano
//  This file is a modified version of Edcel's file "publisher.cpp"
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

#include "Glove.h"

uint8_t gloveData;                                  //This is the data from the glove received from by the computer

//the gesture set up can be moved to Glove.cpp on the communicating computer
//Set up all gestures, gestures declared here are all standardized and are not assigned to specific commands
gesture pointUp;
gesture pointDown;
gesture pointForward;
gesture thumbsBack;
gesture thumbsLeft;
gesture thumbsRight;

//counting gestures
gesture oneGesture;
gesture twoGesture;
gesture threeGesture;
gesture fourGesture;
gesture thumbsDown;
gesture thumbsUp;

//for phase selection purposes
gesture switchPhase;
int currentPhase = PHASE_SELECTION;

//gesture banks
std::vector<gesture> droneGestureBank;                          //Gesture Bank for Drone Command Controls
std::vector<gesture> selectGestureBank;                        //Gesture Bank for Selection

gesture currentGesture;
uint8_t controlledDrones;                                        //drone ID for selected drones
uint8_t selectedDrone;

void setup(void){
    //When it comes to declaring gestures, you have the choice of declaring only orientation,
    //only finger states, or both. If you don't declare one or the other, the gesture will
    //automatically set them as 0. For example, if you declare finger flexion for a gesture
    //you cannot use the gesture's ID for reference, but you can use the gesture's finger states.

    pointUp.setOrientation(FINGER_UP);
    pointUp.setFingerStates(FLEX,EXTD,FLEX,FLEX,FLEX);
    pointDown.addFingerStates(EXTD,EXTD,FLEX,FLEX,FLEX);    
    pointUp.assignDroneCommand(CMD_ASCEND);
    droneGestureBank.push_back(pointUp);

    pointDown.setOrientation(FINGER_DOWN);
    pointDown.setFingerStates(FLEX,EXTD,FLEX,FLEX,FLEX);
    pointDown.addFingerStates(EXTD,EXTD,FLEX,FLEX,FLEX);
    pointDown.assignDroneCommand(CMD_DESCEND);
    droneGestureBank.push_back(pointDown);

    pointForward.setOrientation(THUMB_UP);
    pointForward.addOrientation(PALM_UP);
    pointForward.addOrientation(PALM_DOWN);
    pointForward.setFingerStates(FLEX,EXTD,FLEX,FLEX,FLEX);
    pointForward.assignDroneCommand(CMD_FORWARD);
    droneGestureBank.push_back(pointForward);

    thumbsBack.setOrientation(FINGER_UP);
    thumbsBack.setFingerStates(EXTD,FLEX,FLEX,FLEX,FLEX);
    thumbsBack.assignDroneCommand(CMD_BACK);
    droneGestureBank.push_back(thumbsBack);

    thumbsLeft.setOrientation(PALM_DOWN);
    thumbsLeft.setFingerStates(EXTD,FLEX,FLEX,FLEX,FLEX);
    thumbsLeft.assignDroneCommand(CMD_LEFT);
    droneGestureBank.push_back(thumbsLeft);

    thumbsRight.setOrientation(PALM_UP);
    thumbsRight.setFingerStates(EXTD,FLEX,FLEX,FLEX,FLEX);
    thumbsRight.assignDroneCommand(CMD_RIGHT);
    droneGestureBank.push_back(thumbsRight);

    thumbsDown.setOrientation(THUMB_DOWN);
    thumbsDown.setFingerStates(EXTD,FLEX,FLEX,FLEX,FLEX);
    thumbsDown.assignDroneCommand(CMD_EMERGENCY_SHUTOFF);
    droneGestureBank.push_back(thumbsDown);

    thumbsUp.setOrientation(THUMB_UP);
    thumbsUp.setFingerStates(EXTD,FLEX,FLEX,FLEX,FLEX);
    selectGestureBank.push_back(thumbsUp);

    //general gestures for numbers
    //orientation is set as "FINGER_UP" so that the program only counts it if the user has their hand up
    oneGesture.setFingerStates(FLEX,EXTD,FLEX,FLEX,FLEX);
    oneGesture.addFingerStates(FLEX,FLEX,FLEX,FLEX,EXTD);
    oneGesture.setOrientation(FINGER_UP);
    selectGestureBank.push_back(oneGesture);

    twoGesture.setFingerStates(FLEX,EXTD,EXTD,FLEX,FLEX);
    twoGesture.addFingerStates(FLEX,FLEX,FLEX,EXTD,EXTD);
    twoGesture.setOrientation(FINGER_UP);
    selectGestureBank.push_back(twoGesture);

    threeGesture.setFingerStates(FLEX, EXTD, EXTD, EXTD, FLEX);
    threeGesture.addFingerStates(FLEX, FLEX, EXTD, EXTD, EXTD);
    threeGesture.setOrientation(FINGER_UP);
    selectGestureBank.push_back(threeGesture);

    fourGesture.setFingerStates(FLEX, EXTD, EXTD, EXTD, EXTD);
    fourGesture.setOrientation(FINGER_UP);
    selectGestureBank.push_back(fourGesture);

    switchPhase.setFingerStates(FLEX, EXTD, EXTD, FLEX, FLEX);
    switchPhase.setOrientation(FINGER_DOWN);
    selectGestureBank.push_back(switchPhase);
}

void loop() {
    //Steps will be as follows:
    // 1. Check the gestureID that is received from the glove and identify what gesture is being done.
    //This returns a pointer to the gesture, not the object itself.
    // 2. Check the current phase of the program and execute the corresponding command.
    switch(currentPhase){
        case PHASE_SELECTION:
            std::vector<gesture>::iterator it = std::find_if(selectGestureBank.begin(), selectGestureBank.end(), [](const gesture& g){
                return g.checkGesture(gloveData);   //equality condition
            });

            //case where a gesture is recognized
            if(it != selectGestureBank.end()){
                uint8_t droneID = (*it).getDroneID();

                //Drone selection, if the command has no assigned command then the switch simply breaks
                switch(droneID){
                    case 1:
                        selectedDrone = DRN_1;
                        break;
                    case 2:
                        selectedDrone = DRN_2;
                        break;
                    case 3:
                        selectedDrone = DRN_3;
                        break;
                    case 4:
                        selectedDrone = DRN_4;
                        break;
                    default:
                        break;
                }

                //confirm selection
                if((*it) == thumbsUp){
                    controlledDrones |= selectedDrone;
                }

                //reject selection
                else if((*it) == thumbsDown){
                    uint8_t mask = ~selectedDrone;
                    controlledDrones &= mask;
                }

                else if((*it) == switchPhase){
                    currentPhase = PHASE_CONTROL;
                }

                break;
            }

            //case where a gesture isn't recognized, program ignores
            else if(it == selectGestureBank.end()){
                break;
            }
        case PHASE_CONTROL:
            DroneCommand message;

            std::vector<gesture>::iterator it = std::find_if(droneGestureBank.begin(), droneGestureBank.end(), [](const gesture& g){
                return g.checkGesture(gloveData);   //equality condition
            });

            if(it != droneGestureBank.end()){
            //we want to send droneCommand to the correct drone
            //Control code here

            //Send command to drone
            }

            else if(it == droneGestureBank.end()){
            //Send CMD_INVALID as a default case
            }

    }
}