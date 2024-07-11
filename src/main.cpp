/**************************************************************************************************
Nom du fichier : main.cpp
Auteur : Mathieu Durand
Date de création : 2024/7/11

Description : Fichier principale Arduino
              
Notes : 

Modifications : 

***************************************************************************************************/

// *************************************************************************************************
//  HEADER
// *************************************************************************************************	

void loopPID();

// *************************************************************************************************
//  INCLUDES
// *************************************************************************************************	

#include <Arduino.h>
#include <LibS3GRO.h>
#include "MotorController.h"

#define HALF_DISTANCE 50 // TODO: Measure the half distance to travel

ArduinoX arduinoX;

PID positionPID;
// TODO: Get the real tick per turn and wheel diameter
MotorController positionMotorController(&arduinoX, 0, 0, 50, 25, MotorController::wheelRatio(5));
double positionMeasurement() {return positionMotorController.measurementFunction();}
void positionCommand(double command) {positionMotorController.commandFunction(command);}

PID armPID;
// TODO: Get the real tick per turn, gearbox ratio and discuss the unit of control (Turns, degrees or radians) for now degrees
MotorController armMotorController(&arduinoX, 0, 0, 50, 25, MotorController::degreesRatio());
double armMeasurement() {return armMotorController.measurementFunction();}
void armCommand(double command) {armMotorController.commandFunction(command);}

/**
 * @brief Program initialisation
 * @author Mathieu Durand
 */
void setup()
{   
    arduinoX.init();

    positionPID.setMeasurementFunc(positionMeasurement);
    positionPID.setCommandFunc(positionCommand);
    positionPID.setGains(1, 0, 0); // TODO: Tune the position PID

    armPID.setMeasurementFunc(armMeasurement);
    armPID.setCommandFunc(armCommand);
    armPID.setGains(1, 0, 0); // TODO: Tune the arm PID

    positionPID.setGoal(HALF_DISTANCE);

    Serial.begin(115200); // 115200 ou 9600
}

/**
 * @brief Progam main loop
 * @author Mathieu Durand
*/
void loop()
{
    loopPID();
}

/**
 * @brief PID update loop
 * @author Mathieu Durand
*/
void loopPID()
{
    positionPID.run();
    armPID.run();
}