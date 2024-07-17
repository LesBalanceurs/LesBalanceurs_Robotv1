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
#include "pid.h"

#define HALF_DISTANCE 24 // TODO: Measure the half distance to travel

#define ARM_GOAL_RANGE 5

ArduinoX arduinoX;

PIDI::valeursPID positionPID;
// TODO: Get the real tick per turn
MotorController positionMotorController(&arduinoX, DIRECT, 0, 0, 50, 64, MotorController::wheelRatio(5));
double positionMeasurement() {return positionMotorController.measurementFunction();}
void positionCommand(double command) {positionMotorController.commandFunction(command);}

PIDI::valeursPID armPID;
// TODO: Get the real tick per turn, gearbox ratio and discuss the unit of control (Turns, degrees or radians) for now degrees
MotorController armMotorController(&arduinoX, DIRECT, 1, 1, 50, 64, MotorController::degreesRatio());
double armMeasurement() {return armMotorController.measurementFunction();}
void armCommand(double command) {armMotorController.commandFunction(command);}

bool startArm = false;

/**
 * @brief Program initialisation
 * @author Mathieu Durand
 */
void setup()
{   
    arduinoX.init();

    Serial.begin(9600); // 115200 ou 9600

    //while(!Serial) {}

    delay(500);

    Serial.println("Start!");

    //positionPID.setMeasurementFunc(positionMeasurement);
    //positionPID.setCommandFunc(positionCommand);
    positionPID.set(1, 0, 0, -0.5, 0.5); // TODO: Tune the position PID
    armPID.set(0.005, 0, 0.003, -1, 1); // TODO: Tune the arm PID

    //armPID.enable();

    positionPID.Sp = HALF_DISTANCE;
    armPID.Sp = -90;
}

/**
 * @brief Progam main loop
 * @author Mathieu Durand
*/
void loop()
{
    //Serial.println(positionMeasurement());
    loopPID();
}

/**
 * @brief PID update loop
 * @author Mathieu Durand
*/
void loopPID()
{
    //unsigned long tempTime = millis();
    //period = (tempTime - lastTime);
    //double dt = period / 1000.0;

    //positionPID.enable();

    //positionPID.setPeriod(3);
    //armPID.setPeriod(3);

    //Serial.println(period);

    positionPID.Pv = (float) positionMeasurement();
    armPID.Pv = (float) armMeasurement();

    positionCommand((double) positionPID.update());

    if (abs(positionPID.previous_error) < 2) {
        startArm = true;
    }

    if (startArm) {
        armCommand((double) armPID.update());

        if (abs(armPID.previous_error) < ARM_GOAL_RANGE) {
            armPID.Sp = -armPID.Sp;
        }
    }

    //positionPID.run();
    //armPID.run();

    //lastTime = tempTime;
}