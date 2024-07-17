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

void collectData();

void loopPID();

void transition();
void runStateCode();

// *************************************************************************************************
//  INCLUDES
// *************************************************************************************************	

#include <Arduino.h>
#include <LibS3GRO.h>
#include "MotorController.h"
#include "data_processing/AbsoluteChangeSummator.h"
#include "data_processing/Integrator.h"
#include "timer/Timer.h"
#include "MegaServo/MegaServo.h"

#include "pid.h"

#define HALF_DISTANCE 24
#define ARM_GOAL_RANGE 5

#define POSITION_ENCODER_ID 0
#define ARM_ENCODER_ID 1

#define GRIPPER_PIN 5 // TODO: Get the actual pin number after intalling the servo
#define GRIPPER_ANGLE_OPEN 90
#define GRIPPER_ANGLE_CLOSED 0

ArduinoX arduinoX;

/*============================================================================
							  MOTOR CONTROL
============================================================================*/

PIDI::valeursPID positionPID;
MotorController positionMotorController(&arduinoX, DIRECT, POSITION_ENCODER_ID, POSITION_ENCODER_ID, 50, 64, MotorController::wheelRatio(5));

PIDI::valeursPID armPID;
MotorController armMotorController(&arduinoX, DIRECT, ARM_ENCODER_ID, ARM_ENCODER_ID, 50, 64, MotorController::degreesRatio());

MegaServo gripper;

/*============================================================================
							 DATA COLLECTION
============================================================================*/

AbsoluteChangeSummator<double> distanceTravelled;

double powerConsumption;
Integrator consumedEnergy;

/*============================================================================
							  STATE MACHINE
============================================================================*/

enum RobotState {
    INITIALIZING,
    MOVING_TO_CENTER,      // Robot moves to the center of the track
    ARM_MOVING_TO_PICKUP_ZONE,  // Arm moves toward pickup zone
    WAITING_FOR_CARGO,     // Robot waits for cargo to be loaded
    ARM_MOVING_TO_DROP_ZONE, // Arm moves counter-clockwise
    DROPPING_CARGO_OPEN, // Arm is opening
    DROPPING_CARGO,      // Arm is open and dropping cargo
    DROPPING_CARGO_CLOSE, // Arm is closing after dropping cargo
};

RobotState state = INITIALIZING;

static Timer timer;

/*============================================================================
							  PROGRAM START
============================================================================*/

/**
 * @brief Program initialisation
 * @author Mathieu Durand
 */
void setup()
{   
    arduinoX.init();

    Serial.begin(9600); // 115200 ou 9600

    positionPID.set(1, 0, 0, -0.5, 0.5);
    armPID.set(0.005, 0, 0.003, -1, 1);

    gripper.attach(GRIPPER_PIN);

    transition();
}

/**
 * @brief Progam main loop
 * @author Mathieu Durand
*/
void loop()
{
    runStateCode();
    collectData();
    loopPID();
}

/**
 * @brief Collect data
 * @author Mathieu Durand
*/
void collectData() {
    powerConsumption = (double) arduinoX.getVoltage() * arduinoX.getCurrent();
    consumedEnergy.integrate(powerConsumption);

    distanceTravelled.track(positionMotorController.measurementFunction());
}

/**
 * @brief PID update loop
 * @author Mathieu Durand
*/
void loopPID()
{
    positionPID.Pv = positionMotorController.measurementFunction();
    armPID.Pv = armMotorController.measurementFunction();

    positionMotorController.commandFunction((double) positionPID.update());
    armMotorController.commandFunction((double) armPID.update());
}

/*============================================================================
							  STATE FUNCTIONS
============================================================================*/

// MOVING_TO_CENTER
void movingToCenterInit() {
    positionPID.Sp = HALF_DISTANCE;
    armPID.Sp = 0;
}
void movingToCenter() {
}

// ARM_MOVING_TO_PICKUP_ZONE
void armMovingToPickupZoneInit() {
    armPID.Sp = -90;
}
void armMovingToPickupZone() {
    if (abs(armPID.previous_error) < ARM_GOAL_RANGE) {
        transition();
    }
}

// WAITING_FOR_CARGO
void waitingForCargoInit() {
    timer.start(5.0);
}
void waitingForCargo() {
    if (timer.isTimerOver()) {
        transition();
    }
}

// ARM_MOVING_TO_DROP_ZONE
void armMovingToDropZoneInit() {
    armPID.Sp = 90;
}
void armMovingToDropZone() {
    if (abs(armPID.previous_error) < ARM_GOAL_RANGE) {
        transition();
    }
}

// DROPPING_CARGO_OPEN
void droppingCargoOpenInit() {
    gripper.write(GRIPPER_ANGLE_OPEN);
}
void droppingCargoOpen() {
    if (timer.isTimerOver()) {
        transition();
    }
}

// DROPPING_CARGO
void droppingCargoInit() {
    timer.start(3.0);
}
void droppingCargo() {
    if (timer.isTimerOver()) {
        transition();
    }
}

// DROPPING_CARGO_CLOSE
void droppingCargoCloseInit() {
    gripper.write(GRIPPER_ANGLE_CLOSED);
    timer.start(1.0);
}
void droppingCargoClose() {
    if (timer.isTimerOver()) {
        transition();
    }
}

/*============================================================================
							     STATE LOOP
============================================================================*/

void transition() {
    switch (state) {
        case INITIALIZING:
            state = MOVING_TO_CENTER;
            break;
        case MOVING_TO_CENTER:
            state = ARM_MOVING_TO_PICKUP_ZONE;
            movingToCenterInit();
            break;
        case ARM_MOVING_TO_PICKUP_ZONE:
            state = WAITING_FOR_CARGO;
            armMovingToPickupZoneInit();
            break;
        case WAITING_FOR_CARGO:
            state = ARM_MOVING_TO_DROP_ZONE;
            waitingForCargoInit();
            break;
        case ARM_MOVING_TO_DROP_ZONE:
            state = DROPPING_CARGO_OPEN;
            armMovingToDropZoneInit();
            break;
        case DROPPING_CARGO_OPEN:
            state = DROPPING_CARGO;
            droppingCargoOpenInit();
            break;
        case DROPPING_CARGO:
            state = DROPPING_CARGO_CLOSE;
            droppingCargoInit();
            break;
        case DROPPING_CARGO_CLOSE:
            state = ARM_MOVING_TO_PICKUP_ZONE;
            droppingCargoCloseInit();
            break;
    }
}

void runStateCode() {
    switch (state) {
        case INITIALIZING:
            break;
        case MOVING_TO_CENTER:
            movingToCenter();
            break;
        case ARM_MOVING_TO_PICKUP_ZONE:
            armMovingToPickupZone();
            break;
        case WAITING_FOR_CARGO:
            waitingForCargo();
            break;
        case ARM_MOVING_TO_DROP_ZONE:
            armMovingToDropZone();
            break;
        case DROPPING_CARGO_OPEN:
            droppingCargoOpen();
            break;
        case DROPPING_CARGO:
            droppingCargo();
            break;
        case DROPPING_CARGO_CLOSE:
            droppingCargoClose();
            break;
    }
}