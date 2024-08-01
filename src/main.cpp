/**************************************************************************************************
Nom du fichier : main.cpp
Auteur : Mathieu Durand
Date de création : 2024/7/11

Description : Fichier principale Arduino
              
Notes : 

Modifications : 

***************************************************************************************************/
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
#include "gpsp/GPSP.h"

// *************************************************************************************************
//  HEADER
// *************************************************************************************************    

void collectData();

void loopPID();

void transition();
void runStateCode();

void report();

void sendJson(char* key, double value, bool start = false, bool end = false);
void sendJson(char* key, String value, bool start = false, bool end = false);

double getPendulumAngle();

double getScore();

#define JUMP_START_PIN 3

#define POTENTIOMETER_TO_ANGLE 225 / 1024
#define PENDULUM_ANGLE_PIN A4

#define HALF_DISTANCE 26.5
#define DISTANCE_GOAL_RANGE 1
#define ARM_GOAL_RANGE 5
#define ARM_DERIVATIVE_GOAL_RANGE 0.01

#define POSITION_ENCODER_ID 0
#define ARM_ENCODER_ID 1

#define GRIPPER_PIN 2 // TODO: Get the actual pin number after intalling the servo

#define GRIPPER_ANGLE_OPEN 120 // 120
#define GRIPPER_ANGLE_CLOSED 0 // 35

#define FIR_COUNT_PER_DROP 4

#define DISTANCE_TRAVELLED_SCORE_METER //DISTANCE_TRAVELLED_SCORE_CENTIMETER

ArduinoX arduinoX;

/*============================================================================
                              MOTOR CONTROL
============================================================================*/

PIDI::valeursPID positionPID;
MotorController positionMotorController(&arduinoX, DIRECT, POSITION_ENCODER_ID, POSITION_ENCODER_ID, 50, 64, MotorController::wheelRatio(5));

PIDI::valeursPID armPID;
MotorController armMotorController(&arduinoX, DIRECT, ARM_ENCODER_ID, ARM_ENCODER_ID, 50 * 3, 64, MotorController::degreesRatio());

MegaServo gripper;

/*============================================================================
                             DATA COLLECTION
============================================================================*/

AbsoluteChangeSummator<double> distanceTravelled;

double powerConsumption;
Integrator consumedEnergy;

unsigned long dropped_fir_count = 0;

double startAngle;

/*============================================================================
                              STATE MACHINE
============================================================================*/

enum RobotState {
    INITIALIZING,
    MOVING_TO_CENTER,      // Robot moves to the center of the track
    ARM_MOVING_TO_PICKUP_ZONE,  // Arm moves toward pickup zone
    WAITING_FOR_CARGO,     // Robot waits for cargo to be loaded
    LOADING_CARGO_CLOSE,
    ARM_MOVING_TO_DROP_ZONE, // Arm moves counter-clockwise
    DROPPING_CARGO_OPEN, // Arm is opening
    DROPPING_CARGO,      // Arm is open and dropping cargo
    DONE,
};

RobotState state = INITIALIZING; // INITIALIZING

Timer timer;

String stateToString(RobotState state);

/*============================================================================
                                 SERIAL
============================================================================*/

Timer serialTimer;
Timer competitionTimer;

/*============================================================================
                              PROGRAM START
============================================================================*/

GPSP protocol(Serial);

void PID_S(Stream &stream, const char args[][50], int size) {  
    armPID.Kp = atof(args[0]);
    armPID.Ki = atof(args[1]);
    armPID.Kd = atof(args[2]);
}

void P(Stream &stream, const char args[][50], int size) {  
    armPID.Kp = atof(args[0]);
}

void I(Stream &stream, const char args[][50], int size) {  
    armPID.Ki = atof(args[0]);
}

void D(Stream &stream, const char args[][50], int size) {  
    armPID.Kd = atof(args[0]);
}

void GET_P(Stream &stream, const char args[][50], int size) {
    stream.print("P: ");
    stream.println(armPID.Kp, 10);
}

void GET_I(Stream &stream, const char args[][50], int size) {  
    stream.print("I: ");
    stream.println(armPID.Ki, 10);
}

void GET_D(Stream &stream, const char args[][50], int size) {  
    stream.print("D: ");
    stream.println(armPID.Kd, 10);
}

void GET_PID(Stream &stream, const char args[][50], int size) {  
    stream.print("P: ");
    stream.print(armPID.Kp, 10);

    stream.write('\t');

    stream.print("I: ");
    stream.print(armPID.Ki, 10);

    stream.write('\t');

    stream.print("D: ");
    stream.println(armPID.Kd, 10);
}

void START(Stream &stream, const char args[][50], int size) {
    state = INITIALIZING;
    transition();
}

void STOP(Stream &stream, const char args[][50], int size) {
    state = DONE;
}

/**
 * @brief Program initialisation
 * @author Mathieu Durand
 */
void setup()
{   
    arduinoX.init();

    Serial.begin(115200); // 115200 ou 9600

    positionPID.set(0.5, 0.01, 0.001, -0.5, 0.5);
    armPID.set(0.1 / 2, 0.001, 0.06 / 3.0, -0.75, 0.75); //0.005 //armPID.set(0.1, 0.001, 0.06, -1, 1); //0.005 //armPID.set(0.01, 0, 0, -0.3, 0.3);

    gripper.attach(GRIPPER_PIN);

    pinMode(JUMP_START_PIN, INPUT_PULLUP);
    
    protocol.defineCommand({PID_S, "PID_S", ""});
    protocol.defineCommand({P, "P", ""});
    protocol.defineCommand({I, "I", ""});
    protocol.defineCommand({D, "D", ""});
    protocol.defineCommand({GET_PID, "GET_PID", ""});
    protocol.defineCommand({GET_P, "GET_P", ""});
    protocol.defineCommand({GET_I, "GET_I", ""});
    protocol.defineCommand({GET_D, "GET_D", ""});

    protocol.defineCommand({START, "START", "Start or Restart sequence"});
    protocol.defineCommand({STOP, "STOP", "Stop the sequence"});

    competitionTimer.start(180.0);

    pinMode(PENDULUM_ANGLE_PIN, INPUT);
    startAngle = analogRead(PENDULUM_ANGLE_PIN);
}

/**
 * @brief Progam main loop
 * @author Mathieu Durand
*/
void loop()
{
    protocol.update();
    collectData();
    loopPID();
    runStateCode();

    //Serial.println(analogRead(PENDULUM_ANGLE_PIN));

    if (serialTimer.isTimerOver()) {
        double time = millis() / 1000.0;

        sendJson("energy", consumedEnergy.getIntegral(), true, true);

        sendJson("distance", distanceTravelled.getSum(), true, true);

        sendJson("time", time, true, false);
        sendJson("angle", getPendulumAngle(), false, true);

        sendJson("time", time, true, false);
        sendJson("position", positionMotorController.measurementFunction(), false, true);

        sendJson("score", getScore(), true, true);

        serialTimer.start(1.0);
    }

    // Jump Start Logic
    if (state == INITIALIZING && !digitalRead(JUMP_START_PIN)) {
        transition();
    }

    //if (competitionTimer.isTimerOver()) {
      //  state = DONE;
        //report();
    //}
}

void sendJson(char* key, double value, bool start = false, bool end = false) {
  if (start) {
    Serial.print("{");
  }

  Serial.write('"');
  Serial.print(key);
  Serial.print("\": ");

  Serial.print(value);

  if (!end) {
    Serial.print(", ");
  } else {
    Serial.println("}");
    Serial.flush();
  }
}

void sendJson(char* key, String value, bool start = false, bool end = false) {
  if (start) {
    Serial.print("{");
  }

  Serial.write('"');
  Serial.print(key);
  Serial.print("\": ");

  Serial.print(value);

  if (!end) {
    Serial.print(", ");
  } else {
    Serial.println("}");
    Serial.flush();
  }
}

double getPendulumAngle() {
    return (analogRead(PENDULUM_ANGLE_PIN) - startAngle) * POTENTIOMETER_TO_ANGLE;
}

double getScore() {
    double score = dropped_fir_count * dropped_fir_count + 1.5 * dropped_fir_count;
    #ifdef DISTANCE_TRAVELLED_SCORE_METER
    #endif
    score -= distanceTravelled.getSum() * 0.0254;
    #ifdef DISTANCE_TRAVELLED_SCORE_CENTIMETER
    score -= distanceTravelled.getSum() * 2.54;
    #endif

    return score;
}

void report() {
    Serial.print("Voltage: ");
    Serial.print(arduinoX.getVoltage());
    Serial.print("V");

    Serial.write('\t');
    Serial.print("Power: ");
    Serial.print(powerConsumption);
    Serial.print("W");

    Serial.write('\t');
    Serial.print("Energy consumed: ");
    Serial.print(consumedEnergy.getIntegral());
    Serial.print("J");

    Serial.write('\t');
    Serial.print("Distance: ");
    Serial.print(distanceTravelled.getSum());
    Serial.print(" in");

    Serial.write('\t');
    Serial.print("State: ");
    Serial.print(stateToString(state));

    Serial.write('\t');
    Serial.print("Dropped Fir Count: ");
    Serial.print(dropped_fir_count);

    double score = dropped_fir_count * dropped_fir_count + 1.5 * dropped_fir_count;
    #ifdef DISTANCE_TRAVELLED_SCORE_METER
    #endif
    score -= distanceTravelled.getSum() * 0.0254;
    #ifdef DISTANCE_TRAVELLED_SCORE_CENTIMETER
    score -= distanceTravelled.getSum() * 2.54;
    #endif

    Serial.write('\t');
    Serial.print("Score: ");
    Serial.print(getScore()); // d is in meters but may be in centimeter at the competition

    Serial.write('\t');
    Serial.print("Time left: ");
    Serial.println(competitionTimer.getTimeLeftSeconds()); // d is in meters but may be in centimeter at the competition
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
    gripper.write(GRIPPER_ANGLE_OPEN);
    positionPID.Sp = HALF_DISTANCE;
    armPID.Sp = 0;
}
void movingToCenter() {
    if (abs(positionPID.previous_error) < DISTANCE_GOAL_RANGE) {
        transition();
    }
}

// ARM_MOVING_TO_PICKUP_ZONE
void armMovingToPickupZoneInit() {
    armPID.Sp = 0;
}
void armMovingToPickupZone() {
    if (abs(armPID.previous_error) < ARM_GOAL_RANGE && abs(armPID.previous_derivative) < ARM_DERIVATIVE_GOAL_RANGE) {
        transition();
    }
}

// WAITING_FOR_CARGO
void waitingForCargoInit() {
    timer.start(1.0); // 5.0
}
void waitingForCargo() {
    if (timer.isTimerOver()) {
        transition();
    }
}

// LOADING_CARGO_CLOSE
void loadingCargoCloseInit() {
    gripper.write(GRIPPER_ANGLE_CLOSED);
    timer.start(1.0); // 1.0
}
void loadingCargoClose() {
    if (timer.isTimerOver()) {
        transition();
    }
}

// ARM_MOVING_TO_DROP_ZONE
void armMovingToDropZoneInit() {
    armPID.Sp = 180;
}
void armMovingToDropZone() {
    if (abs(armPID.previous_error) < ARM_GOAL_RANGE && abs(armPID.previous_derivative) < ARM_DERIVATIVE_GOAL_RANGE) {
        transition();
    }
}

// DROPPING_CARGO_OPEN
void droppingCargoOpenInit() {
    gripper.write(GRIPPER_ANGLE_OPEN);
    timer.start(0.0); // 0.0;
}
void droppingCargoOpen() {
    if (timer.isTimerOver()) {
        transition();
    }
}

// DROPPING_CARGO
void droppingCargoInit() {
    dropped_fir_count += FIR_COUNT_PER_DROP;
    timer.start(1.0); // 3.0
}
void droppingCargo() {
    if (timer.isTimerOver()) {
        transition();
    }
}

// DROPPING_CARGO
void stopInit() {

}
void stopCargo() {
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
            state = WAITING_FOR_CARGO;
            break;
        case ARM_MOVING_TO_PICKUP_ZONE:
            state = WAITING_FOR_CARGO;
            break;
        case WAITING_FOR_CARGO:
            state = LOADING_CARGO_CLOSE;
            break;
        case LOADING_CARGO_CLOSE:
            state = ARM_MOVING_TO_DROP_ZONE;
            break;
        case ARM_MOVING_TO_DROP_ZONE:
            state = DROPPING_CARGO_OPEN;
            break;
        case DROPPING_CARGO_OPEN:
            state = DROPPING_CARGO;
            break;
        case DROPPING_CARGO:
            state = ARM_MOVING_TO_PICKUP_ZONE;
            break;
    }

    switch (state) {
        case INITIALIZING:
            break;
        case MOVING_TO_CENTER:
            movingToCenterInit();
            break;
        case ARM_MOVING_TO_PICKUP_ZONE:
            armMovingToPickupZoneInit();
            break;
        case WAITING_FOR_CARGO:
            waitingForCargoInit();
            break;
        case LOADING_CARGO_CLOSE:
            loadingCargoCloseInit();
            break;
        case ARM_MOVING_TO_DROP_ZONE:
            armMovingToDropZoneInit();
            break;
        case DROPPING_CARGO_OPEN:
            droppingCargoOpenInit();
            break;
        case DROPPING_CARGO:
            droppingCargoInit();
            break;
    }


    sendJson("state", stateToString(state).c_str(), true, true);
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
        case LOADING_CARGO_CLOSE:
            loadingCargoClose();
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
    }
}

String stateToString(RobotState state) {
    switch (state) {
        case INITIALIZING:
            return "INITIALIZING";
        case MOVING_TO_CENTER:
            return "MOVING_TO_CENTER";
        case ARM_MOVING_TO_PICKUP_ZONE:
            return "ARM_MOVING_TO_PICKUP_ZONE";
        case WAITING_FOR_CARGO:
            return "WAITING_FOR_CARGO";
        case LOADING_CARGO_CLOSE:
            return "LOADING_CARGO_CLOSE";
        case ARM_MOVING_TO_DROP_ZONE:
            return "ARM_MOVING_TO_DROP_ZONE";
        case DROPPING_CARGO_OPEN:
            return "DROPPING_CARGO_OPEN";
        case DROPPING_CARGO:
            return "DROPPING_CARGO";
        case DONE:
            return "DONE";
        default:
            return "UNKNOWN";
    }
}