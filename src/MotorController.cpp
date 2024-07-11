/****************************************************************************************
Nom du fichier : prototype.h
Auteur :                   
Date de création : 
  
****************************************************************************************/
#include "MotorController.h"

/*============================================================================
								PUBLIC
============================================================================*/

MotorController::MotorController(ArduinoX* arduinoX, PidLayer layer, int motorID, int encoderID, double gearBoxRatio, double encoderTickPerTurn, double convertionRatio) {
	this->arduinoX = arduinoX;
	setLayer(layer);
	setMotorID(motorID);
	setEncoderID(encoderID);
	setGearBoxRatio(gearBoxRatio);
	setEncoderTickPerTurn(encoderTickPerTurn);
	setConversionRatio(convertionRatio);
}
MotorController::MotorController(ArduinoX* arduinoX, PidLayer layer, int motorID, int encoderID, double gearBoxRatio, double encoderTickPerTurn): MotorController(arduinoX, layer, motorID, encoderID, gearBoxRatio, encoderTickPerTurn, DEFAULT_CONVERTION_RATIO) {}
MotorController::MotorController(ArduinoX* arduinoX, PidLayer layer, int motorID, int encoderID, double encoderTickPerTurn): MotorController(arduinoX, layer, motorID, encoderID, DEFAULT_GEAR_BOX_RATIO, encoderTickPerTurn) {}
MotorController::MotorController(ArduinoX* arduinoX, PidLayer layer, int motorID, int encoderID): MotorController(arduinoX, layer, motorID, encoderID, DEFAULT_ENCODER_TICK_PER_TURN) {}


MotorController::MotorController(ArduinoX* arduinoX, int motorID, int encoderID, double gearBoxRatio, double encoderTickPerTurn, double convertionRatio): MotorController(arduinoX, DEFAULT_LAYER, motorID, encoderID, encoderTickPerTurn, convertionRatio) {}
MotorController::MotorController(ArduinoX* arduinoX, int motorID, int encoderID, double gearBoxRatio, double encoderTickPerTurn): MotorController(arduinoX, DEFAULT_LAYER, motorID, encoderID, encoderTickPerTurn) {}
MotorController::MotorController(ArduinoX* arduinoX, int motorID, int encoderID, double encoderTickPerTurn): MotorController(arduinoX, DEFAULT_LAYER, motorID, encoderID) {}
MotorController::MotorController(ArduinoX* arduinoX, int motorID, int encoderID): MotorController(arduinoX, DEFAULT_LAYER, motorID, encoderID) {}

void MotorController::setMotorID(int motorID) {
	this->motorID = motorID;
}
int MotorController::getMotorID() {
	return this->motorID;
}

void MotorController::setEncoderID(int encoderID) {
	this->encoderID = encoderID;
}
int MotorController::getEncoderID() {
	return this->encoderID;
}

void MotorController::setGearBoxRatio(double gearBoxRatio) {
	this->gearBoxRatio = gearBoxRatio;
}
double MotorController::getGearBoxRatio() {
	return this->gearBoxRatio;
}

void MotorController::setEncoderTickPerTurn(double encoderTickPerTurn) {
	this->encoderTickPerTurn = encoderTickPerTurn;
}
double MotorController::getEncoderTickPerTurn() {
	return this->encoderTickPerTurn;
}

void MotorController::setConversionRatio(double convertionRatio) {
	this->convertionRatio = convertionRatio;
}
double MotorController::getConversionRatio() {
	return this->convertionRatio;
}

void MotorController::setLayer(PidLayer layer) {
	this->layer = layer;
}
PidLayer MotorController::getLayer() {
	return this->layer;
}

double MotorController::measurementFunction() {
	return arduinoX->readEncoder(getEncoderID()) / getFirstConversionStage();
}
void MotorController::commandFunction(double command) {
	arduinoX->setMotorPWM(getMotorID(), constrain(command / getLastConversionStage(), -1.0, 1.0));
}

double MotorController::wheelRatio(double diameter) {
	return PI * diameter;
}
double MotorController::degreesRatio() {
	return 360.0;
}
double MotorController::radiansRatio() {
	return 2.0 * PI;
}
double MotorController::turnRatio() {
	return 1.0;
}

/*============================================================================
								PRIVATE
============================================================================*/

double MotorController::getFirstConversionStage() {
	double ratio = 1;

	if (getLayer() >= MOTOR) {
		ratio *= getEncoderTickPerTurn();
	}
	if (getLayer() >= GEARBOX) {
		ratio *= getGearBoxRatio();
	}
	if (getLayer() >= DIRECT) {
		ratio /= getConversionRatio();
	};

	return ratio;
}
double MotorController::getLastConversionStage() {
	double ratio = 1;

	if (getLayer() < MOTOR) {
		ratio *= getEncoderTickPerTurn();
	}
	if (getLayer() < GEARBOX) {
		ratio *= getGearBoxRatio();
	}
	if (getLayer() < DIRECT) {
		ratio /= getConversionRatio();
	}

	return ratio;
}