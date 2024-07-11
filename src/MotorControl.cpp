/****************************************************************************************
Nom du fichier : prototype.h
Auteur :                   
Date de création : 
  
****************************************************************************************/
#include "MotorControl.h"

/*============================================================================
								PUBLIC
============================================================================*/

MotorControl::MotorControl(ArduinoX* arduinoX, PidLayer layer, int motorID, int encoderID, double gearBoxRatio, double encoderTickPerTurn, double convertionRatio) {
	this->arduinoX = arduinoX;
	setLayer(layer);
	setMotorID(motorID);
	setEncoderID(encoderID);
	setGearBoxRatio(gearBoxRatio);
	setEncoderTickPerTurn(encoderTickPerTurn);
	setConversionRatio(convertionRatio);
}
MotorControl::MotorControl(ArduinoX* arduinoX, PidLayer layer, int motorID, int encoderID, double gearBoxRatio, double encoderTickPerTurn): MotorControl(arduinoX, layer, motorID, encoderID, gearBoxRatio, encoderTickPerTurn, DEFAULT_CONVERTION_RATIO) {}
MotorControl::MotorControl(ArduinoX* arduinoX, PidLayer layer, int motorID, int encoderID, double encoderTickPerTurn): MotorControl(arduinoX, layer, motorID, encoderID, DEFAULT_GEAR_BOX_RATIO, encoderTickPerTurn) {}
MotorControl::MotorControl(ArduinoX* arduinoX, PidLayer layer, int motorID, int encoderID): MotorControl(arduinoX, layer, motorID, encoderID, DEFAULT_ENCODER_TICK_PER_TURN) {}


MotorControl::MotorControl::MotorControl(ArduinoX* arduinoX, int motorID, int encoderID, double gearBoxRatio, double encoderTickPerTurn, double convertionRatio): MotorControl(arduinoX, DEFAULT_LAYER, motorID, encoderID, encoderTickPerTurn, convertionRatio) {}
MotorControl::MotorControl(ArduinoX* arduinoX, int motorID, int encoderID, double gearBoxRatio, double encoderTickPerTurn): MotorControl(arduinoX, DEFAULT_LAYER, motorID, encoderID, encoderTickPerTurn) {}
MotorControl::MotorControl(ArduinoX* arduinoX, int motorID, int encoderID, double encoderTickPerTurn): MotorControl(arduinoX, DEFAULT_LAYER, motorID, encoderID) {}
MotorControl::MotorControl(ArduinoX* arduinoX, int motorID, int encoderID): MotorControl(arduinoX, DEFAULT_LAYER, motorID, encoderID) {}

void MotorControl::setMotorID(int motorID) {
	this->motorID = motorID;
}
int MotorControl::getMotorID() {
	return this->motorID;
}

void MotorControl::setEncoderID(int encoderID) {
	this->encoderID = encoderID;
}
int MotorControl::getEncoderID() {
	return this->encoderID;
}

void MotorControl::setGearBoxRatio(double gearBoxRatio) {
	this->gearBoxRatio = gearBoxRatio;
}
double MotorControl::getGearBoxRatio() {
	return this->gearBoxRatio;
}

void MotorControl::setEncoderTickPerTurn(double encoderTickPerTurn) {
	this->encoderTickPerTurn = encoderTickPerTurn;
}
double MotorControl::getEncoderTickPerTurn() {
	return this->encoderTickPerTurn;
}

void MotorControl::setConversionRatio(double convertionRatio) {
	this->convertionRatio = convertionRatio;
}
double MotorControl::getConversionRatio() {
	return this->convertionRatio;
}

void MotorControl::setLayer(PidLayer layer) {
	this->layer = layer;
}
PidLayer MotorControl::getLayer() {
	return this->layer;
}

double MotorControl::measurementFunction() {
	return arduinoX->readEncoder(getEncoderID()) / getFirstConversionStage();
}
void MotorControl::commandFunction(double command) {
	arduinoX->setMotorPWM(getMotorID(), clamp(command / getLastConversionStage(), -1.0, 1.0));
}

double MotorControl::wheelRatio(double diameter) {
	return PI * diameter;
}
double MotorControl::degreeRatio() {
	return 360.0;
}
double MotorControl::radiansRatio() {
	return 2.0 * PI;
}
double MotorControl::turnRatio() {
	return 1.0;
}

/*============================================================================
								PRIVATE
============================================================================*/

double MotorControl::getFirstConversionStage() {
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
double MotorControl::getLastConversionStage() {
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