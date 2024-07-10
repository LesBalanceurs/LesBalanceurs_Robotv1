/****************************************************************************************
Nom du fichier : MotorControl.h
Auteur :                   
Date de création : 
  
****************************************************************************************/
#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H
#include "ArduinoX/ArduinoX.h"
#include "mathX.h"

#define DEFAULT_GEAR_BOX_RATIO 1
#define DEFAULT_ENCODER_TICK_PER_TURN 1
#define DEFAULT_CONVERTION_RATIO 1

class MotorControl
{
    public:
	    MotorControl(ArduinoX* arduinoX, int motorID, int encoderID, double gearBoxRatio, double encoderTickPerTurn, double convertionRatio);
        MotorControl(ArduinoX* arduinoX, int motorID, int encoderID, double gearBoxRatio, double encoderTickPerTurn);
        MotorControl(ArduinoX* arduinoX, int motorID, int encoderID, double encoderTickPerTurn);
        MotorControl(ArduinoX* arduinoX, int motorID, int encoderID);

        void setMotorID(int motorID);
        int getMotorID();

        void setEncoderID(int encoderID);
        int getEncoderID();

        void setGearBoxRatio(double gearBoxRatio);
        double getGearBoxRatio();

        void setEncoderTickPerTurn(double encoderTickPerTurn);
        double getEncoderTickPerTurn();

        void setConversionRatio(double convertionRatio);
        double getConversionRatio();

        double measurementFunction();
        void commandFunction(double command);

        static double wheelRatio(double diameter);
        static double degreeRatio();
        static double radiansRatio();
        static double turnRatio();

    private:
        double getEncoderControlRatio();
        

    private:
        double gearBoxRatio;
        double encoderTickPerTurn;
        double convertionRatio;
        int motorID;
        int encoderID;
        ArduinoX* arduinoX;
};

#endif // MOTOR_CONTROL_H