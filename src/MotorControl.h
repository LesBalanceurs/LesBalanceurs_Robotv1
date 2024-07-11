/****************************************************************************************
Nom du fichier : MotorControl.h
Auteur :                   
Date de création : 
  
****************************************************************************************/
#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H
#include "ArduinoX/ArduinoX.h"
#include "PID/PID.h"
#include "mathX.h"
#include "PidLayer.h"

#define DEFAULT_GEAR_BOX_RATIO 1
#define DEFAULT_ENCODER_TICK_PER_TURN 1
#define DEFAULT_CONVERTION_RATIO 1
#define DEFAULT_LAYER MOTOR

class MotorControl
{
    public:
        typedef void(MotorControl::*CommandFunction)(double);
        typedef double(MotorControl::*MeasureFunction)();

	    MotorControl(ArduinoX* arduinoX, PidLayer layer, int motorID, int encoderID, double gearBoxRatio, double encoderTickPerTurn, double convertionRatio);
        MotorControl(ArduinoX* arduinoX, PidLayer layer, int motorID, int encoderID, double gearBoxRatio, double encoderTickPerTurn);
        MotorControl(ArduinoX* arduinoX, PidLayer layer, int motorID, int encoderID, double encoderTickPerTurn);
        MotorControl(ArduinoX* arduinoX, PidLayer layer, int motorID, int encoderID);

        MotorControl(ArduinoX* arduinoX, int motorID, int encoderID, double gearBoxRatio, double encoderTickPerTurn, double convertionRatio);
        MotorControl(ArduinoX* arduinoX, int motorID, int encoderID, double gearBoxRatio, double encoderTickPerTurn);
        MotorControl(ArduinoX* arduinoX, int motorID, int encoderID, double encoderTickPerTurn);
        MotorControl(ArduinoX* arduinoX, int motorID, int encoderID);

        void setMotorID(int motorID);
        int getMotorID();

        void setEncoderID(int encoderID);
        int getEncoderID();

        void setLayer(PidLayer layer);
        PidLayer getLayer();


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
        double getFirstConversionStage();
        double getLastConversionStage();
        

    private:
        double gearBoxRatio;
        double encoderTickPerTurn;
        double convertionRatio;
        int motorID;
        int encoderID;
        PidLayer layer;
        ArduinoX* arduinoX;
};

#endif // MOTOR_CONTROL_H