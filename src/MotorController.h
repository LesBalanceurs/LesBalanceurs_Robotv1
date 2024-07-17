/****************************************************************************************
Nom du fichier : MotorControllerler.h
Auteur : Mathieu Durand      
Date de création : 11 juillet 2024

****************************************************************************************/

#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "ArduinoX/ArduinoX.h"
#include "PidLayer.h"

#define DEFAULT_GEAR_BOX_RATIO 1
#define DEFAULT_ENCODER_TICK_PER_TURN 1
#define DEFAULT_CONVERTION_RATIO 1
#define DEFAULT_LAYER MOTOR

/**
 * @class MotorController
 * @brief Class for controlling a motor using an Arduino and a PID controller.
 */
class MotorController
{
    public:
        /**
         * @brief Constructor with all parameters.
         * @param arduinoX Pointer to ArduinoX instance.
         * @param layer PID layer.
         * @param motorID ID of the motor.
         * @param encoderID ID of the encoder.
         * @param gearBoxRatio Gearbox ratio.
         * @param encoderTickPerTurn Encoder ticks per turn.
         * @param convertionRatio Conversion ratio.
         */
        MotorController(ArduinoX* arduinoX, PidLayer layer, int motorID, int encoderID, double gearBoxRatio, double encoderTickPerTurn, double convertionRatio);
        
        /**
         * @brief Constructor without conversion ratio.
         * @param arduinoX Pointer to ArduinoX instance.
         * @param layer PID layer.
         * @param motorID ID of the motor.
         * @param encoderID ID of the encoder.
         * @param gearBoxRatio Gearbox ratio.
         * @param encoderTickPerTurn Encoder ticks per turn.
         */
        MotorController(ArduinoX* arduinoX, PidLayer layer, int motorID, int encoderID, double gearBoxRatio, double encoderTickPerTurn);
        
        /**
         * @brief Constructor with default gearbox ratio.
         * @param arduinoX Pointer to ArduinoX instance.
         * @param layer PID layer.
         * @param motorID ID of the motor.
         * @param encoderID ID of the encoder.
         * @param encoderTickPerTurn Encoder ticks per turn.
         */
        MotorController(ArduinoX* arduinoX, PidLayer layer, int motorID, int encoderID, double encoderTickPerTurn);
        
        /**
         * @brief Constructor with default gearbox ratio and encoder ticks per turn.
         * @param arduinoX Pointer to ArduinoX instance.
         * @param layer PID layer.
         * @param motorID ID of the motor.
         * @param encoderID ID of the encoder.
         */
        MotorController(ArduinoX* arduinoX, PidLayer layer, int motorID, int encoderID);

        /**
         * @brief Constructor with all parameters except layer.
         * @param arduinoX Pointer to ArduinoX instance.
         * @param motorID ID of the motor.
         * @param encoderID ID of the encoder.
         * @param gearBoxRatio Gearbox ratio.
         * @param encoderTickPerTurn Encoder ticks per turn.
         * @param convertionRatio Conversion ratio.
         */
        MotorController(ArduinoX* arduinoX, int motorID, int encoderID, double gearBoxRatio, double encoderTickPerTurn, double convertionRatio);
        
        /**
         * @brief Constructor without conversion ratio and layer.
         * @param arduinoX Pointer to ArduinoX instance.
         * @param motorID ID of the motor.
         * @param encoderID ID of the encoder.
         * @param gearBoxRatio Gearbox ratio.
         * @param encoderTickPerTurn Encoder ticks per turn.
         */
        MotorController(ArduinoX* arduinoX, int motorID, int encoderID, double gearBoxRatio, double encoderTickPerTurn);
        
        /**
         * @brief Constructor with default gearbox ratio and without layer.
         * @param arduinoX Pointer to ArduinoX instance.
         * @param motorID ID of the motor.
         * @param encoderID ID of the encoder.
         * @param encoderTickPerTurn Encoder ticks per turn.
         */
        MotorController(ArduinoX* arduinoX, int motorID, int encoderID, double encoderTickPerTurn);
        
        /**
         * @brief Constructor with default gearbox ratio, encoder ticks per turn, and without layer.
         * @param arduinoX Pointer to ArduinoX instance.
         * @param motorID ID of the motor.
         * @param encoderID ID of the encoder.
         */
        MotorController(ArduinoX* arduinoX, int motorID, int encoderID);

        /**
         * @brief Set the motor ID.
         * @param motorID ID of the motor.
         */
        void setMotorID(int motorID);

        /**
         * @brief Get the motor ID.
         * @return ID of the motor.
         */
        int getMotorID();

        /**
         * @brief Set the encoder ID.
         * @param encoderID ID of the encoder.
         */
        void setEncoderID(int encoderID);

        /**
         * @brief Get the encoder ID.
         * @return ID of the encoder.
         */
        int getEncoderID();

        /**
         * @brief Set the PID layer.
         * @param layer PID layer.
         */
        void setLayer(PidLayer layer);

        /**
         * @brief Get the PID layer.
         * @return PID layer.
         */
        PidLayer getLayer();

        /**
         * @brief Set the gearbox ratio.
         * @param gearBoxRatio Gearbox ratio.
         */
        void setGearBoxRatio(double gearBoxRatio);

        /**
         * @brief Get the gearbox ratio.
         * @return Gearbox ratio.
         */
        double getGearBoxRatio();

        /**
         * @brief Set the encoder ticks per turn.
         * @param encoderTickPerTurn Encoder ticks per turn.
         */
        void setEncoderTickPerTurn(double encoderTickPerTurn);

        /**
         * @brief Get the encoder ticks per turn.
         * @return Encoder ticks per turn.
         */
        double getEncoderTickPerTurn();

        /**
         * @brief Set the conversion ratio.
         * @param convertionRatio Conversion ratio.
         */
        void setConversionRatio(double convertionRatio);

        /**
         * @brief Get the conversion ratio.
         * @return Conversion ratio.
         */
        double getConversionRatio();

        /**
         * @brief Measurement function for the motor.
         * @return Measurement value.
         * 
         * This function reads the encoder value and converts it using the
         * first conversion stage to get the measured value.
         */
        double measurementFunction();

        /**
         * @brief Command function for the motor.
         * @param command Command value.
         * 
         * This function sets the motor PWM value based on the given command,
         * constrained between -1.0 and 1.0, after adjusting with the last
         * conversion stage.
         */
        void commandFunction(double command);

        /**
         * @brief Calculate the wheel ratio based on diameter.
         * @param diameter Diameter of the wheel.
         * @return Wheel ratio.
         */
        static double wheelRatio(double diameter);

        /**
         * @brief Calculate the degrees ratio.
         * @return Degrees ratio.
         */
        static double degreesRatio();

        /**
         * @brief Calculate the radians ratio.
         * @return Radians ratio.
         */
        static double radiansRatio();

        /**
         * @brief Calculate the turn ratio.
         * @return Turn ratio.
         */
        static double turnRatio();

    private:
        /**
         * @brief Get the first conversion stage.
         * @return First conversion stage value.
         * 
         * This function calculates the first conversion stage based on the
         * PID layer, encoder ticks per turn, gearbox ratio, and conversion ratio.
         */
        double getFirstConversionStage();

        /**
         * @brief Get the last conversion stage.
         * @return Last conversion stage value.
         * 
         * This function calculates the last conversion stage based on the
         * PID layer, encoder ticks per turn, gearbox ratio, and conversion ratio.
         */
        double getLastConversionStage();

    private:
        double gearBoxRatio;               /**< Gearbox ratio. */
        double encoderTickPerTurn;         /**< Encoder ticks per turn. */
        double convertionRatio;            /**< Conversion ratio. */
        int motorID;                       /**< Motor ID. */
        int encoderID;                     /**< Encoder ID. */
        PidLayer layer;                    /**< PID layer. */
        ArduinoX* arduinoX;                /**< Pointer to ArduinoX instance. */
};

#endif // MOTOR_CONTROLLER_H
