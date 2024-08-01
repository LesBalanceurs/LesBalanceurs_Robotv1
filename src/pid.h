#ifndef PID_H
#define PID_H

#include <Arduino.h>
#include <float.h>

namespace PIDI {
        struct valeursPID {
        valeursPID() : Kp(0.0), Ki(0.0), Kd(0.0), initialTime(0), Sp(0.0), Pv(0.0), integral(0.0), previous_error(0.0), previous_derivative(0.0), Out(0.0), Min(-INFINITY), Max(INFINITY) {}
        float Kp;           // Constante proportionnelle
        float Ki;           // Constante intégrale
        float Kd;           // Constante dérivée
        long initialTime;   // Temps initial
        float Sp;           // Set Point (Valeur voulue)
        float Pv;           // Process Value (Valeur réelle)
        float integral;     // Valeur intégrale
        float previous_error;  // Previous error value
        float previous_derivative;
        float Out;          // Valeur de sortie
        float Min;
        float Max;

        static valeursPID create(float Kp, float Ki, float Kd, float integralCutOff = INFINITY) {
            valeursPID pid = {};
            pid.set(Kp, Ki, Kd, integralCutOff);
            return pid;
        }

        void set(float Kp, float Ki, float Kd, float Min = -INFINITY, float Max = INFINITY) {
            this->Kp = Kp;
            this->Ki = Ki;
            this->Kd = Kd;
            this->Min = Min;
            this->Max = Max;
        }

        float update() {
            unsigned long currentTime = micros();  // Get current time in milliseconds

            // Calculate the time difference (dt) since the last update
            float dt = (currentTime - initialTime) / 1000000.0; // Convert to seconds

            // Calculate the error
            float error = Sp - Pv;

            // Make not NaN or Infinite values are introduced
            if (!isfinite(error) || !isfinite(dt)) {
                return Out;
            }

            // Make sure dt isn't too small or too large
            const float epsilon_min = 0.000005;
            const float epsilon_max = 5;
            if (dt < epsilon_min || dt > epsilon_max || dt == 0) {
                return Out;
            }

            // Update the integral term
            if (Out > Min && Out < Max) {
                integral += error * dt;
            } else {
                integral = 0;
            }

            // Make sure it doesn't overflow
            integral = constrain(integral, -FLT_MAX, FLT_MAX);

            // Calculate the derivative term
            float derivative = (error - previous_error) / dt;

            // Calculate the control output
            Out = constrain((Kp * error) + (Ki * integral) + (Kd * derivative), Min, Max);

            // Store the current time for the next update
            initialTime = currentTime;

            // Store the current error for the next iteration
            previous_error = error;
            previous_derivative = derivative;

            return Out;
        }
    };
};

#endif // PID_H