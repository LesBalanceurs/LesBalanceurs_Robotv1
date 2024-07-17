/****************************************************************************************
Nom du fichier : Integrator.h
Auteur : Mathieu Durand      
Date de création : 17 juillet 2024

****************************************************************************************/

#ifndef INTEGRATOR_H
#define INTEGRATOR_H

#define DEFAULT_INTEGRAL_VALUE 0

#include "delta_time/SafeMicroDeltaTime.h"

/**
 * @class ChangeSummator
 * @brief Class for summing change, similar to the concept of displacement
 */

class Integrator
{
    public:
        Integrator(double initialIntegral);
        Integrator();

        double getIntegral();
        void setIntegral(double integal);

        void integrate(double value);

        SafeMicroDeltaTime* getDeltaTimeProvider();
    
    private:
        double integral;
        SafeMicroDeltaTime deltaTimeProvider;

};

#endif // INTEGRATOR_H
