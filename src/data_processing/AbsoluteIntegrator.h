/****************************************************************************************
Nom du fichier : Integrator.h
Auteur : Mathieu Durand      
Date de création : 17 juillet 2024

****************************************************************************************/

#ifndef ABSOLUTE_INTEGRATOR_H
#define ABSOLUTE_INTEGRATOR_H

#include "Integrator.h"

/**
 * @class ChangeSummator
 * @brief Class for summing change, similar to the concept of displacement
 */

class AbsoluteIntegrator: public Integrator
{
    public:
        AbsoluteIntegrator(double initialIntegral);
        AbsoluteIntegrator();

        void integrate(double value);
};

#endif // ABSOLUTE_INTEGRATOR_H
