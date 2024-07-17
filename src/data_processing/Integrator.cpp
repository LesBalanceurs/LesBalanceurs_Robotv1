/****************************************************************************************
Nom du fichier : Integrator.cpp
Auteur : Mathieu Durand        
Date de création : 17 juillet 2024
  
****************************************************************************************/
#include "Integrator.h"

/*============================================================================
								PUBLIC
============================================================================*/

Integrator::Integrator(double initialIntegral) {
    this->setIntegral(initialIntegral);
}
 
Integrator::Integrator(): Integrator(DEFAULT_INTEGRAL_VALUE) {}

double Integrator::getIntegral() {
    return this->integral;
}

void Integrator::setIntegral(double integral) {
    this->integral = integral;
}

void Integrator::integrate(double value) {
    deltaTimeProvider.tick();
    setIntegral(getIntegral() + value * deltaTimeProvider.getSeconds());
}

SafeMicroDeltaTime* Integrator::getDeltaTimeProvider() {
    return &deltaTimeProvider;
}