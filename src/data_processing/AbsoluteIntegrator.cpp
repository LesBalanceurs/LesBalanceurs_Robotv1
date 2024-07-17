/****************************************************************************************
Nom du fichier : Integrator.cpp
Auteur : Mathieu Durand        
Date de création : 17 juillet 2024
  
****************************************************************************************/
#include "AbsoluteIntegrator.h"
#include "Arduino.h"

/*============================================================================
								PUBLIC
============================================================================*/

AbsoluteIntegrator::AbsoluteIntegrator(double initialIntegral): Integrator(initialIntegral) {}
AbsoluteIntegrator::AbsoluteIntegrator(): Integrator() {}

void AbsoluteIntegrator::integrate(double value) {
    Integrator::integrate(abs(value));
}