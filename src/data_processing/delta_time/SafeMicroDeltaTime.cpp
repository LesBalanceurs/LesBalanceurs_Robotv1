/****************************************************************************************
Nom du fichier : SafeMicroDeltaTime.cpp
Auteur : Mathieu Durand        
Date de création : 17 juillet 2024
  
****************************************************************************************/
#include "SafeMicroDeltaTime.h"
#include "Arduino.h"
#include <limits.h>

/*============================================================================
								PUBLIC
============================================================================*/

SafeMicroDeltaTime::SafeMicroDeltaTime() {
    lastTimeMicro = millis();
}


void SafeMicroDeltaTime::tick() {
    unsigned long tempTime = micros();
    if (tempTime < lastTimeMicro) {
        deltaTimeMicro =  (ULONG_MAX - lastTimeMicro) + tempTime;
    } else {
        deltaTimeMicro = tempTime - lastTimeMicro;
    }

    lastTimeMicro = tempTime;
}

double SafeMicroDeltaTime::getHours() {
    return getMicroseconds() / 3600000000.0;
}

double SafeMicroDeltaTime::getMinutes() {
    return getMicroseconds() / 60000000.0;
}

double SafeMicroDeltaTime::getSeconds() {
    return getMicroseconds() / 1000000.0;
}

double SafeMicroDeltaTime::getMilliseconds() {
    return getMicroseconds() / 1000.0;
}

unsigned long SafeMicroDeltaTime::getMicroseconds() {
    return this->deltaTimeMicro;
}