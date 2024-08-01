/****************************************************************************************
Nom du fichier : Timer.cpp
Auteur : Mathieu Durand        
Date de création : 17 juillet 2024
  
****************************************************************************************/
#include "Timer.h"
#include "Arduino.h"

/*============================================================================
								PUBLIC
============================================================================*/


void Timer::start(double seconds) {
    start((unsigned long) seconds * 1000);
}

void Timer::start(unsigned long milliseconds) {
    this->targetMillis = millis() + milliseconds;
}


double Timer::getTimeLeftSeconds() {
    return getTimeLeftMilliseconds() / 1000.0;
}

unsigned long Timer::getTimeLeftMilliseconds() {
    unsigned long timeLeft = this->targetMillis - millis();

    if (millis() >= this->targetMillis) {
        return 0;
    }

    return timeLeft;
}


bool Timer::isTimerOver() {
    return getTimeLeftMilliseconds() <= 0;
}