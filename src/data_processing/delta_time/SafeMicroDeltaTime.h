/****************************************************************************************
Nom du fichier : SafeMicroDeltaTime.h
Auteur : Mathieu Durand      
Date de création : 17 juillet 2024

****************************************************************************************/

#ifndef SAFE_MICRO_DELTA_TIME_H
#define SAFE_MICRO_DELTA_TIME_H

#define DEFAULT_INTEGRAL_VALUE 0

/**
 * @class SafeMicroDeltaTime
 * @brief Class for computing delta time with microseconds precision while handling overflow in a transparent manner up to delta time of 70 minutes
 */

class SafeMicroDeltaTime
{
    public:
        SafeMicroDeltaTime();

        void tick();

        double getHours();
        double getMinutes();
        double getSeconds();
        double getMilliseconds();
        unsigned long getMicroseconds();
    
    private:
        unsigned long lastTimeMicro;
        unsigned long deltaTimeMicro;

};

#endif // SAFE_MICRO_DELTA_TIME_H
