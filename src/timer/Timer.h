/****************************************************************************************
Nom du fichier : Integrator.h
Auteur : Mathieu Durand      
Date de création : 17 juillet 2024

****************************************************************************************/

#ifndef TIMER_H
#define TIMER_H

/**
 * @class Timer
 * @brief Class for managing waiting for period of time asynchronously
 */

class Timer
{
    public:
        void start(double seconds);
        void start(unsigned long milliseconds);

        double getTimeLeftSeconds();
        unsigned long getTimeLeftMilliseconds();

        bool isTimerOver();
    
    private:
        unsigned long targetMillis;

};

#endif // TIMER_H
