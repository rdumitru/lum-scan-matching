#ifndef _TIMER_H
#define _TIMER_H

//==============================================================================
// Includes.
//==============================================================================
// C++ includes.
#include <iostream>
#include <ostream>
#include <vector>

// C includes.
#include <time.h>
#include <unistd.h>
#include <sys/time.h>

//==============================================================================
// Class definition.
//==============================================================================
class Timer {
private:
    std::vector<clock_t> m_CpuTimes;
    std::vector<timeval> m_RealTimes;

public:
    // Constructors.
    Timer();
    Timer(const Timer &other);
    ~Timer();

    // Methods.
    void start();
    void reset();
    void record();

    void printTime(const std::string &msg,
                   const bool &sinceStart = false);

    // Getters and setters.
    long getCpuTime(const bool &sinceStart = false);
    long getRealTime(const bool &sinceStart = false);
};

#endif // _TIMER_H
